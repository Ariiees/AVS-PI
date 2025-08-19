// g++ -O3 -std=c++17 db_bench.cpp -o db_bench -lrocksdb -lsqlite3 -lpthread -ldl
//
// Assumptions:
// - All files are named as 13-digit millisecond timestamps: "1754957043231.jpg" or ".laz"
// - Directories:
//     images: /home/avs/DATA/SSD/images
//     lidar : /home/avs/DATA/SSD/lidar_laz
//
// What it does:
// 1) Walk both dirs, collect records {type, ts_ms, path}
// 2) Insert into SQLite3 (PRIMARY KEY(type, ts)) & RocksDB (key="type:0000000000000") with timing
// 3) Build a SHARED set of range queries: (type, [start_ms, end_ms])
//    - Default: 1000 queries, each with a 1000 ms window centered on a sample timestamp
// 4) For each DB, run all range queries, scanning all rows in the range (so work is equivalent)
// 5) Print average insert/query latency (ms, 0.0000) and DB size
//
// CLI:
//   ./db_bench [--images DIR] [--lidar DIR] [--sqlite FILE] [--rocks DIR]
//              [--ranges 1000] [--window-ms 1000]
//
// Notes:
// - RocksDB key format keeps lexical ordering: "<type>:<13-digit-ms>"
// - Range query for RocksDB uses iterator Seek(startKey) and scans until endKey/prefix boundary
// - SQLite range query uses: SELECT path FROM entries WHERE type=? AND ts BETWEEN ? AND ?;
// - We fully iterate each result set to make the amount of "work" comparable across DBs.

#include <rocksdb/db.h>
#include <rocksdb/options.h>
#include <sqlite3.h>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <random>
#include <string>
#include <vector>

namespace fs = std::filesystem;
using Clock = std::chrono::high_resolution_clock;
using nanoseconds = std::chrono::nanoseconds;

struct Record {
    std::string type;   // "image" or "lidar"
    int64_t ts_ms;      // 13-digit ms
    std::string path;   // absolute path
};

struct RangeQuery {
    std::string type;   // "image" or "lidar"
    int64_t start_ms;
    int64_t end_ms;
};

// ---------- Helpers ----------
static inline double ns_avg_ms(const std::vector<nanoseconds>& times) {
    if (times.empty()) return 0.0;
    long double sum = 0;
    for (auto& t : times) sum += t.count();
    return static_cast<double>(sum / times.size()) / 1e6;
}

static uint64_t file_size_if_exists(const fs::path& p) {
    std::error_code ec;
    if (fs::exists(p, ec) && fs::is_regular_file(p, ec)) {
        return static_cast<uint64_t>(fs::file_size(p, ec));
    }
    return 0;
}

static uint64_t dir_size_recursive(const fs::path& p) {
    std::error_code ec;
    if (!fs::exists(p, ec)) return 0;
    uint64_t total = 0;
    for (auto const& e : fs::recursive_directory_iterator(p, ec)) {
        if (ec) break;
        if (e.is_regular_file()) {
            total += static_cast<uint64_t>(fs::file_size(e.path(), ec));
            if (ec) ec.clear();
        }
    }
    return total;
}

static std::string padded_ts(int64_t ts) {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%013ld", ts);
    return std::string(buf);
}

static std::string rocks_key(const std::string& type, int64_t ts) {
    // Composite key: "<type>:<13-digit-ms>"
    return type + ":" + padded_ts(ts);
}

// ---------- Collect records ----------
static std::vector<Record> collect_records(const fs::path& images_dir, const fs::path& lidar_dir) {
    std::vector<Record> recs;

    auto add_dir = [&](const fs::path& root, const std::string& type, const std::string& ext){
        if (!fs::exists(root)) return;
        for (auto const& entry : fs::recursive_directory_iterator(root)) {
            if (!entry.is_regular_file()) continue;
            if (entry.path().extension() != ext) continue;
            try {
                int64_t ts = std::stoll(entry.path().stem().string());
                recs.push_back(Record{type, ts, fs::absolute(entry.path()).string()});
            } catch (...) {
                // skip anything not 13-digit numeric
            }
        }
    };

    add_dir(images_dir, "image", ".jpg");
    add_dir(lidar_dir,  "lidar", ".laz");

    std::sort(recs.begin(), recs.end(), [](auto& a, auto& b){
        if (a.type == b.type) return a.ts_ms < b.ts_ms;
        return a.type < b.type;
    });
    return recs;
}

// Build a shared set of range queries by sampling existing records.
// window_ms is the width of each range; we center at a sampled ts.
static std::vector<RangeQuery> build_range_queries(
    const std::vector<Record>& recs,
    size_t N_ranges,
    int64_t window_ms)
{
    std::vector<RangeQuery> qs;
    if (recs.empty() || N_ranges == 0) return qs;

    std::mt19937_64 rng(0xC0FFEE);
    std::uniform_int_distribution<size_t> dist(0, recs.size()-1);

    int64_t half = window_ms / 2;
    qs.reserve(N_ranges);
    for (size_t i=0; i<N_ranges; ++i) {
        const auto& r = recs[dist(rng)];
        int64_t start = r.ts_ms - half;
        int64_t end   = r.ts_ms + half;
        if (start > end) std::swap(start, end);
        qs.push_back(RangeQuery{r.type, start, end});
    }
    return qs;
}

// ---------- SQLite ----------
struct SQLiteBenchResult {
    double avg_insert_ms=0.0;
    double avg_range_query_ms=0.0;
    uint64_t size_bytes=0;
    size_t N_inserted=0;
    size_t N_q=0;        // number of range queries executed
    uint64_t total_rows_scanned=0; // to sanity-check comparable work
};

static SQLiteBenchResult bench_sqlite(
    const std::vector<Record>& recs,
    const fs::path& db_path,
    const std::vector<RangeQuery>& ranges)
{
    SQLiteBenchResult R;
    sqlite3* db=nullptr;

    // clean old
    std::error_code ec;
    fs::remove(db_path, ec);
    fs::remove(db_path.string()+"-wal", ec);
    fs::remove(db_path.string()+"-shm", ec);

    if (sqlite3_open_v2(db_path.string().c_str(), &db, SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE, nullptr) != SQLITE_OK) {
        std::cerr << "[SQLite] open error: " << sqlite3_errmsg(db) << "\n";
        return R;
    }

    auto exec_sql = [&](const char* sql){
        char* err=nullptr;
        if (sqlite3_exec(db, sql, nullptr, nullptr, &err) != SQLITE_OK) {
            if (err) {
                std::cerr << "[SQLite] exec error: " << err << "\n";
                sqlite3_free(err);
            }
        }
    };

    exec_sql("PRAGMA journal_mode=WAL;");
    exec_sql("PRAGMA synchronous=NORMAL;");
    exec_sql("PRAGMA temp_store=MEMORY;");
    exec_sql("PRAGMA mmap_size=268435456;");

    exec_sql("CREATE TABLE IF NOT EXISTS entries ("
             " type TEXT NOT NULL,"
             " ts   INTEGER NOT NULL,"
             " path TEXT NOT NULL,"
             " PRIMARY KEY(type, ts)"
             ");");

    // insert
    sqlite3_stmt* ins=nullptr;
    if (sqlite3_prepare_v2(db, "INSERT INTO entries(type,ts,path) VALUES (?,?,?);", -1, &ins, nullptr) != SQLITE_OK) {
        std::cerr << "[SQLite] prepare insert error: " << sqlite3_errmsg(db) << "\n";
        sqlite3_close(db);
        return R;
    }

    exec_sql("BEGIN TRANSACTION;");
    std::vector<nanoseconds> insert_times;
    insert_times.reserve(recs.size());

    for (const auto& r : recs) {
        sqlite3_reset(ins);
        sqlite3_clear_bindings(ins);
        sqlite3_bind_text (ins, 1, r.type.c_str(), -1, SQLITE_TRANSIENT);
        sqlite3_bind_int64(ins, 2, r.ts_ms);
        sqlite3_bind_text (ins, 3, r.path.c_str(), -1, SQLITE_TRANSIENT);

        auto t0 = Clock::now();
        int rc = sqlite3_step(ins);
        auto t1 = Clock::now();
        if (rc != SQLITE_DONE) {
            std::cerr << "[SQLite] insert error: " << sqlite3_errmsg(db) << "\n";
        } else {
            insert_times.push_back(std::chrono::duration_cast<nanoseconds>(t1 - t0));
            ++R.N_inserted;
        }
    }
    exec_sql("COMMIT;");
    sqlite3_finalize(ins);

    // range query
    sqlite3_stmt* sel=nullptr;
    if (sqlite3_prepare_v2(db,
        "SELECT path FROM entries WHERE type=? AND ts BETWEEN ? AND ?;",
        -1, &sel, nullptr) != SQLITE_OK)
    {
        std::cerr << "[SQLite] prepare select error: " << sqlite3_errmsg(db) << "\n";
        sqlite3_close(db);
        return R;
    }

    std::vector<nanoseconds> range_times;
    range_times.reserve(ranges.size());

    for (const auto& q : ranges) {
        sqlite3_reset(sel);
        sqlite3_clear_bindings(sel);
        sqlite3_bind_text (sel, 1, q.type.c_str(), -1, SQLITE_TRANSIENT);
        sqlite3_bind_int64(sel, 2, q.start_ms);
        sqlite3_bind_int64(sel, 3, q.end_ms);

        uint64_t rows = 0;
        auto t0 = Clock::now();
        while (sqlite3_step(sel) == SQLITE_ROW) {
            // fetch a column to ensure access (optional)
            // const unsigned char* p = sqlite3_column_text(sel, 0);
            ++rows;
        }
        auto t1 = Clock::now();

        range_times.push_back(std::chrono::duration_cast<nanoseconds>(t1 - t0));
        R.total_rows_scanned += rows;
        ++R.N_q;
    }
    sqlite3_finalize(sel);
    sqlite3_close(db);

    R.size_bytes      = file_size_if_exists(db_path)
                      + file_size_if_exists(db_path.string()+"-wal")
                      + file_size_if_exists(db_path.string()+"-shm");
    R.avg_insert_ms   = ns_avg_ms(insert_times);
    R.avg_range_query_ms = ns_avg_ms(range_times);
    return R;
}

// ---------- RocksDB ----------
struct RocksBenchResult {
    double avg_insert_ms=0.0;
    double avg_range_query_ms=0.0;
    uint64_t size_bytes=0;
    size_t N_inserted=0;
    size_t N_q=0;
    uint64_t total_rows_scanned=0;
};

static RocksBenchResult bench_rocks(
    const std::vector<Record>& recs,
    const fs::path& db_dir,
    const std::vector<RangeQuery>& ranges)
{
    RocksBenchResult R;

    // wipe
    std::error_code ec;
    if (fs::exists(db_dir, ec)) fs::remove_all(db_dir, ec);

    rocksdb::Options options;
    options.create_if_missing = true;
    options.compression = rocksdb::kLZ4Compression;

    rocksdb::DB* db=nullptr;
    auto status = rocksdb::DB::Open(options, db_dir.string(), &db);
    if (!status.ok()) {
        std::cerr << "[RocksDB] open error: " << status.ToString() << "\n";
        return R;
    }

    rocksdb::WriteOptions wopt;
    wopt.sync = false;
    rocksdb::ReadOptions ropt;

    // inserts
    std::vector<nanoseconds> insert_times;
    insert_times.reserve(recs.size());

    for (const auto& r : recs) {
        std::string key = rocks_key(r.type, r.ts_ms);
        std::string val = r.type + "|" + r.path;

        auto t0 = Clock::now();
        auto s = db->Put(wopt, key, val);
        auto t1 = Clock::now();
        if (!s.ok()) {
            std::cerr << "[RocksDB] put error: " << s.ToString() << "\n";
        } else {
            insert_times.push_back(std::chrono::duration_cast<nanoseconds>(t1 - t0));
            ++R.N_inserted;
        }
    }

    // range queries with iterator
    std::vector<nanoseconds> range_times;
    range_times.reserve(ranges.size());

    for (const auto& q : ranges) {
        const std::string start_key = rocks_key(q.type, q.start_ms);
        const std::string end_key   = rocks_key(q.type, q.end_ms);
        const std::string prefix    = q.type + ":";

        std::unique_ptr<rocksdb::Iterator> it(db->NewIterator(ropt));

        uint64_t rows = 0;
        auto t0 = Clock::now();

        for (it->Seek(start_key); it->Valid(); it->Next()) {
            const rocksdb::Slice k = it->key();
            // stop if we leave the type prefix
            if (k.size() < prefix.size() || std::memcmp(k.data(), prefix.data(), prefix.size()) != 0) break;
            // stop if beyond end_key
            if (k.ToString() > end_key) break;

            // touch value to ensure access (optional)
            // auto v = it->value();

            ++rows;
        }

        auto t1 = Clock::now();
        range_times.push_back(std::chrono::duration_cast<nanoseconds>(t1 - t0));
        R.total_rows_scanned += rows;
        ++R.N_q;
    }

    delete db;

    R.size_bytes         = dir_size_recursive(db_dir);
    R.avg_insert_ms      = ns_avg_ms(insert_times);
    R.avg_range_query_ms = ns_avg_ms(range_times);
    return R;
}

// ---------- Main ----------
int main(int argc, char** argv) {
    fs::path images = "/home/avs/DATA/SSD/images";
    fs::path lidar  = "/home/avs/DATA/SSD/lidar_laz";
    fs::path sqlite = "/home/avs/DB/bench_sqlite.db";
    fs::path rocks  = "/home/avs/DB/rocks_bench";
    size_t N_ranges = 1000;
    int64_t window_ms = 1000;

    // simple CLI
    for (int i=1;i<argc;i++) {
        std::string a = argv[i];
        auto next = [&](const char* flag)->std::string{
            if (i+1 >= argc) { std::cerr << "Missing value for " << flag << "\n"; std::exit(2); }
            return argv[++i];
        };
        if (a=="--images") images = next("--images");
        else if (a=="--lidar") lidar = next("--lidar");
        else if (a=="--sqlite") sqlite = next("--sqlite");
        else if (a=="--rocks") rocks = next("--rocks");
        else if (a=="--ranges") N_ranges = static_cast<size_t>(std::stoul(next("--ranges")));
        else if (a=="--window-ms") window_ms = static_cast<int64_t>(std::stoll(next("--window-ms")));
        else { std::cerr << "Unknown arg: " << a << "\n"; return 2; }
    }

    auto recs = collect_records(images, lidar);
    if (recs.empty()) {
        std::cerr << "No records found under " << images << " and " << lidar << "\n";
        return 1;
    }

    std::cout << "Total records: " << recs.size() << "\n";

    auto ranges = build_range_queries(recs, N_ranges, window_ms);
    if (ranges.empty()) {
        std::cerr << "No range queries built\n";
        return 1;
    }

    auto sres = bench_sqlite(recs, sqlite, ranges);
    auto rres = bench_rocks(recs, rocks, ranges);

    auto fmt_mb = [](uint64_t bytes){ return bytes / (1024.0*1024.0); };

    std::cout << std::fixed << std::setprecision(4);

    std::cout << "\n=== SQLite3 ===\n";
    std::cout << "Inserted: " << sres.N_inserted
              << " | Range queries: " << sres.N_q
              << " | Rows scanned: " << sres.total_rows_scanned << "\n";
    std::cout << "Avg insert latency (ms): " << sres.avg_insert_ms << "\n";
    std::cout << "Avg range  latency (ms): " << sres.avg_range_query_ms  << "\n";
    std::cout << "DB size: " << fmt_mb(sres.size_bytes) << " MB\n";

    std::cout << "\n=== RocksDB ===\n";
    std::cout << "Inserted: " << rres.N_inserted
              << " | Range queries: " << rres.N_q
              << " | Rows scanned: " << rres.total_rows_scanned << "\n";
    std::cout << "Avg insert latency (ms): " << rres.avg_insert_ms << "\n";
    std::cout << "Avg range  latency (ms): " << rres.avg_range_query_ms  << "\n";
    std::cout << "DB size: " << fmt_mb(rres.size_bytes) << " MB\n";

    std::cout << "\n=== Summary ===\n";
    std::cout << "SQLite  insert(ms): " << sres.avg_insert_ms
              << " | range(ms): " << sres.avg_range_query_ms
              << " | size: " << fmt_mb(sres.size_bytes) << " MB\n";
    std::cout << "RocksDB insert(ms): " << rres.avg_insert_ms
              << " | range(ms): " << rres.avg_range_query_ms
              << " | size: " << fmt_mb(rres.size_bytes) << " MB\n";

    return 0;
}
