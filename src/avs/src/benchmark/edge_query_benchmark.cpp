#include <chrono>
#include <cstdint>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

#include <rocksdb/db.h>
#include <rocksdb/options.h>
#include <sqlite3.h>

#include "avs/retrieve_api.h"
#include "avs/storage_logger.h"

namespace fs = std::filesystem;

namespace {

struct Options {
  std::string backend = "append";
  fs::path ssd_root = "/home/avs/DATA/SSD";
  std::string topic;
  std::uint64_t start_ns = 0;
  std::uint64_t end_ns = 0;
  std::uint64_t max_records = 0;
};

struct BenchResult {
  std::uint64_t records = 0;
  std::uint64_t bytes = 0;
  double total_ms = 0.0;
  double ttfb_ms = 0.0;
};

using TripRow = std::tuple<std::string, std::string, std::string, int, std::uint64_t, std::uint64_t>;

void Usage(const char* argv0) {
  std::cerr
      << "Usage: " << argv0
      << " --backend <append|rocksdb>"
      << " --topic <sensor_topic>"
      << " --start_ns <ts_ns>"
      << " --end_ns <ts_ns>"
      << " [--ssd_root <path>]"
      << " [--max_records <n>]\n";
}

bool ParseArgs(int argc, char** argv, Options* out) {
  if (out == nullptr) {
    return false;
  }

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--backend" && i + 1 < argc) {
      out->backend = avs::NormalizeStorageBackend(argv[++i]);
    } else if (arg == "--topic" && i + 1 < argc) {
      out->topic = argv[++i];
    } else if (arg == "--start_ns" && i + 1 < argc) {
      out->start_ns = std::stoull(argv[++i]);
    } else if (arg == "--end_ns" && i + 1 < argc) {
      out->end_ns = std::stoull(argv[++i]);
    } else if (arg == "--ssd_root" && i + 1 < argc) {
      out->ssd_root = argv[++i];
    } else if (arg == "--max_records" && i + 1 < argc) {
      out->max_records = std::stoull(argv[++i]);
    } else {
      return false;
    }
  }

  return !out->topic.empty() && out->start_ns > 0 && out->end_ns >= out->start_ns &&
         avs::IsSupportedStorageBackend(out->backend);
}

bool OpenDb(const fs::path& dbpath, sqlite3** out, std::string* err) {
  *out = nullptr;
  const int rc = sqlite3_open(dbpath.c_str(), out);
  if (rc != SQLITE_OK) {
    if (err != nullptr) {
      std::ostringstream ss;
      ss << "sqlite open failed " << dbpath << " msg " << sqlite3_errmsg(*out);
      *err = ss.str();
    }
    if (*out != nullptr) {
      sqlite3_close(*out);
      *out = nullptr;
    }
    return false;
  }
  return true;
}

std::vector<TripRow> FindMatchingTrips(sqlite3* db,
                                       const std::string& topic,
                                       std::uint64_t start_ns,
                                       std::uint64_t end_ns,
                                       std::string* err) {
  std::vector<TripRow> out;
  const char* sql =
      "SELECT sensor_topic, topic_folder, day, trip_id, start_ts_ns, end_ts_ns "
      "FROM global "
      "WHERE sensor_topic = ? "
      "  AND CAST(start_ts_ns AS INTEGER) <= ? "
      "  AND (end_ts_ns = '0' OR CAST(end_ts_ns AS INTEGER) >= ?);";

  sqlite3_stmt* stmt = nullptr;
  if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK) {
    if (err != nullptr) {
      std::ostringstream ss;
      ss << "sqlite prepare failed msg " << sqlite3_errmsg(db);
      *err = ss.str();
    }
    return out;
  }

  sqlite3_bind_text(stmt, 1, topic.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_int64(stmt, 2, static_cast<sqlite3_int64>(end_ns));
  sqlite3_bind_int64(stmt, 3, static_cast<sqlite3_int64>(start_ns));

  while (sqlite3_step(stmt) == SQLITE_ROW) {
    const unsigned char* sensor = sqlite3_column_text(stmt, 0);
    const unsigned char* topic_folder = sqlite3_column_text(stmt, 1);
    const unsigned char* day = sqlite3_column_text(stmt, 2);
    const int trip_id = sqlite3_column_int(stmt, 3);
    const auto row_start = static_cast<std::uint64_t>(sqlite3_column_int64(stmt, 4));
    const auto row_end = static_cast<std::uint64_t>(sqlite3_column_int64(stmt, 5));

    out.emplace_back(sensor ? reinterpret_cast<const char*>(sensor) : "",
                     topic_folder ? reinterpret_cast<const char*>(topic_folder) : "",
                     day ? reinterpret_cast<const char*>(day) : "",
                     trip_id,
                     row_start,
                     row_end);
  }

  sqlite3_finalize(stmt);
  return out;
}

std::string FormatRocksKey(std::uint64_t ts_ns, std::uint32_t seq) {
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%020llu_%010u",
                static_cast<unsigned long long>(ts_ns), seq);
  return std::string(buf);
}

bool ParseRocksTimestamp(const rocksdb::Slice& key, std::uint64_t* ts_ns) {
  if (ts_ns == nullptr || key.size() < 20) {
    return false;
  }

  std::uint64_t value = 0;
  for (std::size_t i = 0; i < 20; ++i) {
    const char c = key[i];
    if (c < '0' || c > '9') {
      return false;
    }
    value = value * 10 + static_cast<std::uint64_t>(c - '0');
  }
  *ts_ns = value;
  return true;
}

BenchResult BenchAppend(const Options& options) {
  BenchResult result;
  avs::RetrieveAPI api(options.ssd_root);

  const auto t0 = std::chrono::steady_clock::now();
  std::string err;
  const auto refs = api.QueryRefs(options.topic, options.start_ns, options.end_ns, &err);
  if (!err.empty() && refs.empty()) {
    throw std::runtime_error(err);
  }

  std::vector<std::uint8_t> payload;
  for (const auto& ref : refs) {
    if (options.max_records > 0 && result.records >= options.max_records) {
      break;
    }

    std::string load_err;
    if (!api.LoadPayload(ref, payload, &load_err)) {
      throw std::runtime_error(load_err);
    }

    if (result.records == 0) {
      const auto first = std::chrono::steady_clock::now();
      result.ttfb_ms =
          std::chrono::duration<double, std::milli>(first - t0).count();
    }

    ++result.records;
    result.bytes += payload.size();
  }

  const auto t1 = std::chrono::steady_clock::now();
  result.total_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
  if (result.records == 0) {
    result.ttfb_ms = result.total_ms;
  }
  return result;
}

BenchResult BenchRocksDb(const Options& options) {
  BenchResult result;

  sqlite3* db = nullptr;
  std::string err;
  const fs::path dbpath = options.ssd_root / "rocksdb_global.sqlite3";
  if (!OpenDb(dbpath, &db, &err)) {
    throw std::runtime_error(err);
  }

  const auto trips = FindMatchingTrips(db, options.topic, options.start_ns, options.end_ns, &err);
  sqlite3_close(db);
  if (!err.empty() && trips.empty()) {
    throw std::runtime_error(err);
  }

  const std::string start_key = FormatRocksKey(options.start_ns, 0);
  const auto t0 = std::chrono::steady_clock::now();

  for (const auto& trip : trips) {
    if (options.max_records > 0 && result.records >= options.max_records) {
      break;
    }

    const std::string& topic_folder = std::get<1>(trip);
    const std::string& day = std::get<2>(trip);
    const int trip_id = std::get<3>(trip);

    char trip_buf[64];
    std::snprintf(trip_buf, sizeof(trip_buf), "trip_%02d", trip_id);
    const fs::path trip_dir =
        avs::StorageTopicDayDir("rocksdb", options.ssd_root, topic_folder, day) / trip_buf;

    rocksdb::Options rocks_options;
    rocks_options.create_if_missing = false;

    rocksdb::DB* trip_db = nullptr;
    const rocksdb::Status open_status =
        rocksdb::DB::OpenForReadOnly(rocks_options, trip_dir.string(), &trip_db);
    if (!open_status.ok()) {
      throw std::runtime_error("rocksdb open failed: " + open_status.ToString());
    }

    std::unique_ptr<rocksdb::DB> db_owner(trip_db);
    rocksdb::ReadOptions read_options;
    std::unique_ptr<rocksdb::Iterator> it(trip_db->NewIterator(read_options));
    for (it->Seek(start_key);
         it->Valid() && (options.max_records == 0 || result.records < options.max_records);
         it->Next()) {
      std::uint64_t key_ts = 0;
      if (!ParseRocksTimestamp(it->key(), &key_ts)) {
        continue;
      }
      if (key_ts > options.end_ns) {
        break;
      }

      if (result.records == 0) {
        const auto first = std::chrono::steady_clock::now();
        result.ttfb_ms =
            std::chrono::duration<double, std::milli>(first - t0).count();
      }

      ++result.records;
      result.bytes += static_cast<std::uint64_t>(it->value().size());
    }

    if (!it->status().ok()) {
      throw std::runtime_error("rocksdb iterator failed: " + it->status().ToString());
    }
  }

  const auto t1 = std::chrono::steady_clock::now();
  result.total_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
  if (result.records == 0) {
    result.ttfb_ms = result.total_ms;
  }
  return result;
}

void PrintResult(const Options& options, const BenchResult& result) {
  std::cout << std::fixed << std::setprecision(3);
  std::cout << "backend\t" << options.backend << "\n";
  std::cout << "topic\t" << options.topic << "\n";
  std::cout << "records\t" << result.records << "\n";
  std::cout << "bytes\t" << result.bytes << "\n";
  std::cout << "ttfb_ms\t" << result.ttfb_ms << "\n";
  std::cout << "total_ms\t" << result.total_ms << "\n";
  std::cout << "avg_ms_per_record\t"
            << (result.records == 0 ? 0.0 : result.total_ms / static_cast<double>(result.records))
            << "\n";
}

}  // namespace

int main(int argc, char** argv) {
  Options options;
  if (!ParseArgs(argc, argv, &options)) {
    Usage(argv[0]);
    return 1;
  }

  try {
    BenchResult result;
    if (options.backend == "append") {
      result = BenchAppend(options);
    } else if (options.backend == "rocksdb") {
      result = BenchRocksDb(options);
    } else {
      throw std::runtime_error("unsupported backend");
    }
    PrintResult(options, result);
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "edge_query_benchmark failed: " << e.what() << "\n";
    return 1;
  }
}
