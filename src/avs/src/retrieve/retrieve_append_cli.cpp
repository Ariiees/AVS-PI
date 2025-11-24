// ...new file...
#include <iostream>
#include <filesystem>
#include <string>
#include <vector>
#include <fstream>
#include <memory>
#include <cstdio>
#include <cstdint>
#include <sqlite3.h>

#include "avs/append_logger.h"
#include <cstring>

namespace fs = std::filesystem;

static void usage(const char *p) {
  std::cerr << "Usage: " << p << " --ssd-root <SSD_ROOT> --topic <sensor_topic> --start <ts_ns> --end <ts_ns> [--extract-dir <dir>]\n";
  std::exit(1);
}

static bool open_db(const fs::path &dbpath, sqlite3 **out) {
  int rc = sqlite3_open(dbpath.c_str(), out);
  if (rc != SQLITE_OK) {
    std::cerr << "Failed to open sqlite DB " << dbpath << ": " << sqlite3_errmsg(*out) << "\n";
    if (*out) sqlite3_close(*out);
    *out = nullptr;
    return false;
  }
  return true;
}

static std::vector<std::tuple<std::string,std::string,int,uint64_t,uint64_t>> find_matching_trips(
  sqlite3 *db, const std::string &topic, uint64_t start_ns, uint64_t end_ns)
{
  std::vector<std::tuple<std::string,std::string,int,uint64_t,uint64_t>> out;
  const char *sql = "SELECT sensor_topic, day, trip_id, start_ts_ns, end_ts_ns FROM global "
                    "WHERE sensor_topic = ?;";
  sqlite3_stmt *stmt = nullptr;
  if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK) return out;
  sqlite3_bind_text(stmt, 1, topic.c_str(), -1, SQLITE_TRANSIENT);
  while (sqlite3_step(stmt) == SQLITE_ROW) {
    const unsigned char *s_topic = sqlite3_column_text(stmt, 0);
    const unsigned char *s_day   = sqlite3_column_text(stmt, 1);
    int trip_id                  = sqlite3_column_int(stmt, 2);
    const unsigned char *s_start = sqlite3_column_text(stmt, 3);
    const unsigned char *s_end   = sqlite3_column_text(stmt, 4);
    uint64_t t_start = s_start ? std::stoull(reinterpret_cast<const char*>(s_start)) : 0;
    uint64_t t_end   = s_end   ? std::stoull(reinterpret_cast<const char*>(s_end))   : 0;
    // overlap test
    if (!(t_end < start_ns || t_start > end_ns)) {
      out.emplace_back(reinterpret_cast<const char*>(s_topic),
                       reinterpret_cast<const char*>(s_day),
                       trip_id, t_start, t_end);
    }
  }
  sqlite3_finalize(stmt);
  return out;
}

static bool read_trip_index(const fs::path &idxp, std::vector<avs::TripIndexEntry> &out) {
  out.clear();
  std::ifstream f(idxp, std::ios::binary);
  if (!f.is_open()) return false;
  f.seekg(0, std::ios::end);
  std::streamsize sz = f.tellg();
  if (sz <= 0) return true;
  f.seekg(0, std::ios::beg);
  while (f.tellg() < sz) {
    avs::TripIndexEntry e;
    f.read(reinterpret_cast<char*>(&e), sizeof(e));
    if (!f) break;
    out.push_back(e);
  }
  return true;
}

static void dump_matches(const fs::path &ssd_root,
                         const std::string &topic,
                         uint64_t qstart_ns, uint64_t qend_ns,
                         const fs::path &extract_dir)
{
  sqlite3 *db = nullptr;
  fs::path dbpath = fs::path(ssd_root) / "global.sqlite3";
  if (!open_db(dbpath, &db)) return;

  auto trips = find_matching_trips(db, topic, qstart_ns, qend_ns);
  if (trips.empty()) {
    std::cerr << "[INFO] No matching trips for topic=" << topic << " in window\n";
    sqlite3_close(db);
    return;
  }

  for (auto &t : trips) {
    std::string sensor = std::get<0>(t);
    std::string day    = std::get<1>(t);
    int trip_id        = std::get<2>(t);
    std::cerr << "[INFO] day=" << day << " trip=" << trip_id << " (trip start=" << std::get<3>(t)
              << " end=" << std::get<4>(t) << ")\n";

    fs::path daydir = fs::path(ssd_root) / sensor / day;
    char tb[64];
    snprintf(tb, sizeof(tb), "trip_%02d.idx", trip_id);
    fs::path idxp = daydir / tb;
    snprintf(tb, sizeof(tb), "trip_%02d.log", trip_id);
    fs::path logp = daydir / tb;

    std::vector<avs::TripIndexEntry> idx_entries;
    if (!read_trip_index(idxp, idx_entries)) {
      std::cerr << "[WARN] Unable to open idx: " << idxp << "\n";
      continue;
    }

    // find overlapping chunks (linear scan OK)
    for (const auto &ent : idx_entries) {
      if (ent.end_ts_ns < static_cast<int64_t>(qstart_ns) || ent.start_ts_ns > static_cast<int64_t>(qend_ns))
        continue;

      // read chunk from log
      std::ifstream lf(logp, std::ios::binary);
      if (!lf.is_open()) {
        std::cerr << "[WARN] Cannot open log: " << logp << "\n";
        break;
      }
      lf.seekg(ent.file_offset, std::ios::beg);
      // trip index stored chunk_size_bytes including ChunkHeader (per append_logger)
      uint32_t total_bytes = ent.chunk_size_bytes;
      std::vector<uint8_t> chunkbuf(total_bytes);
      lf.read(reinterpret_cast<char*>(chunkbuf.data()), total_bytes);
      if (!lf) {
        std::cerr << "[WARN] Failed to read chunk at offset " << ent.file_offset << "\n";
        continue;
      }

      // parse ChunkHeader
      if (total_bytes < sizeof(avs::ChunkHeader)) continue;
      avs::ChunkHeader ch;
      std::memcpy(&ch, chunkbuf.data(), sizeof(ch));
      size_t pos = sizeof(ch);
      size_t end = total_bytes;

      // iterate records
      while (pos + sizeof(avs::RecordHeader) <= end) {
        avs::RecordHeader rh;
        std::memcpy(&rh, chunkbuf.data() + pos, sizeof(rh));
        pos += sizeof(rh);
        if (pos + rh.payload_size > end) break;

        uint64_t rts = static_cast<uint64_t>(rh.ts_ns);
        bool match = (rts >= qstart_ns && rts <= qend_ns);
        if (match) {
          std::cout << "MATCH topic=" << sensor << " day=" << day << " trip=" << trip_id
                    << " rec_ts_ns=" << rts << " payload=" << rh.payload_size << " bytes\n";
          if (!extract_dir.empty()) {
            fs::path outdir = extract_dir / sensor / day / ("trip_" + std::to_string(trip_id));
            fs::create_directories(outdir);
            std::string ext = ".bin";
            // choose extension heuristically
            if (sensor.find("camera") != std::string::npos || sensor.find("image") != std::string::npos) ext = ".jpg";
            else if (sensor.find("lidar") != std::string::npos) ext = ".laz";
            fs::path outp = outdir / (std::to_string(rts) + ext);
            std::ofstream of(outp, std::ios::binary);
            of.write(reinterpret_cast<const char*>(chunkbuf.data() + pos), rh.payload_size);
            of.close();
            std::cout << "  -> extracted to: " << outp << "\n";
          }
        }
        pos += rh.payload_size;
      }
    } // idx entries
  } // trips

  sqlite3_close(db);
}

int main(int argc, char ** argv) {
  fs::path ssd_root;
  std::string topic;
  uint64_t start_ns = 0, end_ns = 0;
  fs::path extract_dir;

  if (argc < 7) usage(argv[0]);

  for (int i = 1; i < argc; ++i) {
    std::string a(argv[i]);
    if (a == "--ssd-root" && i + 1 < argc) { ssd_root = argv[++i]; continue; }
    if (a == "--topic" && i + 1 < argc) { topic = argv[++i]; continue; }
    if (a == "--start" && i + 1 < argc) { start_ns = std::stoull(argv[++i]); continue; }
    if (a == "--end" && i + 1 < argc) { end_ns = std::stoull(argv[++i]); continue; }
    if (a == "--extract-dir" && i + 1 < argc) { extract_dir = argv[++i]; continue; }
    usage(argv[0]);
  }

  if (ssd_root.empty() || topic.empty() || start_ns == 0 || end_ns == 0) usage(argv[0]);

  dump_matches(ssd_root.string(), topic, start_ns, end_ns, extract_dir);
  return 0;
}