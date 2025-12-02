// avs_retrieve_view.cpp
#include <iostream>
#include <filesystem>
#include <string>
#include <vector>
#include <fstream>
#include <cstdint>
#include <cstring>

#include <sqlite3.h>
#include <opencv2/opencv.hpp>

#include "avs/append_logger.h"  // TripIndexEntry, ChunkHeader, RecordHeader
#include "avs/trip_manager.h"

namespace fs = std::filesystem;

static void usage(const char *p) {
  std::cerr << "Usage: " << p
            << " --ssd-root <path> --topic <sensor_topic> --start <ts_ns> --end <ts_ns>\n";
  std::exit(1);
}

static bool open_db(const fs::path &dbpath, sqlite3 **out) {
  int rc = sqlite3_open(dbpath.c_str(), out);
  if (rc != SQLITE_OK) {
    std::cerr << "Failed to open sqlite DB " << dbpath << ": "
              << sqlite3_errmsg(*out) << "\n";
    if (*out) sqlite3_close(*out);
    *out = nullptr;
    return false;
  }
  return true;
}

static std::vector<std::tuple<std::string, std::string, std::string, int, uint64_t, uint64_t>>
find_matching_trips(sqlite3 *db,
                    const std::string &topic,
                    uint64_t start_ns,
                    uint64_t end_ns)
{
  std::vector<std::tuple<std::string, std::string, std::string, int, uint64_t, uint64_t>> out;

  // Overlap condition (inclusive):
  //   start_ts_ns <= end_ns
  //   and (end_ts_ns == 0 or end_ts_ns >= start_ns)
  const char *sql =
      "SELECT sensor_topic, topic_folder, day, trip_id, start_ts_ns, end_ts_ns "
      "FROM global "
      "WHERE sensor_topic = ? "
      "  AND CAST(start_ts_ns AS INTEGER) <= ? "
      "  AND (end_ts_ns = '0' OR CAST(end_ts_ns AS INTEGER) >= ?);";

  sqlite3_stmt *stmt = nullptr;
  if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK) {
    return out;
  }

  sqlite3_bind_text(stmt, 1, topic.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_int64(stmt, 2, static_cast<sqlite3_int64>(end_ns));
  sqlite3_bind_int64(stmt, 3, static_cast<sqlite3_int64>(start_ns));

  while (sqlite3_step(stmt) == SQLITE_ROW) {
    const unsigned char *s_topic = sqlite3_column_text(stmt, 0);
    const unsigned char *s_topic_folder   = sqlite3_column_text(stmt, 1);
    const unsigned char *s_day   = sqlite3_column_text(stmt, 2);
    int trip_id                  = sqlite3_column_int(stmt, 3);

    sqlite3_int64 s_start = sqlite3_column_int64(stmt, 4);
    sqlite3_int64 s_end   = sqlite3_column_int64(stmt, 5);

    uint64_t t_start = static_cast<uint64_t>(s_start);
    uint64_t t_end   = static_cast<uint64_t>(s_end);

    out.emplace_back(
        s_topic ? reinterpret_cast<const char*>(s_topic) : "",
        s_topic_folder ? reinterpret_cast<const char*>(s_topic_folder) : "",
        s_day   ? reinterpret_cast<const char*>(s_day)   : "",
        trip_id,
        t_start,
        t_end
    );
  }

  sqlite3_finalize(stmt);
  return out;
}


static bool read_trip_index(const fs::path &idxp,
                            std::vector<avs::TripIndexEntry> &out)
{
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

// -----------------------------------------------------------------------------
// Small references to matching records, no payload in memory
// -----------------------------------------------------------------------------

struct ImageRef {
  std::string sensor_topic;
  std::string day;
  std::string topic_folder;
  int         trip_id;
  uint64_t    ts_ns;

  fs::path    log_path;
  uint64_t    payload_offset;   // offset in trip.log where jpeg payload starts
  uint32_t    payload_size;     // jpeg size in bytes
};

// Build list of ImageRef for the query window.
// This only stores metadata, not image bytes.
static std::vector<ImageRef>
collect_image_refs(const fs::path &ssd_root,
                   const std::string &topic,
                   uint64_t qstart_ns,
                   uint64_t qend_ns)
{
  std::vector<ImageRef> refs;

  sqlite3 *db = nullptr;
  fs::path dbpath = fs::path(ssd_root) / "global.sqlite3";
  if (!open_db(dbpath, &db)) {
    std::cerr << "Cannot open DB " << dbpath << "\n";
    return refs;
  }

  auto trips = find_matching_trips(db, topic, qstart_ns, qend_ns);
  if (trips.empty()) {
    sqlite3_close(db);
    std::cerr << "No matching trips for topic " << topic << " in window\n";
    return refs;
  }

  for (auto &t : trips) {
    std::string sensor = std::get<0>(t);
    std::string topic_folder = std::get<1>(t);
    std::string day    = std::get<2>(t);
    int trip_id        = std::get<3>(t);

    fs::path daydir = fs::path(ssd_root) / topic_folder / day;

    char tb[64];
    std::snprintf(tb, sizeof(tb), "trip_%02d.idx", trip_id);
    fs::path idxp = daydir / tb;
    std::snprintf(tb, sizeof(tb), "trip_%02d.log", trip_id);
    fs::path logp = daydir / tb;

    std::vector<avs::TripIndexEntry> idx_entries;
    if (!read_trip_index(idxp, idx_entries)) {
      std::cerr << "Cannot open index " << idxp << "\n";
      continue;
    }

    std::ifstream lf(logp, std::ios::binary);
    if (!lf.is_open()) {
      std::cerr << "Cannot open log " << logp << "\n";
      continue;
    }

    for (const auto &ent : idx_entries) {
      if (ent.end_ts_ns < static_cast<int64_t>(qstart_ns) ||
          ent.start_ts_ns > static_cast<int64_t>(qend_ns)) {
        continue;
      }

      lf.seekg(ent.file_offset, std::ios::beg);
      uint32_t total_bytes = ent.chunk_size_bytes;
      if (total_bytes < sizeof(avs::ChunkHeader)) {
        continue;
      }

      std::vector<uint8_t> chunkbuf(total_bytes);
      lf.read(reinterpret_cast<char*>(chunkbuf.data()), total_bytes);
      if (!lf) {
        std::cerr << "Failed to read chunk at offset " << ent.file_offset << "\n";
        lf.clear();
        continue;
      }

      avs::ChunkHeader ch;
      std::memcpy(&ch, chunkbuf.data(), sizeof(ch));

      size_t pos = sizeof(ch);
      size_t end = total_bytes;

      while (pos + sizeof(avs::RecordHeader) <= end) {
        size_t record_header_pos = pos;

        avs::RecordHeader rh;
        std::memcpy(&rh, chunkbuf.data() + pos, sizeof(rh));
        pos += sizeof(rh);

        if (pos + rh.payload_size > end) {
          break;
        }

        uint64_t rts = static_cast<uint64_t>(rh.ts_ns);
        bool match = (rts >= qstart_ns && rts <= qend_ns);

        if (match) {
          ImageRef ref;
          ref.sensor_topic  = sensor;
          ref.topic_folder  = topic_folder;
          ref.day           = day;
          ref.trip_id       = trip_id;
          ref.ts_ns         = rts;
          ref.log_path      = logp;
          ref.payload_offset = static_cast<uint64_t>(ent.file_offset + pos);
          ref.payload_size   = rh.payload_size;
          refs.emplace_back(std::move(ref));
        }

        pos += rh.payload_size;
      }
    }
  }

  sqlite3_close(db);
  return refs;
}

// -----------------------------------------------------------------------------
// Public API: load one record payload into memory
// -----------------------------------------------------------------------------

// API 1: load raw bytes for a given ImageRef
// Caller owns 'out_payload' and can interpret it as jpeg.
static bool load_image_payload(const ImageRef &ref,
                               std::vector<uint8_t> &out_payload)
{
  out_payload.clear();

  std::ifstream lf(ref.log_path, std::ios::binary);
  if (!lf.is_open()) {
    std::cerr << "Cannot open log " << ref.log_path << "\n";
    return false;
  }

  lf.seekg(static_cast<std::streamoff>(ref.payload_offset), std::ios::beg);
  out_payload.resize(ref.payload_size);
  lf.read(reinterpret_cast<char*>(out_payload.data()), ref.payload_size);
  if (!lf) {
    std::cerr << "Failed to read payload at offset " << ref.payload_offset << "\n";
    out_payload.clear();
    return false;
  }

  return true;
}

// -----------------------------------------------------------------------------
// Viewer: one image at a time, arrow keys navigate
// -----------------------------------------------------------------------------

static void view_images_interactive(const std::vector<ImageRef> &refs)
{
  if (refs.empty()) {
    std::cout << "No matching images in requested window\n";
    return;
  }

  std::cout << "Found " << refs.size()
            << " images. Right arrow for next, left arrow for previous, q to quit.\n";

  const std::string win_name = "AVS Image Viewer";
  cv::namedWindow(win_name, cv::WINDOW_NORMAL);

  std::size_t idx = 0;

  while (true) {
    const auto &ref = refs[idx];

    // read this one image payload from SSD
    std::vector<uint8_t> payload;
    if (!load_image_payload(ref, payload)) {
      std::cerr << "Skip index " << idx << " due to load error\n";
    } else {
      // decode jpeg from memory
      cv::Mat buf_mat(1, static_cast<int>(payload.size()), CV_8UC1);
      std::memcpy(buf_mat.data, payload.data(), payload.size());
      cv::Mat img = cv::imdecode(buf_mat, cv::IMREAD_COLOR);

      if (img.empty()) {
        std::cerr << "Failed to decode jpeg at index " << idx << "\n";
      } else {
        std::string title = ref.sensor_topic +
                            " day=" + ref.day +
                            " trip=" + std::to_string(ref.trip_id) +
                            " ts=" + std::to_string(ref.ts_ns);
        cv::imshow(win_name, img);
        cv::setWindowTitle(win_name, title);
      }
      // payload and img go out of scope each loop
    }

    int key = cv::waitKey(0);

    bool go_prev = (key == 81 || key == 2424832);     // left arrow
    bool go_next = (key == 83 || key == 2555904);     // right arrow

    if (key == 'q' || key == 'Q' || key == 27) {
      break;
    } else if (go_next) {
      if (idx + 1 < refs.size()) {
        idx += 1;
      }
    } else if (go_prev) {
      if (idx > 0) {
        idx -= 1;
      }
    }
  }

  cv::destroyWindow(win_name);
}

// -----------------------------------------------------------------------------
// main: example entry, uses both query API and viewer
// -----------------------------------------------------------------------------

int main(int argc, char **argv) {
  fs::path ssd_root;
  std::string topic;
  uint64_t start_ns = 0;
  uint64_t end_ns   = 0;

  if (argc < 9) {
    usage(argv[0]);
  }

  for (int i = 1; i < argc; ++i) {
    std::string a(argv[i]);
    if (a == "--ssd-root" && i + 1 < argc) {
      ssd_root = argv[++i];
      continue;
    }
    if (a == "--topic" && i + 1 < argc) {
      topic = argv[++i];
      continue;
    }
    if (a == "--start" && i + 1 < argc) {
      start_ns = std::stoull(argv[++i]);
      continue;
    }
    if (a == "--end" && i + 1 < argc) {
      end_ns = std::stoull(argv[++i]);
      continue;
    }
    usage(argv[0]);
  }

  if (ssd_root.empty() || topic.empty() || start_ns == 0 || end_ns == 0) {
    usage(argv[0]);
  }

  // build references only
  std::vector<ImageRef> refs =
      collect_image_refs(ssd_root, topic, start_ns, end_ns);

  // viewer uses arrow keys, always reading from SSD for each frame
  view_images_interactive(refs);

  return 0;
}
