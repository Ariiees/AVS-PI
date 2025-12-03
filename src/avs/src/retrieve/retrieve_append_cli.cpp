// avs_retrieve_view.cpp
#include <iostream>
#include <filesystem>
#include <string>
#include <vector>
#include <fstream>
#include <cstdint>
#include <cstring>
#include <sstream>
#include <thread>
#include <chrono>
#include <limits>

#include <laszip_api.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <sqlite3.h>
#include <opencv2/opencv.hpp>

#include "avs/append_logger.h" 


namespace fs = std::filesystem;
static const fs::path kSsdRoot("/home/avs/DATA/SSD");


static void usage(const char *p) {
  std::cerr << "Usage: " << p
            << " --topic <sensor_topic> --start <ts_ns> --end <ts_ns> "
               "[--image | --lidar]\n";
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
    std::cerr << "sqlite_prepare failed: " << sqlite3_errmsg(db) << "\n";
    return out;
  }

  sqlite3_bind_text(stmt, 1, topic.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_int64(stmt, 2, static_cast<sqlite3_int64>(end_ns));
  sqlite3_bind_int64(stmt, 3, static_cast<sqlite3_int64>(start_ns));

  while (sqlite3_step(stmt) == SQLITE_ROW) {
    const unsigned char *s_topic         = sqlite3_column_text(stmt, 0);
    const unsigned char *s_topic_folder  = sqlite3_column_text(stmt, 1);
    const unsigned char *s_day           = sqlite3_column_text(stmt, 2);
    int trip_id                          = sqlite3_column_int(stmt, 3);

    sqlite3_int64 s_start = sqlite3_column_int64(stmt, 4);
    sqlite3_int64 s_end   = sqlite3_column_int64(stmt, 5);

    uint64_t t_start = static_cast<uint64_t>(s_start);
    uint64_t t_end   = static_cast<uint64_t>(s_end);

    out.emplace_back(
        s_topic        ? reinterpret_cast<const char*>(s_topic)        : "",
        s_topic_folder ? reinterpret_cast<const char*>(s_topic_folder) : "",
        s_day          ? reinterpret_cast<const char*>(s_day)          : "",
        trip_id,
        t_start,
        t_end
    );
  }

  sqlite3_finalize(stmt);
  return out;
}

// -----------------------------------------------------------------------------
// Small references to matching records, no payload in memory
// -----------------------------------------------------------------------------

struct DataRef {
  std::string sensor_topic;
  std::string day;
  std::string topic_folder;
  int         trip_id;
  uint64_t    ts_ns;

  fs::path    log_path;
  uint64_t    payload_offset;   // offset in trip.log where payload starts
  uint32_t    payload_size;     // payload size in bytes
};

// Build list of DataRef for the query window.
// This only stores metadata, not image or lidar bytes.
static std::vector<DataRef>
collect_refs(const fs::path &ssd_root,
                   const std::string &topic,
                   uint64_t qstart_ns,
                   uint64_t qend_ns)
{
  std::vector<DataRef> refs;

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

  for (const auto &t : trips) {
    const std::string sensor       = std::get<0>(t);
    const std::string topic_folder = std::get<1>(t);
    const std::string day          = std::get<2>(t);
    int trip_id                    = std::get<3>(t);

    fs::path daydir = fs::path(ssd_root) / topic_folder / day;

    char tb[64];
    std::snprintf(tb, sizeof(tb), "trip_%02d.idx", trip_id);
    fs::path idxp = daydir / tb;
    std::snprintf(tb, sizeof(tb), "trip_%02d.log", trip_id);
    fs::path logp = daydir / tb;

    std::ifstream idxf(idxp, std::ios::binary);
    if (!idxf.is_open()) {
      std::cerr << "Cannot open index " << idxp << "\n";
      continue;
    }

    std::ifstream lf(logp, std::ios::binary);
    if (!lf.is_open()) {
      std::cerr << "Cannot open log " << logp << "\n";
      continue;
    }

    avs::TripIndexEntry ent;
    while (idxf.read(reinterpret_cast<char*>(&ent), sizeof(ent))) {
      if (ent.end_ts_ns < static_cast<int64_t>(qstart_ns) ||
          ent.start_ts_ns > static_cast<int64_t>(qend_ns)) {
        continue;
      }

      const std::uint64_t chunk_data_start =
          static_cast<std::uint64_t>(ent.file_offset) + sizeof(avs::ChunkHeader);
      const std::uint64_t chunk_data_end =
          chunk_data_start + static_cast<std::uint64_t>(ent.chunk_size_bytes);

      std::uint64_t pos = chunk_data_start;

      for (std::uint32_t rec = 0;
           rec < ent.record_count &&
           pos + sizeof(avs::RecordHeader) <= chunk_data_end;
           ++rec)
      {
        avs::RecordHeader rh;

        lf.seekg(static_cast<std::streamoff>(pos), std::ios::beg);
        lf.read(reinterpret_cast<char*>(&rh), sizeof(rh));
        if (!lf) {
          std::cerr << "Failed to read record header at offset "
                    << pos << " in " << logp << "\n";
          lf.clear();
          break;
        }

        std::uint64_t next_pos =
            pos + sizeof(avs::RecordHeader) +
            static_cast<std::uint64_t>(rh.payload_size);
        if (next_pos > chunk_data_end) {
          break;
        }

        uint64_t rts = static_cast<uint64_t>(rh.ts_ns);
        if (rts >= qstart_ns && rts <= qend_ns) {
          DataRef ref;
          ref.sensor_topic   = sensor;
          ref.topic_folder   = topic_folder;
          ref.day            = day;
          ref.trip_id        = trip_id;
          ref.ts_ns          = rts;
          ref.log_path       = logp;
          ref.payload_offset = pos + sizeof(avs::RecordHeader);
          ref.payload_size   = rh.payload_size;
          refs.emplace_back(std::move(ref));
        }

        pos = next_pos;
      }
    }
  }

  sqlite3_close(db);
  return refs;
}

// -----------------------------------------------------------------------------
// Public API: load one record payload into memory
// -----------------------------------------------------------------------------

// Single payload loader for both image and lidar
static bool load_payload(const DataRef &ref,
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
    std::cerr << "Failed to read payload at offset " << ref.payload_offset
              << " size " << ref.payload_size << " from " << ref.log_path << "\n";
    out_payload.clear();
    return false;
  }

  return true;
}

static bool decode_laz_payload_to_cloud(const std::vector<uint8_t> &payload,
                                        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
  cloud->clear();
  if (payload.empty()) {
    std::cerr << "Empty LAZ payload\n";
    return false;
  }

  laszip_POINTER reader = nullptr;
  if (laszip_create(&reader) != 0 || !reader) {
    std::cerr << "laszip_create failed\n";
    return false;
  }

  // Wrap payload in a binary stream
  std::string buf;
  buf.assign(reinterpret_cast<const char*>(payload.data()), payload.size());
  std::istringstream iss(buf, std::ios::binary);

  laszip_BOOL is_compressed = 0;
  if (laszip_open_reader_stream(reader, iss, &is_compressed) != 0) {
    std::cerr << "laszip_open_reader_stream failed\n";
    laszip_destroy(reader);
    return false;
  }

  laszip_header *header = nullptr;
  if (laszip_get_header_pointer(reader, &header) != 0 || !header) {
    std::cerr << "laszip_get_header_pointer failed\n";
    laszip_close_reader(reader);
    laszip_destroy(reader);
    return false;
  }

  laszip_point *point = nullptr;
  if (laszip_get_point_pointer(reader, &point) != 0 || !point) {
    std::cerr << "laszip_get_point_pointer failed\n";
    laszip_close_reader(reader);
    laszip_destroy(reader);
    return false;
  }

  const laszip_U64 npts = header->number_of_point_records;
  try {
    cloud->reserve(static_cast<std::size_t>(npts));
  } catch (...) {
    // reserve is best effort
  }

  bool ok = true;

  for (laszip_U64 i = 0; i < npts; ++i) {
    if (laszip_read_point(reader) != 0) {
      std::cerr << "laszip_read_point failed at "
                << static_cast<unsigned long long>(i) << "\n";
      ok = false;
      break;
    }

    const double x = header->x_offset + header->x_scale_factor * static_cast<double>(point->X);
    const double y = header->y_offset + header->y_scale_factor * static_cast<double>(point->Y);
    const double z = header->z_offset + header->z_scale_factor * static_cast<double>(point->Z);

    pcl::PointXYZI pt;
    pt.x = static_cast<float>(x);
    pt.y = static_cast<float>(y);
    pt.z = static_cast<float>(z);
    pt.intensity = static_cast<float>(point->intensity);

    cloud->push_back(pt);
  }

  if (laszip_close_reader(reader) != 0) {
    std::cerr << "laszip_close_reader failed\n";
    ok = false;
  }
  laszip_destroy(reader);
  return ok;
}

// -----------------------------------------------------------------------------
// Viewer: one frame at a time, arrow keys navigate
// -----------------------------------------------------------------------------

static void view_images_interactive(const std::vector<DataRef> &refs)
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
    if (!load_payload(ref, payload)) {
      std::cerr << "Skip index " << idx << " due to load error\n";
    } else {
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


// Simple LiDAR viewer using OpenCV: project XY to 2D image
static void view_lidar_interactive(const std::vector<DataRef> &refs)
{
  if (refs.empty()) {
    std::cout << "No matching LiDAR frames in requested window\n";
    return;
  }

  std::cout << "Found " << refs.size()
            << " LiDAR frames. Left/Right arrow or A/D to navigate, q to quit.\n";

  const std::string win_name = "AVS LiDAR Viewer";
  cv::namedWindow(win_name, cv::WINDOW_NORMAL);

  std::size_t idx = 0;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());

  const int img_w = 1024;
  const int img_h = 512;

  while (true) {
    const auto &ref = refs[idx];

    // load payload
    std::vector<uint8_t> payload;
    if (!load_payload(ref, payload)) {
      std::cerr << "Skip frame " << idx << " due to load error\n";
    } else if (!decode_laz_payload_to_cloud(payload, cloud)) {
      std::cerr << "Skip frame " << idx << " due to decode error\n";
    } else {
      // find XY bounds
      float min_x =  std::numeric_limits<float>::infinity();
      float max_x = -std::numeric_limits<float>::infinity();
      float min_y =  std::numeric_limits<float>::infinity();
      float max_y = -std::numeric_limits<float>::infinity();

      for (const auto &pt : cloud->points) {
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y)) continue;
        if (pt.x < min_x) min_x = pt.x;
        if (pt.x > max_x) max_x = pt.x;
        if (pt.y < min_y) min_y = pt.y;
        if (pt.y > max_y) max_y = pt.y;
      }

      if (!std::isfinite(min_x) || !std::isfinite(max_x) || min_x == max_x) {
        min_x = -50.f; max_x = 50.f;
      }
      if (!std::isfinite(min_y) || !std::isfinite(max_y) || min_y == max_y) {
        min_y = -50.f; max_y = 50.f;
      }

      const float range_x = max_x - min_x;
      const float range_y = max_y - min_y;

      cv::Mat canvas(img_h, img_w, CV_8UC3, cv::Scalar(0, 0, 0));

      for (const auto &pt : cloud->points) {
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y)) continue;

        int px = static_cast<int>((pt.x - min_x) / range_x * (img_w - 1));
        int py = static_cast<int>((pt.y - min_y) / range_y * (img_h - 1));

        py = img_h - 1 - py;

        if (px < 0 || px >= img_w || py < 0 || py >= img_h) continue;

        float inten = pt.intensity;
        if (!std::isfinite(inten)) inten = 0.f;
        if (inten < 0.f) inten = 0.f;
        if (inten > 255.f) inten = 255.f;
        unsigned char v = static_cast<unsigned char>(inten);

        canvas.at<cv::Vec3b>(py, px) = cv::Vec3b(v, v, 255);  // bluish dot
      }

      std::string title = "LiDAR frame "
                          + std::to_string(idx + 1) + "/" + std::to_string(refs.size())
                          + " ts=" + std::to_string(ref.ts_ns);
      cv::imshow(win_name, canvas);
      cv::setWindowTitle(win_name, title);
    }

    // mask key: some OpenCV builds put extra bits in high byte(s)
    int key_raw = cv::waitKey(0);
    int key = key_raw & 0xFF;

    bool go_prev = (key == 81 || key == 'a' || key == 'A');     // left or A
    bool go_next = (key == 83 || key == 'd' || key == 'D');     // right or D

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
// main
// -----------------------------------------------------------------------------

int main(int argc, char **argv) {
  fs::path ssd_root = kSsdRoot;  // fixed SSD root
  std::string topic;
  uint64_t start_ns = 0;
  uint64_t end_ns   = 0;
  bool image_mode   = false;
  bool lidar_mode   = false;

  if (argc < 7) {  // need at least topic, start, end
    usage(argv[0]);
  }

  for (int i = 1; i < argc; ++i) {
    std::string a(argv[i]);
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
    if (a == "--image") {
      image_mode = true;
      continue;
    }
    if (a == "--lidar") {
      lidar_mode = true;
      continue;
    }
    // unknown arg
    usage(argv[0]);
  }

  if (topic.empty() || start_ns == 0 || end_ns == 0) {
    usage(argv[0]);
  }

  // prevent conflicting tags
  if (image_mode && lidar_mode) {
    std::cerr << "Cannot use --image and --lidar together\n";
    usage(argv[0]);
  }

  // default to image if neither tag is given
  if (!image_mode && !lidar_mode) {
    image_mode = true;
  }

  // Build refs once from append log
  std::vector<DataRef> refs =
      collect_refs(ssd_root, topic, start_ns, end_ns);

  if (lidar_mode) {
    view_lidar_interactive(refs);
  } else {  // image_mode
    view_images_interactive(refs);
  }

  return 0;
}
