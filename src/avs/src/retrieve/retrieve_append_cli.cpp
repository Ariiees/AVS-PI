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
#include <pcl/visualization/pcl_visualizer.h>

#include <sqlite3.h>
#include <opencv2/opencv.hpp>

#include "avs/append_logger.h" 

#include <vtkObject.h>
#if defined(VTK_MAJOR_VERSION) && (VTK_MAJOR_VERSION >= 9)
  #include <vtkLogger.h>
#endif

namespace fs = std::filesystem;
static const fs::path kSsdRoot("/home/avs/DATA/SSD");


static void usage(const char *p) {
  std::cerr << "Usage: " << p
            << " --topic <sensor_topic> --start <ts_ns> --end <ts_ns> "
               "[--image | --lidar | --gps]\n";
  std::exit(1);
}


static bool openDb(const fs::path &dbpath, sqlite3 **out) {
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
findMatchingTrips(sqlite3 *db,
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
collectRefs(const fs::path &ssd_root,
            const std::string &topic,
            uint64_t qstart_ns,
            uint64_t qend_ns)
{
  std::vector<DataRef> refs;

  sqlite3 *db = nullptr;
  fs::path dbpath = fs::path(ssd_root) / "global.sqlite3";
  if (!openDb(dbpath, &db)) {
    std::cerr << "Cannot open DB " << dbpath << "\n";
    return refs;
  }

  auto trips = findMatchingTrips(db, topic, qstart_ns, qend_ns);
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

// Single payload loader for image, lidar and gps
static bool loadPayload(const DataRef &ref,
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

// -----------------------------------------------------------------------------
// LiDAR payload decode
// -----------------------------------------------------------------------------

static bool loadLazFromPayload(const std::vector<uint8_t> &payload,
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

  // finalize cloud layout
  cloud->width  = static_cast<std::uint32_t>(cloud->size());
  cloud->height = 1;
  cloud->is_dense = true;

  if (laszip_close_reader(reader) != 0) {
    std::cerr << "laszip_close_reader failed\n";
    ok = false;
  }
  laszip_destroy(reader);

  return ok;
}

// -----------------------------------------------------------------------------
// Viewer: images one frame at a time, arrow keys navigate
// -----------------------------------------------------------------------------

static void viewImagesInteractive(const std::vector<DataRef> &refs)
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
    if (!loadPayload(ref, payload)) {
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


// Simple LiDAR viewer
static void viewLidarInteractive(const std::vector<DataRef>& refs)
{
  if (refs.empty()) {
    std::cout << "No matching LiDAR frames in requested window\n";
    return;
  }

  // Force software GL and silence VTK spam (helps with driver / shader issues).
  setenv("LIBGL_ALWAYS_SOFTWARE", "1", 1);
  setenv("VTK_SILENCE_DEPRECATION", "1", 1);
#if defined(VTK_MAJOR_VERSION) && (VTK_MAJOR_VERSION >= 9)
  vtkLogger::SetStderrVerbosity(vtkLogger::VERBOSITY_ERROR);
#endif
  vtkObject::GlobalWarningDisplayOff();

  using CloudT = pcl::PointCloud<pcl::PointXYZI>;
  pcl::visualization::PCLVisualizer::Ptr viewer(
      new pcl::visualization::PCLVisualizer("LiDAR"));

  viewer->setBackgroundColor(0.0, 0.0, 0.0);
  viewer->addCoordinateSystem(1.0);

  // Disable MSAA so VTK does not try to build the broken resolve shader.
  if (auto rw = viewer->getRenderWindow()) {
    rw->SetMultiSamples(0);
  }

  struct KeyState { char last = 0; } ks;

  viewer->registerKeyboardCallback(
      [](const pcl::visualization::KeyboardEvent& e, void* cookie) {
        if (!e.keyDown()) return;
        auto* state = static_cast<KeyState*>(cookie);
        // Use the same behavior as the working viewClouds(): raw keyCode
        state->last = static_cast<char>(e.getKeyCode());
      },
      static_cast<void*>(&ks));

  CloudT::Ptr cloud(new CloudT);
  int idx = 0;
  const std::string kId = "cloud";

  auto show = [&](int i) {
    cloud->clear();

    std::vector<uint8_t> payload;
    if (!loadPayload(refs[static_cast<std::size_t>(i)], payload)) {
      std::cerr << "Skip frame " << i << " (payload load failed)\n";
      return;
    }
    if (!loadLazFromPayload(payload, cloud)) {
      std::cerr << "Skip frame " << i << " (LAZ decode failed)\n";
      return;
    }
    if (cloud->empty()) {
      std::cerr << "Frame " << i << " decoded to empty cloud\n";
      return;
    }

    CloudT::ConstPtr ccloud(cloud);
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity(
        ccloud, "intensity");

    if (!viewer->updatePointCloud<pcl::PointXYZI>(ccloud, intensity, kId)) {
      viewer->addPointCloud<pcl::PointXYZI>(ccloud, intensity, kId);
      viewer->setPointCloudRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, kId);
    }
  };

  // Show first frame
  show(idx);

  // Main loop: same key behavior as your working tool
  while (!viewer->wasStopped()) {
    viewer->spinOnce(50);

    if (ks.last == 'd') {  // next
      idx = (idx + 1) % static_cast<int>(refs.size());
      show(idx);
      ks.last = 0;
    } else if (ks.last == 'a') {  // previous
      idx = (idx - 1 + static_cast<int>(refs.size())) % static_cast<int>(refs.size());
      show(idx);
      ks.last = 0;
    } else if (ks.last == 'q' || ks.last == 27) {  // q or ESC
      break;
    }
  }
}

// -----------------------------------------------------------------------------
// GPS viewer
// -----------------------------------------------------------------------------

struct GpsPayload {
  double latitude;
  double longitude;
  double altitude;
  double cov_xx;
  double cov_yy;
  double cov_zz;
};

static void viewGpsInteractive(const std::vector<DataRef>& refs)
{
  if (refs.empty()) {
    std::cout << "No matching GPS records in requested window\n";
    return;
  }

  std::cout << "Found " << refs.size()
            << " GPS records. Use n for next, p for previous, q to quit.\n";

  std::size_t idx = 0;
  // flush any leftover newline
  std::cin.clear();
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

  while (true) {
    const auto &ref = refs[idx];

    std::vector<uint8_t> payload;
    if (!loadPayload(ref, payload)) {
      std::cerr << "Skip index " << idx << " due to load error\n";
    } else if (payload.size() < sizeof(GpsPayload)) {
      std::cerr << "Payload too small for GpsPayload at index " << idx << "\n";
    } else {
      GpsPayload gp{};
      std::memcpy(&gp, payload.data(), sizeof(GpsPayload));

            std::time_t t_sec = static_cast<std::time_t>(ref.ts_ns / 1000000000ULL);
            std::tm tm_buf{};
            char time_str[64];
            // POSIX: use gmtime_r(const time_t* time, tm* tmBuf)
            if (gmtime_r(&t_sec, &tm_buf)) {
              std::strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S UTC", &tm_buf);
            } else {
              std::snprintf(time_str, sizeof(time_str), "ts_ns=%llu",
                            static_cast<unsigned long long>(ref.ts_ns));
            }

      std::cout << "\n[" << idx + 1 << "/" << refs.size() << "] "
                << "topic=" << ref.sensor_topic
                << " day=" << ref.day
                << " trip=" << ref.trip_id
                << " ts_ns=" << ref.ts_ns
                << " (" << time_str << ")\n";

      std::cout << "  latitude   : " << gp.latitude  << "\n"
                << "  longitude  : " << gp.longitude << "\n"
                << "  altitude   : " << gp.altitude  << "\n"
                << "  cov_xx     : " << gp.cov_xx    << "\n"
                << "  cov_yy     : " << gp.cov_yy    << "\n"
                << "  cov_zz     : " << gp.cov_zz    << "\n";
    }

    std::cout << "\nCommand [n=next, p=prev, q=quit] > ";
    std::string line;
    if (!std::getline(std::cin, line)) {
      break;
    }
    if (line.empty()) {
      continue;
    }
    char cmd = line[0];
    if (cmd == 'q' || cmd == 'Q') {
      break;
    } else if (cmd == 'n' || cmd == 'N') {
      if (idx + 1 < refs.size()) {
        idx += 1;
      }
    } else if (cmd == 'p' || cmd == 'P') {
      if (idx > 0) {
        idx -= 1;
      }
    }
  }
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
  bool gps_mode     = false;

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
    if (a == "--gps") {
      gps_mode = true;
      continue;
    }
    // unknown arg
    usage(argv[0]);
  }

  if (topic.empty() || start_ns == 0 || end_ns == 0) {
    usage(argv[0]);
  }

  // prevent conflicting tags
  int mode_count = (image_mode ? 1 : 0) +
                   (lidar_mode ? 1 : 0) +
                   (gps_mode   ? 1 : 0);
  if (mode_count > 1) {
    std::cerr << "Cannot combine --image, --lidar, or --gps\n";
    usage(argv[0]);
  }

  // default to image if no mode tag is given
  if (!image_mode && !lidar_mode && !gps_mode) {
    image_mode = true;
  }

  // Build refs once from append log
  std::vector<DataRef> refs =
      collectRefs(ssd_root, topic, start_ns, end_ns);

  if (lidar_mode) {
    viewLidarInteractive(refs);
  } else if (gps_mode) {
    viewGpsInteractive(refs);
  } else {  // image_mode
    viewImagesInteractive(refs);
  }

  return 0;
}
