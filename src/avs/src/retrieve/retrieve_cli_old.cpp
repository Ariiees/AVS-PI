#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <thread>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <cstdio>
#include <ctime>
#include <iomanip>
#include <sstream>


#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "avs/retrieve_api.h"

#include <vtkObject.h>
#if defined(VTK_MAJOR_VERSION) && (VTK_MAJOR_VERSION >= 9)
  #include <vtkLogger.h>
#endif

#include <sqlite3.h>  // NEW

namespace fs = std::filesystem;

namespace {

struct Args {
  std::string data;        // "image" | "lidar" | "gps"
  std::string start_wall;
  std::string end_wall;
  std::string sensor;      // for image/lidar exact topic id in DB; ignored for gps
  std::string image_db;
  std::string lidar_db;
  std::string gps_root = "/home/avs/DATA/SSD/gps";  // NEW
  bool do_list = false;
  bool do_view = false;    // not used for gps
  bool use_sw = false;
  bool bench = false;
  int  bench_max = 0;      // 0 = no limit
  bool quiet = true;
};

bool parseArgs(int argc, char** argv, Args& a) {
  for (int i = 1; i < argc; ++i) {
    std::string k = argv[i];
    if (k == "--data" && i+1 < argc)   { a.data = argv[++i]; continue; }
    if (k == "--start" && i+1 < argc)  { a.start_wall = argv[++i]; continue; }
    if (k == "--end" && i+1 < argc)    { a.end_wall = argv[++i]; continue; }
    if (k == "--sensor" && i+1 < argc) { a.sensor = argv[++i]; continue; }
    if (k == "--image-db" && i+1 < argc){ a.image_db = argv[++i]; continue; }
    if (k == "--lidar-db" && i+1 < argc){ a.lidar_db = argv[++i]; continue; }
    if (k == "--gps-root" && i+1 < argc){ a.gps_root = argv[++i]; continue; } // NEW
    if (k == "--list") { a.do_list = true; continue; }
    if (k == "--view") { a.do_view = true; continue; }
    if (k == "--sw")   { a.use_sw  = true; continue; }
    if (k == "--bench"){ a.bench = true; continue; }
    if (k == "--max" && i+1 < argc){ a.bench_max = std::atoi(argv[++i]); continue; }
    if (k == "--no-quiet"){ a.quiet = false; continue; }
  }
  if (a.data.empty() || a.start_wall.empty() || a.end_wall.empty())
    return false;
  return true;
}

std::vector<std::string> resolveDbPaths(const Args& a) {
  auto pick = [](const std::string& prefer, const std::vector<std::string>& fallbacks){
    if (!prefer.empty() && fs::exists(prefer)) return std::vector<std::string>{prefer};
    for (auto& p : fallbacks) if (fs::exists(p)) return std::vector<std::string>{p};
    return std::vector<std::string>{};
  };

  if (a.data == "image") {
    return pick(a.image_db, {
      "/home/avs/DATA/SSD/db/avs_image.sqlite3",
      "/home/avs/DATA/DB/avs_image.sqlite3"
    });
  }
  if (a.data == "lidar") {
    return pick(a.lidar_db, {
      "/home/avs/DATA/SSD/db/avs_lidar.sqlite3",
      "/home/avs/DATA/DB/avs_lidar.sqlite3"
    });
  }
  return {};
}

void printRecords(const std::vector<avs::AvsRecord>& recs) {
  for (auto& r : recs) {
    std::cout << r.sensor_id << " | "
              << r.data_type << " | "
              << r.ts_ms    << " | "
              << r.path     << "\n";
  }
}

// ---------- time helpers (local time) ----------
static inline int64_t tmToEpochMs(std::tm& tm) {
  tm.tm_isdst = -1;
  std::time_t tt = std::mktime(&tm); // local time
  return static_cast<int64_t>(tt) * 1000;
}

// Accepts YYYY-M-D_HH-MM or YYYY-M-D_HH-MM-SS
int64_t parseWallMs(const std::string& s) {
  int y=0,m=0,d=0,H=0,M=0,S=0;
  if (std::sscanf(s.c_str(), "%d-%d-%d_%d-%d-%d", &y,&m,&d,&H,&M,&S) < 5) {
    // try without seconds
    if (std::sscanf(s.c_str(), "%d-%d-%d_%d-%d", &y,&m,&d,&H,&M) < 5) {
      return 0;
    }
  }
  std::tm tm{}; tm.tm_year = y-1900; tm.tm_mon = m-1; tm.tm_mday = d;
  tm.tm_hour = H; tm.tm_min = M; tm.tm_sec = S;
  return tmToEpochMs(tm);
}

std::string dateStrFromMs(int64_t ms) {
  std::time_t tt = static_cast<std::time_t>(ms / 1000);
  std::tm tm{};
#if defined(_WIN32)
  localtime_s(&tm, &tt);
#else
  localtime_r(&tt, &tm);
#endif
  std::ostringstream oss;
  oss << std::setfill('0')
      << std::setw(4) << (tm.tm_year + 1900) << "-"
      << std::setw(2) << (tm.tm_mon + 1)    << "-"
      << std::setw(2) << tm.tm_mday;
  return oss.str();
}

// ---------- GPS support ----------
struct GpsRow {
  int64_t ts_ms = 0;
  double lat = 0, lon = 0, alt = 0;
  double cov_xx = 0, cov_yy = 0, cov_zz = 0;
};

bool queryGpsInFile(const std::string& dbpath, int64_t lo_ms, int64_t hi_ms,
                    std::vector<GpsRow>& out, bool quiet) {
  if (!fs::exists(dbpath)) return true; // silently skip
  sqlite3* db = nullptr;
  if (sqlite3_open_v2(dbpath.c_str(), &db, SQLITE_OPEN_READONLY, nullptr) != SQLITE_OK) {
    if (!quiet) std::cerr << "[gps] open fail: " << dbpath << " : " << sqlite3_errmsg(db) << "\n";
    if (db) sqlite3_close(db);
    return false;
  }
  const char* SQL =
    "SELECT ts_ms, latitude, longitude, altitude, cov_xx, cov_yy, cov_zz "
    "FROM gps_data WHERE ts_ms BETWEEN ?1 AND ?2 ORDER BY ts_ms;";
  sqlite3_stmt* st = nullptr;
  if (sqlite3_prepare_v2(db, SQL, -1, &st, nullptr) != SQLITE_OK) {
    if (!quiet) std::cerr << "[gps] prepare fail: " << sqlite3_errmsg(db) << "\n";
    sqlite3_close(db);
    return false;
  }
  sqlite3_bind_int64(st, 1, static_cast<sqlite3_int64>(lo_ms));
  sqlite3_bind_int64(st, 2, static_cast<sqlite3_int64>(hi_ms));
  for (;;) {
    int rc = sqlite3_step(st);
    if (rc == SQLITE_ROW) {
      GpsRow g;
      g.ts_ms = sqlite3_column_int64(st, 0);
      g.lat   = sqlite3_column_double(st, 1);
      g.lon   = sqlite3_column_double(st, 2);
      g.alt   = sqlite3_column_double(st, 3);
      g.cov_xx= sqlite3_column_double(st, 4);
      g.cov_yy= sqlite3_column_double(st, 5);
      g.cov_zz= sqlite3_column_double(st, 6);
      out.push_back(g);
    } else if (rc == SQLITE_DONE) {
      break;
    } else {
      if (!quiet) std::cerr << "[gps] step fail: " << sqlite3_errmsg(db) << "\n";
      sqlite3_finalize(st);
      sqlite3_close(db);
      return false;
    }
  }
  sqlite3_finalize(st);
  sqlite3_close(db);
  return true;
}


static inline int64_t addDaysMs(int64_t ms, int days) {
  using namespace std::chrono;
  auto tp = system_clock::time_point(milliseconds(ms));
  tp += hours(24) * days;            // advance by whole days
  return duration_cast<milliseconds>(tp.time_since_epoch()).count();
}


std::vector<GpsRow> listGpsRows(const std::string& root, const std::string& start_wall,
                                const std::string& end_wall, bool quiet) {
  int64_t lo_ms = parseWallMs(start_wall);
  int64_t hi_ms = parseWallMs(end_wall);
  if (hi_ms <= lo_ms) return {};
  // iterate per-day db files
  std::vector<GpsRow> rows;
  int64_t d_ms = lo_ms;
  while (d_ms <= hi_ms) {
    std::string day = dateStrFromMs(d_ms);
    std::string dbpath = (fs::path(root) / (day + ".sqlite3")).string();
    queryGpsInFile(dbpath, lo_ms, hi_ms, rows, quiet);
    d_ms = addDaysMs(d_ms, 1);
  }
  std::sort(rows.begin(), rows.end(), [](auto& a, auto& b){ return a.ts_ms < b.ts_ms; });
  return rows;
}


int listGps(const std::string& root, const std::string& start_wall, const std::string& end_wall, bool quiet) {
  auto rows = listGpsRows(root, start_wall, end_wall, quiet);
  // Match the 4-column output shape: sensor_id | data_type | ts_ms | "details"
  for (auto& g : rows) {
    std::cout << "gps"
              << " | " << "gps"
              << " | " << g.ts_ms
              << " | " << "lat=" << g.lat << ",lon=" << g.lon << ",alt=" << g.alt
              << ",cov_xx=" << g.cov_xx << ",cov_yy=" << g.cov_yy << ",cov_zz=" << g.cov_zz
              << "\n";
  }
  return 0;
}

// Bench GPS by stepping the SELECT and timing the first row (TTFB) and per-row step latencies (DECODE)
int benchGps(const std::string& root, const std::string& t0_wall, const std::string& t1_wall,
             int max_items, bool quiet) {
  using clock = std::chrono::steady_clock;
  int64_t lo_ms = parseWallMs(t0_wall);
  int64_t hi_ms = parseWallMs(t1_wall);
  if (hi_ms <= lo_ms) { std::cout << "NO_DATA\nEND\n"; return 0; }

  // Build the per-day file list first
  std::vector<std::string> files;
  for (int64_t d_ms = lo_ms; d_ms <= hi_ms; d_ms = addDaysMs(d_ms, 1)) {
    std::string day = dateStrFromMs(d_ms);
    std::string dbpath = (fs::path(root) / (day + ".sqlite3")).string();
    if (fs::exists(dbpath)) files.push_back(dbpath);
  }
  if (files.empty()) { std::cout << "NO_DATA\nEND\n"; return 0; }

  const char* SQL =
    "SELECT ts_ms, latitude, longitude, altitude, cov_xx, cov_yy, cov_zz "
    "FROM gps_data WHERE ts_ms BETWEEN ?1 AND ?2 ORDER BY ts_ms;";

  bool first_done = false;
  int emitted = 0;

  auto t_start = clock::now();

  for (auto& dbpath : files) {
    sqlite3* db = nullptr;
    if (sqlite3_open_v2(dbpath.c_str(), &db, SQLITE_OPEN_READONLY, nullptr) != SQLITE_OK) {
      if (!quiet) std::cerr << "[gps] open fail: " << dbpath << " : " << sqlite3_errmsg(db) << "\n";
      if (db) sqlite3_close(db);
      continue;
    }
    sqlite3_stmt* st = nullptr;
    if (sqlite3_prepare_v2(db, SQL, -1, &st, nullptr) != SQLITE_OK) {
      if (!quiet) std::cerr << "[gps] prepare fail: " << sqlite3_errmsg(db) << "\n";
      sqlite3_close(db);
      continue;
    }
    sqlite3_bind_int64(st, 1, static_cast<sqlite3_int64>(lo_ms));
    sqlite3_bind_int64(st, 2, static_cast<sqlite3_int64>(hi_ms));

    for (;;) {
      auto t0 = clock::now();
      int rc = sqlite3_step(st);
      auto t1 = clock::now();

      if (rc == SQLITE_ROW) {
        int64_t ts = sqlite3_column_int64(st, 0);
        double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        if (!first_done) {
          first_done = true;
          double ttfb = std::chrono::duration<double, std::milli>(t1 - t_start).count();
          std::cout << "TTFB\t" << ttfb << "\n";
        } else {
          // "bytes" proxy: 7 doubles ~ 56 bytes (not used by parser but keep shape)
          std::cout << "DECODE\t" << ts << "\t" << ms << "\t" << 56 << "\n";
          emitted++;
          if (max_items > 0 && emitted >= max_items) { sqlite3_finalize(st); sqlite3_close(db); goto DONE; }
        }
      } else if (rc == SQLITE_DONE) {
        break;
      } else {
        if (!quiet) std::cerr << "[gps] step fail: " << sqlite3_errmsg(db) << "\n";
        break;
      }
    }
    sqlite3_finalize(st);
    sqlite3_close(db);
  }

DONE:
  if (!first_done) std::cout << "NO_FIRST\n";
  std::cout << "END\n";
  return 0;
}


int viewImages(const std::vector<avs::AvsRecord>& recs, avs::RetrieveAPI& api) {
  if (recs.empty()) return 0;
  int idx = 0;
  cv::namedWindow("AVS Image", cv::WINDOW_NORMAL);
  cv::resizeWindow("AVS Image", 1280, 720);
  for (;;) {
    cv::Mat bgr;
    if (api.loadImage(recs[idx], &bgr)) cv::imshow("AVS Image", bgr);
    int key = cv::waitKey(0) & 0xFF;
    if (key == 'q' || key == 27) break;
    if (key == 'd') idx = (idx + 1) % recs.size();
    if (key == 'a') idx = (idx - 1 + recs.size()) % recs.size();
  }
  return 0;
}

int viewClouds(const std::vector<avs::AvsRecord>& recs, avs::RetrieveAPI& api) {
  if (recs.empty()) return 0;

  setenv("VTK_SILENCE_DEPRECATION", "1", 1);
#if defined(VTK_MAJOR_VERSION) && (VTK_MAJOR_VERSION >= 9)
  vtkLogger::SetStderrVerbosity(vtkLogger::VERBOSITY_ERROR);
#endif
  vtkObject::GlobalWarningDisplayOff();

  using CloudT = pcl::PointCloud<pcl::PointXYZI>;
  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("LiDAR"));
  viewer->setBackgroundColor(0, 0, 0);
  viewer->addCoordinateSystem(1.0);
  if (auto rw = viewer->getRenderWindow()) rw->SetMultiSamples(0);

  struct KeyState { char last = 0; } ks;
  viewer->registerKeyboardCallback(
      [](const pcl::visualization::KeyboardEvent& e, void* cookie){
        if (e.keyDown()) static_cast<KeyState*>(cookie)->last = static_cast<char>(e.getKeyCode());
      }, (void*)&ks);

  CloudT::Ptr cloud(new CloudT);
  int idx = 0;
  const std::string kId = "cloud";

  auto show = [&](int i) {
    cloud->clear();
    if (!api.loadLaz(recs[i], cloud)) return;
    CloudT::ConstPtr ccloud(cloud);
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity(ccloud, "intensity");
    if (!viewer->updatePointCloud<pcl::PointXYZI>(ccloud, intensity, kId)) {
      viewer->addPointCloud<pcl::PointXYZI>(ccloud, intensity, kId);
      viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, kId);
    }
  };

  show(idx);
  while (!viewer->wasStopped()) {
    viewer->spinOnce(50);
    if (ks.last == 'd') { idx = (idx + 1) % recs.size(); show(idx); ks.last = 0; }
    if (ks.last == 'a') { idx = (idx - 1 + recs.size()) % recs.size(); show(idx); ks.last = 0; }
    if (ks.last == 'q' || ks.last == 27) break;
  }
  return 0;
}

// --- For retrive_report.py (prints TTFB/DECODE lines) ---
int benchDecode(const std::vector<avs::AvsRecord>& recs, avs::RetrieveAPI& api, const std::string& modality, int max_items, bool quiet) {
  using clock = std::chrono::steady_clock;
  if (recs.empty()) { std::cout << "NO_DATA\n"; std::cout << "END\n"; return 0; }

  bool first_done = false;
  int emitted = 0;

  if (!quiet) {
    std::cerr << "[bench] items=" << recs.size()
              << " modality=" << modality
              << " max=" << max_items << "\n";
  }

  for (size_t i = 0; i < recs.size(); ++i) {
    auto t0 = clock::now();
    double ms = 0.0;
    size_t bytes = 0;

    if (modality == "image") {
      cv::Mat bgr;
      bool ok = api.loadImage(recs[i], &bgr);
      auto t1 = clock::now();
      ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
      if (!ok || bgr.empty()) continue;
      bytes = static_cast<size_t>(bgr.total() * bgr.elemSize());
    } else {
      using CloudT = pcl::PointCloud<pcl::PointXYZI>;
      CloudT::Ptr cloud(new CloudT);
      bool ok = api.loadLaz(recs[i], cloud);
      auto t1 = clock::now();
      ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
      if (!ok || !cloud || cloud->empty()) continue;
      bytes = cloud->size() * sizeof(pcl::PointXYZI);
    }

    if (!first_done) {
      first_done = true;
      std::cout << "TTFB\t" << ms << "\n";
      continue; // first success is NOT included in steady metrics
    }

    std::cout << "DECODE\t" << recs[i].ts_ms << "\t" << ms << "\t" << bytes << "\n";
    emitted++;
    if (max_items > 0 && emitted >= max_items) break;
  }

  if (!first_done) std::cout << "NO_FIRST\n";
  std::cout << "END\n";
  return 0;
}

} // namespace

int main(int argc, char** argv) {
  Args a;
  if (!parseArgs(argc, argv, a)) {
    std::cerr << "Usage: " << argv[0]
              << " --data image|lidar|gps"
              << " --start YYYY-M-D_HH-MM[(-SS)] --end YYYY-M-D_HH-MM[(-SS)]"
              << " [--sensor <exact_topic_in_db>]               # image/lidar"
              << " [--image-db </path/to/avs_image.sqlite3>]" 
              << " [--lidar-db </path/to/avs_lidar.sqlite3>]"
              << " [--gps-root </path/to/gps_dir>]              # default /home/avs/DATA/SSD/gps"
              << " [--list] [--view] [--sw]"
              << " [--bench] [--max N] [--no-quiet]\n";
    return 1;
  }

  if (a.use_sw) {
    setenv("LIBGL_ALWAYS_SOFTWARE", "1", 1);
    if (!a.quiet) std::cerr << "[INFO] LIBGL_ALWAYS_SOFTWARE=1\n";
  }

  // GPS mode is independent of image/lidar DBs
  if (a.data == "gps") {
    if (a.do_list || (!a.do_view && !a.bench)) {
      return listGps(a.gps_root, a.start_wall, a.end_wall, a.quiet);
    }
    if (a.do_view) {
      std::cerr << "[WARN] --view unsupported for gps; using --list.\n";
      return listGps(a.gps_root, a.start_wall, a.end_wall, a.quiet);
    }
    if (a.bench) {
      return benchGps(a.gps_root, a.start_wall, a.end_wall, a.bench_max, a.quiet);
    }
    return listGps(a.gps_root, a.start_wall, a.end_wall, a.quiet);
  }

  // image / lidar flow (unchanged)
  auto dbs = resolveDbPaths(a);
  if (dbs.empty()) {
    std::cerr << "DB path not found. Use --image-db/--lidar-db.\n";
    return 2;
  }

  std::string sensor_id = a.sensor;
  std::string data_type;
  if (a.data == "image") {
    if (sensor_id.empty()) sensor_id = "/my_camera/pylon_ros2_camera_node/image_raw";
    data_type = "jpg";
  } else if (a.data == "lidar") {
    if (sensor_id.empty()) sensor_id = "/sensing/lidar/top/pointcloud";
    data_type = "laz";
  } else {
    std::cerr << "[ERR] Unsupported --data: " << a.data << "\n";
    return 1;
  }

  std::vector<avs::AvsRecord> all;
  for (auto& db : dbs) {
    avs::RetrieveAPI api(db, "");
    if (!api.isOpen()) {
      if (!a.quiet) std::cerr << "DB open fail: " << db << " : " << api.lastError() << "\n";
      continue;
    }
    std::vector<avs::AvsRecord> recs;
    if (!api.list(sensor_id, data_type, a.start_wall, a.end_wall, &recs)) {
      if (!a.quiet) std::cerr << "Query fail (" << db << "): " << api.lastError() << "\n";
      continue;
    }
    all.insert(all.end(), recs.begin(), recs.end());
  }
  std::sort(all.begin(), all.end(), [](auto& x, auto& y){ return x.ts_ms < y.ts_ms; });

  if (a.do_list) {
    printRecords(all);
    return 0;
  }
  if (a.do_view) {
    for (auto& db : dbs) {
      avs::RetrieveAPI api(db,"");
      std::vector<avs::AvsRecord> recs;
      if (!api.list(sensor_id, data_type, a.start_wall, a.end_wall, &recs)) continue;
      if (a.data == "image") viewImages(recs, api);
      else viewClouds(recs, api);
    }
    return 0;
  }
  if (a.bench) {
    avs::RetrieveAPI api(dbs.front(), "");
    if (!api.isOpen()) { std::cerr << "DB open fail\n"; return 2; }
    std::vector<avs::AvsRecord> recs;
    if (!api.list(sensor_id, data_type, a.start_wall, a.end_wall, &recs)) {
      std::cout << "NO_DATA\nEND\n"; return 0;
    }
    std::sort(recs.begin(), recs.end(), [](auto& x, auto& y){ return x.ts_ms < y.ts_ms; });
    return benchDecode(recs, api, a.data, a.bench_max, a.quiet);
  }

  // default: list to stdout
  printRecords(all);
  return 0;
}
