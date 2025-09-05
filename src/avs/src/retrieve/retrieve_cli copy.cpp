#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <thread>
#include <chrono>
#include <cstdlib>
#include <filesystem>

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

namespace fs = std::filesystem;

namespace {

struct Args {
  std::string data;        // "image", "lidar", or "image+lidar"
  std::string start_wall;
  std::string end_wall;
  std::string sensor;      // NEW: exact sensor/topic id in DB
  std::string image_db;    // NEW
  std::string lidar_db;    // NEW
  bool do_list = false;
  bool do_view = false;
  bool use_sw = false;
};

bool parseArgs(int argc, char** argv, Args& a) {
  for (int i = 1; i < argc; ++i) {
    std::string k = argv[i];
    if (k == "--data" && i+1 < argc)   { a.data = argv[++i]; continue; }
    if (k == "--start" && i+1 < argc)  { a.start_wall = argv[++i]; continue; }
    if (k == "--end" && i+1 < argc)    { a.end_wall = argv[++i]; continue; }
    if (k == "--sensor" && i+1 < argc) { a.sensor = argv[++i]; continue; }          // NEW
    if (k == "--image-db" && i+1 < argc){ a.image_db = argv[++i]; continue; }       // NEW
    if (k == "--lidar-db" && i+1 < argc){ a.lidar_db = argv[++i]; continue; }       // NEW
    if (k == "--list") { a.do_list = true; continue; }
    if (k == "--view") { a.do_view = true; continue; }
    if (k == "--sw")   { a.use_sw  = true; continue; }
  }
  if (a.data.empty() || a.start_wall.empty() || a.end_wall.empty())
    return false;
  return true;
}

std::vector<std::string> resolveDbPaths(const Args& a) {                     // CHANGED
  auto pick = [](const std::string& prefer, const std::vector<std::string>& fallbacks){
    if (!prefer.empty() && fs::exists(prefer)) return std::vector<std::string>{prefer};
    for (auto& p : fallbacks) if (fs::exists(p)) return std::vector<std::string>{p};
    return std::vector<std::string>{}; // none found
  };

  if (a.data == "image") {
    return pick(a.image_db, {
      "/home/avs/DATA/SSD/db/avs_image.sqlite3",   // your actual path
      "/home/avs/DATA/DB/avs_image.sqlite3"        // legacy fallback
    });
  }
  if (a.data == "lidar") {
    return pick(a.lidar_db, {
      "/home/avs/DATA/SSD/db/avs_lidar.sqlite3",
      "/home/avs/DATA/DB/avs_lidar.sqlite3"
    });
  }
  // image+lidar not used for benchmarking; keep empty
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

int viewImages(const std::vector<avs::AvsRecord>& recs, avs::RetrieveAPI& api) {
  if (recs.empty()) return 0;
  int idx = 0;
  cv::namedWindow("AVS Image", cv::WINDOW_NORMAL);
  cv::resizeWindow("AVS Image", 1280, 720);
  for (;;) {
    cv::Mat bgr;
    if (api.loadImage(recs[idx], &bgr)) cv::imshow("AVS Image", bgr);
    else std::cerr << "Image load fail: " << api.lastError() << "\n";
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
    if (!api.loadLaz(recs[i], cloud)) {
      std::cerr << "LAZ load fail: " << api.lastError() << "\n";
      return;
    }
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

} // namespace

int main(int argc, char** argv) {
  Args a;
  if (!parseArgs(argc, argv, a)) {
    std::cerr << "Usage: " << argv[0]
              << " --data image|lidar"
              << " --start YYYY-M-D_HH-MM --end YYYY-M-D_HH-MM"
              << " [--sensor <exact_topic_in_db>]"
              << " [--image-db </path/to/avs_image.sqlite3>]"
              << " [--lidar-db </path/to/avs_lidar.sqlite3>]"
              << " [--list] [--view] [--sw]\n";
    return 1;
  }

  if (a.use_sw) {
    setenv("LIBGL_ALWAYS_SOFTWARE", "1", 1);
    std::cerr << "[INFO] Running with LIBGL_ALWAYS_SOFTWARE=1\n";
  }

  // Resolve DB(s)
  auto dbs = resolveDbPaths(a);
  if (dbs.empty()) {
    std::cerr << "DB path not found. Use --image-db/--lidar-db.\n";
    return 2;
  }

  // Sensor + datatype
  std::string sensor_id = a.sensor;
  std::string data_type;
  if (a.data == "image") {
    if (sensor_id.empty())
      sensor_id = "/my_camera/pylon_ros2_camera_node/image_raw";   // default for your setup
    data_type = "jpg";
  } else if (a.data == "lidar") {
    if (sensor_id.empty())
      sensor_id = "/sensing/lidar/top/pointcloud";                 // default for your setup
    data_type = "laz";
  } else {
    std::cerr << "[ERR] Unsupported --data: " << a.data << "\n";
    return 1;
  }

  // Query
  std::vector<avs::AvsRecord> all;
  for (auto& db : dbs) {
    avs::RetrieveAPI api(db, "");
    if (!api.isOpen()) {
      std::cerr << "DB open fail: " << db << " : " << api.lastError() << "\n";
      continue;
    }
    std::vector<avs::AvsRecord> recs;
    if (!api.list(sensor_id, data_type, a.start_wall, a.end_wall, &recs)) {
      std::cerr << "Query fail (" << db << "): " << api.lastError() << "\n";
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

  // Default action if neither --list nor --view: list to stdout (useful for piping)
  printRecords(all);
  return all.empty() ? 0 : 0;
}
