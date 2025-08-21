#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <thread>
#include <chrono>
#include <cstdlib>

#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "avs/retrieve_api.h"

#include <vtkObject.h>    // vtkObject::GlobalWarningDisplayOff
#if defined(VTK_MAJOR_VERSION) && (VTK_MAJOR_VERSION >= 9)
  #include <vtkLogger.h>  // vtkLogger (VTK 9+)
#endif

namespace {

struct Args {
  std::string data;        // "image", "lidar", or "image+lidar"
  std::string start_wall;
  std::string end_wall;
  bool do_list = false;
  bool do_view = false;
  bool use_sw = false;
};

bool parseArgs(int argc, char** argv, Args& a) {
  for (int i = 1; i < argc; ++i) {
    std::string k = argv[i];
    if (k == "--data" && i+1 < argc) { a.data = argv[++i]; continue; }
    if (k == "--start" && i+1 < argc) { a.start_wall = argv[++i]; continue; }
    if (k == "--end" && i+1 < argc) { a.end_wall = argv[++i]; continue; }
    if (k == "--list") { a.do_list = true; continue; }
    if (k == "--view") { a.do_view = true; continue; }
    if (k == "--sw")   { a.use_sw  = true; continue; }
  }
  if (a.data.empty() || a.start_wall.empty() || a.end_wall.empty())
    return false;
  return true;
}

std::vector<std::string> resolveDbPaths(const std::string& data) {
  if (data == "image") return {"/home/avs/DATA/DB/avs_image.sqlite3"};
  if (data == "lidar") return {"/home/avs/DATA/DB/avs_lidar.sqlite3"};
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
    if (api.loadImage(recs[idx], &bgr)) {
      cv::imshow("AVS Image", bgr);
    } else {
      std::cerr << "Image load fail: " << api.lastError() << "\n";
    }
    int key = cv::waitKey(0) & 0xFF;
    if (key == 'q' || key == 27) break;
    if (key == 'd') idx = (idx + 1) % recs.size();
    if (key == 'a') idx = (idx - 1 + recs.size()) % recs.size();
  }
  return 0;
}

struct KeyState { char last = 0; };
void kbCallback(const pcl::visualization::KeyboardEvent& e, void* cookie) {
  if (!e.keyDown()) return;
  auto* ks = static_cast<KeyState*>(cookie);
  ks->last = static_cast<char>(e.getKeyCode());
}

int viewClouds(const std::vector<avs::AvsRecord>& recs, avs::RetrieveAPI& api) {
  if (recs.empty()) return 0;

  // Hide VTK deprecation spam (safe; does not affect functionality)
  setenv("VTK_SILENCE_DEPRECATION", "1", 1);   // env-based silence (VTK honors this)
  #if defined(VTK_MAJOR_VERSION) && (VTK_MAJOR_VERSION >= 9)
  vtkLogger::SetStderrVerbosity(vtkLogger::VERBOSITY_ERROR);  // only errors+
  #endif
  vtkObject::GlobalWarningDisplayOff();  // last resort: suppress all VTK warnings

  using CloudT = pcl::PointCloud<pcl::PointXYZI>;

  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("LiDAR"));
  viewer->setBackgroundColor(0, 0, 0);
  viewer->addCoordinateSystem(1.0);

  auto rw = viewer->getRenderWindow();
  if (rw) {
    rw->SetMultiSamples(0);            // critical line
    // (Optional) rw->SetStereoRender(0); // keep things simple
  }


  struct KeyState { char last = 0; } ks;
  viewer->registerKeyboardCallback(
      [](const pcl::visualization::KeyboardEvent& e, void* cookie){
        if (!e.keyDown()) return;
        static_cast<KeyState*>(cookie)->last = static_cast<char>(e.getKeyCode());
      }, (void*)&ks);

  CloudT::Ptr cloud(new CloudT);
  int idx = 0;
  const std::string kId = "cloud";

  auto show = [&](int i) {
    cloud->clear(); // keep memory bounded
    if (!api.loadLaz(recs[i], cloud)) {
      std::cerr << "LAZ load fail: " << api.lastError() << "\n";
      return;
    }

    // Use ConstPtr for update/add
    CloudT::ConstPtr ccloud(cloud);

    // Keep intensity coloring consistent
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity(ccloud, "intensity");

    // Try update first; if it doesn't exist yet, add
    if (!viewer->updatePointCloud<pcl::PointXYZI>(ccloud, intensity, kId)) {
      viewer->addPointCloud<pcl::PointXYZI>(ccloud, intensity, kId);
      viewer->setPointCloudRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, kId);
    }

    // (Optional) you can update the window name with ts/path if you want:
    // viewer->setWindowName(std::to_string(recs[i].ts_ms) + " | " + recs[i].path);
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
              << " --data image|lidar|image+lidar"
              << " [--sensor <id>] --start YYYY-M-D_HH-MM --end YYYY-M-D_HH-MM"
              << " --list|--view\n";
    return 1;
  }

  if (a.use_sw) {
    // Force Mesa software rasterizer
    setenv("LIBGL_ALWAYS_SOFTWARE", "1", 1);
    std::cerr << "[INFO] Running with LIBGL_ALWAYS_SOFTWARE=1\n";
  }

  std::vector<avs::AvsRecord> all;

  // Map "image" -> (/camera/image, jpg), "lidar" -> (/lidar/pointcloud, laz)
  std::string sensor_id, data_type;
  if (a.data == "image") {
    sensor_id = "/camera/image";
    data_type = "jpg";
  } else if (a.data == "lidar") {
    sensor_id = "/lidar/pointcloud";
    data_type = "laz";
  } 

  for (auto& db : resolveDbPaths(a.data)) {
    avs::RetrieveAPI api(db, "");
    if (!api.isOpen()) { std::cerr << "DB open fail: " << api.lastError() << "\n"; continue; }
    std::vector<avs::AvsRecord> recs;
    if (!api.list(sensor_id, data_type, a.start_wall, a.end_wall, &recs)) {
      std::cerr << "Query fail: " << api.lastError() << "\n"; continue;
    }
    all.insert(all.end(), recs.begin(), recs.end());
  }

  std::sort(all.begin(), all.end(), [](auto& x, auto& y){ return x.ts_ms < y.ts_ms; });

  if (a.do_list) {
    printRecords(all);
  } else if (a.do_view) {
    for (auto& db : resolveDbPaths(a.data)) {
      avs::RetrieveAPI api(db,"");
      std::vector<avs::AvsRecord> recs;
      api.list(sensor_id,data_type,a.start_wall,a.end_wall,&recs);
      if (recs.empty()) continue;
      if (db.find("image") != std::string::npos) viewImages(recs, api);
      if (db.find("lidar") != std::string::npos) viewClouds(recs, api);
    }
  }

  return 0;
}
