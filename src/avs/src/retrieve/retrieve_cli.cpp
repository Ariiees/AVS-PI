// retrieve_cli.cpp
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

#include <opencv2/opencv.hpp>

#include <vtkObject.h>
#if defined(VTK_MAJOR_VERSION) && (VTK_MAJOR_VERSION >= 9)
  #include <vtkLogger.h>
#endif

#include "avs/retrieve_api.h"

namespace fs = std::filesystem;

static const fs::path kSsdRoot("/home/avs/DATA/SSD");

static void usage(const char* p) {
  std::cerr << "Usage: " << p
            << " --topic <sensor_topic> --start <ts_ns> --end <ts_ns> "
               "[--image | --lidar | --gps]\n";
  std::exit(1);
}

// -----------------------------------------------------------------------------
// LiDAR payload decode
// -----------------------------------------------------------------------------

static bool loadLazFromPayload(const std::vector<std::uint8_t>& payload,
                               pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) {
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

  std::string buf;
  buf.assign(reinterpret_cast<const char*>(payload.data()), payload.size());
  std::istringstream iss(buf, std::ios::binary);

  laszip_BOOL is_compressed = 0;
  if (laszip_open_reader_stream(reader, iss, &is_compressed) != 0) {
    std::cerr << "laszip_open_reader_stream failed\n";
    laszip_destroy(reader);
    return false;
  }

  laszip_header* header = nullptr;
  if (laszip_get_header_pointer(reader, &header) != 0 || !header) {
    std::cerr << "laszip_get_header_pointer failed\n";
    laszip_close_reader(reader);
    laszip_destroy(reader);
    return false;
  }

  laszip_point* point = nullptr;
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

  cloud->width = static_cast<std::uint32_t>(cloud->size());
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
// Viewer: images one frame at a time
// -----------------------------------------------------------------------------

static void viewImagesInteractive(const avs::RetrieveAPI& api,
                                  const std::vector<avs::DataRef>& refs) {
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
    const auto& ref = refs[idx];

    std::vector<std::uint8_t> payload;
    std::string err;
    if (!api.LoadPayload(ref, payload, &err)) {
      std::cerr << "Skip index " << idx << " load error " << err << "\n";
    } else {
      cv::Mat buf_mat(1, static_cast<int>(payload.size()), CV_8UC1);
      std::memcpy(buf_mat.data, payload.data(), payload.size());
      cv::Mat img = cv::imdecode(buf_mat, cv::IMREAD_COLOR);

      if (img.empty()) {
        std::cerr << "Failed to decode image at index " << idx << "\n";
      } else {
        std::string title = ref.sensor_topic +
                            " day=" + ref.day +
                            " trip=" + std::to_string(ref.trip_id) +
                            " ts=" + std::to_string(ref.ts_ns);
        cv::imshow(win_name, img);
        cv::setWindowTitle(win_name, title);
      }
    }

    int key = cv::waitKey(0);

    bool go_prev = (key == 81 || key == 2424832);
    bool go_next = (key == 83 || key == 2555904);

    if (key == 'q' || key == 'Q' || key == 27) {
      break;
    } else if (go_next) {
      if (idx + 1 < refs.size()) idx += 1;
    } else if (go_prev) {
      if (idx > 0) idx -= 1;
    }
  }

  cv::destroyWindow(win_name);
}

// -----------------------------------------------------------------------------
// LiDAR viewer
// -----------------------------------------------------------------------------

static void viewLidarInteractive(const avs::RetrieveAPI& api,
                                 const std::vector<avs::DataRef>& refs) {
  if (refs.empty()) {
    std::cout << "No matching LiDAR frames in requested window\n";
    return;
  }

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

  if (auto rw = viewer->getRenderWindow()) {
    rw->SetMultiSamples(0);
  }

  struct KeyState { char last = 0; } ks;

  viewer->registerKeyboardCallback(
      [](const pcl::visualization::KeyboardEvent& e, void* cookie) {
        if (!e.keyDown()) return;
        auto* state = static_cast<KeyState*>(cookie);
        state->last = static_cast<char>(e.getKeyCode());
      },
      static_cast<void*>(&ks));

  CloudT::Ptr cloud(new CloudT);
  int idx = 0;
  const std::string kId = "cloud";

  auto show = [&](int i) {
    cloud->clear();

    std::vector<std::uint8_t> payload;
    std::string err;
    if (!api.LoadPayload(refs[static_cast<std::size_t>(i)], payload, &err)) {
      std::cerr << "Skip frame " << i << " payload load failed " << err << "\n";
      return;
    }
    if (!loadLazFromPayload(payload, cloud)) {
      std::cerr << "Skip frame " << i << " LAZ decode failed\n";
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

  show(idx);

  while (!viewer->wasStopped()) {
    viewer->spinOnce(50);

    if (ks.last == 'd') {
      idx = (idx + 1) % static_cast<int>(refs.size());
      show(idx);
      ks.last = 0;
    } else if (ks.last == 'a') {
      idx = (idx - 1 + static_cast<int>(refs.size())) % static_cast<int>(refs.size());
      show(idx);
      ks.last = 0;
    } else if (ks.last == 'q' || ks.last == 27) {
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

static void viewGpsInteractive(const avs::RetrieveAPI& api,
                               const std::vector<avs::DataRef>& refs) {
  if (refs.empty()) {
    std::cout << "No matching GPS records in requested window\n";
    return;
  }

  std::cout << "Found " << refs.size()
            << " GPS records. Use n for next, p for previous, q to quit.\n";

  std::size_t idx = 0;
  std::cin.clear();
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

  while (true) {
    const auto& ref = refs[idx];

    std::vector<std::uint8_t> payload;
    std::string err;
    if (!api.LoadPayload(ref, payload, &err)) {
      std::cerr << "Skip index " << idx << " load error " << err << "\n";
    } else if (payload.size() < sizeof(GpsPayload)) {
      std::cerr << "Payload too small for GpsPayload at index " << idx << "\n";
    } else {
      GpsPayload gp{};
      std::memcpy(&gp, payload.data(), sizeof(GpsPayload));

      std::time_t t_sec = static_cast<std::time_t>(ref.ts_ns / 1000000000ULL);
      std::tm tm_buf{};
      char time_str[64];
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
    if (!std::getline(std::cin, line)) break;
    if (line.empty()) continue;

    char cmd = line[0];
    if (cmd == 'q' || cmd == 'Q') {
      break;
    } else if (cmd == 'n' || cmd == 'N') {
      if (idx + 1 < refs.size()) idx += 1;
    } else if (cmd == 'p' || cmd == 'P') {
      if (idx > 0) idx -= 1;
    }
  }
}

// -----------------------------------------------------------------------------
// main
// -----------------------------------------------------------------------------

int main(int argc, char** argv) {
  fs::path ssd_root = kSsdRoot;
  std::string topic;
  std::uint64_t start_ns = 0;
  std::uint64_t end_ns = 0;
  bool image_mode = false;
  bool lidar_mode = false;
  bool gps_mode = false;

  if (argc < 7) {
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
    usage(argv[0]);
  }

  if (topic.empty() || start_ns == 0 || end_ns == 0) {
    usage(argv[0]);
  }

  int mode_count = (image_mode ? 1 : 0) + (lidar_mode ? 1 : 0) + (gps_mode ? 1 : 0);
  if (mode_count > 1) {
    std::cerr << "Cannot combine --image, --lidar, or --gps\n";
    usage(argv[0]);
  }

  if (!image_mode && !lidar_mode && !gps_mode) {
    image_mode = true;
  }

  avs::RetrieveAPI api(ssd_root);
  std::string err;
  std::vector<avs::DataRef> refs = api.QueryRefs(topic, start_ns, end_ns, &err);

  if (refs.empty()) {
    if (!err.empty()) std::cerr << err << "\n";
    if (lidar_mode) {
      std::cout << "No matching LiDAR frames in requested window\n";
    } else if (gps_mode) {
      std::cout << "No matching GPS records in requested window\n";
    } else {
      std::cout << "No matching images in requested window\n";
    }
    return 0;
  }

  if (lidar_mode) {
    viewLidarInteractive(api, refs);
  } else if (gps_mode) {
    viewGpsInteractive(api, refs);
  } else {
    viewImagesInteractive(api, refs);
  }

  return 0;
}
