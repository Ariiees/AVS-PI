// retrieve_view.cpp
#include <iostream>
#include <filesystem>
#include <string>
#include <vector>
#include <fstream>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <sstream>
#include <thread>
#include <chrono>

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

enum class NavigationKey {
  kNone,
  kPrevious,
  kNext,
  kQuit,
};

static NavigationKey navigationKey(int key) {
  // OpenCV HighGUI returns backend-specific codes for special keys.
  constexpr int kQtLeft = 0x01000012;
  constexpr int kQtRight = 0x01000014;
  constexpr int kX11Left = 0xff51;
  constexpr int kX11Right = 0xff53;
  constexpr int kWindowsLeft = 0x250000;
  constexpr int kWindowsRight = 0x270000;

  if (key == 'q' || key == 'Q' || key == 27) {
    return NavigationKey::kQuit;
  }
  if (key == 'p' || key == 'P' ||
      key == kQtLeft || key == kX11Left || key == kWindowsLeft) {
    return NavigationKey::kPrevious;
  }
  if (key == 'n' || key == 'N' ||
      key == kQtRight || key == kX11Right || key == kWindowsRight) {
    return NavigationKey::kNext;
  }
  return NavigationKey::kNone;
}

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
            << " images. Use n/right for next, p/left for previous, q to quit.\n";

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

    const NavigationKey key = navigationKey(cv::waitKeyEx(0));
    if (key == NavigationKey::kQuit) {
      break;
    } else if (key == NavigationKey::kNext) {
      if (idx + 1 < refs.size()) idx += 1;
    } else if (key == NavigationKey::kPrevious) {
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

  struct KeyState {
    int step = 0;
    bool quit = false;
  } ks;

  viewer->registerKeyboardCallback(
      [](const pcl::visualization::KeyboardEvent& e, void* cookie) {
        if (!e.keyDown()) return;
        auto* state = static_cast<KeyState*>(cookie);
        const std::string& key = e.getKeySym();
        if (key == "n" || key == "N" || key == "Right") {
          state->step = 1;
        } else if (key == "p" || key == "P" || key == "Left") {
          state->step = -1;
        } else if (key == "q" || key == "Q" || key == "Escape") {
          state->quit = true;
        }
      },
      static_cast<void*>(&ks));

  CloudT::Ptr cloud(new CloudT);
  std::size_t idx = 0;
  const std::string kId = "cloud";

  auto show = [&](std::size_t i) {
    cloud->clear();

    std::vector<std::uint8_t> payload;
    std::string err;
    if (!api.LoadPayload(refs[i], payload, &err)) {
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

    viewer->removePointCloud(kId);
    viewer->addPointCloud<pcl::PointXYZI>(ccloud, intensity, kId);
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, kId);

    const auto& ref = refs[i];
    std::cout << "LiDAR frame [" << i + 1 << "/" << refs.size()
              << "] ts_ns=" << ref.ts_ns
              << " points=" << cloud->size() << "\n";
  };

  std::cout << "Found " << refs.size()
            << " LiDAR frames. Use n/right for next, p/left for previous, q to quit.\n";
  show(idx);

  while (!viewer->wasStopped()) {
    viewer->spinOnce(50);

    if (ks.quit) {
      break;
    }
    if (ks.step > 0) {
      if (idx + 1 < refs.size()) {
        idx += 1;
        show(idx);
      }
      ks.step = 0;
    } else if (ks.step < 0) {
      if (idx > 0) {
        idx -= 1;
        show(idx);
      }
      ks.step = 0;
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

struct GpsRecord {
  bool valid = false;
  GpsPayload payload{};
};

struct GpsTrajectoryView {
  cv::Mat base;
  std::vector<cv::Point> pixels;
  std::vector<std::size_t> record_indices;
};

struct GpsMouseState {
  const std::vector<cv::Point>* pixels = nullptr;
  const std::vector<std::size_t>* record_indices = nullptr;
  std::size_t hovered_record = 0;
  std::size_t hovered_pixel = 0;
  bool has_hover = false;
  bool changed = true;
};

static GpsTrajectoryView buildGpsTrajectory(const std::vector<GpsRecord>& records) {
  constexpr int kWidth = 1000;
  constexpr int kHeight = 760;
  constexpr int kMargin = 80;
  constexpr double kMetersPerDegree = 111320.0;
  constexpr double kPi = 3.14159265358979323846;

  GpsTrajectoryView view;
  view.base = cv::Mat(kHeight, kWidth, CV_8UC3, cv::Scalar(25, 25, 25));
  cv::rectangle(view.base, cv::Point(kMargin, kMargin),
                cv::Point(kWidth - kMargin, kHeight - kMargin),
                cv::Scalar(80, 80, 80), 1);

  std::size_t origin_idx = 0;
  while (origin_idx < records.size() && !records[origin_idx].valid) {
    ++origin_idx;
  }
  if (origin_idx == records.size()) {
    cv::putText(view.base, "No valid GPS records in the requested time range",
                cv::Point(kMargin, kHeight / 2), cv::FONT_HERSHEY_SIMPLEX,
                0.8, cv::Scalar(230, 230, 230), 2);
    return view;
  }

  const double lat0 = records[origin_idx].payload.latitude;
  const double lon0 = records[origin_idx].payload.longitude;
  const double lon_scale = kMetersPerDegree * std::cos(lat0 * kPi / 180.0);

  std::vector<cv::Point2d> local_points;
  local_points.reserve(records.size());
  view.record_indices.reserve(records.size());
  for (std::size_t i = 0; i < records.size(); ++i) {
    if (!records[i].valid) continue;
    local_points.emplace_back(
        (records[i].payload.longitude - lon0) * lon_scale,
        (records[i].payload.latitude - lat0) * kMetersPerDegree);
    view.record_indices.push_back(i);
  }

  double min_x = local_points.front().x;
  double max_x = local_points.front().x;
  double min_y = local_points.front().y;
  double max_y = local_points.front().y;
  for (const auto& point : local_points) {
    min_x = std::min(min_x, point.x);
    max_x = std::max(max_x, point.x);
    min_y = std::min(min_y, point.y);
    max_y = std::max(max_y, point.y);
  }

  const double range_x = std::max(max_x - min_x, 1.0);
  const double range_y = std::max(max_y - min_y, 1.0);
  const double scale = std::min(
      static_cast<double>(kWidth - 2 * kMargin) / range_x,
      static_cast<double>(kHeight - 2 * kMargin) / range_y);
  const double center_x = (min_x + max_x) / 2.0;
  const double center_y = (min_y + max_y) / 2.0;

  auto toPixel = [&](const cv::Point2d& point) {
    return cv::Point(
        static_cast<int>(std::lround(kWidth / 2.0 + (point.x - center_x) * scale)),
        static_cast<int>(std::lround(kHeight / 2.0 - (point.y - center_y) * scale)));
  };

  view.pixels.reserve(local_points.size());
  for (const auto& point : local_points) {
    view.pixels.push_back(toPixel(point));
  }

  if (view.pixels.size() > 1) {
    cv::polylines(view.base, view.pixels, false, cv::Scalar(0, 210, 255), 2, cv::LINE_AA);
  }
  cv::circle(view.base, view.pixels.front(), 7, cv::Scalar(0, 220, 0), cv::FILLED, cv::LINE_AA);
  cv::circle(view.base, view.pixels.back(), 8, cv::Scalar(0, 0, 255), cv::FILLED, cv::LINE_AA);
  cv::putText(view.base, "Move mouse over trajectory to inspect GPS record. q=quit",
              cv::Point(25, kHeight - 25), cv::FONT_HERSHEY_SIMPLEX,
              0.55, cv::Scalar(210, 210, 210), 1, cv::LINE_AA);
  return view;
}

static cv::Mat drawGpsHover(const GpsTrajectoryView& view,
                            const std::vector<GpsRecord>& records,
                            const std::vector<avs::DataRef>& refs,
                            const GpsMouseState& mouse) {
  cv::Mat canvas = view.base.clone();
  if (!mouse.has_hover) {
    cv::putText(canvas, "Full GPS trajectory",
                cv::Point(25, 32), cv::FONT_HERSHEY_SIMPLEX,
                0.7, cv::Scalar(235, 235, 235), 2, cv::LINE_AA);
    return canvas;
  }

  const std::size_t idx = mouse.hovered_record;
  const GpsPayload& gp = records[idx].payload;
  const avs::DataRef& ref = refs[idx];
  if (mouse.hovered_pixel < view.pixels.size()) {
    cv::circle(canvas, view.pixels[mouse.hovered_pixel],
               7, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
  }

  std::ostringstream metadata;
  metadata << "record " << idx + 1 << "/" << records.size()
           << "  topic " << ref.sensor_topic
           << "  day " << ref.day
           << "  trip " << ref.trip_id
           << "  ts_ns " << ref.ts_ns;
  cv::putText(canvas, metadata.str(), cv::Point(25, 22),
              cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(235, 235, 235), 1, cv::LINE_AA);

  std::ostringstream position;
  position.precision(10);
  position << "latitude " << gp.latitude
           << "  longitude " << gp.longitude
           << "  altitude " << gp.altitude << " m";
  cv::putText(canvas, position.str(), cv::Point(25, 46),
              cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(235, 235, 235), 1, cv::LINE_AA);

  std::ostringstream covariance;
  covariance.precision(8);
  covariance << "cov_xx " << gp.cov_xx
             << "  cov_yy " << gp.cov_yy
             << "  cov_zz " << gp.cov_zz;
  cv::putText(canvas, covariance.str(), cv::Point(25, 70),
              cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(235, 235, 235), 1, cv::LINE_AA);
  return canvas;
}

static void onGpsMouse(int event, int x, int y, int, void* userdata) {
  if (event != cv::EVENT_MOUSEMOVE) return;
  auto* state = static_cast<GpsMouseState*>(userdata);
  if (!state || !state->pixels || !state->record_indices) return;

  constexpr int kHoverRadius = 12;
  double best_distance_sq = kHoverRadius * kHoverRadius + 1.0;
  std::size_t best_record = 0;
  std::size_t best_pixel = 0;
  const cv::Point2d mouse(x, y);

  for (std::size_t i = 0; i < state->pixels->size(); ++i) {
    const cv::Point2d start((*state->pixels)[i]);
    cv::Point2d closest = start;
    std::size_t closest_pixel = i;
    if (i + 1 < state->pixels->size()) {
      const cv::Point2d end((*state->pixels)[i + 1]);
      const cv::Point2d segment = end - start;
      const double length_sq = segment.dot(segment);
      const double t = length_sq > 0.0
          ? std::clamp((mouse - start).dot(segment) / length_sq, 0.0, 1.0)
          : 0.0;
      closest = start + segment * t;
      if (t >= 0.5) closest_pixel = i + 1;
    }
    const cv::Point2d delta = closest - mouse;
    const double distance_sq = delta.dot(delta);
    if (distance_sq < best_distance_sq) {
      best_distance_sq = distance_sq;
      best_pixel = closest_pixel;
      best_record = (*state->record_indices)[closest_pixel];
    }
  }

  const bool has_hover = best_distance_sq <= kHoverRadius * kHoverRadius;
  if (has_hover != state->has_hover ||
      (has_hover && (best_record != state->hovered_record ||
                     best_pixel != state->hovered_pixel))) {
    state->has_hover = has_hover;
    state->hovered_record = best_record;
    state->hovered_pixel = best_pixel;
    state->changed = true;
  }
}

static void viewGpsInteractive(const avs::RetrieveAPI& api,
                               const std::vector<avs::DataRef>& refs) {
  if (refs.empty()) {
    std::cout << "No matching GPS records in requested window\n";
    return;
  }

  std::vector<GpsRecord> records(refs.size());
  for (std::size_t i = 0; i < refs.size(); ++i) {
    std::vector<std::uint8_t> payload;
    std::string err;
    if (!api.LoadPayload(refs[i], payload, &err)) {
      std::cerr << "Skip index " << i << " load error " << err << "\n";
    } else if (payload.size() < sizeof(GpsPayload)) {
      std::cerr << "Payload too small for GpsPayload at index " << i << "\n";
    } else {
      std::memcpy(&records[i].payload, payload.data(), sizeof(GpsPayload));
      const auto& gp = records[i].payload;
      records[i].valid = std::isfinite(gp.latitude) && std::isfinite(gp.longitude) &&
                         std::isfinite(gp.altitude) &&
                         gp.latitude >= -90.0 && gp.latitude <= 90.0 &&
                         gp.longitude >= -180.0 && gp.longitude <= 180.0;
    }
  }

  const GpsTrajectoryView trajectory = buildGpsTrajectory(records);
  std::cout << "Found " << refs.size()
            << " GPS records. Showing the full trajectory; move the mouse over it to inspect a record.\n";

  const std::string win_name = "AVS GPS Trajectory";
  cv::namedWindow(win_name, cv::WINDOW_NORMAL);
  GpsMouseState mouse;
  mouse.pixels = &trajectory.pixels;
  mouse.record_indices = &trajectory.record_indices;
  cv::setMouseCallback(win_name, onGpsMouse, &mouse);

  while (true) {
    if (mouse.changed) {
      cv::imshow(win_name, drawGpsHover(trajectory, records, refs, mouse));
      mouse.changed = false;
    }
    const NavigationKey key = navigationKey(cv::waitKeyEx(20));
    if (key == NavigationKey::kQuit) {
      break;
    }
  }

  cv::setMouseCallback(win_name, nullptr);
  cv::destroyWindow(win_name);
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
