#include <chrono>
#include <cmath>
#include <cstring>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <gps_msgs/msg/gps_fix.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <yaml-cpp/yaml.h>

#include "avs/img_dedup.h"
#include "avs/lidar_compress.h"
#include "avs/lidar_downsample.h"

namespace {

struct SensorStats {
  std::uint64_t msg_count = 0;
  std::uint64_t raw_bytes = 0;
  std::uint64_t reduced_bytes = 0;
  std::uint64_t errors = 0;
};

struct SensorSamples {
  std::vector<double> raw_mb_s;
  std::vector<double> reduced_mb_s;
  std::vector<double> ratio_raw_over_reduced;
};

struct TopicDefaults {
  std::string camera_topic;
  std::string lidar_topic;
  std::string gps_topic;
};

struct GpsPayload {
  double latitude;
  double longitude;
  double altitude;
  double cov_xx;
  double cov_yy;
  double cov_zz;
};

static void usage(const char * prog) {
  std::cerr
    << "Usage:\n"
    << "  " << prog
    << " --duration <seconds>"
    << " [--sensors camera,lidar,gps]"
    << " [--camera-topic <topic>]"
    << " [--lidar-topic <topic>]"
    << " [--gps-topic <topic>]"
    << " [--config-path <yaml>]\n\n"
    << "Example:\n"
    << "  " << prog
    << " --duration 120 --sensors camera,lidar,gps"
    << " --camera-topic /my_camera/pylon_ros2_camera_node/image_raw"
    << " --gps-topic /novatel/oem7/gps"
    << " --lidar-topic /sensing/lidar/top/pointcloud\n";
}

static std::string trim(const std::string & in) {
  std::size_t b = 0;
  while (b < in.size() && std::isspace(static_cast<unsigned char>(in[b]))) {
    ++b;
  }
  std::size_t e = in.size();
  while (e > b && std::isspace(static_cast<unsigned char>(in[e - 1]))) {
    --e;
  }
  return in.substr(b, e - b);
}

static std::vector<std::string> split_csv(const std::string & s) {
  std::vector<std::string> out;
  std::string cur;
  for (char ch : s) {
    if (ch == ',') {
      const std::string token = trim(cur);
      if (!token.empty()) out.push_back(token);
      cur.clear();
    } else {
      cur.push_back(ch);
    }
  }
  const std::string token = trim(cur);
  if (!token.empty()) out.push_back(token);
  return out;
}

static double mean(const std::vector<double> & v) {
  if (v.empty()) return 0.0;
  double s = 0.0;
  for (double x : v) s += x;
  return s / static_cast<double>(v.size());
}

static double stdev_sample(const std::vector<double> & v) {
  if (v.size() < 2) return 0.0;
  const double m = mean(v);
  double s2 = 0.0;
  for (double x : v) {
    const double d = x - m;
    s2 += d * d;
  }
  return std::sqrt(s2 / static_cast<double>(v.size() - 1));
}

static double t95(int df) {
  static const double tab[31] = {
    0.0,
    12.706, 4.303, 3.182, 2.776, 2.571,
    2.447, 2.365, 2.306, 2.262, 2.228,
    2.201, 2.179, 2.160, 2.145, 2.131,
    2.120, 2.110, 2.101, 2.093, 2.086,
    2.080, 2.074, 2.069, 2.064, 2.060,
    2.056, 2.052, 2.048, 2.045, 2.042
  };
  if (df <= 0) return 0.0;
  if (df < 31) return tab[df];
  return 1.96;
}

static double ci_halfwidth95(const std::vector<double> & v) {
  const int n = static_cast<int>(v.size());
  if (n < 2) return 0.0;
  return t95(n - 1) * stdev_sample(v) / std::sqrt(static_cast<double>(n));
}

static bool load_topics_from_config(
  const std::string & config_path,
  TopicDefaults * out,
  std::string * err)
{
  try {
    const YAML::Node root = YAML::LoadFile(config_path);
    const YAML::Node common = root["common"];
    if (!common) {
      *err = "missing 'common' section in config";
      return false;
    }

    if (common["img_topic"]) out->camera_topic = common["img_topic"].as<std::string>();
    if (common["lidar_topic"]) out->lidar_topic = common["lidar_topic"].as<std::string>();
    if (common["gps_topic"]) out->gps_topic = common["gps_topic"].as<std::string>();
    return true;
  } catch (const std::exception & e) {
    *err = e.what();
    return false;
  }
}

template<typename MsgT>
static std::size_t serialized_size(const MsgT & msg, rclcpp::Serialization<MsgT> * serializer) {
  rclcpp::SerializedMessage serialized_msg;
  serializer->serialize_message(&msg, &serialized_msg);
  return serialized_msg.size();
}

static std::size_t reduced_gps_payload_size(const gps_msgs::msg::GPSFix & msg) {
  GpsPayload gp{};
  gp.latitude = msg.latitude;
  gp.longitude = msg.longitude;
  gp.altitude = msg.altitude;
  gp.cov_xx = msg.position_covariance[0];
  gp.cov_yy = msg.position_covariance[4];
  gp.cov_zz = msg.position_covariance[8];

  std::vector<std::uint8_t> payload(sizeof(GpsPayload));
  std::memcpy(payload.data(), &gp, sizeof(GpsPayload));
  return payload.size();
}

class ReductionReportNode : public rclcpp::Node {
public:
  ReductionReportNode(
    const std::string & config_path,
    bool use_camera,
    bool use_lidar,
    bool use_gps,
    const std::string & camera_topic,
    const std::string & gps_topic,
    const std::string & lidar_topic)
  : Node("reduction_report_node")
  {
    if (use_camera) {
      // Use an existing directory so processing stays in-memory.
      deduplicator_ = std::make_shared<avs::ImgDeduplicator>("/tmp", config_path);
      camera_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        camera_topic,
        rclcpp::SensorDataQoS(),
        std::bind(&ReductionReportNode::on_camera, this, std::placeholders::_1));
      RCLCPP_INFO(get_logger(), "Subscribed camera topic: %s", camera_topic.c_str());
    }

    if (use_lidar) {
      downsampler_ = std::make_shared<avs::LidarDownsampler>(config_path);
      // Use an existing directory so processing stays in-memory.
      compressor_ = std::make_shared<avs::LidarCompressor>("/tmp");
      lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        lidar_topic,
        rclcpp::SensorDataQoS(),
        std::bind(&ReductionReportNode::on_lidar, this, std::placeholders::_1));
      RCLCPP_INFO(get_logger(), "Subscribed lidar topic: %s", lidar_topic.c_str());
    }

    if (use_gps) {
      gps_sub_ = this->create_subscription<gps_msgs::msg::GPSFix>(
        gps_topic,
        rclcpp::SensorDataQoS(),
        std::bind(&ReductionReportNode::on_gps, this, std::placeholders::_1));
      RCLCPP_INFO(get_logger(), "Subscribed GPS topic: %s", gps_topic.c_str());
    }
  }

  SensorStats camera_stats() const {
    std::lock_guard<std::mutex> lk(mu_);
    return camera_stats_;
  }

  SensorStats lidar_stats() const {
    std::lock_guard<std::mutex> lk(mu_);
    return lidar_stats_;
  }

  SensorStats gps_stats() const {
    std::lock_guard<std::mutex> lk(mu_);
    return gps_stats_;
  }

private:
  void on_camera(const sensor_msgs::msg::Image::SharedPtr msg) {
    const std::size_t raw = serialized_size(*msg, &image_serializer_);

    std::size_t reduced = 0;
    try {
      std::vector<std::uint8_t> payload;
      const bool unique = deduplicator_->isUniqueAndGetBytes(*msg, payload);
      if (unique) reduced = payload.size();
    } catch (const std::exception & e) {
      std::lock_guard<std::mutex> lk(mu_);
      camera_stats_.msg_count += 1;
      camera_stats_.raw_bytes += raw;
      camera_stats_.errors += 1;
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "Camera processing failed: %s", e.what());
      return;
    }

    std::lock_guard<std::mutex> lk(mu_);
    camera_stats_.msg_count += 1;
    camera_stats_.raw_bytes += raw;
    camera_stats_.reduced_bytes += reduced;
  }

  void on_lidar(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    const std::size_t raw = serialized_size(*msg, &pc2_serializer_);

    std::size_t reduced = 0;
    try {
      auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
      pcl::fromROSMsg(*msg, *cloud);
      auto downsampled = downsampler_->downsample(cloud);
      std::vector<std::uint8_t> payload;
      compressor_->getLAZ(downsampled, payload);
      reduced = payload.size();
    } catch (const std::exception & e) {
      std::lock_guard<std::mutex> lk(mu_);
      lidar_stats_.msg_count += 1;
      lidar_stats_.raw_bytes += raw;
      lidar_stats_.errors += 1;
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "Lidar processing failed: %s", e.what());
      return;
    }

    std::lock_guard<std::mutex> lk(mu_);
    lidar_stats_.msg_count += 1;
    lidar_stats_.raw_bytes += raw;
    lidar_stats_.reduced_bytes += reduced;
  }

  void on_gps(const gps_msgs::msg::GPSFix::SharedPtr msg) {
    const std::size_t raw = serialized_size(*msg, &gps_serializer_);

    std::size_t reduced = 0;
    try {
      reduced = reduced_gps_payload_size(*msg);
    } catch (const std::exception & e) {
      std::lock_guard<std::mutex> lk(mu_);
      gps_stats_.msg_count += 1;
      gps_stats_.raw_bytes += raw;
      gps_stats_.errors += 1;
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "GPS processing failed: %s", e.what());
      return;
    }

    std::lock_guard<std::mutex> lk(mu_);
    gps_stats_.msg_count += 1;
    gps_stats_.raw_bytes += raw;
    gps_stats_.reduced_bytes += reduced;
  }

  mutable std::mutex mu_;
  SensorStats camera_stats_;
  SensorStats lidar_stats_;
  SensorStats gps_stats_;

  std::shared_ptr<avs::ImgDeduplicator> deduplicator_;
  std::shared_ptr<avs::LidarDownsampler> downsampler_;
  std::shared_ptr<avs::LidarCompressor> compressor_;

  rclcpp::Serialization<sensor_msgs::msg::Image> image_serializer_;
  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> pc2_serializer_;
  rclcpp::Serialization<gps_msgs::msg::GPSFix> gps_serializer_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Subscription<gps_msgs::msg::GPSFix>::SharedPtr gps_sub_;
};

static std::string ratio_string(std::uint64_t raw, std::uint64_t reduced) {
  if (raw == 0 && reduced == 0) return "n/a";
  if (reduced == 0) return "inf";
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(2) << (static_cast<double>(raw) / static_cast<double>(reduced));
  return oss.str();
}

static std::string ratio_ci_string(std::uint64_t raw, std::uint64_t reduced, double ci95) {
  if (raw == 0 && reduced == 0) return "n/a";
  if (reduced == 0) return "n/a";
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(2) << ci95;
  return oss.str();
}

static void add_interval_sample(
  const SensorStats & prev,
  const SensorStats & cur,
  double dt_s,
  SensorSamples * out)
{
  if (dt_s <= 0.0) return;

  const std::uint64_t d_raw = cur.raw_bytes - prev.raw_bytes;
  const std::uint64_t d_reduced = cur.reduced_bytes - prev.reduced_bytes;

  const double raw_mb_s = static_cast<double>(d_raw) / 1e6 / dt_s;
  const double reduced_mb_s = static_cast<double>(d_reduced) / 1e6 / dt_s;
  out->raw_mb_s.push_back(raw_mb_s);
  out->reduced_mb_s.push_back(reduced_mb_s);

  if (d_reduced > 0) {
    out->ratio_raw_over_reduced.push_back(static_cast<double>(d_raw) / static_cast<double>(d_reduced));
  }
}

static void print_row(const std::string & name, const SensorStats & s, const SensorSamples & samples, double duration_s) {
  const double raw_mb_s = static_cast<double>(s.raw_bytes) / 1e6 / duration_s;
  const double reduced_mb_s = static_cast<double>(s.reduced_bytes) / 1e6 / duration_s;
  const double raw_ci95 = ci_halfwidth95(samples.raw_mb_s);
  const double reduced_ci95 = ci_halfwidth95(samples.reduced_mb_s);
  const double ratio_ci95 = ci_halfwidth95(samples.ratio_raw_over_reduced);
  std::cout << std::left << std::setw(10) << name
            << std::right << std::setw(14) << std::fixed << std::setprecision(3) << raw_mb_s
            << std::setw(14) << std::fixed << std::setprecision(3) << raw_ci95
            << std::setw(18) << std::fixed << std::setprecision(3) << reduced_mb_s
            << std::setw(16) << std::fixed << std::setprecision(3) << reduced_ci95
            << std::setw(22) << ratio_string(s.raw_bytes, s.reduced_bytes)
            << std::setw(14) << ratio_ci_string(s.raw_bytes, s.reduced_bytes, ratio_ci95)
            << "\n";
}

}  // namespace

int main(int argc, char ** argv) {
  const std::string default_config = "/home/avs/AVS-PI/src/avs/config/avs_config.yaml";

  rclcpp::InitOptions init_options;
  auto non_ros_args = rclcpp::init_and_remove_ros_arguments(argc, argv, init_options);

  double duration_s = 0.0;
  std::string sensors_csv = "camera,lidar";
  std::string camera_topic;
  std::string lidar_topic;
  std::string gps_topic;
  std::string config_path = default_config;

  for (std::size_t i = 1; i < non_ros_args.size(); ++i) {
    const std::string & a = non_ros_args[i];
    if (a == "--duration" && i + 1 < non_ros_args.size()) {
      duration_s = std::stod(non_ros_args[++i]);
      continue;
    }
    if (a == "--sensors" && i + 1 < non_ros_args.size()) {
      sensors_csv = non_ros_args[++i];
      continue;
    }
    if (a == "--camera-topic" && i + 1 < non_ros_args.size()) {
      camera_topic = non_ros_args[++i];
      continue;
    }
    if (a == "--lidar-topic" && i + 1 < non_ros_args.size()) {
      lidar_topic = non_ros_args[++i];
      continue;
    }
    if (a == "--gps-topic" && i + 1 < non_ros_args.size()) {
      gps_topic = non_ros_args[++i];
      continue;
    }
    if (a == "--config-path" && i + 1 < non_ros_args.size()) {
      config_path = non_ros_args[++i];
      continue;
    }
    if (a == "--help" || a == "-h") {
      usage(non_ros_args[0].c_str());
      rclcpp::shutdown();
      return 0;
    }
    std::cerr << "Unknown or incomplete arg: " << a << "\n";
    usage(non_ros_args[0].c_str());
    rclcpp::shutdown();
    return 2;
  }

  if (duration_s <= 0.0) {
    std::cerr << "--duration must be > 0\n";
    usage(non_ros_args[0].c_str());
    rclcpp::shutdown();
    return 2;
  }

  bool use_camera = false;
  bool use_lidar = false;
  bool use_gps = false;
  const auto sensors = split_csv(sensors_csv);
  for (const auto & s : sensors) {
    if (s == "camera") use_camera = true;
    else if (s == "lidar") use_lidar = true;
    else if (s == "gps") use_gps = true;
    else {
      std::cerr << "Unsupported sensor in --sensors: " << s << " (supported: camera,lidar,gps)\n";
      rclcpp::shutdown();
      return 2;
    }
  }
  if (!use_camera && !use_lidar && !use_gps) {
    std::cerr << "At least one sensor must be selected in --sensors\n";
    rclcpp::shutdown();
    return 2;
  }

  TopicDefaults defaults;
  std::string load_err;
  if (!load_topics_from_config(config_path, &defaults, &load_err)) {
    std::cerr << "Failed to load config '" << config_path << "': " << load_err << "\n";
    rclcpp::shutdown();
    return 2;
  }
  if (camera_topic.empty()) camera_topic = defaults.camera_topic;
  if (lidar_topic.empty()) lidar_topic = defaults.lidar_topic;
  if (gps_topic.empty()) gps_topic = defaults.gps_topic;

  if (use_camera && camera_topic.empty()) {
    std::cerr << "Camera topic is empty. Set --camera-topic or common.img_topic in config.\n";
    rclcpp::shutdown();
    return 2;
  }
  if (use_lidar && lidar_topic.empty()) {
    std::cerr << "Lidar topic is empty. Set --lidar-topic or common.lidar_topic in config.\n";
    rclcpp::shutdown();
    return 2;
  }
  if (use_gps && gps_topic.empty()) {
    std::cerr << "GPS topic is empty. Set --gps-topic or common.gps_topic in config.\n";
    rclcpp::shutdown();
    return 2;
  }

  auto node = std::make_shared<ReductionReportNode>(
    config_path, use_camera, use_lidar, use_gps, camera_topic, gps_topic, lidar_topic);

  using clock = std::chrono::steady_clock;
  const auto start = clock::now();
  const auto stop = start + std::chrono::duration<double>(duration_s);
  const auto sample_period = std::chrono::seconds(1);
  auto prev_sample_t = start;
  auto next_sample_t = start + sample_period;

  SensorSamples camera_samples;
  SensorSamples lidar_samples;
  SensorSamples gps_samples;
  SensorStats prev_camera_stats;
  SensorStats prev_lidar_stats;
  SensorStats prev_gps_stats;
  if (use_camera) prev_camera_stats = node->camera_stats();
  if (use_lidar) prev_lidar_stats = node->lidar_stats();
  if (use_gps) prev_gps_stats = node->gps_stats();

  while (rclcpp::ok() && clock::now() < stop) {
    rclcpp::spin_some(node);

    const auto now = clock::now();
    if (now >= next_sample_t) {
      const double dt_s = std::chrono::duration<double>(now - prev_sample_t).count();
      if (use_camera) {
        const SensorStats cur = node->camera_stats();
        add_interval_sample(prev_camera_stats, cur, dt_s, &camera_samples);
        prev_camera_stats = cur;
      }
      if (use_lidar) {
        const SensorStats cur = node->lidar_stats();
        add_interval_sample(prev_lidar_stats, cur, dt_s, &lidar_samples);
        prev_lidar_stats = cur;
      }
      if (use_gps) {
        const SensorStats cur = node->gps_stats();
        add_interval_sample(prev_gps_stats, cur, dt_s, &gps_samples);
        prev_gps_stats = cur;
      }
      prev_sample_t = now;
      next_sample_t = now + sample_period;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  rclcpp::spin_some(node);
  const auto end = clock::now();
  const double tail_dt_s = std::chrono::duration<double>(end - prev_sample_t).count();
  if (tail_dt_s > 0.0) {
    if (use_camera) {
      const SensorStats cur = node->camera_stats();
      add_interval_sample(prev_camera_stats, cur, tail_dt_s, &camera_samples);
    }
    if (use_lidar) {
      const SensorStats cur = node->lidar_stats();
      add_interval_sample(prev_lidar_stats, cur, tail_dt_s, &lidar_samples);
    }
    if (use_gps) {
      const SensorStats cur = node->gps_stats();
      add_interval_sample(prev_gps_stats, cur, tail_dt_s, &gps_samples);
    }
  }

  std::cout << "\nSensor, Raw MB per s, Reduced MB per s, Ratio (Raw/Reduced)\n";
  std::cout << std::left << std::setw(10) << "Sensor"
            << std::right << std::setw(14) << "Raw MB per s"
            << std::setw(14) << "Raw CI95"
            << std::setw(18) << "Reduced MB per s"
            << std::setw(16) << "Reduced CI95"
            << std::setw(22) << "Ratio Raw/Reduced"
            << std::setw(14) << "Ratio CI95"
            << "\n";

  if (use_camera) print_row("camera", node->camera_stats(), camera_samples, duration_s);
  if (use_lidar) print_row("lidar", node->lidar_stats(), lidar_samples, duration_s);
  if (use_gps) print_row("gps", node->gps_stats(), gps_samples, duration_s);

  rclcpp::shutdown();
  return 0;
}
