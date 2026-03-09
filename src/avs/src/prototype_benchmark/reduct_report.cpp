#include <chrono>
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

struct TopicDefaults {
  std::string camera_topic;
  std::string lidar_topic;
};

static void usage(const char * prog) {
  std::cerr
    << "Usage:\n"
    << "  " << prog
    << " --duration <seconds>"
    << " [--sensors camera,lidar]"
    << " [--camera-topic <topic>]"
    << " [--lidar-topic <topic>]"
    << " [--config-path <yaml>]\n\n"
    << "Example:\n"
    << "  " << prog
    << " --duration 30 --sensors camera,lidar"
    << " --camera-topic /my_camera/pylon_ros2_camera_node/image_raw"
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

class ReductionReportNode : public rclcpp::Node {
public:
  ReductionReportNode(
    const std::string & config_path,
    bool use_camera,
    bool use_lidar,
    const std::string & camera_topic,
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
  }

  SensorStats camera_stats() const {
    std::lock_guard<std::mutex> lk(mu_);
    return camera_stats_;
  }

  SensorStats lidar_stats() const {
    std::lock_guard<std::mutex> lk(mu_);
    return lidar_stats_;
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

  mutable std::mutex mu_;
  SensorStats camera_stats_;
  SensorStats lidar_stats_;

  std::shared_ptr<avs::ImgDeduplicator> deduplicator_;
  std::shared_ptr<avs::LidarDownsampler> downsampler_;
  std::shared_ptr<avs::LidarCompressor> compressor_;

  rclcpp::Serialization<sensor_msgs::msg::Image> image_serializer_;
  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> pc2_serializer_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
};

static std::string ratio_string(std::uint64_t raw, std::uint64_t reduced) {
  if (raw == 0 && reduced == 0) return "n/a";
  if (reduced == 0) return "inf";
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(2) << (static_cast<double>(raw) / static_cast<double>(reduced));
  return oss.str();
}

static void print_row(const std::string & name, const SensorStats & s, double duration_s) {
  const double raw_mb_s = static_cast<double>(s.raw_bytes) / 1e6 / duration_s;
  const double reduced_mb_s = static_cast<double>(s.reduced_bytes) / 1e6 / duration_s;
  std::cout << std::left << std::setw(10) << name
            << std::right << std::setw(14) << std::fixed << std::setprecision(3) << raw_mb_s
            << std::setw(18) << std::fixed << std::setprecision(3) << reduced_mb_s
            << std::setw(22) << ratio_string(s.raw_bytes, s.reduced_bytes)
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
  const auto sensors = split_csv(sensors_csv);
  for (const auto & s : sensors) {
    if (s == "camera") use_camera = true;
    else if (s == "lidar") use_lidar = true;
    else {
      std::cerr << "Unsupported sensor in --sensors: " << s << " (supported: camera,lidar)\n";
      rclcpp::shutdown();
      return 2;
    }
  }
  if (!use_camera && !use_lidar) {
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

  auto node = std::make_shared<ReductionReportNode>(
    config_path, use_camera, use_lidar, camera_topic, lidar_topic);

  using clock = std::chrono::steady_clock;
  const auto start = clock::now();
  const auto stop = start + std::chrono::duration<double>(duration_s);

  while (rclcpp::ok() && clock::now() < stop) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  rclcpp::spin_some(node);

  std::cout << "\nSensor, Raw MB per s, Reduced MB per s, Ratio (Raw/Reduced)\n";
  std::cout << std::left << std::setw(10) << "Sensor"
            << std::right << std::setw(14) << "Raw MB per s"
            << std::setw(18) << "Reduced MB per s"
            << std::setw(22) << "Ratio Raw/Reduced"
            << "\n";

  if (use_camera) print_row("camera", node->camera_stats(), duration_s);
  if (use_lidar) print_row("lidar", node->lidar_stats(), duration_s);

  rclcpp::shutdown();
  return 0;
}
