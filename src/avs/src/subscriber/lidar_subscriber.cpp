#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/int64.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include "avs/lidar_downsample.h"
#include "avs/lidar_compress.h"
#include "avs/common.h"
#include "avs/db_operation.h"
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <string>
#include <memory>
#include <chrono>

using std::placeholders::_1;
namespace fs = std::filesystem;

namespace avs {

class LidarProcessNode : public rclcpp::Node
{
public:
  LidarProcessNode()
  : Node("lidar_process_node")
  {
    this->declare_parameter<std::string>("config_path", "/home/avs/AVS-PI/src/avs/config/avs_config.yaml");
     std::string config_path = this->get_parameter("config_path").as_string();

    YAML::Node root = YAML::LoadFile(config_path);
    YAML::Node common = root["common"];
    YAML::Node dedup = root["lidar_compress"];

    lidar_topic_ = common["lidar_topic"].as<std::string>();
    lidar_ssd_dir_ = common["lidar_ssd_dir"].as<std::string>();
    db_dir_ = common["hot_db_dir"].as<std::string>();
    lidar_ext_ = dedup["lidar_format"].as<std::string>();

    lidar_path_ = (fs::path(lidar_ssd_dir_) / avs::getCurrentDayFolder()).string();
    if (!avs::ensureDirectory(lidar_path_, &ec)) {
      RCLCPP_WARN(this->get_logger(),
                  "Lidar directory %s did not exist. Attempted to create it (ec=%d: %s).",
                  lidar_path_.c_str(),
                  ec.value(),
                  ec.message().c_str());
    }
    
    downsampler_ = std::make_shared<LidarDownsampler>(config_path);
    compressor_ = std::make_shared<LidarCompressor>(lidar_path_);

    fs::create_directories(db_dir_);
    const std::string db_path = (fs::path(db_dir_) / "avs_lidar.sqlite3").string();
   
    std::string err;
    if (!db_.open(db_path, &err)) {
      throw std::runtime_error("Failed to open DB: " + err);
    }
    
    // subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    //   lidar_topic_, rclcpp::SensorDataQoS(), std::bind(&LidarProcessNode::lidarCallback, this, _1));
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      lidar_topic_, make_auto_qos(lidar_topic_),
      std::bind(&LidarProcessNode::lidarCallback, this, _1));
    
    latency_pub_ = this->create_publisher<std_msgs::msg::Int64>("/avs/record_latency_us", 10);
    
    RCLCPP_INFO(this->get_logger(),
                "AVS Lidar node started. Subscribed to %s; saving to %s; DB at %s",
                lidar_topic_.c_str(), lidar_path_.c_str(), db_path.c_str());
  }

private:
  void lidarCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    auto t0 = std::chrono::steady_clock::now();

    long long ts_ms = msg->header.stamp.sec * 1000LL + msg->header.stamp.nanosec / 1000000LL;
    std::string filepath = lidar_path_ + '/' + std::to_string(ts_ms) + "." + lidar_ext_;

    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *pcl_cloud);

    pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled = downsampler_->downsample(pcl_cloud);

    try {
      compressor_->saveAsLAZ(downsampled, filepath);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Downsampe/compress/store failed: %s", e.what());
      return;
    }

    AvsRow row;
    row.sensor_id = lidar_topic_;
    row.data_type = lidar_ext_;  
    row.ts_ms     = ts_ms;
    row.path      = filepath;

    std::string err;
    if (!db_.insertRow(row, &err)) {
      RCLCPP_ERROR(this->get_logger(), "DB insert failed: %s", err.c_str());
    } else {
      RCLCPP_DEBUG(this->get_logger(), "DB insert OK: %s", filepath.c_str());
    }


    auto t1 = std::chrono::steady_clock::now();
    auto latency_us = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();

    std_msgs::msg::Int64 m;
    m.data = latency_us;
    latency_pub_->publish(m);
    // RCLCPP_INFO(this->get_logger(), "Latency: %ld µs", latency_us);
  }

  rclcpp::QoS make_auto_qos(const std::string& topic)
  {
    using rclcpp::ReliabilityPolicy;

    // Fallback: KEEP_LAST(10), RELIABLE, VOLATILE
    rclcpp::QoS qos(10);
    qos.reliability(ReliabilityPolicy::Reliable);
    qos.durability(rclcpp::DurabilityPolicy::Volatile);

    for (int i = 0; i < 20; ++i) {
      auto infos = this->get_publishers_info_by_topic(topic);
      if (!infos.empty()) {
        auto offered = infos.front().qos_profile();

        qos.reliability(offered.reliability());

        RCLCPP_INFO(
          this->get_logger(),
          "QoS for %s -> depth=10 reliability=%d durability=%d",
          topic.c_str(),
          static_cast<int>(qos.get_rmw_qos_profile().reliability),
          static_cast<int>(qos.get_rmw_qos_profile().durability));

        return qos;
      }
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    RCLCPP_WARN(this->get_logger(),
                "No publisher QoS detected on %s; using fallback QoS(KEEP_LAST depth=10, RELIABLE, VOLATILE)",
                topic.c_str());
    return qos;
  }



private:
  // Config/state
  std::string lidar_topic_;
  std::string lidar_ssd_dir_;
  std::string db_dir_;
  std::string lidar_ext_;

  std::string lidar_path_;

  std::error_code ec;

  std::shared_ptr<LidarDownsampler> downsampler_;
  std::shared_ptr<LidarCompressor> compressor_;
  AvsDb db_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr latency_pub_;
};

} // namespace avs

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<avs::LidarProcessNode>());
  rclcpp::shutdown();
  return 0;
}
