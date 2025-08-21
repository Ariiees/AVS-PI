#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
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
    db_dir_ = common["db_dir"].as<std::string>();
    lidar_ext_ = dedup["lidar_format"].as<std::string>();

    lidar_path_ = (fs::path(lidar_ssd_dir_) / avs::getCurrentDayFolder()).string();

    downsampler_ = std::make_shared<LidarDownsampler>(config_path);
    compressor_ = std::make_shared<LidarCompressor>(lidar_path_);

    fs::create_directories(db_dir_);
    const std::string db_path = (fs::path(db_dir_) / "avs_lidar.sqlite3").string();
    std::string err;
    if (!db_.open(db_path, &err)) {
      throw std::runtime_error("Failed to open DB: " + err);
    }
    
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      lidar_topic_, rclcpp::SensorDataQoS(), std::bind(&LidarProcessNode::lidarCallback, this, _1));
    
    RCLCPP_INFO(this->get_logger(),
                "AVS Lidar node started. Subscribed to %s; saving to %s; DB at %s",
                lidar_topic_.c_str(), lidar_path_.c_str(), db_path.c_str());
  }

private:
  void lidarCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    auto t0 = std::chrono::steady_clock::now();
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *pcl_cloud);

    pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled = downsampler_->downsample(pcl_cloud);

    auto [filepath, ts_ms] = avs::getTimestampAndFilename(this->lidar_path_, lidar_ext_);

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

    RCLCPP_INFO(this->get_logger(), "Latency: %ld µs", latency_us);
  }


private:
  // Config/state
  std::string lidar_topic_;
  std::string lidar_ssd_dir_;
  std::string db_dir_;
  std::string lidar_ext_;

  std::string lidar_path_;

  std::shared_ptr<LidarDownsampler> downsampler_;
  std::shared_ptr<LidarCompressor> compressor_;
  AvsDb db_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

} // namespace avs

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<avs::LidarProcessNode>());
  rclcpp::shutdown();
  return 0;
}
