#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/int64.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include "avs/lidar_downsample.h"
#include "avs/lidar_compress.h"
#include "avs/topic_map.h"
#include "avs/trip_manager.h"
#include "avs/storage_logger.h"
#include "avs/common.h"
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
    this->declare_parameter<std::string>("topic_map_path", "/home/avs/AVS-PI/src/avs/config/topics.yaml");
    this->declare_parameter<std::string>("storage_backend", "append");
    
    std::string config_path = this->get_parameter("config_path").as_string();
    std::string topic_map_path = this->get_parameter("topic_map_path").as_string();
    storage_backend_ = avs::NormalizeStorageBackend(
      this->get_parameter("storage_backend").as_string());

    YAML::Node root = YAML::LoadFile(config_path);
    YAML::Node common = root["common"];

    lidar_topic_ = common["lidar_topic"].as<std::string>();
    std::string ssd_root = common["ssd_root"].as<std::string>();

    std::string folder_name = avs::GetTopicFolder(avs::LoadTopicMap(topic_map_path), lidar_topic_);
    std::string current_day = avs::getCurrentDayFolder();

    lidar_path_ = avs::StorageTopicDayDir(
      storage_backend_, fs::path(ssd_root), folder_name, current_day).string();

    if (!avs::ensureDirectory(lidar_path_, &ec)) {
      RCLCPP_WARN(this->get_logger(),
                  "Lidar directory %s did not exist. Attempted to create it (ec=%d: %s).",
                  lidar_path_.c_str(),
                  ec.value(),
                  ec.message().c_str());
    }
    
    downsampler_ = std::make_shared<LidarDownsampler>(config_path);
    compressor_ = std::make_shared<LidarCompressor>(lidar_path_);
    trip_mgr_     = std::make_shared<TripManager>();
    storage_logger_ = avs::CreateStorageLogger(storage_backend_, ssd_root, lidar_topic_);

    int trip_id = trip_mgr_->GetTripId(lidar_path_);
    const uint64_t trip_strat_ns = static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count());
    
    storage_logger_->startTrip(current_day, folder_name, trip_id, trip_strat_ns);

    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      lidar_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LidarProcessNode::lidarCallback, this, _1));
    
    // latency_pub_ = this->create_publisher<std_msgs::msg::Int64>("/avs/lidar_latency_us", 10);
    
    RCLCPP_INFO(this->get_logger(),
                "AVS Lidar node started. backend=%s topic=%s path=%s trip=%s",
                storage_backend_.c_str(), lidar_topic_.c_str(), lidar_path_.c_str(),
                std::to_string(trip_id).c_str());
  }

  ~LidarProcessNode() override
  {
    subscription_.reset(); 
    uint64_t trip_end_ns = static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count());
    try {
      if (storage_logger_) storage_logger_->endTrip(trip_end_ns);
    } catch (...) {}
  }

private:
  void lidarCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    // auto t0 = std::chrono::steady_clock::now();

    // long long ts_ms = msg->header.stamp.sec * 1000LL + msg->header.stamp.nanosec / 1000000LL;
    // tamperaily set it to current system time
    uint64_t ts_ns = static_cast<uint64_t>(
       std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count());

    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *pcl_cloud);

    pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled = downsampler_->downsample(pcl_cloud);

    std::vector<uint8_t> playload;
    try {
      compressor_->getLAZ(downsampled, playload);
      storage_logger_->appendRecord(ts_ns, playload);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Downsampe/compress/store failed: %s", e.what());
      return;
    }

    // auto t1 = std::chrono::steady_clock::now();
    // auto latency_us = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();

    // std_msgs::msg::Int64 m;
    // m.data = latency_us;
    // latency_pub_->publish(m);
    // RCLCPP_INFO(this->get_logger(), "Latency: %ld µs", latency_us);
  }


private:
  // Config/state
  std::string lidar_topic_;
  std::string lidar_path_;
  std::string storage_backend_;

  std::error_code ec;

  std::shared_ptr<LidarDownsampler> downsampler_;
  std::shared_ptr<LidarCompressor> compressor_;
  std::shared_ptr<TripManager> trip_mgr_;
  std::shared_ptr<avs::StorageLogger> storage_logger_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  // rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr latency_pub_;
};

} // namespace avs

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<avs::LidarProcessNode>());
  rclcpp::shutdown();
  return 0;
}
