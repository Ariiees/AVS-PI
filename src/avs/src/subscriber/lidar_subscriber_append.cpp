#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/int64.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include "avs/lidar_downsample.h"
#include "avs/lidar_compress.h"
#include "avs/topic_map.h"
#include "avs/trip_manager.h"
#include "avs/append_logger.h"
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
    this->declare_parameter<std::string>("topic_map_path", "/home/avs/AVS-PI/src/avs/config/topics.yaml");
    
    std::string config_path = this->get_parameter("config_path").as_string();
    std::string topic_map_path = this->get_parameter("topic_map_path").as_string();

    YAML::Node root = YAML::LoadFile(config_path);
    YAML::Node common = root["common"];

    lidar_topic_ = common["lidar_topic"].as<std::string>();
    std::string ssd_root = common["ssd_root"].as<std::string>();

    std::string folder_name = avs::GetTopicFolder(avs::LoadTopicMap(topic_map_path), lidar_topic_);
    std::string current_day = avs::getCurrentDayFolder();

    lidar_path_ = (fs::path(ssd_root) / fs::path(folder_name) / fs::path(current_day)).string();

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
    append_logger_ = std::make_shared<avs::AppendLogger>(ssd_root, lidar_topic_);

    int trip_id = trip_mgr_->GetTripId(lidar_path_);
    const uint64_t trip_strat_ns = static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count());
    
    append_logger_->startTrip(current_day, folder_name, trip_id, trip_strat_ns);

    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      lidar_topic_, make_auto_qos(lidar_topic_),
      std::bind(&LidarProcessNode::lidarCallback, this, _1));
    
    // latency_pub_ = this->create_publisher<std_msgs::msg::Int64>("/avs/record_latency_us", 10);
    
    RCLCPP_INFO(this->get_logger(),
                "AVS Lidar node started. Subscribed to %s; writing to append-logger (path=%s trip=%s)",
                lidar_topic_.c_str(), lidar_path_.c_str(), std::to_string(trip_id).c_str());
  }

  ~LidarProcessNode() override
  {
    subscription_.reset(); 
    uint64_t trip_end_ns = static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count());
    try {
      if (append_logger_) append_logger_->endTrip(trip_end_ns);
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
      append_logger_->appendRecord(ts_ns, playload);
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

  rclcpp::QoS make_auto_qos(const std::string& topic)
  {
    using rclcpp::ReliabilityPolicy;

    rclcpp::QoS qos = rclcpp::SensorDataQoS();  // BestEffort, small depth
    qos.durability(rclcpp::DurabilityPolicy::Volatile);

    for (int i = 0; i < 50; ++i) {
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
                "No publisher QoS detected on %s; using fallback QoS(KEEP_LAST depth=10, BestEffort, VOLATILE)",
                topic.c_str());
    return qos;
  }



private:
  // Config/state
  std::string lidar_topic_;
  std::string lidar_path_;

  std::error_code ec;

  std::shared_ptr<LidarDownsampler> downsampler_;
  std::shared_ptr<LidarCompressor> compressor_;
  std::shared_ptr<TripManager> trip_mgr_;
  std::shared_ptr<avs::AppendLogger> append_logger_;

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
