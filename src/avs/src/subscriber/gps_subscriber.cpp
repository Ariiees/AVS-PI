#include "rclcpp/rclcpp.hpp"
#include "gps_msgs/msg/gps_fix.hpp"
#include "std_msgs/msg/int64.hpp"
#include "yaml-cpp/yaml.h"

#include "avs/common.h"
#include "avs/append_logger.h"
#include "avs/topic_map.h"
#include "avs/trip_manager.h"

#include <filesystem>
#include <chrono>
#include <string>
#include <vector>
#include <cstring>

namespace fs = std::filesystem;
using std::placeholders::_1;

namespace avs
{ 

class GpsProcessNode : public rclcpp::Node {
public:
  GpsProcessNode()
  : Node("gps_process_node")
  {
    this->declare_parameter<std::string>("config_path", "/home/avs/AVS-PI/src/avs/config/avs_config.yaml");
    this->declare_parameter<std::string>("topic_map_path", "/home/avs/AVS-PI/src/avs/config/topics.yaml");

    std::string config_path = this->get_parameter("config_path").as_string();
    std::string topic_map_path = this->get_parameter("topic_map_path").as_string();

    YAML::Node root   = YAML::LoadFile(config_path);
    YAML::Node common = root["common"];
    YAML::Node gps_downscale = root["gps_downscale"];

    gps_topic_   = common["gps_topic"].as<std::string>();
    std::string ssd_root = common["ssd_root"].as<std::string>();
    downscale_ = gps_downscale["open"].as<bool>(false);
    downscale_frequency_ = gps_downscale["frequency"].as<int>(50);

    std::string folder_name =
      avs::GetTopicFolder(avs::LoadTopicMap(topic_map_path), gps_topic_);
    std::string current_day = avs::getCurrentDayFolder();

    
    gps_path_ = (fs::path(ssd_root) / fs::path(folder_name) / fs::path(current_day)).string();

    if (!avs::ensureDirectory(gps_path_, &ec)) {
      RCLCPP_WARN(
        this->get_logger(),
        "GPS directory %s did not exist. Attempted to create it (ec=%d: %s).",
        gps_path_.c_str(),
        ec.value(),
        ec.message().c_str());
    }

    trip_mgr_     = std::make_shared<TripManager>();
    append_logger_ = std::make_shared<avs::AppendLogger>(ssd_root, gps_topic_);

    int trip_id = trip_mgr_->GetTripId(gps_path_);

    const uint64_t trip_start_ns = static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count());

    append_logger_->startTrip(current_day, folder_name, trip_id, trip_start_ns);


    subscription_ = this->create_subscription<gps_msgs::msg::GPSFix>(
      gps_topic_, rclcpp::SensorDataQoS(),
      std::bind(&GpsProcessNode::gpsCallback, this, std::placeholders::_1));

    // latency_pub_ = this->create_publisher<std_msgs::msg::Int64>("/avs/gps_latency_us", 10);

    RCLCPP_INFO(this->get_logger(),
                "AVS GPS node started. Subscribed to %s; writing to append-logger (path=%s trip=%s)",
                gps_topic_.c_str(), gps_path_.c_str(), std::to_string(trip_id).c_str());
  }

  ~GpsProcessNode() override {
    subscription_.reset(); 
    uint64_t trip_end_ns = static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count());
    try {
      if (append_logger_) append_logger_->endTrip(trip_end_ns);
    } catch (...) {}
  }

private:

  void gpsCallback(const gps_msgs::msg::GPSFix::SharedPtr msg)
  {
    // const uint64_t ts_ns =
    //   static_cast<uint64_t>(msg->header.stamp.sec) * 1000000000ULL
    //   + static_cast<uint64_t>(msg->header.stamp.nanosec);
    if (downscale_) {
      records_count_++;
      if (records_count_ % downscale_frequency_ != 0) {
        return;
      }
    }
    
    records_count_ = 0;

    // const auto t0 = std::chrono::steady_clock::now();

    uint64_t ts_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                       std::chrono::system_clock::now().time_since_epoch()
                     ).count();

    GpsPayload gp{};
    gp.latitude  = msg->latitude;
    gp.longitude = msg->longitude;
    gp.altitude  = msg->altitude;

    // position_covariance is a 9 element array (row major)
    gp.cov_xx = msg->position_covariance[0];  // xx
    gp.cov_yy = msg->position_covariance[4];  // yy
    gp.cov_zz = msg->position_covariance[8];  // zz

    std::vector<uint8_t> payload(sizeof(GpsPayload));
    std::memcpy(payload.data(), &gp, sizeof(GpsPayload));

    try {
      append_logger_->appendRecord(ts_ns, payload);
      // const auto t1 = std::chrono::steady_clock::now();
      // const auto latency_us = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();
      // std_msgs::msg::Int64 m;
      // m.data = latency_us;
      // latency_pub_->publish(m);
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "appendRecord failed: %s", e.what());
    }
  }

private:
  struct GpsPayload {
    double latitude;
    double longitude;
    double altitude;
    double cov_xx;
    double cov_yy;
    double cov_zz;
  };

  std::string gps_topic_;
  std::string gps_path_;
  bool downscale_;
  int downscale_frequency_; // hz
  int records_count_ = 0;

  std::error_code ec;

  std::shared_ptr<avs::AppendLogger> append_logger_;
  std::shared_ptr<avs::TripManager> trip_mgr_;

  rclcpp::Subscription<gps_msgs::msg::GPSFix>::SharedPtr subscription_;
  // rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr latency_pub_;
};

};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<avs::GpsProcessNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
