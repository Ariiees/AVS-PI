#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/int64.hpp"
#include "avs/img_dedup.h"
#include "avs/common.h"
#include "avs/append_logger.h"
#include "avs/trip_manager.h"
#include "avs/topic_map.h"
#include <cv_bridge/cv_bridge.hpp>
#include <chrono>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <string>
#include <memory>
#include <vector>
#include <opencv2/imgcodecs.hpp>

using std::placeholders::_1;
namespace fs = std::filesystem;

namespace avs
{

class ImgProcessNode : public rclcpp::Node
{
public:
  ImgProcessNode()
  : Node("img_process_node")
  {
    this->declare_parameter<std::string>("config_path", "/home/avs/AVS-PI/src/avs/config/avs_config.yaml");
    this->declare_parameter<std::string>("topic_map_path", "/home/avs/AVS-PI/src/avs/config/topics.yaml");

    std::string config_path    = this->get_parameter("config_path").as_string();
    std::string topic_map_path = this->get_parameter("topic_map_path").as_string();

    YAML::Node root   = YAML::LoadFile(config_path);
    YAML::Node common = root["common"];
    // YAML::Node dedup = root["image_dedup"]; // not used here

    image_topic_ = common["img_topic"].as<std::string>();
    std::string ssd_root = common["ssd_root"].as<std::string>();

    std::string folder_name =
      avs::GetTopicFolder(avs::LoadTopicMap(topic_map_path), image_topic_);
    std::string current_day = avs::getCurrentDayFolder();

    image_path_ = (fs::path(ssd_root) / fs::path(folder_name) / fs::path(current_day)).string();
    if (!avs::ensureDirectory(image_path_, &ec)) {
      RCLCPP_WARN(this->get_logger(),
                  "Image directory %s did not exist. Attempted to create it (ec=%d: %s).",
                  image_path_.c_str(),
                  ec.value(),
                  ec.message().c_str());
    }

    deduplicator_ = std::make_shared<ImgDeduplicator>(image_path_, config_path);
    trip_mgr_     = std::make_shared<TripManager>();
    append_logger_ = std::make_shared<avs::AppendLogger>(ssd_root, image_topic_);

    int trip_id = trip_mgr_->GetTripId(image_path_);

    const uint64_t trip_start_ns = static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count());

    append_logger_->startTrip(current_day, folder_name, trip_id, trip_start_ns);

    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      image_topic_, make_auto_qos(image_topic_),
      std::bind(&ImgProcessNode::imageCallback, this, _1));
    
    latency_pub_ = this->create_publisher<std_msgs::msg::Int64>("/avs/record_latency_us", 10);

    RCLCPP_INFO(this->get_logger(),
                "AVS Image node started. Subscribed to %s; writing to append-logger (path=%s trip=%s)",
                image_topic_.c_str(), image_path_.c_str(), std::to_string(trip_id).c_str());
  }

  ~ImgProcessNode() override
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

  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    // const auto t0 = std::chrono::steady_clock::now();
    uint64_t ts_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                       std::chrono::system_clock::now().time_since_epoch()
                     ).count();

    bool is_unique = false;
    std::vector<uint8_t> payload;

    try {
      is_unique = deduplicator_->isUniqueAndGetBytes(*msg, payload);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Dedup/GetBytes failed: %s", e.what());
      return;
    }

    if (is_unique) {
      // uint64_t ts_ns = static_cast<uint64_t>(msg->header.stamp.sec) * 1000000000ULL
      //                + static_cast<uint64_t>(msg->header.stamp.nanosec);

      try {
        append_logger_->appendRecord(ts_ns, payload);
      } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "AppendLogger appendRecord failed: %s", e.what());
      }

      // const auto t1 = std::chrono::steady_clock::now();
      // const auto latency_us = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();
      // std_msgs::msg::Int64 m;
      // m.data = latency_us;
      // latency_pub_->publish(m);
    }
  }

  rclcpp::QoS make_auto_qos(const std::string& topic)
  {
    using rclcpp::ReliabilityPolicy;

    rclcpp::QoS qos = rclcpp::SensorDataQoS();  // BestEffort, small depth
    // qos.durability(rclcpp::DurabilityPolicy::Volatile);

    // for (int i = 0; i < 50; ++i) {
    //   auto infos = this->get_publishers_info_by_topic(topic);
    //   if (!infos.empty()) {
    //     auto offered = infos.front().qos_profile();
    //     qos.reliability(offered.reliability());
    //     RCLCPP_INFO(
    //       this->get_logger(),
    //       "QoS for %s -> depth=10 reliability=%d durability=%d",
    //       topic.c_str(),
    //       static_cast<int>(qos.get_rmw_qos_profile().reliability),
    //       static_cast<int>(qos.get_rmw_qos_profile().durability));
    //     return qos;
    //   }
    //   rclcpp::sleep_for(std::chrono::milliseconds(100));
    // }

    // RCLCPP_WARN(this->get_logger(),
    //             "No publisher QoS detected on %s; using fallback QoS(KEEP_LAST depth=10, BEST_EFFORT, VOLATILE)",
    //             topic.c_str());
    return qos;
  }

private:
  std::string image_topic_;
  std::string image_path_;

  std::error_code ec;

  std::shared_ptr<ImgDeduplicator> deduplicator_;
  std::shared_ptr<avs::AppendLogger> append_logger_;
  std::shared_ptr<avs::TripManager> trip_mgr_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr latency_pub_;
};

} // namespace avs

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<avs::ImgProcessNode>());
  rclcpp::shutdown();
  return 0;
}
