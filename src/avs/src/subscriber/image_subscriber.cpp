#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/int64.hpp"
#include "avs/img_dedup.h"
#include "avs/img_compress.h"
#include "avs/db_operation.h"
#include "avs/common.h"
#include <cv_bridge/cv_bridge.hpp>
#include <chrono>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <string>
#include <memory>

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
     std::string config_path = this->get_parameter("config_path").as_string();

    YAML::Node root = YAML::LoadFile(config_path);
    YAML::Node common = root["common"];
    YAML::Node dedup = root["image_dedup"];

    image_topic_ = common["img_topic"].as<std::string>();
    image_ssd_dir_ = common["img_ssd_dir"].as<std::string>();
    db_dir_ = common["db_dir"].as<std::string>();
    image_ext_ = dedup["img_format"].as<std::string>();

    image_path_ = (fs::path(image_ssd_dir_) / avs::getCurrentDayFolder()).string();

    deduplicator_ = std::make_shared<ImgDeduplicator>(image_path_, config_path);

    fs::create_directories(db_dir_);
    const std::string db_path = (fs::path(db_dir_) / "avs_image.sqlite3").string();
    std::string err;
    if (!db_.open(db_path, &err)) {
      throw std::runtime_error("Failed to open DB: " + err);
    }

    // subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
    //   image_topic_, 10, std::bind(&ImgProcessNode::imageCallback, this, _1));
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      image_topic_, make_auto_qos(image_topic_),
      std::bind(&ImgProcessNode::imageCallback, this, _1));
    
    latency_pub_ = this->create_publisher<std_msgs::msg::Int64>("/avs/record_latency_us", 10);

    RCLCPP_INFO(this->get_logger(),
                "AVS Image node started. Subscribed to %s; saving to %s; DB at %s",
                image_topic_.c_str(), image_path_.c_str(), db_path.c_str());
  }

private:

  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    const auto t0 = std::chrono::steady_clock::now();

    // Build filename from wall-clock ms so the on-disk name is friendly/stable.
    auto [filepath, ts_ms] = avs::getTimestampAndFilename(image_path_, image_ext_);

    bool is_unique = false;
    try {
      // Save only if unique
      is_unique = deduplicator_->isUniqueAndStore(*msg, filepath);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Dedup/store failed: %s", e.what());
      return;
    }

    // If we saved, record metadata to DB
    if (is_unique) {
      AvsRow row;
      row.sensor_id = image_topic_;
      row.data_type = image_ext_;  
      row.ts_ms     = ts_ms;
      row.path      = filepath;

      std::string err;
      if (!db_.insertRow(row, &err)) {
        RCLCPP_ERROR(this->get_logger(), "DB insert failed: %s", err.c_str());
      } 
      
      const auto t1 = std::chrono::steady_clock::now();
      const auto latency_us = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();
      std_msgs::msg::Int64 m;
      m.data = latency_us;
      latency_pub_->publish(m);
    }

    // RCLCPP_INFO(this->get_logger(), "Latency: %ld µs", latency_us);
    // if (is_unique) {
    //   RCLCPP_INFO(this->get_logger(),
    //               "[UNIQUE] saved %s | latency=%ldus",
    //               filepath.c_str(), latency_us);
    // } else {
    //   RCLCPP_INFO(this->get_logger(), "[DUP] skipped | latency=%ldus",
    //               latency_us);
    // }
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
  std::string image_topic_;
  std::string image_ssd_dir_;
  std::string db_dir_;
  std::string image_ext_;
  std::string image_path_;

  // Helpers
  std::shared_ptr<ImgDeduplicator> deduplicator_;
  AvsDb db_;

  // ROS
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
