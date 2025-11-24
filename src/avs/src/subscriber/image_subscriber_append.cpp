// ...existing code...
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/int64.hpp"
#include "avs/img_dedup.h"
#include "avs/img_compress.h"
#include "avs/db_operation.h"
#include "avs/common.h"
#include "avs/append_logger.h"
#include <cv_bridge/cv_bridge.hpp>
#include <chrono>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <string>
#include <memory>
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
    std::string config_path = this->get_parameter("config_path").as_string();

    YAML::Node root = YAML::LoadFile(config_path);
    YAML::Node common = root["common"];
    YAML::Node dedup = root["image_dedup"];

    image_topic_ = common["img_topic"].as<std::string>();
    image_ssd_dir_ = common["img_ssd_dir"].as<std::string>();
    db_dir_ = common["hot_db_dir"].as<std::string>();
    image_ext_ = dedup["img_format"].as<std::string>();

    image_path_ = (fs::path(image_ssd_dir_) / avs::getCurrentDayFolder()).string();
    if (!avs::ensureDirectory(image_path_, &ec)) {
      RCLCPP_WARN(this->get_logger(),
                  "Image directory %s did not exist. Attempted to create it (ec=%d: %s).",
                  image_path_.c_str(),
                  ec.value(),
                  ec.message().c_str());
    }

    deduplicator_ = std::make_shared<ImgDeduplicator>(image_path_, config_path);

    fs::create_directories(db_dir_);
    const std::string db_path = (fs::path(db_dir_) / "avs_image.sqlite3").string();
    std::string err;
    if (!db_.open(db_path, &err)) {
      throw std::runtime_error("Failed to open DB: " + err);
    }

    // Setup append logger: derive SSD root and topic name from image_ssd_dir_
    fs::path p(image_ssd_dir_);
    std::string topic_name = p.filename().string();               // e.g. "camera_front"
    std::string ssd_root   = p.parent_path().string();           // parent as SSD root
    // fall back to a known path if config was absolute SSD root
    if (ssd_root.empty()) ssd_root = "/home/avs/DATA/SSD";
    append_logger_ = std::make_shared<avs::AppendLogger>(ssd_root, topic_name);

    // start a single trip (bench / node lifetime)
    const std::string day = avs::getCurrentDayFolder();
    const int trip_id = 0; // need to add function about trip id management per day
    const uint64_t trip_start_ns = static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count());
    append_logger_->startTrip(day, trip_id, trip_start_ns);

    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      image_topic_, make_auto_qos(image_topic_),
      std::bind(&ImgProcessNode::imageCallback, this, _1));
    
    RCLCPP_INFO(this->get_logger(),
                "AVS Image node started. Subscribed to %s; writing to append-logger (SSD root=%s topic=%s); DB at %s",
                image_topic_.c_str(), ssd_root.c_str(), topic_name.c_str(), db_path.c_str());
  }

  ~ImgProcessNode() override
  {
    // end trip on destruction
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
    // compute ms timestamp (kept for compatibility)
    long long ts_ms = msg->header.stamp.sec * 1000LL + msg->header.stamp.nanosec / 1000000LL;
    std::string filepath = image_path_ + '/' + std::to_string(ts_ms) + "." + image_ext_;

    bool is_unique = false;
    // try {
    //   // Keep original dedup logic (attempt to use existing helper).
    //   // If the existing API writes files, we still use it to preserve behavior,
    //   // but we will also obtain the JPEG bytes directly and send to append logger.
    //   is_unique = deduplicator_->isUniqueAndStore(*msg, filepath);
    // } catch (const std::exception& e) {
    //   RCLCPP_ERROR(this->get_logger(), "Dedup/store failed: %s", e.what());
    //   return;
    // }
    is_unique = deduplicator_->isUnique(*msg);

    if (is_unique) {
      // Produce JPEG bytes from the incoming Image message (cv::imencode)
      std::vector<uint8_t> jpeg_buf;
      try {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, msg->encoding);
        std::vector<int> params;
        params.push_back(cv::IMWRITE_JPEG_QUALITY);
        params.push_back(90);
        cv::imencode("." + image_ext_, cv_ptr->image, jpeg_buf, params);
      } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to encode JPEG: %s", e.what());
        return;
      }

      // Compute timestamp in nanoseconds for append logger (use ROS header if present)
      uint64_t ts_ns = static_cast<uint64_t>(msg->header.stamp.sec) * 1000000000ULL
                     + static_cast<uint64_t>(msg->header.stamp.nanosec);

      // Append to trip log (replaces DB insert + direct .jpg write)
      try {
        append_logger_->appendRecord(ts_ns, jpeg_buf);
      } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "AppendLogger appendRecord failed: %s", e.what());
      }
    }
  }

  rclcpp::QoS make_auto_qos(const std::string& topic)
  {
    using rclcpp::ReliabilityPolicy;

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
  std::string image_topic_;
  std::string image_ssd_dir_;
  std::string db_dir_;
  std::string image_ext_;
  std::string image_path_;

  std::error_code ec;

  std::shared_ptr<ImgDeduplicator> deduplicator_;
  AvsDb db_;

  std::shared_ptr<avs::AppendLogger> append_logger_;

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