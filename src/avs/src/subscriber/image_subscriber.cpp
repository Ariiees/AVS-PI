#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "img_dedup.h"
#include "img_compress.h"
#include <cv_bridge/cv_bridge.hpp>
#include <chrono>

using std::placeholders::_1;

namespace avs
{

class ImgDedupNode : public rclcpp::Node
{
public:
  ImgDedupNode()
  : Node("img_dedup_node")
  {
    // Declare parameters
    this->declare_parameter<std::string>("image_topic", "/camera/image_raw");
    this->declare_parameter<std::string>("dedup_output_dir", "/home/avs/DATA/SSD/images");
    // this->declare_parameter<std::string>("video_output_dir", "/home/avs/DATA/HDD/videos");
    this->declare_parameter<std::string>("config_path", "/home/avs/AVS-PI/src/avs/config/avs_config.yaml");

    std::string image_topic      = this->get_parameter("image_topic").as_string();
    std::string dedup_output_dir = this->get_parameter("dedup_output_dir").as_string();
    // std::string video_output_dir = this->get_parameter("video_output_dir").as_string();
    std::string config     = this->get_parameter("config_path").as_string();

    deduplicator_ = std::make_shared<ImgDeduplicator>(dedup_output_dir, config);
    // compressor_   = std::make_shared<VideoCompressor>(video_output_dir, config);

    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      image_topic, 10, std::bind(&ImgDedupNode::imageCallback, this, _1));

    RCLCPP_INFO(this->get_logger(), "Image deduplication + video compression node started. Subscribed to %s", image_topic.c_str());
  }

  // ~ImgDedupNode()
  // {
  //   compressor_->finalize();
  // }

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    auto start = std::chrono::steady_clock::now();

    // Convert once for both dedup and video compression
    cv::Mat image;
    try {
      image = cv_bridge::toCvShare(msg, "bgr8")->image;
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge conversion failed: %s", e.what());
      return;
    }

    // compressor_->addImage(image);  // Always write to HDD (video)

    bool is_unique = deduplicator_->isUniqueAndStore(*msg);  // Conditionally store to SSD

    auto end = std::chrono::steady_clock::now();
    auto latency_us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

    RCLCPP_INFO(this->get_logger(), "Latency: %ld µs", latency_us);

    // if (is_unique)
    // {
    //   RCLCPP_INFO(this->get_logger(), "Unique image stored. Callback latency: %ld µs", latency_us);
    // }
    // else
    // {
    //   RCLCPP_INFO(this->get_logger(), "Duplicate image skipped. Callback latency: %ld µs", latency_us);
    // }
  }

  std::shared_ptr<ImgDeduplicator> deduplicator_;
  // std::shared_ptr<VideoCompressor> compressor_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

} // namespace avs

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<avs::ImgDedupNode>());
  rclcpp::shutdown();
  return 0;
}
