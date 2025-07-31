#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <chrono>
#include "lidar_downsample.h"
#include "lidar_compress.h"

using std::placeholders::_1;

namespace avs {

class LidarProcessNode : public rclcpp::Node
{
public:
  LidarProcessNode()
  : Node("lidar_process_node")
  {
    this->declare_parameter<std::string>("lidar_topic", "/kitti/velo/pointcloud");
    this->declare_parameter<std::string>("config_path", "/home/avs/AVS-PI/src/avs/config/avs_config.yaml");
    this->declare_parameter<std::string>("output_dir", "/home/avs/DATA/SSD/lidar_laz");

    std::string lidar_topic = this->get_parameter("lidar_topic").as_string();
    std::string config_path = this->get_parameter("config_path").as_string();
    std::string output_dir = this->get_parameter("output_dir").as_string();

    downsampler_ = std::make_shared<LidarDownsampler>(config_path);
    compressor_ = std::make_shared<LidarCompressor>(output_dir);

    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      lidar_topic, rclcpp::SensorDataQoS(), std::bind(&LidarProcessNode::lidarCallback, this, _1));
    
    RCLCPP_INFO(this->get_logger(), "Lidar downsample + compression node started. Subscribed to %s", lidar_topic.c_str());
  }

private:
  void lidarCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    auto start = std::chrono::steady_clock::now();
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *pcl_cloud);

    pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled = downsampler_->downsample(pcl_cloud);

    compressor_->saveAsLAZ(downsampled);

    auto end = std::chrono::steady_clock::now();
    auto latency_us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

    RCLCPP_INFO(this->get_logger(), "Latency: %ld µs", latency_us);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  std::shared_ptr<LidarDownsampler> downsampler_;
  std::shared_ptr<LidarCompressor> compressor_;
};

} // namespace avs

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<avs::LidarProcessNode>());
  rclcpp::shutdown();
  return 0;
}
