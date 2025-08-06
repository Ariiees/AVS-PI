// image_logger_node.cpp
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgcodecs.hpp>
#include <chrono>
#include <fstream>
#include <filesystem>
#include <cstdlib>
#include <cstdio>
#include <unistd.h>
#include "avs/fs_benchmark_logger.h"

using std::placeholders::_1;
namespace fs = std::filesystem;

class ImageLoggerNode : public rclcpp::Node
{
public:
  ImageLoggerNode()
  : Node("image_logger_node")
  {
    this->declare_parameter<std::string>("image_topic", "/camera/image_raw");
    this->declare_parameter<std::string>("output_dir", "/home/avs/DATA/SSD/images");
    this->declare_parameter<std::string>("benchmark_csv", "/home/avs/DATA/image_ext4_benchmark.csv");

    this->get_parameter("image_topic", image_topic_);
    this->get_parameter("output_dir", output_dir_);
    this->get_parameter("benchmark_csv", benchmark_csv_);

    fs::create_directories(output_dir_);
    fs_logger_ = std::make_shared<FSBenchmarkLogger>(benchmark_csv_);

    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      image_topic_, 10, std::bind(&ImageLoggerNode::image_callback, this, _1));

    RCLCPP_INFO(this->get_logger(), "Filesystem image write benchmark: ImageLoggerNOde started. Subscribed to %s", image_topic_.c_str());
  }

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    auto start = std::chrono::high_resolution_clock::now();
    
    // Convert ROS image to OpenCV
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    std::string filename = output_dir_ + "/" + std::to_string(msg->header.stamp.sec) + "_" + std::to_string(msg->header.stamp.nanosec) + ".jpg";

    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 95};
    cv::imwrite(filename, cv_ptr->image, params);

    // Get fsync latency using perf
    std::string fsync_cmd = "perf stat -e syscalls:sys_enter_fsync -x, -r 1 true 2>&1 | grep fsync | awk -F',' '{print $2}'";
    double fsync_latency_ms = run_shell_command(fsync_cmd);

    // Get write throughput using iotop
    std::string iotop_cmd = "sudo iotop -b -n 1 -p " + std::to_string(getpid()) + " | grep -v Total | awk '{print $10}'";
    double write_MBps = run_shell_command(iotop_cmd);

    double cpu_usage = fs_logger_->get_cpu_usage();
    auto end = std::chrono::high_resolution_clock::now();
    double duration_ms = std::chrono::duration<double, std::milli>(end - start).count();

    fs_logger_->log(duration_ms, fsync_latency_ms, write_MBps, 0.0, cpu_usage);
  }

  double run_shell_command(const std::string &cmd)
  {
    FILE* pipe = popen(cmd.c_str(), "r");
    if (!pipe) return 0.0;
    char buffer[128];
    std::string result;
    if (fgets(buffer, sizeof(buffer), pipe)) {
      result = buffer;
    }
    pclose(pipe);
    try {
      return std::stod(result);
    } catch (...) {
      return 0.0;
    }
  }

  std::string image_topic_;
  std::string output_dir_;
  std::string benchmark_csv_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  std::shared_ptr<FSBenchmarkLogger> fs_logger_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageLoggerNode>());
  rclcpp::shutdown();
  return 0;
}
