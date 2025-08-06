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
#include <sstream>
#include <fcntl.h>

using std::placeholders::_1;
namespace fs = std::filesystem;

class ImageLoggerNode : public rclcpp::Node
{
public:
  ImageLoggerNode()
  : Node("image_logger_node"), logical_written_bytes_(0), image_count_(0), fsync_total_ms_(0.0)
  {
    this->declare_parameter<std::string>("image_topic", "/camera/image_raw");
    this->declare_parameter<std::string>("output_dir", "/home/avs/DATA/SSD/images");
    this->declare_parameter<std::string>("device_name", "nvme0n1p3");

    this->get_parameter("image_topic", image_topic_);
    this->get_parameter("output_dir", output_dir_);
    this->get_parameter("device_name", device_name_);

    fs::create_directories(output_dir_);

    sectors_written_start_ = get_partition_sectors_written(device_name_);
    first_write_time_ = std::chrono::steady_clock::time_point();
    last_write_time_ = std::chrono::steady_clock::time_point();
    last_msg_time_ = std::chrono::steady_clock::now();
    shutdown_timeout_sec_ = 3.0;

    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      image_topic_, 10, std::bind(&ImageLoggerNode::image_callback, this, _1));

    timer_ = this->create_wall_timer(
      std::chrono::seconds(1), std::bind(&ImageLoggerNode::check_shutdown_condition, this));

    RCLCPP_INFO(this->get_logger(), "Image logger node started. Subscribed to %s", image_topic_.c_str());
  }

  ~ImageLoggerNode() {
    if (image_count_ == 0 || first_write_time_ == std::chrono::steady_clock::time_point() || last_write_time_ == std::chrono::steady_clock::time_point()) {
      RCLCPP_WARN(this->get_logger(), "No images written. Skipping summary.");
      return;
    }

    system("sync");
    double elapsed_sec = std::chrono::duration<double>(last_write_time_ - first_write_time_).count();

    uint64_t sectors_written_end = get_partition_sectors_written(device_name_);
    double physical_written_bytes = (sectors_written_end - sectors_written_start_) * 512.0;

    double write_throughput_MBps = (logical_written_bytes_ / 1e6) / elapsed_sec;
    double write_amplification = (logical_written_bytes_ > 0) ? (physical_written_bytes / logical_written_bytes_) : 0.0;
    double avg_fsync_latency_ms = image_count_ > 0 ? fsync_total_ms_ / image_count_ : 0.0;

    RCLCPP_INFO(this->get_logger(), "=== Filesystem Benchmark Summary ===");
    RCLCPP_INFO(this->get_logger(), "Elapsed Time (actual write): %.2f s", elapsed_sec);
    RCLCPP_INFO(this->get_logger(), "Logical Data Written: %.2f MB", logical_written_bytes_ / 1e6);
    RCLCPP_INFO(this->get_logger(), "Physical Data Written: %.2f MB", physical_written_bytes / 1e6);
    RCLCPP_INFO(this->get_logger(), "Write Throughput: %.2f MB/s", write_throughput_MBps);
    RCLCPP_INFO(this->get_logger(), "Write Amplification: %.2fx", write_amplification);
    RCLCPP_INFO(this->get_logger(), "Average Fsync Latency: %.2f ms", avg_fsync_latency_ms);
  }

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    last_msg_time_ = std::chrono::steady_clock::now();
    auto start = std::chrono::steady_clock::now();

    if (first_write_time_ == std::chrono::steady_clock::time_point()) {
      first_write_time_ = start;
    }

    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    std::string filename = output_dir_ + "/" + std::to_string(msg->header.stamp.sec) + "_" + std::to_string(msg->header.stamp.nanosec) + ".jpg";

    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 95};
    cv::imwrite(filename, cv_ptr->image, params);

    std::error_code ec;
    size_t file_size = fs::file_size(filename, ec);
    logical_written_bytes_ += file_size;
    ++image_count_;

    int fd = open(filename.c_str(), O_RDONLY);
    auto fsync_start = std::chrono::steady_clock::now();
    fsync(fd);
    auto fsync_end = std::chrono::steady_clock::now();
    close(fd);
    double fsync_latency_ms = std::chrono::duration<double, std::milli>(fsync_end - fsync_start).count();
    fsync_total_ms_ += fsync_latency_ms;

    auto end = std::chrono::steady_clock::now();
    last_write_time_ = end;
    double duration_ms = std::chrono::duration<double, std::milli>(end - start).count();

    RCLCPP_INFO(this->get_logger(), "Wrote %s | Size: %.2f KB | Write Time: %.2f ms | Fsync Latency: %.2f ms",
                filename.c_str(), file_size / 1024.0, duration_ms, fsync_latency_ms);
  }

  void check_shutdown_condition()
  {
    auto now = std::chrono::steady_clock::now();
    double elapsed = std::chrono::duration<double>(now - last_msg_time_).count();
    if (elapsed > shutdown_timeout_sec_) {
      RCLCPP_INFO(this->get_logger(), "No image received for %.1f seconds. Shutting down...", elapsed);
      rclcpp::shutdown();
    }
  }

  uint64_t get_partition_sectors_written(const std::string &partition)
  {
    std::ifstream diskstats("/proc/diskstats");
    std::string line;
    while (std::getline(diskstats, line)) {
      std::istringstream iss(line);
      std::string field;
      std::vector<std::string> fields;
      while (iss >> field) {
        fields.push_back(field);
      }
      if (fields.size() > 10 && fields[2] == partition) {
        return std::stoull(fields[9]);  // field 10 is "# of sectors written"
      }
    }
    return 0;
  }

  std::string image_topic_;
  std::string output_dir_;
  std::string device_name_;
  uint64_t logical_written_bytes_;
  uint64_t sectors_written_start_;
  size_t image_count_;
  double fsync_total_ms_;
  std::chrono::steady_clock::time_point first_write_time_;
  std::chrono::steady_clock::time_point last_write_time_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::chrono::steady_clock::time_point last_msg_time_;
  double shutdown_timeout_sec_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageLoggerNode>());
  rclcpp::shutdown();
  return 0;
}