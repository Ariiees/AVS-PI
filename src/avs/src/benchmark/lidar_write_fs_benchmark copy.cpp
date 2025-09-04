#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <fcntl.h>
#include <unistd.h>

#include "avs/lidar_compress.h"
#include "avs/common.h"

using std::placeholders::_1;
namespace fs = std::filesystem;

class LidarLoggerNode : public rclcpp::Node
{
public:
  LidarLoggerNode(const rclcpp::NodeOptions & options)
  : Node("lidar_logger_node", options), logical_written_bytes_(0), cloud_count_(0), fsync_total_ms_(0.0)
  {
    this->get_parameter_or<std::string>("output_dir", output_dir_, "/home/avs/DATA/SSD/lidar_laz");
    this->get_parameter_or<std::string>("device_name", device_name_, "nvme0n1p3");
    
    this->declare_parameter<std::string>("lidar_topic", "/kitti/velo/pointcloud");
    this->get_parameter("lidar_topic", lidar_topic_);


    compressor_ = std::make_shared<avs::LidarCompressor>(output_dir_); // will create directories inside the compressor creater

    sectors_written_start_ = get_partition_sectors_written(device_name_);
    first_write_time_ = std::chrono::steady_clock::time_point();
    last_write_time_ = std::chrono::steady_clock::time_point();
    last_msg_time_ = std::chrono::steady_clock::now();
    shutdown_timeout_sec_ = 3.0;

    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      lidar_topic_, rclcpp::SensorDataQoS(), std::bind(&LidarLoggerNode::lidar_callback, this, _1));

    timer_ = this->create_wall_timer(
      std::chrono::seconds(1), std::bind(&LidarLoggerNode::check_shutdown_condition, this));

    RCLCPP_INFO(this->get_logger(), "Lidar logger node started. Subscribed to %s, writing to %s", lidar_topic_.c_str(), output_dir_.c_str());
  }

  ~LidarLoggerNode()
  {
    if (cloud_count_ == 0 || first_write_time_ == std::chrono::steady_clock::time_point()) {
      RCLCPP_WARN(this->get_logger(), "No point clouds written. Skipping summary.");
      return;
    }

    system("sync");
    double elapsed_sec = std::chrono::duration<double>(last_write_time_ - first_write_time_).count();

    uint64_t sectors_written_end = get_partition_sectors_written(device_name_);
    double physical_written_bytes = (sectors_written_end - sectors_written_start_) * 512.0;

    double write_throughput_MBps = (logical_written_bytes_ / 1e6) / elapsed_sec;
    double write_amplification = logical_written_bytes_ > 0 ? physical_written_bytes / logical_written_bytes_ : 0.0;
    double avg_fsync_latency_ms = cloud_count_ > 0 ? fsync_total_ms_ / cloud_count_ : 0.0;

    RCLCPP_INFO(this->get_logger(), "=== Filesystem Benchmark Summary ===");
    RCLCPP_INFO(this->get_logger(), "Elapsed Time: %.2f s", elapsed_sec);
    RCLCPP_INFO(this->get_logger(), "Logical Data Written: %.2f MB", logical_written_bytes_ / 1e6);
    RCLCPP_INFO(this->get_logger(), "Physical Data Written: %.2f MB", physical_written_bytes / 1e6);
    RCLCPP_INFO(this->get_logger(), "Write Throughput: %.2f MB/s", write_throughput_MBps);
    RCLCPP_INFO(this->get_logger(), "Write Amplification: %.2fx", write_amplification);
    RCLCPP_INFO(this->get_logger(), "Average Fsync Latency: %.2f ms", avg_fsync_latency_ms);
  }

private:
  void lidar_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    last_msg_time_ = std::chrono::steady_clock::now();
    auto start = std::chrono::steady_clock::now();

    if (first_write_time_ == std::chrono::steady_clock::time_point()) {
      first_write_time_ = start;
    }

    // Convert to PCL
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *pcl_cloud);

    // Save LAZ file
    std::string filename = avs::getTimestampFilename(output_dir_, ".laz");

    compressor_->saveAsLAZ(pcl_cloud, filename);

    // Get file size and stats
    std::error_code ec;
    size_t file_size = fs::file_size(filename, ec);
    logical_written_bytes_ += file_size;
    ++cloud_count_;

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

    RCLCPP_INFO(this->get_logger(), "Saved %s | Size: %.2f KB | Write Time: %.2f ms | Fsync: %.2f ms",
                filename.c_str(), file_size / 1024.0, duration_ms, fsync_latency_ms);
  }

  void check_shutdown_condition()
  {
    auto now = std::chrono::steady_clock::now();
    double elapsed = std::chrono::duration<double>(now - last_msg_time_).count();
    if (elapsed > shutdown_timeout_sec_) {
      RCLCPP_INFO(this->get_logger(), "No LiDAR message received for %.1f seconds. Shutting down...", elapsed);
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
      while (iss >> field) fields.push_back(field);
      if (fields.size() > 10 && fields[2] == partition) {
        return std::stoull(fields[9]);  // field 10 is "# of sectors written"
      }
    }
    return 0;
  }

  std::string lidar_topic_;
  std::string output_dir_;
  std::string device_name_;
  uint64_t logical_written_bytes_;
  uint64_t sectors_written_start_;
  size_t cloud_count_;
  double fsync_total_ms_;
  std::chrono::steady_clock::time_point first_write_time_;
  std::chrono::steady_clock::time_point last_write_time_;
  std::chrono::steady_clock::time_point last_msg_time_;
  double shutdown_timeout_sec_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  std::shared_ptr<avs::LidarCompressor> compressor_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto options = rclcpp::NodeOptions()
    .allow_undeclared_parameters(true)
    .automatically_declare_parameters_from_overrides(true);
  rclcpp::spin(std::make_shared<LidarLoggerNode>(options));
  rclcpp::shutdown();
  return 0;
}
