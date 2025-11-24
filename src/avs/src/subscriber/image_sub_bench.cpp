// avs_image_node.cpp
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/int64.hpp"
#include "std_msgs/msg/float64.hpp"

#include "yaml-cpp/yaml.h"
#include "avs/img_dedup.h"
#include "avs/img_compress.h"
#include "avs/db_operation.h"
#include "avs/common.h"

#include <filesystem>
#include <string>
#include <memory>
#include <chrono>
#include <deque>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <fstream>
#include <fcntl.h>
#include <unistd.h>

using std::placeholders::_1;
namespace fs = std::filesystem;

static inline void fsync_path_and_dir(const std::string& filepath) {
  int fd = ::open(filepath.c_str(), O_RDONLY);
  if (fd >= 0) { ::fsync(fd); ::close(fd); }
  auto slash = filepath.find_last_of('/');
  if (slash != std::string::npos) {
    std::string dir = filepath.substr(0, slash);
    int dfd = ::open(dir.c_str(), O_RDONLY | O_DIRECTORY);
    if (dfd >= 0) { ::fsync(dfd); ::close(dfd); }
  }
}

static inline double read_self_rss_mb() {
  std::ifstream f("/proc/self/status");
  std::string line;
  while (std::getline(f, line)) {
    if (line.rfind("VmRSS:", 0) == 0) {
      std::string num;
      for (char c : line) if (isdigit(c)) num.push_back(c);
      if (!num.empty()) {
        double kb = std::stod(num);
        return kb / 1024.0;
      }
      break;
    }
  }
  return 0.0;
}

namespace avs {

class ImgProcessNode : public rclcpp::Node
{
public:
  ImgProcessNode()
  : Node("img_sub_bench")
  {
    this->declare_parameter<std::string>("config_path", "/home/avs/AVS-PI/src/avs/config/avs_config.yaml");
    std::string config_path = this->get_parameter("config_path").as_string();

    YAML::Node root   = YAML::LoadFile(config_path);
    YAML::Node common = root["common"];
    YAML::Node dedup  = root["image_dedup"];

    image_topic_   = common["img_topic"].as<std::string>();
    image_ssd_dir_ = common["img_ssd_dir"].as<std::string>();
    db_dir_        = common["hot_db_dir"].as<std::string>();
    image_ext_     = dedup["img_format"].as<std::string>();

    int qparam = this->declare_parameter<int>("max_queue", 10);
    max_queue_ = std::max(1, qparam);

    image_path_ = (fs::path(image_ssd_dir_) / avs::getCurrentDayFolder()).string();
    if (!avs::ensureDirectory(image_path_, &ec)) {
      RCLCPP_WARN(this->get_logger(),
        "Image dir %s create attempt (ec=%d: %s).",
        image_path_.c_str(), ec.value(), ec.message().c_str());
    }

    deduplicator_ = std::make_shared<ImgDeduplicator>(image_path_, config_path);

    fs::create_directories(db_dir_);
    const std::string db_path = (fs::path(db_dir_) / "avs_image.sqlite3").string();

    std::string err;
    if (!db_.open(db_path, &err)) {
      throw std::runtime_error("Failed to open DB: " + err);
    }

    lat_pub_  = this->create_publisher<std_msgs::msg::Int64>   ("/avs/latency_us/image",  10);
    q_pub_    = this->create_publisher<std_msgs::msg::Int64>   ("/avs/queue_depth/image", 10);
    drop_pub_ = this->create_publisher<std_msgs::msg::Int64>   ("/avs/dropped/image",     10);
    rss_pub_  = this->create_publisher<std_msgs::msg::Float64> ("/avs/rss_mb/image",      10);

    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      image_topic_, make_auto_qos(image_topic_),
      std::bind(&ImgProcessNode::enqueueCb, this, _1));

    worker_thread_ = std::thread([this]{ workerLoop(); });

    metrics_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      [this]{
        std_msgs::msg::Int64 q;
        {
          std::lock_guard<std::mutex> lk(m_);
          q.data = static_cast<int64_t>(q_.size());
        }
        q_pub_->publish(q);

        std_msgs::msg::Int64 d; d.data = static_cast<int64_t>(dropped_.load());
        drop_pub_->publish(d);

        std_msgs::msg::Float64 rss; rss.data = read_self_rss_mb();
        rss_pub_->publish(rss);
      });

    RCLCPP_INFO(this->get_logger(),
      "AVS Image node (bounded queue=%d) Subscribed to %s → %s ; DB at %s",
      max_queue_, image_topic_.c_str(), image_path_.c_str(), db_path.c_str());
  }

  ~ImgProcessNode() override {
    {
      std::lock_guard<std::mutex> lk(m_);
      stop_ = true;
    }
    cv_.notify_all();
    if (worker_thread_.joinable()) worker_thread_.join();
  }

private:
  struct Item {
    sensor_msgs::msg::Image::ConstSharedPtr msg;
    std::chrono::steady_clock::time_point t0;
  };

  void enqueueCb(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
    const auto now = std::chrono::steady_clock::now();
    {
      std::lock_guard<std::mutex> lk(m_);
      if (q_.size() >= static_cast<size_t>(max_queue_)) {
        dropped_++;
        return;
      }
      q_.push_back({msg, now});
      if (q_.size() > max_seen_depth_) max_seen_depth_ = q_.size();
    }
    cv_.notify_one();
  }

  void workerLoop() {
    while (rclcpp::ok()) {
      Item it;
      {
        std::unique_lock<std::mutex> lk(m_);
        cv_.wait(lk, [this]{ return stop_ || !q_.empty(); });
        if (stop_) break;
        it = std::move(q_.front());
        q_.pop_front();
      }

      const auto& msg = it.msg;
      long long image_ts_ms = msg->header.stamp.sec * 1000LL + msg->header.stamp.nanosec / 1000000LL;
      std::string filepath = image_path_ + '/' + std::to_string(image_ts_ms) + "." + image_ext_;

      bool saved = false;
      try {
        // Save only if unique; this function writes the image file atomically
        saved = deduplicator_->isUniqueAndStore(*msg, filepath);
        if (saved) {
          fsync_path_and_dir(filepath); // ensure durable
        }
      } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Image dedup/store failed: %s", e.what());
        saved = false;
      }

      if (saved) {
        AvsRow row;
        row.sensor_id = image_topic_;
        row.data_type = image_ext_;
        row.ts_ms     = image_ts_ms;
        row.path      = filepath;

        std::string err;
        if (!db_.insertRow(row, &err)) {
          RCLCPP_ERROR(this->get_logger(), "Image DB insert failed: %s", err.c_str());
        }

        const auto t1 = std::chrono::steady_clock::now();
        const auto us = std::chrono::duration_cast<std::chrono::microseconds>(t1 - it.t0).count();
        std_msgs::msg::Int64 m; m.data = static_cast<int64_t>(us);
        lat_pub_->publish(m);
      }
      // If duplicate, we intentionally do not publish a latency sample.
      // (You can change this behavior if you want dup-aware stats.)
    }
  }

  rclcpp::QoS make_auto_qos(const std::string& topic)
  {
    using rclcpp::ReliabilityPolicy;
    rclcpp::QoS qos(10);
    qos.reliability(ReliabilityPolicy::Reliable);
    qos.durability(rclcpp::DurabilityPolicy::Volatile);

    for (int i = 0; i < 50; ++i) {
      auto infos = this->get_publishers_info_by_topic(topic);
      if (!infos.empty()) {
        auto offered = infos.front().qos_profile();
        qos.reliability(offered.reliability());
        RCLCPP_INFO(this->get_logger(),
          "QoS for %s -> depth=10 reliability=%d durability=%d",
          topic.c_str(),
          static_cast<int>(qos.get_rmw_qos_profile().reliability),
          static_cast<int>(qos.get_rmw_qos_profile().durability));
        return qos;
      }
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
    RCLCPP_WARN(this->get_logger(),
      "No publisher QoS detected on %s; using fallback RELIABLE/Volatile, depth=10", topic.c_str());
    return qos;
  }

private:
  // Config/state
  std::string image_topic_;
  std::string image_ssd_dir_;
  std::string db_dir_;
  std::string image_ext_;
  std::string image_path_;
  std::error_code ec;

  int max_queue_{8};
  std::deque<Item> q_;
  std::mutex m_;
  std::condition_variable cv_;
  std::thread worker_thread_;
  std::atomic<bool> stop_{false};
  std::atomic<uint64_t> dropped_{0};
  size_t max_seen_depth_{0};

  std::shared_ptr<ImgDeduplicator> deduplicator_;
  AvsDb db_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;

  // Metrics
  rclcpp::TimerBase::SharedPtr metrics_timer_;
  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr   lat_pub_;
  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr   q_pub_;
  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr   drop_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rss_pub_;
};

} // namespace avs

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<avs::ImgProcessNode>());
  rclcpp::shutdown();
  return 0;
}
