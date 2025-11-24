// avs_gps_node.cpp
#include "rclcpp/rclcpp.hpp"
#include "gps_msgs/msg/gps_fix.hpp"
#include "std_msgs/msg/int64.hpp"
#include "std_msgs/msg/float64.hpp"

#include "yaml-cpp/yaml.h"
#include "avs/common.h"

#include <sqlite3.h>
#include <filesystem>
#include <string>
#include <chrono>
#include <deque>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <fstream>

using std::placeholders::_1;
namespace fs = std::filesystem;

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

class GpsSubscriber : public rclcpp::Node {
public:
  GpsSubscriber()
  : Node("gps_sub_bench")
  {
    this->declare_parameter<std::string>("config_path", "/home/avs/AVS-PI/src/avs/config/avs_config.yaml");
    std::string config_path = this->get_parameter("config_path").as_string();

    YAML::Node root   = YAML::LoadFile(config_path);
    YAML::Node common = root["common"];
    gps_topic_    = common["gps_topic"].as<std::string>();
    gps_ssd_dir_  = common["gps_ssd_dir"].as<std::string>();

    int qparam = this->declare_parameter<int>("max_queue", 10);
    max_queue_ = std::max(1, qparam);

    // Open SQLite DB (per-day path, same as your original design)
    openDatabase();

    // Metrics publishers
    lat_pub_  = this->create_publisher<std_msgs::msg::Int64>   ("/avs/latency_us/gps",  10);
    q_pub_    = this->create_publisher<std_msgs::msg::Int64>   ("/avs/queue_depth/gps", 10);
    drop_pub_ = this->create_publisher<std_msgs::msg::Int64>   ("/avs/dropped/gps",     10);
    rss_pub_  = this->create_publisher<std_msgs::msg::Float64> ("/avs/rss_mb/gps",      10);

    // Subscription → bounded queue
    sub_ = this->create_subscription<gps_msgs::msg::GPSFix>(
      gps_topic_, make_auto_qos(gps_topic_),
      std::bind(&GpsSubscriber::enqueueCb, this, _1));

    // Worker thread
    worker_thread_ = std::thread([this]{ workerLoop(); });

    // Metrics timer
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
      "AVS GPS node (bounded queue=%d). Subscribed to %s; DB at %s",
      max_queue_, gps_topic_.c_str(), db_path_.c_str());
  }

  ~GpsSubscriber() override {
    {
      std::lock_guard<std::mutex> lk(m_);
      stop_ = true;
    }
    cv_.notify_all();
    if (worker_thread_.joinable()) worker_thread_.join();
    if (db_) sqlite3_close(db_);
  }

private:
  struct Item {
    gps_msgs::msg::GPSFix::SharedPtr msg;
    std::chrono::steady_clock::time_point t0;
  };

  void openDatabase() {
    gps_path_ = (fs::path(gps_ssd_dir_) / avs::getCurrentDayFolder()).string();
    // ensure directory exists (for parity, even though DB lives at parent)
    std::error_code ec;
    avs::ensureDirectory(gps_path_, &ec);

    db_path_ = gps_path_ + ".sqlite3";
    if (sqlite3_open(db_path_.c_str(), &db_) != SQLITE_OK) {
      RCLCPP_FATAL(this->get_logger(), "Failed to open DB: %s", sqlite3_errmsg(db_));
      db_ = nullptr;
      return;
    }

    char *errmsg = nullptr;
    sqlite3_exec(db_, "PRAGMA journal_mode=WAL;", nullptr, nullptr, &errmsg);
    if (errmsg) { RCLCPP_WARN(this->get_logger(), "PRAGMA WAL: %s", errmsg); sqlite3_free(errmsg); errmsg = nullptr; }
    sqlite3_exec(db_, "PRAGMA synchronous=FULL;", nullptr, nullptr, &errmsg);
    if (errmsg) { RCLCPP_WARN(this->get_logger(), "PRAGMA synchronous=FULL: %s", errmsg); sqlite3_free(errmsg); errmsg = nullptr; }

    const char *sql =
      "CREATE TABLE IF NOT EXISTS gps_data ("
      " ts_ms INTEGER PRIMARY KEY, "
      " latitude REAL, "
      " longitude REAL, "
      " altitude REAL, "
      " cov_xx REAL, "
      " cov_yy REAL, "
      " cov_zz REAL"
      ");";

    if (sqlite3_exec(db_, sql, nullptr, nullptr, &errmsg) != SQLITE_OK) {
      RCLCPP_FATAL(this->get_logger(), "Failed to create table: %s", errmsg);
      sqlite3_free(errmsg);
    }
  }

  void enqueueCb(const gps_msgs::msg::GPSFix::SharedPtr msg) {
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

      if (!db_) continue;

      sqlite3_stmt *stmt = nullptr;
      const char *sql =
        "INSERT OR IGNORE INTO gps_data (ts_ms, latitude, longitude, altitude, cov_xx, cov_yy, cov_zz) "
        "VALUES (?, ?, ?, ?, ?, ?, ?);";

      if (sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr) != SQLITE_OK) {
        RCLCPP_ERROR(this->get_logger(), "Prepare failed: %s", sqlite3_errmsg(db_));
        continue;
      }

      auto &msg = it.msg;
      long long gps_ts_ms = msg->header.stamp.sec * 1000LL + msg->header.stamp.nanosec / 1000000LL;

      sqlite3_bind_int64 (stmt, 1, gps_ts_ms);
      sqlite3_bind_double(stmt, 2, msg->latitude);
      sqlite3_bind_double(stmt, 3, msg->longitude);
      sqlite3_bind_double(stmt, 4, msg->altitude);

      // position_covariance is 9-element (row-major)
      sqlite3_bind_double(stmt, 5, msg->position_covariance[0]); // xx
      sqlite3_bind_double(stmt, 6, msg->position_covariance[4]); // yy
      sqlite3_bind_double(stmt, 7, msg->position_covariance[8]); // zz

      if (sqlite3_step(stmt) != SQLITE_DONE) {
        RCLCPP_ERROR(this->get_logger(), "Insert failed: %s", sqlite3_errmsg(db_));
      }
      sqlite3_finalize(stmt);

      // With synchronous=FULL and WAL, commit is durable at step() boundary (per SQLite docs).
      const auto t1 = std::chrono::steady_clock::now();
      const auto us = std::chrono::duration_cast<std::chrono::microseconds>(t1 - it.t0).count();
      std_msgs::msg::Int64 m; m.data = static_cast<int64_t>(us);
      lat_pub_->publish(m);
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
  rclcpp::Subscription<gps_msgs::msg::GPSFix>::SharedPtr sub_;
  sqlite3 *db_{nullptr};
  std::string gps_ssd_dir_;
  std::string gps_topic_;
  std::string db_path_;
  std::string gps_path_;

  int max_queue_{64};
  std::deque<Item> q_;
  std::mutex m_;
  std::condition_variable cv_;
  std::thread worker_thread_;
  std::atomic<bool> stop_{false};
  std::atomic<uint64_t> dropped_{0};
  size_t max_seen_depth_{0};

  // Metrics
  rclcpp::TimerBase::SharedPtr metrics_timer_;
  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr   lat_pub_;
  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr   q_pub_;
  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr   drop_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rss_pub_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GpsSubscriber>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
