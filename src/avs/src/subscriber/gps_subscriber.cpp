#include "rclcpp/rclcpp.hpp"
#include "gps_msgs/msg/gps_fix.hpp" 
#include "std_msgs/msg/int64.hpp"
#include "yaml-cpp/yaml.h"
#include "avs/common.h"
#include <sqlite3.h>
#include <filesystem>
#include <chrono>
#include <string>
#include <iomanip>
#include <sstream>

namespace fs = std::filesystem;

class GpsSubscriber : public rclcpp::Node {
public:
  GpsSubscriber()
  : Node("gps_subscriber")
  {
    this->declare_parameter<std::string>("config_path", "/home/avs/AVS-PI/src/avs/config/avs_config.yaml");
     std::string config_path = this->get_parameter("config_path").as_string();

    YAML::Node root = YAML::LoadFile(config_path);
    YAML::Node common = root["common"];
    gps_topic_ = common["gps_topic"].as<std::string>();
    gps_ssd_dir_ = common["gps_ssd_dir"].as<std::string>();

    if (!avs::ensureDirectory(gps_ssd_dir_, &ec)) {
      RCLCPP_WARN(this->get_logger(),
                  "GPS directory %s did not exist. Attempted to create it (ec=%d: %s).",
                  gps_ssd_dir_.c_str(),
                  ec.value(),
                  ec.message().c_str());
    }

    // Open SQLite DB
    openDatabase();

    subscription_ = this->create_subscription<gps_msgs::msg::GPSFix>(
      gps_topic_, make_auto_qos(gps_topic_),
      std::bind(&GpsSubscriber::gpsCallback, this, std::placeholders::_1));
    
    latency_pub_ = this->create_publisher<std_msgs::msg::Int64>("/avs/record_latency_us", 10);

    RCLCPP_INFO(this->get_logger(),
                "AVS GPS node started. Subscribed to %s; saving to %s",
                gps_topic_.c_str(), db_path_.c_str());
  }

  ~GpsSubscriber() {
    if (db_) sqlite3_close(db_);
  }

private:
  rclcpp::Subscription<gps_msgs::msg::GPSFix>::SharedPtr sub_;
  sqlite3 *db_{nullptr};
  std::string gps_ssd_dir_;
  std::string gps_topic_;
  std::string db_path_;
  std::string gps_path_;

  std::error_code ec;

  rclcpp::Subscription<gps_msgs::msg::GPSFix>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr latency_pub_;


  void openDatabase() {
    // Create directory
    gps_path_ = (fs::path(gps_ssd_dir_) / avs::getCurrentDayFolder()).string();
    db_path_ = gps_path_ + ".sqlite3";

    if (sqlite3_open(db_path_.c_str(), &db_) != SQLITE_OK) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open DB: %s", sqlite3_errmsg(db_));
      return;
    }

    const char *sql =
      "CREATE TABLE IF NOT EXISTS gps_data ("
      "ts_ms INTEGER PRIMARY KEY, "
      "latitude REAL, "
      "longitude REAL, "
      "altitude REAL, "
      "cov_xx REAL, "
      "cov_yy REAL, "
      "cov_zz REAL"
      ");";

    char *errmsg;
    if (sqlite3_exec(db_, sql, nullptr, nullptr, &errmsg) != SQLITE_OK) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create table: %s", errmsg);
      sqlite3_free(errmsg);
    }
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

  void gpsCallback(const gps_msgs::msg::GPSFix::SharedPtr msg) {
    if (!db_) return;

    const auto t0 = std::chrono::steady_clock::now();

    sqlite3_stmt *stmt;
    const char *sql =
      "INSERT INTO gps_data (ts_ms, latitude, longitude, altitude, cov_xx, cov_yy, cov_zz) "
      "VALUES (?, ?, ?, ?, ?, ?, ?);";

    if (sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr) != SQLITE_OK) {
      RCLCPP_ERROR(this->get_logger(), "Prepare failed: %s", sqlite3_errmsg(db_));
      return;
    }

    long long ts_ms = msg->header.stamp.sec * 1000LL + msg->header.stamp.nanosec / 1000000LL;

    sqlite3_bind_int64(stmt, 1, ts_ms);
    sqlite3_bind_double(stmt, 2, msg->latitude);
    sqlite3_bind_double(stmt, 3, msg->longitude);
    sqlite3_bind_double(stmt, 4, msg->altitude);

    // position_covariance is a 9-element array (row-major)
    sqlite3_bind_double(stmt, 5, msg->position_covariance[0]); // xx
    sqlite3_bind_double(stmt, 6, msg->position_covariance[4]); // yy
    sqlite3_bind_double(stmt, 7, msg->position_covariance[8]); // zz

    if (sqlite3_step(stmt) != SQLITE_DONE) {
      RCLCPP_ERROR(this->get_logger(), "Insert failed: %s", sqlite3_errmsg(db_));
    }
    sqlite3_finalize(stmt);

    const auto t1 = std::chrono::steady_clock::now();
    const auto latency_us = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();
    std_msgs::msg::Int64 m;
    m.data = latency_us;
    latency_pub_->publish(m);
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GpsSubscriber>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
