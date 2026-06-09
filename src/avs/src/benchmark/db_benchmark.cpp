#include <rocksdb/db.h>
#include <rocksdb/options.h>
#include <sqlite3.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cctype>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <random>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include <gps_msgs/msg/gps_fix.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <yaml-cpp/yaml.h>

#include "avs/append_logger.h"
#include "avs/common.h"
#include "avs/img_dedup.h"
#include "avs/lidar_compress.h"
#include "avs/lidar_downsample.h"
#include "avs/retrieve_api.h"
#include "avs/topic_map.h"

namespace fs = std::filesystem;
using Clock = std::chrono::high_resolution_clock;
using nanoseconds = std::chrono::nanoseconds;

namespace {

enum class SensorId : std::uint8_t {
  kCamera = 0,
  kLidar = 1,
  kGps = 2,
};

struct TopicDefaults {
  std::string camera_topic;
  std::string lidar_topic;
  std::string gps_topic;
};

struct BenchmarkOptions {
  double duration_s = 30.0;
  bool use_camera = true;
  bool use_lidar = true;
  bool use_gps = true;
  std::string camera_topic;
  std::string lidar_topic;
  std::string gps_topic;
  std::string config_path = "/home/avs/AVS-PI/src/avs/config/avs_config.yaml";
  std::string topic_map_path = "/home/avs/AVS-PI/src/avs/config/topics.yaml";
  fs::path out_root = "/home/avs/DATA/SSD/db_benchmark";
  fs::path capture_file;
  fs::path sqlite_path;
  fs::path rocks_dir;
  fs::path append_root;
  std::size_t range_count = 1000;
  std::int64_t window_ms = 1000;
};

struct SensorConfig {
  SensorId id;
  std::string label;
  std::string sensor_type;
  std::string topic;
  std::string folder_name;
  bool enabled = false;
};

struct GpsPayload {
  double latitude;
  double longitude;
  double altitude;
  double cov_xx;
  double cov_yy;
  double cov_zz;
};

struct CapturedRecord {
  SensorId id;
  std::uint64_t ts_ns = 0;
  std::uint64_t payload_offset = 0;
  std::uint32_t payload_size = 0;
  std::uint32_t seq = 0;
};

struct RangeQuery {
  SensorId id;
  std::uint64_t start_ns = 0;
  std::uint64_t end_ns = 0;
};

struct BenchResult {
  std::string backend;
  double avg_insert_ms = 0.0;
  double avg_query_ms = 0.0;
  std::uint64_t size_bytes = 0;
  std::size_t inserted = 0;
  std::size_t query_count = 0;
  std::uint64_t total_rows_scanned = 0;
};

struct CaptureSummary {
  std::uint64_t raw_messages = 0;
  std::uint64_t kept_records = 0;
  std::uint64_t errors = 0;
};

static void Usage(const char* prog) {
  std::cerr
      << "Usage:\n"
      << "  " << prog
      << " --duration <seconds>"
      << " [--sensors camera,lidar,gps]"
      << " [--camera-topic <topic>]"
      << " [--lidar-topic <topic>]"
      << " [--gps-topic <topic>]"
      << " [--config-path <yaml>]"
      << " [--topic-map-path <yaml>]"
      << " [--out-root <dir>]"
      << " [--capture-file <file>]"
      << " [--sqlite <file>]"
      << " [--rocks <dir>]"
      << " [--append-root <dir>]"
      << " [--ranges <n>]"
      << " [--window-ms <ms>]\n\n"
      << "The benchmark subscribes once to the live ROS2 topics, captures the same\n"
      << "processed payload stream, then benchmarks SQLite, RocksDB, and the\n"
      << "append-only logger sequentially on that identical input.\n";
}

static std::string trim(const std::string& in) {
  std::size_t begin = 0;
  while (begin < in.size() && std::isspace(static_cast<unsigned char>(in[begin]))) {
    ++begin;
  }
  std::size_t end = in.size();
  while (end > begin && std::isspace(static_cast<unsigned char>(in[end - 1]))) {
    --end;
  }
  return in.substr(begin, end - begin);
}

static std::vector<std::string> split_csv(const std::string& input) {
  std::vector<std::string> tokens;
  std::string current;
  for (char ch : input) {
    if (ch == ',') {
      const std::string token = trim(current);
      if (!token.empty()) {
        tokens.push_back(token);
      }
      current.clear();
    } else {
      current.push_back(ch);
    }
  }
  const std::string token = trim(current);
  if (!token.empty()) {
    tokens.push_back(token);
  }
  return tokens;
}

static bool load_topics_from_config(const std::string& config_path,
                                    TopicDefaults* out,
                                    std::string* err) {
  try {
    const YAML::Node root = YAML::LoadFile(config_path);
    const YAML::Node common = root["common"];
    if (!common) {
      if (err != nullptr) {
        *err = "missing 'common' section in config";
      }
      return false;
    }

    if (common["img_topic"]) {
      out->camera_topic = common["img_topic"].as<std::string>();
    }
    if (common["lidar_topic"]) {
      out->lidar_topic = common["lidar_topic"].as<std::string>();
    }
    if (common["gps_topic"]) {
      out->gps_topic = common["gps_topic"].as<std::string>();
    }
    return true;
  } catch (const std::exception& e) {
    if (err != nullptr) {
      *err = e.what();
    }
    return false;
  }
}

static bool ParseArgs(int argc, char** argv, BenchmarkOptions* options) {
  std::string sensors_csv = "camera,lidar,gps";

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto next = [&](const char* flag) {
      if (i + 1 >= argc) {
        std::cerr << "Missing value for " << flag << "\n";
        std::exit(2);
      }
      return std::string(argv[++i]);
    };

    if (arg == "--duration") {
      options->duration_s = std::stod(next("--duration"));
    } else if (arg == "--sensors") {
      sensors_csv = next("--sensors");
    } else if (arg == "--camera-topic") {
      options->camera_topic = next("--camera-topic");
    } else if (arg == "--lidar-topic") {
      options->lidar_topic = next("--lidar-topic");
    } else if (arg == "--gps-topic") {
      options->gps_topic = next("--gps-topic");
    } else if (arg == "--config-path") {
      options->config_path = next("--config-path");
    } else if (arg == "--topic-map-path") {
      options->topic_map_path = next("--topic-map-path");
    } else if (arg == "--out-root") {
      options->out_root = next("--out-root");
    } else if (arg == "--capture-file") {
      options->capture_file = next("--capture-file");
    } else if (arg == "--sqlite") {
      options->sqlite_path = next("--sqlite");
    } else if (arg == "--rocks") {
      options->rocks_dir = next("--rocks");
    } else if (arg == "--append-root") {
      options->append_root = next("--append-root");
    } else if (arg == "--ranges") {
      options->range_count = static_cast<std::size_t>(std::stoull(next("--ranges")));
    } else if (arg == "--window-ms") {
      options->window_ms = static_cast<std::int64_t>(std::stoll(next("--window-ms")));
    } else if (arg == "--help" || arg == "-h") {
      return false;
    } else {
      std::cerr << "Unknown arg: " << arg << "\n";
      return false;
    }
  }

  options->use_camera = false;
  options->use_lidar = false;
  options->use_gps = false;
  for (const auto& token : split_csv(sensors_csv)) {
    if (token == "camera") {
      options->use_camera = true;
    } else if (token == "lidar") {
      options->use_lidar = true;
    } else if (token == "gps") {
      options->use_gps = true;
    } else {
      std::cerr << "Unknown sensor in --sensors: " << token << "\n";
      return false;
    }
  }

  if (!options->use_camera && !options->use_lidar && !options->use_gps) {
    std::cerr << "At least one sensor must be enabled.\n";
    return false;
  }
  if (options->duration_s <= 0.0) {
    std::cerr << "--duration must be positive.\n";
    return false;
  }

  if (options->capture_file.empty()) {
    options->capture_file = options->out_root / "capture.payloads";
  }
  if (options->sqlite_path.empty()) {
    options->sqlite_path = options->out_root / "sqlite" / "bench.db";
  }
  if (options->rocks_dir.empty()) {
    options->rocks_dir = options->out_root / "rocksdb";
  }
  if (options->append_root.empty()) {
    options->append_root = options->out_root / "append_only";
  }

  return true;
}

static std::size_t sensor_index(SensorId id) {
  return static_cast<std::size_t>(id);
}

static std::string default_folder_name(SensorId id) {
  switch (id) {
    case SensorId::kCamera:
      return "bench_camera";
    case SensorId::kLidar:
      return "bench_lidar";
    case SensorId::kGps:
      return "bench_gps";
  }
  return "bench_unknown";
}

static std::array<SensorConfig, 3> BuildSensorConfigs(const BenchmarkOptions& options) {
  TopicDefaults defaults;
  std::string load_err;
  if (!load_topics_from_config(options.config_path, &defaults, &load_err)) {
    throw std::runtime_error("Failed to load topic defaults: " + load_err);
  }

  const avs::TopicMap topic_map = avs::LoadTopicMap(options.topic_map_path);

  std::array<SensorConfig, 3> sensors{{
      {SensorId::kCamera, "camera", "image",
       options.camera_topic.empty() ? defaults.camera_topic : options.camera_topic, "", options.use_camera},
      {SensorId::kLidar, "lidar", "lidar",
       options.lidar_topic.empty() ? defaults.lidar_topic : options.lidar_topic, "", options.use_lidar},
      {SensorId::kGps, "gps", "gps",
       options.gps_topic.empty() ? defaults.gps_topic : options.gps_topic, "", options.use_gps},
  }};

  for (auto& sensor : sensors) {
    if (!sensor.enabled) {
      continue;
    }
    if (sensor.topic.empty()) {
      throw std::runtime_error("Configured topic is empty for sensor " + sensor.label);
    }
    sensor.folder_name = avs::GetTopicFolder(topic_map, sensor.topic);
    if (sensor.folder_name.empty()) {
      sensor.folder_name = default_folder_name(sensor.id);
    }
  }

  return sensors;
}

static double ns_avg_ms(const std::vector<nanoseconds>& times) {
  if (times.empty()) {
    return 0.0;
  }
  long double total = 0;
  for (const auto& time : times) {
    total += time.count();
  }
  return static_cast<double>(total / times.size()) / 1e6;
}

static std::uint64_t file_size_if_exists(const fs::path& path) {
  std::error_code ec;
  if (fs::exists(path, ec) && fs::is_regular_file(path, ec)) {
    return static_cast<std::uint64_t>(fs::file_size(path, ec));
  }
  return 0;
}

static std::uint64_t dir_size_recursive(const fs::path& root) {
  std::error_code ec;
  if (!fs::exists(root, ec)) {
    return 0;
  }

  std::uint64_t total = 0;
  for (const auto& entry : fs::recursive_directory_iterator(root, ec)) {
    if (ec) {
      break;
    }
    if (entry.is_regular_file()) {
      total += static_cast<std::uint64_t>(fs::file_size(entry.path(), ec));
      if (ec) {
        ec.clear();
      }
    }
  }
  return total;
}

static std::string rocks_key(SensorId id, std::uint64_t ts_ns, std::uint32_t seq) {
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%02u:%020llu:%010u",
                static_cast<unsigned>(id),
                static_cast<unsigned long long>(ts_ns),
                seq);
  return std::string(buf);
}

class CaptureStore {
 public:
  explicit CaptureStore(const fs::path& capture_file) {
    std::error_code ec;
    fs::create_directories(capture_file.parent_path(), ec);
    stream_.open(capture_file, std::ios::binary | std::ios::trunc);
    if (!stream_.is_open()) {
      throw std::runtime_error("Failed to open capture file: " + capture_file.string());
    }
  }

  bool append(SensorId id, std::uint64_t ts_ns, const std::vector<std::uint8_t>& payload) {
    std::lock_guard<std::mutex> lk(mu_);
    if (!stream_.is_open()) {
      return false;
    }

    const auto idx = sensor_index(id);
    const std::uint64_t offset = static_cast<std::uint64_t>(stream_.tellp());
    if (!payload.empty()) {
      stream_.write(reinterpret_cast<const char*>(payload.data()),
                    static_cast<std::streamsize>(payload.size()));
      if (!stream_) {
        return false;
      }
    }

    records_.push_back(CapturedRecord{
        id,
        ts_ns,
        offset,
        static_cast<std::uint32_t>(payload.size()),
        next_seq_[idx]++,
    });
    ++summary_[idx].kept_records;
    return true;
  }

  void count_raw(SensorId id) {
    std::lock_guard<std::mutex> lk(mu_);
    ++summary_[sensor_index(id)].raw_messages;
  }

  void count_error(SensorId id) {
    std::lock_guard<std::mutex> lk(mu_);
    ++summary_[sensor_index(id)].errors;
  }

  const std::vector<CapturedRecord>& records() const { return records_; }
  const std::array<CaptureSummary, 3>& summary() const { return summary_; }

 private:
  mutable std::mutex mu_;
  std::ofstream stream_;
  std::vector<CapturedRecord> records_;
  std::array<std::uint32_t, 3> next_seq_{{0, 0, 0}};
  std::array<CaptureSummary, 3> summary_{};
};

static bool ReadPayloadAt(std::ifstream* input,
                          std::uint64_t offset,
                          std::uint32_t size,
                          std::vector<std::uint8_t>* out) {
  out->clear();
  input->clear();
  if (size == 0) {
    return true;
  }
  input->seekg(static_cast<std::streamoff>(offset), std::ios::beg);
  out->resize(size);
  input->read(reinterpret_cast<char*>(out->data()), size);
  return static_cast<bool>(*input);
}

class TopicCaptureNode : public rclcpp::Node {
 public:
  TopicCaptureNode(const BenchmarkOptions& options,
                   const std::array<SensorConfig, 3>& sensors,
                   std::shared_ptr<CaptureStore> capture_store)
      : Node("db_benchmark_capture_node"),
        sensors_(sensors),
        capture_store_(std::move(capture_store)),
        deduplicator_(std::make_shared<avs::ImgDeduplicator>(
            (options.out_root / "capture_stage" / "image").string(),
            options.config_path)),
        downsampler_(std::make_shared<avs::LidarDownsampler>(options.config_path)),
        compressor_(std::make_shared<avs::LidarCompressor>(
            (options.out_root / "capture_stage" / "lidar").string())) {
    loadGpsConfig(options.config_path);

    if (sensors_[sensor_index(SensorId::kCamera)].enabled) {
      image_sub_ = create_subscription<sensor_msgs::msg::Image>(
          sensors_[sensor_index(SensorId::kCamera)].topic,
          rclcpp::SensorDataQoS(),
          std::bind(&TopicCaptureNode::onImage, this, std::placeholders::_1));
    }
    if (sensors_[sensor_index(SensorId::kLidar)].enabled) {
      lidar_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
          sensors_[sensor_index(SensorId::kLidar)].topic,
          rclcpp::SensorDataQoS(),
          std::bind(&TopicCaptureNode::onLidar, this, std::placeholders::_1));
    }
    if (sensors_[sensor_index(SensorId::kGps)].enabled) {
      gps_sub_ = create_subscription<gps_msgs::msg::GPSFix>(
          sensors_[sensor_index(SensorId::kGps)].topic,
          rclcpp::SensorDataQoS(),
          std::bind(&TopicCaptureNode::onGps, this, std::placeholders::_1));
    }
  }

 private:
  void loadGpsConfig(const std::string& config_path) {
    const YAML::Node root = YAML::LoadFile(config_path);
    const YAML::Node gps_cfg = root["gps_downscale"];
    gps_downscale_enabled_ = gps_cfg && gps_cfg["open"] ? gps_cfg["open"].as<bool>() : false;
    gps_downscale_frequency_ =
        gps_cfg && gps_cfg["frequency"] ? gps_cfg["frequency"].as<int>() : 5;
    if (gps_downscale_frequency_ <= 0) {
      gps_downscale_frequency_ = 1;
    }
  }

  std::uint64_t now_ns() const {
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch())
            .count());
  }

  void onImage(const sensor_msgs::msg::Image::SharedPtr msg) {
    capture_store_->count_raw(SensorId::kCamera);

    try {
      std::vector<std::uint8_t> payload;
      if (!deduplicator_->isUniqueAndGetBytes(*msg, payload)) {
        return;
      }
      if (!capture_store_->append(SensorId::kCamera, now_ns(), payload)) {
        capture_store_->count_error(SensorId::kCamera);
      }
    } catch (...) {
      capture_store_->count_error(SensorId::kCamera);
    }
  }

  void onLidar(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
    capture_store_->count_raw(SensorId::kLidar);

    try {
      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
      pcl::fromROSMsg(*msg, *cloud);

      pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled = downsampler_->downsample(cloud);
      std::vector<std::uint8_t> payload;
      compressor_->getLAZ(downsampled, payload);
      if (!capture_store_->append(SensorId::kLidar, now_ns(), payload)) {
        capture_store_->count_error(SensorId::kLidar);
      }
    } catch (...) {
      capture_store_->count_error(SensorId::kLidar);
    }
  }

  void onGps(const gps_msgs::msg::GPSFix::SharedPtr msg) {
    capture_store_->count_raw(SensorId::kGps);

    if (gps_downscale_enabled_) {
      ++gps_record_counter_;
      if (gps_record_counter_ % gps_downscale_frequency_ != 0) {
        return;
      }
      gps_record_counter_ = 0;
    }

    try {
      GpsPayload payload{};
      payload.latitude = msg->latitude;
      payload.longitude = msg->longitude;
      payload.altitude = msg->altitude;
      payload.cov_xx = msg->position_covariance[0];
      payload.cov_yy = msg->position_covariance[4];
      payload.cov_zz = msg->position_covariance[8];

      std::vector<std::uint8_t> bytes(sizeof(GpsPayload));
      std::memcpy(bytes.data(), &payload, sizeof(GpsPayload));

      if (!capture_store_->append(SensorId::kGps, now_ns(), bytes)) {
        capture_store_->count_error(SensorId::kGps);
      }
    } catch (...) {
      capture_store_->count_error(SensorId::kGps);
    }
  }

  std::array<SensorConfig, 3> sensors_;
  std::shared_ptr<CaptureStore> capture_store_;

  std::shared_ptr<avs::ImgDeduplicator> deduplicator_;
  std::shared_ptr<avs::LidarDownsampler> downsampler_;
  std::shared_ptr<avs::LidarCompressor> compressor_;

  bool gps_downscale_enabled_ = false;
  int gps_downscale_frequency_ = 1;
  int gps_record_counter_ = 0;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Subscription<gps_msgs::msg::GPSFix>::SharedPtr gps_sub_;
};

static std::vector<RangeQuery> BuildRangeQueries(const std::vector<CapturedRecord>& records,
                                                 std::size_t count,
                                                 std::int64_t window_ms) {
  std::vector<RangeQuery> queries;
  if (records.empty() || count == 0) {
    return queries;
  }

  const std::uint64_t half_window_ns =
      static_cast<std::uint64_t>(window_ms > 0 ? window_ms : 0) * 1000000ULL / 2ULL;
  std::mt19937_64 rng(0xC0FFEE);
  std::uniform_int_distribution<std::size_t> dist(0, records.size() - 1);

  queries.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    const CapturedRecord& record = records[dist(rng)];
    const std::uint64_t start_ns =
        record.ts_ns > half_window_ns ? record.ts_ns - half_window_ns : 0;
    const std::uint64_t end_ns = record.ts_ns + half_window_ns;
    queries.push_back(RangeQuery{record.id, start_ns, end_ns});
  }
  return queries;
}

static const SensorConfig& SensorById(const std::array<SensorConfig, 3>& sensors, SensorId id) {
  return sensors[sensor_index(id)];
}

static BenchResult BenchSQLite(const std::vector<CapturedRecord>& records,
                               const std::array<SensorConfig, 3>& sensors,
                               const fs::path& capture_file,
                               const fs::path& db_path,
                               const std::vector<RangeQuery>& ranges) {
  BenchResult result;
  result.backend = "SQLite";

  std::error_code ec;
  fs::create_directories(db_path.parent_path(), ec);
  fs::remove(db_path, ec);
  fs::remove(db_path.string() + "-wal", ec);
  fs::remove(db_path.string() + "-shm", ec);

  sqlite3* db = nullptr;
  if (sqlite3_open_v2(db_path.string().c_str(), &db,
                      SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE, nullptr) != SQLITE_OK) {
    std::cerr << "[SQLite] open error: " << sqlite3_errmsg(db) << "\n";
    return result;
  }

  auto exec_sql = [&](const char* sql) {
    char* err = nullptr;
    if (sqlite3_exec(db, sql, nullptr, nullptr, &err) != SQLITE_OK) {
      if (err != nullptr) {
        std::cerr << "[SQLite] exec error: " << err << "\n";
        sqlite3_free(err);
      }
    }
  };

  exec_sql("PRAGMA journal_mode=WAL;");
  exec_sql("PRAGMA synchronous=NORMAL;");
  exec_sql("PRAGMA temp_store=MEMORY;");
  exec_sql("PRAGMA mmap_size=268435456;");
  exec_sql("CREATE TABLE IF NOT EXISTS entries ("
           " topic_id INTEGER NOT NULL,"
           " sensor_topic TEXT NOT NULL,"
           " ts INTEGER NOT NULL,"
           " seq INTEGER NOT NULL,"
           " payload BLOB NOT NULL,"
           " PRIMARY KEY(topic_id, ts, seq)"
           ");");

  sqlite3_stmt* insert_stmt = nullptr;
  if (sqlite3_prepare_v2(
          db,
          "INSERT INTO entries(topic_id, sensor_topic, ts, seq, payload) VALUES (?, ?, ?, ?, ?);",
          -1,
          &insert_stmt,
          nullptr) != SQLITE_OK) {
    std::cerr << "[SQLite] prepare insert error: " << sqlite3_errmsg(db) << "\n";
    sqlite3_close(db);
    return result;
  }

  std::ifstream capture(capture_file, std::ios::binary);
  if (!capture.is_open()) {
    std::cerr << "[SQLite] failed to open capture file: " << capture_file << "\n";
    sqlite3_finalize(insert_stmt);
    sqlite3_close(db);
    return result;
  }

  exec_sql("BEGIN TRANSACTION;");
  std::vector<nanoseconds> insert_times;
  insert_times.reserve(records.size());
  std::vector<std::uint8_t> payload;

  for (const auto& record : records) {
    if (!ReadPayloadAt(&capture, record.payload_offset, record.payload_size, &payload)) {
      std::cerr << "[SQLite] failed to read capture payload at offset "
                << record.payload_offset << "\n";
      continue;
    }

    const auto& sensor = SensorById(sensors, record.id);
    sqlite3_reset(insert_stmt);
    sqlite3_clear_bindings(insert_stmt);
    sqlite3_bind_int(insert_stmt, 1, static_cast<int>(record.id));
    sqlite3_bind_text(insert_stmt, 2, sensor.topic.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_int64(insert_stmt, 3, static_cast<sqlite3_int64>(record.ts_ns));
    sqlite3_bind_int(insert_stmt, 4, static_cast<int>(record.seq));
    sqlite3_bind_blob(insert_stmt, 5, payload.data(),
                      static_cast<int>(payload.size()), SQLITE_TRANSIENT);

    const auto t0 = Clock::now();
    const int rc = sqlite3_step(insert_stmt);
    const auto t1 = Clock::now();
    if (rc != SQLITE_DONE) {
      std::cerr << "[SQLite] insert error: " << sqlite3_errmsg(db) << "\n";
      continue;
    }

    insert_times.push_back(std::chrono::duration_cast<nanoseconds>(t1 - t0));
    ++result.inserted;
  }
  exec_sql("COMMIT;");
  sqlite3_finalize(insert_stmt);

  sqlite3_stmt* query_stmt = nullptr;
  if (sqlite3_prepare_v2(
          db,
          "SELECT payload FROM entries WHERE topic_id = ? AND ts BETWEEN ? AND ?;",
          -1,
          &query_stmt,
          nullptr) != SQLITE_OK) {
    std::cerr << "[SQLite] prepare select error: " << sqlite3_errmsg(db) << "\n";
    sqlite3_close(db);
    return result;
  }

  std::vector<nanoseconds> query_times;
  query_times.reserve(ranges.size());

  for (const auto& range : ranges) {
    sqlite3_reset(query_stmt);
    sqlite3_clear_bindings(query_stmt);
    sqlite3_bind_int(query_stmt, 1, static_cast<int>(range.id));
    sqlite3_bind_int64(query_stmt, 2, static_cast<sqlite3_int64>(range.start_ns));
    sqlite3_bind_int64(query_stmt, 3, static_cast<sqlite3_int64>(range.end_ns));

    std::uint64_t rows = 0;
    const auto t0 = Clock::now();
    while (sqlite3_step(query_stmt) == SQLITE_ROW) {
      (void)sqlite3_column_bytes(query_stmt, 0);
      ++rows;
    }
    const auto t1 = Clock::now();
    query_times.push_back(std::chrono::duration_cast<nanoseconds>(t1 - t0));
    result.total_rows_scanned += rows;
    ++result.query_count;
  }

  sqlite3_finalize(query_stmt);
  sqlite3_close(db);

  result.size_bytes = file_size_if_exists(db_path) +
                      file_size_if_exists(db_path.string() + "-wal") +
                      file_size_if_exists(db_path.string() + "-shm");
  result.avg_insert_ms = ns_avg_ms(insert_times);
  result.avg_query_ms = ns_avg_ms(query_times);
  return result;
}

static BenchResult BenchRocksDb(const std::vector<CapturedRecord>& records,
                                const fs::path& capture_file,
                                const fs::path& db_dir,
                                const std::vector<RangeQuery>& ranges) {
  BenchResult result;
  result.backend = "RocksDB";

  std::error_code ec;
  if (fs::exists(db_dir, ec)) {
    fs::remove_all(db_dir, ec);
  }
  fs::create_directories(db_dir.parent_path(), ec);

  rocksdb::Options options;
  options.create_if_missing = true;
  options.compression = rocksdb::kLZ4Compression;

  rocksdb::DB* db = nullptr;
  const auto open_status = rocksdb::DB::Open(options, db_dir.string(), &db);
  if (!open_status.ok()) {
    std::cerr << "[RocksDB] open error: " << open_status.ToString() << "\n";
    return result;
  }

  std::ifstream capture(capture_file, std::ios::binary);
  if (!capture.is_open()) {
    std::cerr << "[RocksDB] failed to open capture file: " << capture_file << "\n";
    delete db;
    return result;
  }

  rocksdb::WriteOptions write_options;
  write_options.sync = false;
  rocksdb::ReadOptions read_options;

  std::vector<nanoseconds> insert_times;
  insert_times.reserve(records.size());
  std::vector<std::uint8_t> payload;

  for (const auto& record : records) {
    if (!ReadPayloadAt(&capture, record.payload_offset, record.payload_size, &payload)) {
      std::cerr << "[RocksDB] failed to read capture payload at offset "
                << record.payload_offset << "\n";
      continue;
    }

    const std::string key = rocks_key(record.id, record.ts_ns, record.seq);
    const auto t0 = Clock::now();
    const auto status =
        db->Put(write_options, key,
                rocksdb::Slice(reinterpret_cast<const char*>(payload.data()), payload.size()));
    const auto t1 = Clock::now();
    if (!status.ok()) {
      std::cerr << "[RocksDB] put error: " << status.ToString() << "\n";
      continue;
    }

    insert_times.push_back(std::chrono::duration_cast<nanoseconds>(t1 - t0));
    ++result.inserted;
  }

  std::vector<nanoseconds> query_times;
  query_times.reserve(ranges.size());

  for (const auto& range : ranges) {
    const std::string start_key = rocks_key(range.id, range.start_ns, 0);
    const std::string end_key = rocks_key(range.id, range.end_ns, std::numeric_limits<std::uint32_t>::max());
    char prefix_buf[8];
    std::snprintf(prefix_buf, sizeof(prefix_buf), "%02u:", static_cast<unsigned>(range.id));
    const std::string prefix(prefix_buf);

    std::unique_ptr<rocksdb::Iterator> it(db->NewIterator(read_options));
    std::uint64_t rows = 0;
    const auto t0 = Clock::now();
    for (it->Seek(start_key); it->Valid(); it->Next()) {
      const rocksdb::Slice key = it->key();
      if (key.size() < prefix.size() ||
          std::memcmp(key.data(), prefix.data(), prefix.size()) != 0) {
        break;
      }
      if (key.ToString() > end_key) {
        break;
      }
      (void)it->value().size();
      ++rows;
    }
    const auto t1 = Clock::now();
    query_times.push_back(std::chrono::duration_cast<nanoseconds>(t1 - t0));
    result.total_rows_scanned += rows;
    ++result.query_count;
  }

  delete db;

  result.size_bytes = dir_size_recursive(db_dir);
  result.avg_insert_ms = ns_avg_ms(insert_times);
  result.avg_query_ms = ns_avg_ms(query_times);
  return result;
}

static BenchResult BenchAppendOnly(const std::vector<CapturedRecord>& records,
                                   const std::array<SensorConfig, 3>& sensors,
                                   const fs::path& capture_file,
                                   const fs::path& append_root,
                                   const std::vector<RangeQuery>& ranges) {
  BenchResult result;
  result.backend = "AppendOnly";

  std::error_code ec;
  if (fs::exists(append_root, ec)) {
    fs::remove_all(append_root, ec);
  }
  fs::create_directories(append_root, ec);

  std::ifstream capture(capture_file, std::ios::binary);
  if (!capture.is_open()) {
    std::cerr << "[AppendOnly] failed to open capture file: " << capture_file << "\n";
    return result;
  }

  const std::string day = avs::getCurrentDayFolder();
  std::unordered_map<int, std::shared_ptr<avs::AppendLogger>> loggers;
  std::unordered_map<int, std::uint64_t> last_ts_by_sensor;

  auto logger_for = [&](SensorId id, std::uint64_t start_ts_ns) {
    const int key = static_cast<int>(id);
    auto it = loggers.find(key);
    if (it != loggers.end()) {
      return it->second;
    }

    const auto& sensor = SensorById(sensors, id);
    auto logger = std::make_shared<avs::AppendLogger>(append_root.string(), sensor.topic);
    logger->startTrip(day, sensor.folder_name, 0, start_ts_ns);
    loggers.emplace(key, logger);
    return logger;
  };

  std::vector<nanoseconds> insert_times;
  insert_times.reserve(records.size());
  std::vector<std::uint8_t> payload;

  for (const auto& record : records) {
    if (!ReadPayloadAt(&capture, record.payload_offset, record.payload_size, &payload)) {
      std::cerr << "[AppendOnly] failed to read capture payload at offset "
                << record.payload_offset << "\n";
      continue;
    }

    auto logger = logger_for(record.id, record.ts_ns);
    const auto t0 = Clock::now();
    logger->appendRecord(record.ts_ns, payload);
    const auto t1 = Clock::now();

    insert_times.push_back(std::chrono::duration_cast<nanoseconds>(t1 - t0));
    ++result.inserted;
    last_ts_by_sensor[static_cast<int>(record.id)] = record.ts_ns;
  }

  for (auto& entry : loggers) {
    entry.second->endTrip(last_ts_by_sensor[entry.first]);
  }

  avs::RetrieveAPI api(append_root);
  std::vector<nanoseconds> query_times;
  query_times.reserve(ranges.size());

  for (const auto& range : ranges) {
    const auto& sensor = SensorById(sensors, range.id);
    const auto t0 = Clock::now();
    std::string err;
    const auto refs = api.QueryRefs(sensor.topic, range.start_ns, range.end_ns, &err);
    if (!err.empty() && refs.empty()) {
      std::cerr << "[AppendOnly] query error: " << err << "\n";
      continue;
    }

    std::uint64_t rows = 0;
    std::vector<std::uint8_t> payload_out;
    for (const auto& ref : refs) {
      std::string load_err;
      if (!api.LoadPayload(ref, payload_out, &load_err)) {
        std::cerr << "[AppendOnly] payload read error: " << load_err << "\n";
        continue;
      }
      ++rows;
    }

    const auto t1 = Clock::now();
    query_times.push_back(std::chrono::duration_cast<nanoseconds>(t1 - t0));
    result.total_rows_scanned += rows;
    ++result.query_count;
  }

  result.size_bytes = dir_size_recursive(append_root);
  result.avg_insert_ms = ns_avg_ms(insert_times);
  result.avg_query_ms = ns_avg_ms(query_times);
  return result;
}

static void PrintCaptureSummary(const std::array<SensorConfig, 3>& sensors,
                                const std::array<CaptureSummary, 3>& summary) {
  std::cout << "Capture summary:\n";
  for (const auto& sensor : sensors) {
    if (!sensor.enabled) {
      continue;
    }
    const auto& item = summary[sensor_index(sensor.id)];
    std::cout << "  " << sensor.label
              << " raw=" << item.raw_messages
              << " kept=" << item.kept_records
              << " errors=" << item.errors
              << " topic=" << sensor.topic << "\n";
  }
}

static void PrintMarkdownTable(const std::vector<BenchResult>& results) {
  auto mb = [](std::uint64_t bytes) { return bytes / (1024.0 * 1024.0); };

  std::cout << std::fixed << std::setprecision(4);
  std::cout << "\n| Backend | Inserted | Avg Insert Latency (ms) | Avg Query Latency (ms) | Final Size (MB) |\n";
  std::cout << "|---|---:|---:|---:|---:|\n";
  for (const auto& result : results) {
    std::cout << "| " << result.backend
              << " | " << result.inserted
              << " | " << result.avg_insert_ms
              << " | " << result.avg_query_ms
              << " | " << mb(result.size_bytes)
              << " |\n";
  }
}

}  // namespace

int main(int argc, char** argv) {
  BenchmarkOptions options;
  if (!ParseArgs(argc, argv, &options)) {
    Usage(argv[0]);
    return 1;
  }

  try {
    const auto sensors = BuildSensorConfigs(options);

    std::error_code ec;
    fs::create_directories(options.out_root, ec);

    rclcpp::init(argc, argv);
    auto capture_store = std::make_shared<CaptureStore>(options.capture_file);
    auto node = std::make_shared<TopicCaptureNode>(options, sensors, capture_store);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    const auto capture_start = std::chrono::steady_clock::now();
    while (rclcpp::ok()) {
      executor.spin_some();
      const auto elapsed = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - capture_start);
      if (elapsed.count() >= options.duration_s) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    executor.remove_node(node);
    rclcpp::shutdown();

    const auto& records = capture_store->records();
    if (records.empty()) {
      std::cerr << "No records were captured from the configured ROS2 topics.\n";
      return 1;
    }

    const auto ranges = BuildRangeQueries(records, options.range_count, options.window_ms);
    if (ranges.empty()) {
      std::cerr << "No range queries were generated from the captured records.\n";
      return 1;
    }

    std::cout << "Captured records: " << records.size() << "\n";
    std::cout << "Range queries: " << ranges.size() << "\n";
    PrintCaptureSummary(sensors, capture_store->summary());

    std::vector<BenchResult> results;
    results.push_back(BenchSQLite(records, sensors, options.capture_file, options.sqlite_path, ranges));
    results.push_back(BenchRocksDb(records, options.capture_file, options.rocks_dir, ranges));
    results.push_back(BenchAppendOnly(records, sensors, options.capture_file, options.append_root, ranges));

    PrintMarkdownTable(results);
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "db_benchmark failed: " << e.what() << "\n";
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    return 1;
  }
}
