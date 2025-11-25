// append logger implementation : segment(chunk(messages...), chunk(messages...), ...)
#include "avs/append_logger.h"

#include <filesystem>
#include <system_error>
#include <iostream>
#include <cstring>
#include <stdexcept>

namespace fs = std::filesystem;

namespace avs {

AppendLogger::AppendLogger(const std::string &ssd_root, const std::string &topic)
  : ssd_root_(ssd_root), topic_(topic)
{
  openGlobalDB();
  ensureGlobalSchema();
}

AppendLogger::~AppendLogger()
{
  try {
    std::lock_guard<std::mutex> lk(mu_);
    if (trip_id_ != -1) {
      flushChunkLocked();
    }
  } catch(...) {}
  if (ssd_db_) sqlite3_close(ssd_db_);
}

void AppendLogger::openGlobalDB()
{
  fs::path dbpath = fs::path(ssd_root_) / "global.sqlite3";
  int rc = sqlite3_open(dbpath.c_str(), &ssd_db_);
  if (rc != SQLITE_OK) {
    std::string err = sqlite3_errmsg(ssd_db_) ? sqlite3_errmsg(ssd_db_) : "unknown";
    if (ssd_db_) sqlite3_close(ssd_db_);
    ssd_db_ = nullptr;
    throw std::runtime_error("sqlite3_open failed: " + err);
  }
}

void AppendLogger::ensureGlobalSchema()
{
  const char *sql =
    "CREATE TABLE IF NOT EXISTS global ("
    "  sensor_topic TEXT NOT NULL,"
    "  day TEXT NOT NULL,"
    "  trip_id INTEGER NOT NULL,"
    "  start_ts_ns TEXT NOT NULL,"
    "  end_ts_ns TEXT NOT NULL,"
    "  PRIMARY KEY(sensor_topic, day, trip_id)"
    ");";
  char *errmsg = nullptr;
  int rc = sqlite3_exec(ssd_db_, sql, nullptr, nullptr, &errmsg);
  if (rc != SQLITE_OK) {
    std::string e = errmsg ? errmsg : "unknown";
    sqlite3_free(errmsg);
    throw std::runtime_error("Failed to create global schema: " + e);
  }
}

void AppendLogger::insertGlobalRow(const std::string &day, int trip_id, uint64_t start_ts_ns)
{
  const char *sql = "INSERT OR REPLACE INTO global(sensor_topic, day, trip_id, start_ts_ns, end_ts_ns) VALUES(?, ?, ?, ?, ?);";
  sqlite3_stmt *stmt = nullptr;
  int rc = sqlite3_prepare_v2(ssd_db_, sql, -1, &stmt, nullptr);
  if (rc != SQLITE_OK) {
    throw std::runtime_error("sqlite3_prepare_v2 failed (insert)");
  }
  sqlite3_bind_text(stmt, 1, topic_.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(stmt, 2, day.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_int(stmt, 3, trip_id);
  sqlite3_bind_text(stmt, 4, std::to_string(start_ts_ns).c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(stmt, 5, "0", -1, SQLITE_TRANSIENT);

  rc = sqlite3_step(stmt);
  if (rc != SQLITE_DONE) {
    sqlite3_finalize(stmt);
    throw std::runtime_error("sqlite3_step insert failed");
  }
  sqlite3_finalize(stmt);
}

void AppendLogger::updateGlobalRowEnd(const std::string &day, int trip_id, uint64_t end_ts_ns)
{
  const char *sql = "UPDATE global SET end_ts_ns = ? WHERE sensor_topic = ? AND day = ? AND trip_id = ?;";
  sqlite3_stmt *stmt = nullptr;
  int rc = sqlite3_prepare_v2(ssd_db_, sql, -1, &stmt, nullptr);
  if (rc != SQLITE_OK) throw std::runtime_error("sqlite3_prepare_v2 failed (update)");
  sqlite3_bind_text(stmt, 1, std::to_string(end_ts_ns).c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(stmt, 2, topic_.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(stmt, 3, day.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_int(stmt, 4, trip_id);

  rc = sqlite3_step(stmt);
  if (rc != SQLITE_DONE) {
    sqlite3_finalize(stmt);
    throw std::runtime_error("sqlite3_step update failed");
  }
  sqlite3_finalize(stmt);
}

void AppendLogger::startTrip(const std::string &day, const std::string &topic_folder, int trip_id, uint64_t start_ts_ns)
{
  std::lock_guard<std::mutex> lk(mu_);
  if (trip_id_ != -1) throw std::runtime_error("Trip already active");

  day_ = day;
  trip_id_ = trip_id;
  topic_folder_ = topic_folder;

  fs::path daydir = fs::path(ssd_root_) / topic_folder_ / day_;
  std::error_code ec;
  fs::create_directories(daydir, ec);
  if (ec) throw std::system_error(ec);

  openTripFiles();

  TripHeader th;
  std::memset(&th, 0, sizeof(th));
  const char *magic = "AVS_TRIP_v1";
  std::strncpy(th.magic, magic, sizeof(th.magic)-1);
  th.trip_start_ts_ns = start_ts_ns;
  th.reserved = 0;
  trip_log_.write(reinterpret_cast<const char*>(&th), sizeof(th));
  trip_log_.flush();

  trip_idx_.seekp(0, std::ios::end);

  insertGlobalRow(day_, trip_id_, start_ts_ns);

  chunk_buf_.clear();
  chunk_start_ts_ns_ = 0;
  chunk_end_ts_ns_ = 0;
  chunk_record_count_ = 0;
  trip_index_.clear();
}

void AppendLogger::openTripFiles()
{
  fs::path daydir = fs::path(ssd_root_) / topic_folder_ / day_;
  char tb[64];
  snprintf(tb, sizeof(tb), "trip_%02d.log", trip_id_);
  fs::path logp = daydir / tb;
  snprintf(tb, sizeof(tb), "trip_%02d.idx", trip_id_);
  fs::path idxp = daydir / tb;

  trip_log_.open(logp, std::ios::binary | std::ios::app);
  if (!trip_log_.is_open()) throw std::runtime_error("Failed to open trip log: " + logp.string());
  trip_idx_.open(idxp, std::ios::binary | std::ios::app);
  if (!trip_idx_.is_open()) throw std::runtime_error("Failed to open trip idx: " + idxp.string());
}

void AppendLogger::closeTripFiles()
{
  if (trip_log_.is_open()) {
    trip_log_.flush();
    trip_log_.close();
  }
  if (trip_idx_.is_open()) {
    trip_idx_.flush();
    trip_idx_.close();
  }
}

void AppendLogger::appendRecord(uint64_t ts_ns, const std::vector<uint8_t> &payload)
{
  std::lock_guard<std::mutex> lk(mu_);
  if (trip_id_ == -1) throw std::runtime_error("No active trip");

  if (chunk_record_count_ == 0) {
    chunk_start_ts_ns_ = static_cast<int64_t>(ts_ns);
    chunk_end_ts_ns_ = static_cast<int64_t>(ts_ns);
  } else {
    chunk_end_ts_ns_ = static_cast<int64_t>(ts_ns);
  }

  RecordHeader rh;
  rh.ts_ns = static_cast<int64_t>(ts_ns);
  rh.payload_size = static_cast<uint32_t>(payload.size());

  size_t prev_size = chunk_buf_.size();
  chunk_buf_.resize(prev_size + sizeof(rh) + payload.size());
  std::memcpy(chunk_buf_.data() + prev_size, &rh, sizeof(rh));
  if (!payload.empty()) {
    std::memcpy(chunk_buf_.data() + prev_size + sizeof(rh), payload.data(), payload.size());
  }
  ++chunk_record_count_;

  bool size_exceeded = chunk_buf_.size() >= chunk_target_bytes_;
  bool time_exceeded = (static_cast<uint64_t>(chunk_end_ts_ns_ - chunk_start_ts_ns_) >= chunk_target_ns_);

  if (size_exceeded || time_exceeded) {
    flushChunkLocked();
  }
}

void AppendLogger::flushChunkLocked()
{
  if (chunk_record_count_ == 0) return;

  trip_log_.seekp(0, std::ios::end);
  uint64_t file_offset = static_cast<uint64_t>(trip_log_.tellp());

  ChunkHeader ch;
  ch.start_ts_ns = chunk_start_ts_ns_;
  ch.end_ts_ns = chunk_end_ts_ns_;
  ch.record_count = chunk_record_count_;
  ch.chunk_size_bytes = static_cast<uint32_t>(chunk_buf_.size());

  trip_log_.write(reinterpret_cast<const char*>(&ch), sizeof(ch));
  if (!chunk_buf_.empty())
    trip_log_.write(reinterpret_cast<const char*>(chunk_buf_.data()), chunk_buf_.size());
  trip_log_.flush();

  TripIndexEntry tie;
  tie.start_ts_ns = ch.start_ts_ns;
  tie.end_ts_ns = ch.end_ts_ns;
  tie.file_offset = file_offset;
  tie.chunk_size_bytes = ch.chunk_size_bytes + sizeof(ch);
  tie.record_count = ch.record_count;

  trip_index_.push_back(tie);

  trip_idx_.write(reinterpret_cast<const char*>(&tie), sizeof(tie));
  trip_idx_.flush();

  chunk_buf_.clear();
  chunk_record_count_ = 0;
  chunk_start_ts_ns_ = 0;
  chunk_end_ts_ns_ = 0;
}

void AppendLogger::endTrip(uint64_t end_ts_ns)
{
  std::lock_guard<std::mutex> lk(mu_);
  if (trip_id_ == -1) return;

  flushChunkLocked();

  updateGlobalRowEnd(day_, trip_id_, end_ts_ns);

  closeTripFiles();
  trip_id_ = -1;
  day_.clear();
  trip_index_.clear();
}

} // namespace avs
// ...existing code...