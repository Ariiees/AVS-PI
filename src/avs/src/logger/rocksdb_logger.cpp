#include "avs/rocksdb_logger.h"

#include <chrono>
#include <cstdio>
#include <filesystem>
#include <mutex>
#include <stdexcept>
#include <string>
#include <system_error>

#include <rocksdb/db.h>
#include <rocksdb/options.h>
#include <rocksdb/write_batch.h>

#include "avs/storage_logger.h"

namespace fs = std::filesystem;

namespace avs {

RocksDbLogger::RocksDbLogger(const std::string& ssd_root, const std::string& topic)
    : ssd_root_(ssd_root), topic_(topic) {
  openMetadataDb();
  ensureMetadataSchema();
}

RocksDbLogger::~RocksDbLogger() {
  try {
    if (trip_id_ != -1) {
      const auto now_ns = static_cast<std::uint64_t>(
          std::chrono::duration_cast<std::chrono::nanoseconds>(
              std::chrono::system_clock::now().time_since_epoch())
              .count());
      endTrip(now_ns);
    }
  } catch (...) {
  }
  closeTripDb();
  if (metadata_db_) {
    sqlite3_close(metadata_db_);
    metadata_db_ = nullptr;
  }
}

void RocksDbLogger::openMetadataDb() {
  const fs::path dbpath = fs::path(ssd_root_) / "rocksdb_global.sqlite3";
  const int rc = sqlite3_open(dbpath.c_str(), &metadata_db_);
  if (rc != SQLITE_OK) {
    const std::string err = metadata_db_ && sqlite3_errmsg(metadata_db_)
                                ? sqlite3_errmsg(metadata_db_)
                                : "unknown";
    if (metadata_db_) {
      sqlite3_close(metadata_db_);
      metadata_db_ = nullptr;
    }
    throw std::runtime_error("sqlite3_open failed: " + err);
  }
}

void RocksDbLogger::ensureMetadataSchema() {
  const char* sql =
      "CREATE TABLE IF NOT EXISTS global ("
      "  sensor_topic TEXT NOT NULL,"
      "  topic_folder TEXT NOT NULL,"
      "  day TEXT NOT NULL,"
      "  trip_id INTEGER NOT NULL,"
      "  number_of_records INTEGER NOT NULL,"
      "  start_ts_ns TEXT NOT NULL,"
      "  end_ts_ns TEXT NOT NULL,"
      "  PRIMARY KEY(sensor_topic, day, trip_id)"
      ");";
  char* errmsg = nullptr;
  const int rc = sqlite3_exec(metadata_db_, sql, nullptr, nullptr, &errmsg);
  if (rc != SQLITE_OK) {
    const std::string err = errmsg ? errmsg : "unknown";
    sqlite3_free(errmsg);
    throw std::runtime_error("Failed to create RocksDB metadata schema: " + err);
  }
}

void RocksDbLogger::insertMetadataRow(const std::string& topic_folder,
                                      const std::string& day,
                                      int trip_id,
                                      std::uint64_t start_ts_ns) {
  const char* sql =
      "INSERT OR REPLACE INTO global("
      "sensor_topic, topic_folder, day, trip_id, number_of_records, start_ts_ns, end_ts_ns"
      ") VALUES(?, ?, ?, ?, ?, ?, ?);";

  sqlite3_stmt* stmt = nullptr;
  if (sqlite3_prepare_v2(metadata_db_, sql, -1, &stmt, nullptr) != SQLITE_OK) {
    throw std::runtime_error("sqlite3_prepare_v2 failed (rocksdb insert)");
  }

  sqlite3_bind_text(stmt, 1, topic_.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(stmt, 2, topic_folder.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(stmt, 3, day.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_int(stmt, 4, trip_id);
  sqlite3_bind_int64(stmt, 5, 0);
  sqlite3_bind_text(stmt, 6, std::to_string(start_ts_ns).c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(stmt, 7, "0", -1, SQLITE_TRANSIENT);

  const int rc = sqlite3_step(stmt);
  sqlite3_finalize(stmt);
  if (rc != SQLITE_DONE) {
    throw std::runtime_error("sqlite3_step failed (rocksdb insert)");
  }
}

void RocksDbLogger::updateMetadataRowEnd(const std::string& day,
                                         int trip_id,
                                         std::uint64_t end_ts_ns,
                                         std::uint64_t number_of_records) {
  const char* sql =
      "UPDATE global "
      "SET end_ts_ns = ?, number_of_records = ? "
      "WHERE sensor_topic = ? AND day = ? AND trip_id = ?;";

  sqlite3_stmt* stmt = nullptr;
  if (sqlite3_prepare_v2(metadata_db_, sql, -1, &stmt, nullptr) != SQLITE_OK) {
    throw std::runtime_error("sqlite3_prepare_v2 failed (rocksdb update)");
  }

  sqlite3_bind_text(stmt, 1, std::to_string(end_ts_ns).c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_int64(stmt, 2, static_cast<sqlite3_int64>(number_of_records));
  sqlite3_bind_text(stmt, 3, topic_.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(stmt, 4, day.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_int(stmt, 5, trip_id);

  const int rc = sqlite3_step(stmt);
  sqlite3_finalize(stmt);
  if (rc != SQLITE_DONE) {
    throw std::runtime_error("sqlite3_step failed (rocksdb update)");
  }
}

std::string RocksDbLogger::formatKey(std::uint64_t ts_ns, std::uint32_t seq) const {
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%020llu_%010u",
                static_cast<unsigned long long>(ts_ns), seq);
  return std::string(buf);
}

void RocksDbLogger::closeTripDb() {
  if (trip_db_) {
    delete trip_db_;
    trip_db_ = nullptr;
  }
}

void RocksDbLogger::startTrip(const std::string& day,
                              const std::string& topic_folder,
                              int trip_id,
                              std::uint64_t start_ts_ns) {
  std::lock_guard<std::mutex> lk(mu_);
  if (trip_id_ != -1) {
    throw std::runtime_error("Trip already active");
  }

  day_ = day;
  topic_folder_ = topic_folder;
  trip_id_ = trip_id;
  next_seq_ = 0;
  trip_total_record_count_ = 0;

  const fs::path daydir = StorageTopicDayDir("rocksdb", fs::path(ssd_root_), topic_folder_, day_);
  std::error_code ec;
  fs::create_directories(daydir, ec);
  if (ec) {
    throw std::system_error(ec);
  }

  char name_buf[64];
  std::snprintf(name_buf, sizeof(name_buf), "trip_%02d", trip_id_);
  const fs::path db_dir = daydir / name_buf;

  rocksdb::Options options;
  options.create_if_missing = true;
  options.compression = rocksdb::kNoCompression;
  options.max_open_files = 256;

  rocksdb::DB* db = nullptr;
  const rocksdb::Status status =
      rocksdb::DB::Open(options, db_dir.string(), &db);
  if (!status.ok()) {
    trip_id_ = -1;
    day_.clear();
    topic_folder_.clear();
    throw std::runtime_error("rocksdb open failed: " + status.ToString());
  }

  trip_db_ = db;
  insertMetadataRow(topic_folder_, day_, trip_id_, start_ts_ns);
}

void RocksDbLogger::appendRecord(std::uint64_t ts_ns,
                                 const std::vector<std::uint8_t>& payload) {
  std::lock_guard<std::mutex> lk(mu_);
  if (trip_id_ == -1 || trip_db_ == nullptr) {
    throw std::runtime_error("No active trip");
  }

  rocksdb::WriteOptions write_options;
  write_options.disableWAL = false;
  const std::string key = formatKey(ts_ns, next_seq_++);
  const rocksdb::Slice value(reinterpret_cast<const char*>(payload.data()), payload.size());
  const rocksdb::Status status = trip_db_->Put(write_options, key, value);
  if (!status.ok()) {
    throw std::runtime_error("rocksdb put failed: " + status.ToString());
  }

  ++trip_total_record_count_;
}

void RocksDbLogger::endTrip(std::uint64_t end_ts_ns) {
  std::lock_guard<std::mutex> lk(mu_);
  if (trip_id_ == -1) {
    return;
  }

  if (trip_db_) {
    rocksdb::FlushOptions flush_options;
    flush_options.wait = true;
    const rocksdb::Status flush_status = trip_db_->Flush(flush_options);
    if (!flush_status.ok()) {
      throw std::runtime_error("rocksdb flush failed: " + flush_status.ToString());
    }
  }

  updateMetadataRowEnd(day_, trip_id_, end_ts_ns, trip_total_record_count_);
  closeTripDb();

  trip_id_ = -1;
  day_.clear();
  topic_folder_.clear();
  next_seq_ = 0;
  trip_total_record_count_ = 0;
}

}  // namespace avs
