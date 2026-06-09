#pragma once

#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include <sqlite3.h>

#include "avs/storage_logger.h"

namespace rocksdb {
class DB;
}

namespace avs {

class RocksDbLogger : public StorageLogger {
 public:
  RocksDbLogger(const std::string& ssd_root, const std::string& topic);
  ~RocksDbLogger() override;

  void startTrip(const std::string& day,
                 const std::string& topic_folder,
                 int trip_id,
                 std::uint64_t start_ts_ns) override;
  void appendRecord(std::uint64_t ts_ns,
                    const std::vector<std::uint8_t>& payload) override;
  void endTrip(std::uint64_t end_ts_ns) override;

 private:
  void openMetadataDb();
  void ensureMetadataSchema();
  void insertMetadataRow(const std::string& topic_folder,
                         const std::string& day,
                         int trip_id,
                         std::uint64_t start_ts_ns);
  void updateMetadataRowEnd(const std::string& day,
                            int trip_id,
                            std::uint64_t end_ts_ns,
                            std::uint64_t number_of_records);

  std::string formatKey(std::uint64_t ts_ns, std::uint32_t seq) const;
  void closeTripDb();

  std::string ssd_root_;
  std::string topic_;
  std::string day_;
  std::string topic_folder_;
  int trip_id_ = -1;

  sqlite3* metadata_db_ = nullptr;
  rocksdb::DB* trip_db_ = nullptr;

  std::uint32_t next_seq_ = 0;
  std::uint64_t trip_total_record_count_ = 0;

  std::mutex mu_;
};

}  // namespace avs
