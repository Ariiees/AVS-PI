// ...existing code...
#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <mutex>
#include <fstream>
#include <sqlite3.h>

namespace avs {

#pragma pack(push,1)
struct TripIndexEntry {
  int64_t  start_ts_ns;
  int64_t  end_ts_ns;
  uint64_t file_offset;       // offset of each ChunkHeader in trip.log
  uint32_t chunk_size_bytes;  // header plus records for each chunk
  uint32_t record_count;      // number of records in this chunk
};

struct TripHeader {
  char     magic[16];          // "AVS_TRIP_v1"
  uint64_t trip_start_ts_ns;   // absolute start time
  uint64_t reserved;
};

struct ChunkHeader {
  int64_t  start_ts_ns;       // first record time in this chunk
  int64_t  end_ts_ns;         // last record time in this chunk
  uint32_t record_count;
  uint32_t chunk_size_bytes;  // bytes after this header for this chunk
};

struct RecordHeader {
  int64_t  ts_ns;             // frame timestamp
  uint32_t payload_size;      // bytes following
};
#pragma pack(pop)

class AppendLogger {
public:
  AppendLogger(const std::string &ssd_root, const std::string &topic);
  ~AppendLogger();

  // Start a trip: creates directory, opens files, inserts global row (end_ts_ns = 0)
  void startTrip(const std::string &day, int trip_id, uint64_t start_ts_ns);

  // Append a record (encoded payload). Thread-safe.
  void appendRecord(uint64_t ts_ns, const std::vector<uint8_t> &payload);

  // End a trip: flush pending chunk, update global end_ts_ns, close files.
  void endTrip(uint64_t end_ts_ns);

  // optional tuning
  void setChunkTargetBytes(size_t bytes) { chunk_target_bytes_ = bytes; }
  void setChunkTargetNs(uint64_t ns) { chunk_target_ns_ = ns; }

private:
  void openGlobalDB();
  void ensureGlobalSchema();
  void insertGlobalRow(const std::string &day, int trip_id, uint64_t start_ts_ns);
  void updateGlobalRowEnd(const std::string &day, int trip_id, uint64_t end_ts_ns);

  void openTripFiles();
  void closeTripFiles();
  void flushChunkLocked(); // requires mu_ locked

  std::string ssd_root_;
  std::string topic_;
  std::string day_;
  int trip_id_ = -1;

  std::ofstream trip_log_;
  std::ofstream trip_idx_;

  sqlite3 *ssd_db_ = nullptr;

  std::vector<uint8_t> chunk_buf_;
  int64_t chunk_start_ts_ns_ = 0;
  int64_t chunk_end_ts_ns_ = 0;
  uint32_t chunk_record_count_ = 0;

  std::vector<TripIndexEntry> trip_index_;
  std::mutex mu_;

  size_t chunk_target_bytes_ = 256 * 1024;
  uint64_t chunk_target_ns_ = 250000000ULL; // 250 ms
};

} // namespace avs
// ...existing code...