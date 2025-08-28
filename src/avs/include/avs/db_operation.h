#pragma once
#include <functional>
#include <string>
#include <sqlite3.h>

namespace avs {

struct AvsRow {
  std::string sensor_id;
  std::string data_type;
  long long   ts_ms{0};
  std::string path;
};

struct AvsArchRow {
  // target table name: "archive_images" | "archive_lidar" | "archive_gps"
  std::string table;
  std::string sensor_group;  // "images" | "lidar" | "gps"
  std::string day;           // "YYYY-MM-DD"
  std::string path;          // HDD path to .tar (images/lidar) or .sqlite3 (gps)
  long long   start_ms{0};   // earliest ts in container (or per-day gps DB)
  long long   end_ms{0};     // latest ts
  long long   file_count{0}; // images/lidar: file count; gps: row_count
  long long   archived_ms{0};
  std::string sha256_hex;    // optional
};

class AvsDb {
public:
  AvsDb() = default;
  ~AvsDb();

  AvsDb(const AvsDb&) = delete;
  AvsDb& operator=(const AvsDb&) = delete;
  AvsDb(AvsDb&&) = delete;
  AvsDb& operator=(AvsDb&&) = delete;

  // ---------- Common ----------
  void close();

  bool beginTx(std::string* err = nullptr);     // BEGIN IMMEDIATE
  bool commitTx(std::string* err = nullptr);
  bool rollbackTx(std::string* err = nullptr);

  // ---------- Hot DB (SSD) ----------
  bool open(const std::string& db_path, std::string* err = nullptr);
  bool insertRow(const AvsRow& row, std::string* err = nullptr);
  bool hotDbDeleteRangeByType(const std::string& db_path,
                              const std::string& data_type,
                              long long start_ms,
                              long long end_ms,
                              std::string* err); 

  // Range query using local wall times "YYYY-M-D_HH-MM" (seconds assumed 00)
  bool queryByWallRange(const std::string& sensor_id,
                        const std::string& data_type,
                        const std::string& start_wall,
                        const std::string& end_wall,
                        const std::function<void(const AvsRow&)>& cb,
                        std::string* err = nullptr) const;

  bool updatePath(const std::string& sensor_id,
                  const std::string& data_type,
                  long long ts_ms,
                  const std::string& new_path,
                  std::string* err = nullptr);

  // ---------- Cold DB (HDD / archive) ----------
  bool openArchive(const std::string& db_path, std::string* err = nullptr);
  bool insertArchive(const AvsArchRow& row, std::string* err = nullptr);

private:
  // state
  sqlite3*      db_{nullptr};
  sqlite3_stmt* insert_stmt_{nullptr};
  sqlite3_stmt* insert_archive_stmt_{nullptr};

  // schema & ops
  bool ensureSchema(std::string* err = nullptr);
  bool ensureArchiveSchema(std::string* err = nullptr);

  bool queryByTimeRange(const std::string& sensor_id,
                        const std::string& data_type,
                        long long start_ms,
                        long long end_ms,
                        const std::function<void(const AvsRow&)>& cb,
                        std::string* err = nullptr) const;

  // helpers
  bool exec(const char* sql, std::string* err = nullptr) const;
  void finalizeStatements();
  bool configurePragmas(std::string* err = nullptr);
};

} // namespace avs
