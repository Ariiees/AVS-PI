#pragma once
#include <string>
#include <functional>
#include <sqlite3.h>

namespace avs {

struct AvsRow {
  std::string sensor_id;
  std::string data_type;
  long long   ts_ms{0};
  std::string path;
};

class AvsDb {
public:
  AvsDb() = default;
  ~AvsDb();

  // Open and configure DB (WAL, pragmas). Non-copyable.
  bool open(const std::string& db_path, std::string* err = nullptr);
  void close();

  // Single-row insert (OR IGNORE on PK).
  bool insertRow(const AvsRow& row, std::string* err = nullptr);

  // Optional helpers
  bool beginTx(std::string* err = nullptr);
  bool commitTx(std::string* err = nullptr);
  bool rollbackTx(std::string* err = nullptr);

  // Convenience: range query using wall times "YYYY-M-D_HH-MM" (local time, seconds = 00)
  bool queryByWallRange(const std::string& sensor_id,
                        const std::string& data_type,
                        const std::string& start_wall,  // "YYYY-M-D_HH-MM"
                        const std::string& end_wall,    // "YYYY-M-D_HH-MM"
                        const std::function<void(const AvsRow&)>& cb,
                        std::string* err = nullptr) const;

  // Optional update path
  bool updatePath(const std::string& sensor_id,
                  const std::string& data_type,
                  long long ts_ms,
                  const std::string& new_path,
                  std::string* err = nullptr);

private:
  sqlite3* db_{nullptr};
  sqlite3_stmt* insert_stmt_{nullptr};

  // Create table if not exists.
  bool ensureSchema(std::string* err = nullptr);

  // Simple range query: invoke cb(row) for each match. Return false on error.
  bool queryByTimeRange(const std::string& sensor_id,
                        const std::string& data_type,
                        long long start_ms,
                        long long end_ms,
                        const std::function<void(const AvsRow&)>& cb,
                        std::string* err = nullptr) const;

  bool exec(const char* sql, std::string* err = nullptr) const;
  void finalizeStatements();
};

} // namespace avs
