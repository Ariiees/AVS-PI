#include "avs/db_operation.h"
#include "avs/common.h"
#include <sstream>

namespace fs = std::filesystem;

namespace avs {

AvsDb::~AvsDb() { close(); }

bool AvsDb::open(const std::string& db_path, std::string* err) {
  if (sqlite3_open(db_path.c_str(), &db_) != SQLITE_OK) {
    if (err) *err = sqlite3_errmsg(db_);
    return false;
  }

  // Pragmas tuned for WAL ingestion
  if (!exec("PRAGMA journal_mode=WAL;", err)) return false;
  if (!exec("PRAGMA synchronous=NORMAL;", err)) return false;
  if (!exec("PRAGMA temp_store=MEMORY;", err)) return false;
  if (!exec("PRAGMA cache_size=-20000;", err)) return false; // ~20MB

  if (!ensureSchema(err)) return false;

  // Prepare insert
  const char* sql =
      "INSERT OR IGNORE INTO avs_data (sensor_id, data_type, ts_ms, path) "
      "VALUES (?, ?, ?, ?);";
  if (sqlite3_prepare_v2(db_, sql, -1, &insert_stmt_, nullptr) != SQLITE_OK) {
    if (err) *err = sqlite3_errmsg(db_);
    return false;
  }
  return true;
}

void AvsDb::close() {
  finalizeStatements();
  if (db_) {
    sqlite3_close(db_);
    db_ = nullptr;
  }
}

void AvsDb::finalizeStatements() {
  if (insert_stmt_) {
    sqlite3_finalize(insert_stmt_);
    insert_stmt_ = nullptr;
  }
}

bool AvsDb::ensureSchema(std::string* err) {
  const char* ddl =
      "CREATE TABLE IF NOT EXISTS avs_data ("
      "  sensor_id TEXT NOT NULL,"
      "  data_type TEXT NOT NULL,"
      "  ts_ms     INTEGER NOT NULL,"
      "  path      TEXT NOT NULL,"
      "  PRIMARY KEY(sensor_id, data_type, ts_ms)"
      ");";
  return exec(ddl, err);
}

bool AvsDb::insertRow(const AvsRow& row, std::string* err) {
  if (!db_ || !insert_stmt_) { if (err) *err = "DB not initialized"; return false; }

  sqlite3_reset(insert_stmt_);
  sqlite3_clear_bindings(insert_stmt_);

  if (sqlite3_bind_text(insert_stmt_, 1, row.sensor_id.c_str(), -1, SQLITE_TRANSIENT) != SQLITE_OK) { if (err) *err = sqlite3_errmsg(db_); return false; }
  if (sqlite3_bind_text(insert_stmt_, 2, row.data_type.c_str(), -1, SQLITE_TRANSIENT) != SQLITE_OK) { if (err) *err = sqlite3_errmsg(db_); return false; }
  if (sqlite3_bind_int64(insert_stmt_, 3, row.ts_ms) != SQLITE_OK) { if (err) *err = sqlite3_errmsg(db_); return false; }
  if (sqlite3_bind_text(insert_stmt_, 4, row.path.c_str(), -1, SQLITE_TRANSIENT) != SQLITE_OK) { if (err) *err = sqlite3_errmsg(db_); return false; }

  if (sqlite3_step(insert_stmt_) != SQLITE_DONE) {
    if (err) *err = sqlite3_errmsg(db_);
    return false;
  }
  return true;
}

bool AvsDb::beginTx(std::string* err)    { return exec("BEGIN;", err); }
bool AvsDb::commitTx(std::string* err)   { return exec("COMMIT;", err); }
bool AvsDb::rollbackTx(std::string* err) { return exec("ROLLBACK;", err); }

bool AvsDb::queryByTimeRange(const std::string& sensor_id,
                             const std::string& data_type,
                             long long start_ms,
                             long long end_ms,
                             const std::function<void(const AvsRow&)>& cb,
                             std::string* err) const {
  if (!db_) { if (err) *err = "DB not initialized"; return false; }

  const char* sql =
      "SELECT sensor_id, data_type, ts_ms, path "
      "FROM avs_data WHERE sensor_id=? AND data_type=? AND ts_ms BETWEEN ? AND ? "
      "ORDER BY ts_ms ASC;";

  sqlite3_stmt* stmt{nullptr};
  if (sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr) != SQLITE_OK) {
    if (err) *err = sqlite3_errmsg(db_);
    return false;
  }

  bool ok = true;
  do {
    if (sqlite3_bind_text(stmt, 1, sensor_id.c_str(), -1, SQLITE_TRANSIENT) != SQLITE_OK) { ok = false; break; }
    if (sqlite3_bind_text(stmt, 2, data_type.c_str(), -1, SQLITE_TRANSIENT) != SQLITE_OK) { ok = false; break; }
    if (sqlite3_bind_int64(stmt, 3, start_ms) != SQLITE_OK) { ok = false; break; }
    if (sqlite3_bind_int64(stmt, 4, end_ms)   != SQLITE_OK) { ok = false; break; }

    while (ok) {
      int rc = sqlite3_step(stmt);
      if (rc == SQLITE_ROW) {
        AvsRow r;
        r.sensor_id = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 0));
        r.data_type = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 1));
        r.ts_ms     = sqlite3_column_int64(stmt, 2);
        r.path      = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 3));
        cb(r);
      } else if (rc == SQLITE_DONE) {
        break;
      } else {
        ok = false;
        if (err) *err = sqlite3_errmsg(db_);
      }
    }
  } while (false);

  sqlite3_finalize(stmt);
  return ok;
}

bool AvsDb::queryByWallRange(const std::string& sensor_id,
                             const std::string& data_type,
                             const std::string& start_wall,  // "YYYY-M-D_HH-MM"
                             const std::string& end_wall,    // "YYYY-M-D_HH-MM"
                             const std::function<void(const AvsRow&)>& cb,
                             std::string* err) const {
  long long start_ms = 0, end_ms = 0;
  if (!avs::wallRangeYmdHmToEpochMsLocal(start_wall, end_wall, &start_ms, &end_ms, err))
    return false;
  return queryByTimeRange(sensor_id, data_type, start_ms, end_ms, cb, err);
}


bool AvsDb::updatePath(const std::string& sensor_id,
                       const std::string& data_type,
                       long long ts_ms,
                       const std::string& new_path,
                       std::string* err) {
  if (!db_) { if (err) *err = "DB not initialized"; return false; }
  const char* sql = "UPDATE avs_data SET path=? WHERE sensor_id=? AND data_type=? AND ts_ms=?;";
  sqlite3_stmt* stmt{nullptr};
  if (sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr) != SQLITE_OK) { if (err) *err = sqlite3_errmsg(db_); return false; }

  bool ok = true;
  do {
    if (sqlite3_bind_text(stmt, 1, new_path.c_str(), -1, SQLITE_TRANSIENT) != SQLITE_OK) { ok = false; break; }
    if (sqlite3_bind_text(stmt, 2, sensor_id.c_str(), -1, SQLITE_TRANSIENT) != SQLITE_OK) { ok = false; break; }
    if (sqlite3_bind_text(stmt, 3, data_type.c_str(), -1, SQLITE_TRANSIENT) != SQLITE_OK) { ok = false; break; }
    if (sqlite3_bind_int64(stmt, 4, ts_ms) != SQLITE_OK) { ok = false; break; }

    if (sqlite3_step(stmt) != SQLITE_DONE) { ok = false; if (err) *err = sqlite3_errmsg(db_); }
  } while (false);

  sqlite3_finalize(stmt);
  return ok;
}

bool AvsDb::exec(const char* sql, std::string* err) const {
  char* e = nullptr;
  if (sqlite3_exec(db_, sql, nullptr, nullptr, &e) != SQLITE_OK) {
    if (err) *err = e ? e : "unknown error";
    if (e) sqlite3_free(e);
    return false;
  }
  return true;
}

} // namespace avs
