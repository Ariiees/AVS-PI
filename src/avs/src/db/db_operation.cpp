#include "avs/db_operation.h"
#include "avs/common.h"  // wallRangeYmdHmToEpochMsLocal
#include <algorithm>
#include <cstring>
#include <sstream>

namespace avs {

// ---------- lifecycle ----------

AvsDb::~AvsDb() { close(); }

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
  if (insert_archive_stmt_) {
    sqlite3_finalize(insert_archive_stmt_);
    insert_archive_stmt_ = nullptr;
  }
}

bool AvsDb::exec(const char* sql, std::string* err) const {
  if (!db_) { if (err) *err = "DB not initialized"; return false; }
  char* e = nullptr;
  if (sqlite3_exec(db_, sql, nullptr, nullptr, &e) != SQLITE_OK) {
    if (err) *err = e ? e : "unknown sqlite error";
    if (e) sqlite3_free(e);
    return false;
  }
  return true;
}

bool AvsDb::configurePragmas(std::string* err) {
  // Order matters for some pragmas (page_size only before first table).
  // Use conservative, ingestion-friendly settings.
  if (!exec("PRAGMA journal_mode=WAL;", err)) return false;
  if (!exec("PRAGMA synchronous=NORMAL;", err)) return false;
  if (!exec("PRAGMA temp_store=MEMORY;", err)) return false;
  if (!exec("PRAGMA cache_size=-20000;", err)) return false;       // ~20MB cache
  if (!exec("PRAGMA wal_autocheckpoint=1000;", err)) return false; // checkpoint roughly every ~1k pages
  (void)exec("PRAGMA page_size=4096;", nullptr);                    // best-effort; noop if already set

  // Busy handler via API (applies even if exec errors are ignored)
  sqlite3_busy_timeout(db_, 3000);
  sqlite3_extended_result_codes(db_, 1);
  return true;
}

// ---------- transactions ----------

bool AvsDb::beginTx(std::string* err)    { return exec("BEGIN IMMEDIATE;", err); }
bool AvsDb::commitTx(std::string* err)   { return exec("COMMIT;", err); }
bool AvsDb::rollbackTx(std::string* err) { return exec("ROLLBACK;", err); }

// ---------- hot DB ----------

bool AvsDb::ensureSchema(std::string* err) {
  // Composite PK gives us an index on (sensor_id, data_type, ts_ms)
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

bool AvsDb::open(const std::string& db_path, std::string* err) {
  // If this instance was previously opened, close it first.
  close();

  if (sqlite3_open_v2(db_path.c_str(),
                      &db_,
                      SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE | SQLITE_OPEN_FULLMUTEX,
                      nullptr) != SQLITE_OK) {
    if (err) *err = sqlite3_errmsg(db_);
    return false;
  }

  if (!configurePragmas(err)) return false;
  if (!ensureSchema(err)) return false;

  // Prepare insert
  static const char* kInsertSql =
      "INSERT OR IGNORE INTO avs_data (sensor_id, data_type, ts_ms, path) "
      "VALUES (?, ?, ?, ?);";
  if (sqlite3_prepare_v2(db_, kInsertSql, -1, &insert_stmt_, nullptr) != SQLITE_OK) {
    if (err) *err = sqlite3_errmsg(db_);
    return false;
  }
  return true;
}

bool AvsDb::insertRow(const AvsRow& row, std::string* err) {
  if (!db_ || !insert_stmt_) { if (err) *err = "DB not initialized"; return false; }

  sqlite3_reset(insert_stmt_);
  sqlite3_clear_bindings(insert_stmt_);

  if (sqlite3_bind_text(insert_stmt_, 1, row.sensor_id.c_str(), -1, SQLITE_TRANSIENT) != SQLITE_OK) { if (err) *err = sqlite3_errmsg(db_); return false; }
  if (sqlite3_bind_text(insert_stmt_, 2, row.data_type.c_str(), -1, SQLITE_TRANSIENT) != SQLITE_OK) { if (err) *err = sqlite3_errmsg(db_); return false; }
  if (sqlite3_bind_int64(insert_stmt_, 3, row.ts_ms) != SQLITE_OK)                                     { if (err) *err = sqlite3_errmsg(db_); return false; }
  if (sqlite3_bind_text(insert_stmt_, 4, row.path.c_str(), -1, SQLITE_TRANSIENT) != SQLITE_OK)        { if (err) *err = sqlite3_errmsg(db_); return false; }

  if (sqlite3_step(insert_stmt_) != SQLITE_DONE) {
    if (err) *err = sqlite3_errmsg(db_);
    return false;
  }
  return true;
}

bool AvsDb::hotDbDeleteRangeByType(const std::string& db_path,
                                   const std::string& data_type,
                                   long long start_ms,
                                   long long end_ms,
                                   std::string* err) {
  sqlite3* db = nullptr;
  if (sqlite3_open_v2(db_path.c_str(), &db, SQLITE_OPEN_READWRITE, nullptr) != SQLITE_OK) {
    if (err) *err = sqlite3_errmsg(db);
    if (db) sqlite3_close(db);
    return false;
  }
  sqlite3_busy_timeout(db, 3000);
  sqlite3_exec(db, "PRAGMA journal_mode=WAL;", nullptr, nullptr, nullptr);
  sqlite3_exec(db, "PRAGMA synchronous=NORMAL;", nullptr, nullptr, nullptr);

  const char* sql = "DELETE FROM avs_data WHERE data_type=? AND ts_ms BETWEEN ? AND ?;";
  sqlite3_stmt* stmt = nullptr;
  if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK) {
    if (err) *err = sqlite3_errmsg(db);
    sqlite3_close(db);
    return false;
  }
  sqlite3_bind_text (stmt, 1, data_type.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_int64(stmt, 2, start_ms);
  sqlite3_bind_int64(stmt, 3, end_ms);

  bool ok = true;
  if (sqlite3_step(stmt) != SQLITE_DONE) {
    ok = false;
    if (err) *err = sqlite3_errmsg(db);
  }
  sqlite3_finalize(stmt);
  sqlite3_close(db);
  return ok;
}

bool AvsDb::queryByTimeRange(const std::string& sensor_id,
                             const std::string& data_type,
                             long long start_ms,
                             long long end_ms,
                             const std::function<void(const AvsRow&)>& cb,
                             std::string* err) const {
  if (!db_) { if (err) *err = "DB not initialized"; return false; }

  if (start_ms > end_ms) std::swap(start_ms, end_ms);

  static const char* kSql =
      "SELECT sensor_id, data_type, ts_ms, path "
      "FROM avs_data "
      "WHERE sensor_id=? AND data_type=? AND ts_ms BETWEEN ? AND ? "
      "ORDER BY ts_ms ASC;";

  sqlite3_stmt* stmt{nullptr};
  if (sqlite3_prepare_v2(db_, kSql, -1, &stmt, nullptr) != SQLITE_OK) {
    if (err) *err = sqlite3_errmsg(db_);
    return false;
  }

  bool ok = true;
  do {
    if (sqlite3_bind_text(stmt, 1, sensor_id.c_str(), -1, SQLITE_TRANSIENT) != SQLITE_OK) { ok = false; break; }
    if (sqlite3_bind_text(stmt, 2, data_type.c_str(), -1, SQLITE_TRANSIENT) != SQLITE_OK) { ok = false; break; }
    if (sqlite3_bind_int64(stmt, 3, start_ms) != SQLITE_OK)                                { ok = false; break; }
    if (sqlite3_bind_int64(stmt, 4, end_ms)   != SQLITE_OK)                                { ok = false; break; }

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
                             const std::string& start_wall,
                             const std::string& end_wall,
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

  static const char* kSql =
      "UPDATE avs_data SET path=? WHERE sensor_id=? AND data_type=? AND ts_ms=?;";
  sqlite3_stmt* stmt{nullptr};
  if (sqlite3_prepare_v2(db_, kSql, -1, &stmt, nullptr) != SQLITE_OK) {
    if (err) *err = sqlite3_errmsg(db_);
    return false;
  }

  bool ok = true;
  do {
    if (sqlite3_bind_text(stmt, 1, new_path.c_str(), -1, SQLITE_TRANSIENT) != SQLITE_OK) { ok = false; break; }
    if (sqlite3_bind_text(stmt, 2, sensor_id.c_str(), -1, SQLITE_TRANSIENT) != SQLITE_OK) { ok = false; break; }
    if (sqlite3_bind_text(stmt, 3, data_type.c_str(), -1, SQLITE_TRANSIENT) != SQLITE_OK) { ok = false; break; }
    if (sqlite3_bind_int64(stmt, 4, ts_ms) != SQLITE_OK)                                   { ok = false; break; }

    if (sqlite3_step(stmt) != SQLITE_DONE) { ok = false; if (err) *err = sqlite3_errmsg(db_); }
  } while (false);

  sqlite3_finalize(stmt);
  return ok;
}

// ---------- archive DB ----------

bool AvsDb::ensureArchiveSchema(std::string* err) {
  // 1) Per-day archive summary tables (images / lidar / gps)
  const char* sql_images =
      "CREATE TABLE IF NOT EXISTS archive_images ("
      "  sensor_group TEXT NOT NULL,"      /* 'images' */
      "  day          TEXT NOT NULL,"      /* 'YYYY-MM-DD' */
      "  path         TEXT NOT NULL,"      /* /HDD/images/YYYY/MM/YYYY-MM-DD.tar */
      "  start_ms     INTEGER NOT NULL,"
      "  end_ms       INTEGER NOT NULL,"
      "  file_count   INTEGER NOT NULL,"
      "  archived_ms  INTEGER NOT NULL,"
      "  sha256_hex   TEXT"
      ");";

  const char* sql_lidar =
      "CREATE TABLE IF NOT EXISTS archive_lidar ("
      "  sensor_group TEXT NOT NULL,"      /* 'lidar'  */
      "  day          TEXT NOT NULL,"
      "  path         TEXT NOT NULL,"
      "  start_ms     INTEGER NOT NULL,"
      "  end_ms       INTEGER NOT NULL,"
      "  file_count   INTEGER NOT NULL,"
      "  archived_ms  INTEGER NOT NULL,"
      "  sha256_hex   TEXT"
      ");";

  const char* sql_gps =
      "CREATE TABLE IF NOT EXISTS archive_gps ("
      "  sensor_group TEXT NOT NULL,"      /* 'gps'    */
      "  day          TEXT NOT NULL,"
      "  path         TEXT NOT NULL,"      /* /HDD/gps/YYYY/MM/YYYY-MM-DD.sqlite3 */
      "  start_ms     INTEGER NOT NULL,"
      "  end_ms       INTEGER NOT NULL,"
      "  row_count    INTEGER,"            /* optional */
      "  archived_ms  INTEGER NOT NULL,"
      "  sha256_hex   TEXT"
      ");";


  if (!exec(sql_images, err)) return false;
  if (!exec(sql_lidar,  err)) return false;
  if (!exec(sql_gps,    err)) return false;

  return true;
}


bool AvsDb::openArchive(const std::string& db_path, std::string* err) {
  // Close any previous handle on this instance.
  close();

  if (sqlite3_open_v2(db_path.c_str(),
                      &db_,
                      SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE | SQLITE_OPEN_FULLMUTEX,
                      nullptr) != SQLITE_OK) {
    if (err) *err = sqlite3_errmsg(db_);
    return false;
  }

  if (!configurePragmas(err)) return false;
  if (!ensureArchiveSchema(err)) return false;

  return true;
}

bool AvsDb::insertArchive(const AvsArchRow& row, std::string* err) {
  if (!db_) { if (err) *err = "DB not initialized"; return false; }

  // Ensure the correct archive table exists
  if (!ensureArchiveSchema(err)) return false;

  std::ostringstream oss;
  if (row.sensor_group == "gps") {
    oss << "INSERT INTO " << row.table
      << " (sensor_group, day, path, start_ms, end_ms, row_count, archived_ms, sha256_hex) "
      << "VALUES (?1, ?2, ?3, ?4, ?5, ?6, ?7, ?8);";
  } else {
    oss << "INSERT INTO " << row.table
      << " (sensor_group, day, path, start_ms, end_ms, file_count, archived_ms, sha256_hex) "
      << "VALUES (?1, ?2, ?3, ?4, ?5, ?6, ?7, ?8);";
  }
  const std::string sql = oss.str();

  sqlite3_stmt* stmt{nullptr};
  if (sqlite3_prepare_v2(db_, sql.c_str(), -1, &stmt, nullptr) != SQLITE_OK) {
    if (err) *err = sqlite3_errmsg(db_);
    return false;
  }

  sqlite3_bind_text (stmt, 1, row.sensor_group.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_text (stmt, 2, row.day.c_str(),          -1, SQLITE_TRANSIENT);
  sqlite3_bind_text (stmt, 3, row.path.c_str(),         -1, SQLITE_TRANSIENT);
  sqlite3_bind_int64(stmt, 4, row.start_ms);
  sqlite3_bind_int64(stmt, 5, row.end_ms);
  sqlite3_bind_int64(stmt, 6, row.file_count);
  sqlite3_bind_int64(stmt, 7, row.archived_ms);
  if (!row.sha256_hex.empty())
    sqlite3_bind_text(stmt, 8, row.sha256_hex.c_str(), -1, SQLITE_TRANSIENT);
  else
    sqlite3_bind_null(stmt, 8);

  bool ok = true;
  if (sqlite3_step(stmt) != SQLITE_DONE) {
    ok = false;
    if (err) *err = sqlite3_errmsg(db_);
  }

  sqlite3_finalize(stmt);
  return ok;
}

} // namespace avs
