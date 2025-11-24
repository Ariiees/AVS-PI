// ...new file...
#include <iostream>
#include <filesystem>
#include <vector>
#include <string>
#include <fstream>
#include <chrono>
#include <system_error>
#include <sqlite3.h>
#include <archive.h>
#include <archive_entry.h>

#include "avs/append_logger.h"

namespace fs = std::filesystem;

// Simple archive manager that:
// - Finds (sensor_topic, day) pairs in SSD global.sqlite3 older than cutoff_day
// - Builds an HDD tar at HDD_ROOT/<topic>/YYYY/MM/YYYY_MM_DD.tar containing day/ trip files
// - Copies trip metadata rows from SSD DB to HDD DB
// - Removes SSD day folder and deletes SSD DB rows
//
// Usage: archive_manager --ssd-root <SSD_ROOT> --hdd-root <HDD_ROOT> --cutoff-day YYYY_MM_DD

static void usage(const char *p) {
  std::cerr << "Usage: " << p << " --ssd-root <SSD_ROOT> --hdd-root <HDD_ROOT> --cutoff-day YYYY_MM_DD\n";
  std::exit(1);
}

static bool open_db(const fs::path &dbpath, sqlite3 **out) {
  int rc = sqlite3_open(dbpath.c_str(), out);
  if (rc != SQLITE_OK) {
    std::cerr << "sqlite_open failed: " << sqlite3_errmsg(*out) << "\n";
    if (*out) sqlite3_close(*out);
    *out = nullptr;
    return false;
  }
  return true;
}

static std::vector<std::pair<std::string,std::string>> list_days_to_archive(sqlite3 *ssd_db, const std::string &cutoff_day) {
  std::vector<std::pair<std::string,std::string>> out;
  const char *sql =
    "SELECT DISTINCT sensor_topic, day FROM global WHERE day < ?;";
  sqlite3_stmt *stmt = nullptr;
  if (sqlite3_prepare_v2(ssd_db, sql, -1, &stmt, nullptr) != SQLITE_OK) return out;
  sqlite3_bind_text(stmt, 1, cutoff_day.c_str(), -1, SQLITE_TRANSIENT);
  while (sqlite3_step(stmt) == SQLITE_ROW) {
    const unsigned char *topic = sqlite3_column_text(stmt, 0);
    const unsigned char *day   = sqlite3_column_text(stmt, 1);
    out.emplace_back(reinterpret_cast<const char*>(topic), reinterpret_cast<const char*>(day));
  }
  sqlite3_finalize(stmt);
  return out;
}

static bool add_file_to_archive(struct archive *a, const fs::path &src, const std::string &path_in_tar) {
  struct archive_entry *entry = archive_entry_new();
  archive_entry_set_pathname(entry, path_in_tar.c_str());
  std::error_code ec;
  auto st = fs::status(src, ec);
  if (ec) { archive_entry_free(entry); return false; }
  archive_entry_set_size(entry, fs::file_size(src, ec));
  archive_entry_set_filetype(entry, AE_IFREG);
  archive_entry_set_perm(entry, 0644);
  archive_write_header(a, entry);
  std::ifstream ifs(src, std::ios::binary);
  const size_t bufsize = 64*1024;
  std::vector<char> buf(bufsize);
  while (ifs) {
    ifs.read(buf.data(), buf.size());
    std::streamsize r = ifs.gcount();
    if (r > 0) archive_write_data(a, buf.data(), r);
  }
  archive_entry_free(entry);
  return true;
}

static bool create_tar_for_day(const fs::path &ssd_day_dir, const fs::path &hdd_tar_path, const std::string &day) {
  fs::create_directories(hdd_tar_path.parent_path());
  struct archive *a = archive_write_new();
  archive_write_add_filter_none(a);
  archive_write_set_format_pax_restricted(a);
  if (archive_write_open_filename(a, hdd_tar_path.c_str()) != ARCHIVE_OK) {
    std::cerr << "archive_write_open_filename failed: " << archive_error_string(a) << "\n";
    archive_write_free(a);
    return false;
  }

  // Add all trip_*.log and trip_*.idx under internal path day/...
  for (auto &p : fs::directory_iterator(ssd_day_dir)) {
    if (!p.is_regular_file()) continue;
    std::string name = p.path().filename().string();
    if (name.rfind("trip_", 0) != 0) continue;
    std::string path_in_tar = (fs::path(day) / name).string();
    if (!add_file_to_archive(a, p.path(), path_in_tar)) {
      std::cerr << "Failed adding file to tar: " << p.path() << "\n";
    }
  }

  archive_write_close(a);
  archive_write_free(a);
  return true;
}

static bool copy_trip_rows(sqlite3 *ssd_db, sqlite3 *hdd_db, const std::string &topic, const std::string &day) {
  // select rows for topic+day
  const char *sel = "SELECT sensor_topic, day, trip_id, start_ts_ns, end_ts_ns FROM global WHERE sensor_topic = ? AND day = ?;";
  sqlite3_stmt *sstmt = nullptr;
  if (sqlite3_prepare_v2(ssd_db, sel, -1, &sstmt, nullptr) != SQLITE_OK) return false;
  sqlite3_bind_text(sstmt, 1, topic.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(sstmt, 2, day.c_str(), -1, SQLITE_TRANSIENT);

  const char *ins = "INSERT OR REPLACE INTO global(sensor_topic, day, trip_id, start_ts_ns, end_ts_ns) VALUES(?, ?, ?, ?, ?);";
  sqlite3_stmt *istmt = nullptr;
  if (sqlite3_prepare_v2(hdd_db, ins, -1, &istmt, nullptr) != SQLITE_OK) {
    sqlite3_finalize(sstmt);
    return false;
  }

  // begin transaction
  char *errmsg = nullptr;
  sqlite3_exec(hdd_db, "BEGIN IMMEDIATE;", nullptr, nullptr, &errmsg);

  while (sqlite3_step(sstmt) == SQLITE_ROW) {
    const unsigned char *s_topic = sqlite3_column_text(sstmt, 0);
    const unsigned char *s_day   = sqlite3_column_text(sstmt, 1);
    int trip_id                  = sqlite3_column_int(sstmt, 2);
    const unsigned char *s_start = sqlite3_column_text(sstmt, 3);
    const unsigned char *s_end   = sqlite3_column_text(sstmt, 4);

    sqlite3_bind_text(istmt, 1, reinterpret_cast<const char*>(s_topic), -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(istmt, 2, reinterpret_cast<const char*>(s_day), -1, SQLITE_TRANSIENT);
    sqlite3_bind_int(istmt, 3, trip_id);
    sqlite3_bind_text(istmt, 4, reinterpret_cast<const char*>(s_start), -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(istmt, 5, reinterpret_cast<const char*>(s_end), -1, SQLITE_TRANSIENT);

    if (sqlite3_step(istmt) != SQLITE_DONE) {
      std::cerr << "Insert into HDD DB failed: " << sqlite3_errmsg(hdd_db) << "\n";
    }
    sqlite3_reset(istmt);
  }

  sqlite3_finalize(istmt);
  sqlite3_finalize(sstmt);
  sqlite3_exec(hdd_db, "COMMIT;", nullptr, nullptr, &errmsg);
  return true;
}

static bool remove_ssd_day_and_rows(sqlite3 *ssd_db, const fs::path &ssd_day_dir, const std::string &topic, const std::string &day) {
  // delete files
  std::error_code ec;
  fs::remove_all(ssd_day_dir, ec);
  if (ec) {
    std::cerr << "Failed to remove SSD day dir: " << ec.message() << "\n";
    return false;
  }
  // remove rows
  const char *del = "DELETE FROM global WHERE sensor_topic = ? AND day = ?;";
  sqlite3_stmt *dstmt = nullptr;
  if (sqlite3_prepare_v2(ssd_db, del, -1, &dstmt, nullptr) != SQLITE_OK) return false;
  sqlite3_bind_text(dstmt, 1, topic.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(dstmt, 2, day.c_str(), -1, SQLITE_TRANSIENT);
  if (sqlite3_step(dstmt) != SQLITE_DONE) {
    std::cerr << "Failed to delete SSD DB rows: " << sqlite3_errmsg(ssd_db) << "\n";
    sqlite3_finalize(dstmt);
    return false;
  }
  sqlite3_finalize(dstmt);
  return true;
}

int main(int argc, char ** argv) {
  fs::path ssd_root, hdd_root;
  std::string cutoff_day;
  if (argc < 7) usage(argv[0]);

  for (int i = 1; i < argc; ++i) {
    std::string a(argv[i]);
    if (a == "--ssd-root" && i + 1 < argc) { ssd_root = argv[++i]; continue; }
    if (a == "--hdd-root" && i + 1 < argc) { hdd_root = argv[++i]; continue; }
    if (a == "--cutoff-day" && i + 1 < argc) { cutoff_day = argv[++i]; continue; }
    usage(argv[0]);
  }

  if (ssd_root.empty() || hdd_root.empty() || cutoff_day.empty()) usage(argv[0]);

  sqlite3 *ssd_db = nullptr;
  if (!open_db(ssd_root / "global.sqlite3", &ssd_db)) return 1;

  auto pairs = list_days_to_archive(ssd_db, cutoff_day);
  if (pairs.empty()) {
    std::cerr << "[INFO] Nothing to archive (no topic/day older than " << cutoff_day << ")\n";
    sqlite3_close(ssd_db);
    return 0;
  }

  // ensure hdd global DB exists and schema
  sqlite3 *hdd_db = nullptr;
  fs::create_directories(hdd_root);
  if (!open_db(hdd_root / "global.sqlite3", &hdd_db)) { sqlite3_close(ssd_db); return 1; }
  // ensure schema on HDD DB
  const char *schema =
    "CREATE TABLE IF NOT EXISTS global ("
    "  sensor_topic TEXT NOT NULL,"
    "  day TEXT NOT NULL,"
    "  trip_id INTEGER NOT NULL,"
    "  start_ts_ns TEXT NOT NULL,"
    "  end_ts_ns TEXT NOT NULL,"
    "  PRIMARY KEY(sensor_topic, day, trip_id)"
    ");";
  char *errmsg = nullptr;
  sqlite3_exec(hdd_db, schema, nullptr, nullptr, &errmsg);

  for (auto &pr : pairs) {
    const std::string &topic = pr.first;
    const std::string &day   = pr.second;
    std::cerr << "[ARCHIVE] topic=" << topic << " day=" << day << "\n";

    fs::path ssd_day_dir = ssd_root / topic / day;
    if (!fs::exists(ssd_day_dir)) {
      std::cerr << "[WARN] SSD day dir missing: " << ssd_day_dir << "\n";
      continue;
    }

    // build hdd tar path: HDD_ROOT/topic/YYYY/MM/YYYY_MM_DD.tar
    std::string year = day.substr(0,4);
    std::string month = day.substr(5,2);
    fs::path hdd_tar = hdd_root / topic / year / month / (day + ".tar");

    if (!create_tar_for_day(ssd_day_dir, hdd_tar, day)) {
      std::cerr << "[ERR] Failed to create tar for " << ssd_day_dir << "\n";
      continue;
    }

    // copy metadata rows from SSD->HDD
    if (!copy_trip_rows(ssd_db, hdd_db, topic, day)) {
      std::cerr << "[ERR] Failed to copy trip rows for " << topic << " " << day << "\n";
      continue;
    }

    // remove SSD data files and rows
    if (!remove_ssd_day_and_rows(ssd_db, ssd_day_dir, topic, day)) {
      std::cerr << "[ERR] Failed to cleanup SSD for " << topic << " " << day << "\n";
      continue;
    }

    std::cerr << "[ARCHIVE] Completed for " << topic << " " << day << " -> " << hdd_tar << "\n";
  }

  sqlite3_close(ssd_db);
  sqlite3_close(hdd_db);
  return 0;
}