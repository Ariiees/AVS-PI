// move_manager.cpp
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <regex>
#include <string>
#include <system_error>
#include <vector>

#include <archive.h>
#include <archive_entry.h>
#include <sqlite3.h>

#include "avs/common.h"         // ensureDirectory, yearMonthFromDay, sha256File
#include "avs/db_operation.h"   // AvsDb: openArchive(), insertArchive(), pragmas

namespace fs = std::filesystem;

static const std::string SSD_IMAGES_ROOT = "/home/avs/DATA/SSD/images";
static const std::string SSD_LIDAR_ROOT  = "/home/avs/DATA/SSD/lidar";
static const std::string SSD_GPS_DIR     = "/home/avs/DATA/SSD/gps";

static const std::string HDD_IMAGES_DATA_ROOT = "/home/avs/DATA/HDD/images";
static const std::string HDD_LIDAR_DATA_ROOT  = "/home/avs/DATA/HDD/lidar";
static const std::string HDD_GPS_ROOT         = "/home/avs/DATA/HDD/gps";

static const std::string SSD_DB_IMAGE = "/home/avs/DATA/SSD/db/avs_image.sqlite3";
static const std::string SSD_DB_LIDAR = "/home/avs/DATA/SSD/db/avs_lidar.sqlite3";
static const std::string HDD_DB_ARCH  = "/home/avs/DATA/HDD/db/archive.sqlite3";

// ---------- Small helpers ----------
static bool isYmd(const std::string& s) {
  // strict YYYY-MM-DD
  static const std::regex re(R"(^\d{4}-\d{2}-\d{2}$)");
  return std::regex_match(s, re);
}
static bool lessYmd(const std::string& a, const std::string& b) {
  // Lexicographic compare works for YYYY-MM-DD
  return a < b;
}

static bool parseBeforeArg(int argc, char** argv, std::string& cutoff_day) {
  for (int i = 1; i + 1 < argc; ++i) {
    if (std::string(argv[i]) == "--before") {
      cutoff_day = argv[i + 1];
      if (!isYmd(cutoff_day)) {
        std::cerr << "[ERR] --before expects YYYY-MM-DD (got '" << cutoff_day << "')\n";
        return false;
      }
      return true;
    }
  }
  std::cerr << "Usage: " << argv[0] << " --before YYYY-MM-DD\n";
  return false;
}

static bool listDayFoldersBefore(const std::string& root, const std::string& cutoff,
                                 std::vector<std::string>& days_out) {
  days_out.clear();
  std::error_code ec;
  if (!fs::exists(root, ec)) return true; // nothing to do
  for (auto& de : fs::directory_iterator(root, ec)) {
    if (!de.is_directory(ec)) continue;
    const auto name = de.path().filename().string();
    if (isYmd(name) && lessYmd(name, cutoff)) {
      days_out.push_back(name);
    }
  }
  std::sort(days_out.begin(), days_out.end());
  return true;
}

// Collect files with given extension; returns sorted by numeric stem (ts_ms).
static void scanDayFiles(const fs::path& day_dir,
                         const std::string& ext,
                         std::vector<fs::path>& files,
                         long long& start_ms,
                         long long& end_ms) {
  files.clear();
  start_ms = LLONG_MAX;
  end_ms   = LLONG_MIN;

  std::error_code ec;
  if (!fs::exists(day_dir, ec)) return;

  for (auto& de : fs::directory_iterator(day_dir, ec)) {
    if (!de.is_regular_file(ec)) continue;
    if (de.path().extension().string() != ext) continue;
    files.push_back(de.path());
  }
  auto to_ts = [](const fs::path& p)->long long {
    try {
      return std::stoll(p.stem().string());
    } catch (...) { return -1; }
  };
  std::sort(files.begin(), files.end(),
            [&](const fs::path& a, const fs::path& b){ return to_ts(a) < to_ts(b); });

  if (!files.empty()) {
    start_ms = to_ts(files.front());
    end_ms   = to_ts(files.back());
  }
}

// archive per day folder to tar
static bool archiveDayStreamingToTar(const fs::path& day_dir,
                                     const std::string& ext,             // ".jpg" or ".laz"
                                     const fs::path& out_tar,
                                     const std::string& day_basename,    // "YYYY-MM-DD" for tar prefix
                                     long long& start_ms,
                                     long long& end_ms,
                                     long long& file_count,
                                     std::string* err) {
  start_ms   = LLONG_MAX;
  end_ms     = LLONG_MIN;
  file_count = 0;

  std::error_code ec;
  if (!fs::exists(day_dir, ec)) return true; // nothing to do is not an error

  struct archive* a = archive_write_new();
  if (!a) { if (err) *err = "archive_write_new failed"; return false; }
  archive_write_set_format_pax_restricted(a);
  if (archive_write_open_filename(a, out_tar.string().c_str()) != ARCHIVE_OK) {
    if (err) *err = archive_error_string(a);
    archive_write_free(a);
    return false;
  }

  char buf[1 << 16];
  auto to_ts = [](const fs::path& p)->long long {
    try { return std::stoll(p.stem().string()); } catch (...) { return -1; }
  };

  for (auto it = fs::directory_iterator(day_dir, ec);
       !ec && it != fs::end(it); ++it) {
    const auto& de = *it;
    if (!de.is_regular_file(ec)) continue;
    const auto& p = de.path();
    if (p.extension().string() != ext) continue;

    const long long ts = to_ts(p);
    if (ts >= 0) {
      if (ts < start_ms) start_ms = ts;
      if (ts > end_ms)   end_ms   = ts;
    }
    ++file_count;

    // tar path: YYYY-MM-DD/<filename>
    const std::string tar_path = (fs::path(day_basename) / p.filename()).string();

    std::ifstream ifs(p, std::ios::binary);
    if (!ifs) {
      if (err) *err = "open file failed: " + p.string();
      archive_write_close(a);
      archive_write_free(a);
      return false;
    }

    struct archive_entry* entry = archive_entry_new();
    archive_entry_set_pathname(entry, tar_path.c_str());
    archive_entry_set_filetype(entry, AE_IFREG);
    archive_entry_set_perm(entry, 0644);
    const auto sz = fs::file_size(p, ec);
    archive_entry_set_size(entry, static_cast<la_int64_t>(ec ? 0 : sz));

    if (archive_write_header(a, entry) != ARCHIVE_OK) {
      if (err) *err = std::string("write_header failed: ") + archive_error_string(a);
      archive_entry_free(entry);
      archive_write_close(a);
      archive_write_free(a);
      return false;
    }

    while (ifs.good()) {
      ifs.read(buf, sizeof(buf));
      std::streamsize n = ifs.gcount();
      if (n > 0) {
        if (archive_write_data(a, buf, static_cast<size_t>(n)) < 0) {
          if (err) *err = std::string("write_data failed: ") + archive_error_string(a);
          archive_entry_free(entry);
          archive_write_close(a);
          archive_write_free(a);
          return false;
        }
      }
    }
    archive_entry_free(entry);
  }

  archive_write_close(a);
  archive_write_free(a);

  if (file_count == 0) {
    // no files found; treat as no-op
    start_ms = 0; end_ms = 0;
  }
  return true;
}


// Try to read min/max/row_count from a GPS per-day sqlite DB.
// Tries several (table, column) candidates; returns true on first success.
static bool probeGpsDayStats(const std::string& per_day_db,
                             long long& out_min_ms,
                             long long& out_max_ms,
                             long long& out_rows) {
  sqlite3* db = nullptr;
  if (sqlite3_open_v2(per_day_db.c_str(), &db, SQLITE_OPEN_READONLY, nullptr) != SQLITE_OK) {
    if (db) sqlite3_close(db);
    return false;
  }
  sqlite3_busy_timeout(db, 2000);

  struct Candidate { const char* table; const char* col; };
  static const Candidate CANDS[] = {
    {"gps_data", "ts_ms"},
    {"gps",      "ts_ms"},
    {"oem7",     "ts_ms"},
    {"gps_data", "timestamp_ms"},
    {"gps",      "timestamp_ms"}
  };

  bool ok = false;
  for (const auto& c : CANDS) {
    std::string sql = std::string("SELECT MIN(") + c.col + "), MAX(" + c.col + "), COUNT(1) FROM " + c.table + ";";
    sqlite3_stmt* stmt = nullptr;
    if (sqlite3_prepare_v2(db, sql.c_str(), -1, &stmt, nullptr) != SQLITE_OK) continue;

    if (sqlite3_step(stmt) == SQLITE_ROW) {
      if (sqlite3_column_type(stmt, 0) != SQLITE_NULL &&
          sqlite3_column_type(stmt, 1) != SQLITE_NULL) {
        out_min_ms = sqlite3_column_int64(stmt, 0);
        out_max_ms = sqlite3_column_int64(stmt, 1);
        out_rows   = sqlite3_column_int64(stmt, 2);
        ok = true;
        sqlite3_finalize(stmt);
        break;
      }
    }
    sqlite3_finalize(stmt);
  }
  sqlite3_close(db);
  return ok;
}

// ---------- Archive flows ----------

static bool archiveOneSensorDay(const std::string& group,          // "images" | "lidar"
                                const std::string& ssd_root,       // per-day folder root on SSD
                                const std::string& hdd_data_root,  // HDD data root
                                const std::string& ssd_hot_db,     // SSD avs_data DB
                                const std::string& day,
                                const std::string& ext_in,            // "jpg" or "laz"
                                avs::AvsDb& archiveDb) {
  const fs::path day_dir = fs::path(ssd_root) / day;

  std::vector<fs::path> files;
  long long start_ms = 0, end_ms = 0, file_count = 0;
  
  std::string ext = "." + ext_in; // file size move need .jpg or .laz, db sized move need jpg or laz
  scanDayFiles(day_dir, ext, files, start_ms, end_ms);

  if (files.empty()) {
    // Nothing to archive for this day; remove empty dir to keep tidy (best-effort)
    std::error_code ec;
    fs::remove(day_dir, ec);
    std::cout << "[INFO] " << group << " " << day << ": no files\n";
    return true;
  }

  const auto [Y, M] = avs::yearMonthFromDay(day);
  fs::path hdd_dir = fs::path(hdd_data_root) / Y / M;
  std::error_code ec;
  if (!avs::ensureDirectory(hdd_dir.string(), &ec)) {
    std::cerr << "[ERR] ensureDirectory failed: " << hdd_dir << " (" << ec.message() << ")\n";
    return false;
  }
  fs::path out_tar = hdd_dir / (day + ".tar");

  std::string why;
  if (!archiveDayStreamingToTar(day_dir, ext, out_tar, day, start_ms, end_ms, file_count, &why)) {
    std::cerr << "[ERR] writeTar failed for " << day_dir << " -> " << out_tar << ": " << why << "\n";
    return false;
  }

  // Compute sha256 for integrity (optional)
  std::string sha_hex;
  (void)avs::sha256File(out_tar.string(), sha_hex);

  // Insert archive row
  avs::AvsArchRow row;
  row.table        = (group == "images") ? "archive_images" : "archive_lidar";
  row.sensor_group = group;
  row.day          = day;
  row.path         = out_tar.string();
  row.start_ms     = start_ms;
  row.end_ms       = end_ms;
  row.file_count   = static_cast<long long>(files.size());
  row.archived_ms  = static_cast<long long>(std::chrono::duration_cast<std::chrono::milliseconds>(
                         std::chrono::system_clock::now().time_since_epoch()).count());
  row.sha256_hex   = sha_hex;

  std::string dberr;
  if (!archiveDb.insertArchive(row, &dberr)) {
    std::cerr << "[ERR] insertArchive failed: " << dberr << "\n";
    return false;
  }

  // Delete rows from hot DB by type/range
  if (!archiveDb.hotDbDeleteRangeByType(ssd_hot_db, ext_in, start_ms, end_ms, &dberr)) {
    std::cerr << "[WARN] hotDb deletion failed (continuing): " << dberr << "\n";
  }

  // Remove the original day folder from SSD
  fs::remove_all(day_dir, ec);
  if (ec) {
    std::cerr << "[WARN] remove_all(" << day_dir << ") failed: " << ec.message() << "\n";
  }

  std::cout << "[OK]  " << group << " " << day << " -> " << out_tar << " ("
            << files.size() << " files, " << start_ms << "-" << end_ms << ")\n";
  return true;
}

static bool archiveGpsDay(const std::string& day, avs::AvsDb& archiveDb) {
  // Source and destination paths
  const fs::path src = fs::path(SSD_GPS_DIR) / (day + ".sqlite3");
  if (!fs::exists(src)) {
    // nothing to do
    return true;
  }

  const auto [Y, M] = avs::yearMonthFromDay(day);
  fs::path dst_dir = fs::path(HDD_GPS_ROOT) / Y / M;
  std::error_code ec;
  if (!avs::ensureDirectory(dst_dir.string(), &ec)) {
    std::cerr << "[ERR] ensureDirectory failed: " << dst_dir << " (" << ec.message() << ")\n";
    return false;
  }
  fs::path dst = dst_dir / (day + ".sqlite3");

  // Probe stats BEFORE move (if possible), else after move.
  long long start_ms = 0, end_ms = 0, row_cnt = 0;
  bool probed = probeGpsDayStats(src.string(), start_ms, end_ms, row_cnt);

  // First insert to archive_gps
  std::string sha_hex;
  (void)avs::sha256File(dst.string(), sha_hex);

  avs::AvsArchRow row;
  row.table        = "archive_gps";
  row.sensor_group = "gps";
  row.day          = day;
  row.path         = dst.string();
  row.start_ms     = start_ms;
  row.end_ms       = end_ms;
  row.file_count   = row_cnt;   // for gps we store row_count
  row.archived_ms  = static_cast<long long>(std::chrono::duration_cast<std::chrono::milliseconds>(
                         std::chrono::system_clock::now().time_since_epoch()).count());
  row.sha256_hex   = sha_hex;

  std::string dberr;
  if (!archiveDb.insertArchive(row, &dberr)) {
    std::cerr << "[ERR] insertArchive(gps) failed: " << dberr << "\n";
    return false;
  }

  // Move (rename will copy across devices if needed)
  std::error_code rec;
  fs::rename(src, dst, rec);
  if (rec) {
    // fallback: copy + remove
    rec.clear();
    fs::copy_file(src, dst, fs::copy_options::overwrite_existing, rec);
    if (rec) {
      std::cerr << "[ERR] copy_file " << src << " -> " << dst << " failed: " << rec.message() << "\n";
      return false;
    }
    fs::remove(src, rec);
    if (rec) std::cerr << "[WARN] remove src failed: " << rec.message() << "\n";
  }

  std::cout << "[OK]  gps " << day << " -> " << dst
            << " (rows=" << row_cnt << ", " << start_ms << "-" << end_ms << ")\n";
  return true;
}

// ---------- Main ----------

int main(int argc, char** argv) {
  std::ios::sync_with_stdio(false);

  std::string cutoff;
  if (!parseBeforeArg(argc, argv, cutoff)) return 2;

  // Prepare archive DB on HDD (single DB with all three tables).
  avs::AvsDb archiveDb;
  {
    std::string err;
    if (!archiveDb.openArchive(HDD_DB_ARCH, &err)) {
      std::cerr << "[ERR] openArchive failed: " << err << "\n";
      return 3;
    }
  }

  // 1) IMAGES
  const auto t0 = std::chrono::steady_clock::now();
  std::vector<std::string> img_days;
  (void)listDayFoldersBefore(SSD_IMAGES_ROOT, cutoff, img_days);
  for (const auto& day : img_days) {
    if (!archiveOneSensorDay("images",
                             SSD_IMAGES_ROOT,
                             HDD_IMAGES_DATA_ROOT,
                             SSD_DB_IMAGE,
                             day,
                             "jpg",
                             archiveDb)) {
      std::cerr << "[ERR] images day failed: " << day << "\n";
    }
  }
  const auto t1 = std::chrono::steady_clock::now();
  // 2) LIDAR
  std::vector<std::string> lidar_days;
  (void)listDayFoldersBefore(SSD_LIDAR_ROOT, cutoff, lidar_days);
  for (const auto& day : lidar_days) {
    if (!archiveOneSensorDay("lidar",
                             SSD_LIDAR_ROOT,
                             HDD_LIDAR_DATA_ROOT,
                             SSD_DB_LIDAR,
                             day,
                             "laz",
                             archiveDb)) {
      std::cerr << "[ERR] lidar day failed: " << day << "\n";
    }
  }

  const auto t2 = std::chrono::steady_clock::now();
  // 3) GPS (per-day sqlite files)
  // We iterate SSD_GPS_DIR for files "YYYY-MM-DD.sqlite3" < cutoff
  {
    std::error_code ec;
    if (fs::exists(SSD_GPS_DIR, ec)) {
      std::vector<std::string> gps_days;
      for (auto& de : fs::directory_iterator(SSD_GPS_DIR, ec)) {
        if (!de.is_regular_file(ec)) continue;
        const auto name = de.path().filename().string(); // e.g., "YYYY-MM-DD.sqlite3"
        if (name.size() != 18 || name.substr(10) != ".sqlite3") continue;
        const std::string day = name.substr(0, 10);
        if (isYmd(day) && lessYmd(day, cutoff)) gps_days.push_back(day);
      }
      std::sort(gps_days.begin(), gps_days.end());
      for (const auto& day : gps_days) {
        if (!archiveGpsDay(day, archiveDb)) {
          std::cerr << "[ERR] gps day failed: " << day << "\n";
        }
      }
    }
  }
  const auto t3 = std::chrono::steady_clock::now();

  const auto image_latency_us = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();
  const auto lidar_latency_us = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
  const auto gps_latency_us = std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2).count();
  const auto archive_latency_us = std::chrono::duration_cast<std::chrono::microseconds>(t3 - t0).count();
  
  std::cout << "[DONE] Archive completed for days < " << cutoff << "\n";
  std::cout << "[REPORT] Image archive latency: " << image_latency_us << "\n";
  std::cout << "[REPORT] Lidar archive latency: " << lidar_latency_us << "\n";
  std::cout << "[REPORT] GPS archive latency: " << gps_latency_us << "\n";
  std::cout << "[REPORT] Total archive latency: " << archive_latency_us << "\n";
  return 0;
}
