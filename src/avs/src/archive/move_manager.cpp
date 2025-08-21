#include <algorithm>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <regex>
#include <string>
#include <system_error>
#include <vector>

#include <archive.h>
#include <archive_entry.h>
#include <yaml-cpp/yaml.h>

#include "avs/db_operation.h"

namespace fs = std::filesystem;

// ----------------------- Config from YAML -----------------------
struct Config {
  // paths
  std::string img_ssd_dir;
  std::string lidar_ssd_dir;
  std::string img_hdd_dir;
  std::string lidar_hdd_dir;
  std::string db_dir;

  // topics (sensor_id)
  std::string img_topic;
  std::string lidar_topic;

  // formats (data_type / file extension)
  std::string img_format;   // "jpg" or "png"
  std::string lidar_format; // "laz"
};

static bool loadConfig(const std::string& yaml_path, Config& c, std::string& err) {
  try {
    YAML::Node root = YAML::LoadFile(yaml_path);
    auto common = root["common"];
    if (!common) { err = "YAML missing 'common' section"; return false; }

    c.img_ssd_dir  = common["img_ssd_dir"].as<std::string>();
    c.lidar_ssd_dir= common["lidar_ssd_dir"].as<std::string>();
    c.img_hdd_dir  = common["img_hdd_dir"].as<std::string>();
    c.lidar_hdd_dir= common["lidar_hdd_dir"].as<std::string>();
    c.db_dir       = common["db_dir"].as<std::string>();
    c.img_topic    = common["img_topic"].as<std::string>();
    c.lidar_topic  = common["lidar_topic"].as<std::string>();

    // formats
    auto img_dedup = root["image_dedup"];
    auto lidar_comp= root["lidar_compress"];
    if (!img_dedup || !lidar_comp) {
      err = "YAML missing 'image_dedup' or 'lidar_compress' section";
      return false;
    }
    c.img_format   = img_dedup["img_format"].as<std::string>();      // jpg|png
    c.lidar_format = lidar_comp["lidar_format"].as<std::string>();   // laz

    return true;
  } catch (const std::exception& e) {
    err = std::string("YAML parse failed: ") + e.what();
    return false;
  }
}

// ----------------------- CLI -----------------------
struct Args {
  std::string cfg_path;
  std::string before_date;  // YYYY-MM-DD (strictly before)
  bool dry_run{false};
};

static void usage() {
  std::cerr << "Usage: move_manager --before YYYY-MM-DD [--dry-run]\n";
}

static bool parseArgs(int argc, char** argv, Args& a) {
  // Fixed config path
  a.cfg_path = "/home/avs/AVS-PI/src/avs/config/avs_config.yaml";

  for (int i = 1; i < argc; ++i) {
    std::string s(argv[i]);
    if (s == "--before" && i + 1 < argc) {
      a.before_date = argv[++i];
    } else if (s == "--dry-run") {
      a.dry_run = true;
    } else {
      std::cerr << "Unknown or incomplete arg: " << s << "\n";
      return false;
    }
  }

  // Validate required --before YYYY-MM-DD
  static const std::regex date_re(R"(^\d{4}-\d{2}-\d{2}$)");
  if (!std::regex_match(a.before_date, date_re)) {
    std::cerr << "Missing or invalid --before (expected YYYY-MM-DD)\n";
    return false;
  }
  return true;
}


// ----------------------- Utilities -----------------------
static long long nowMs() {
  using namespace std::chrono;
  return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

static bool dateDirIsBefore(const std::string& dirName, const std::string& cutoff) {
  // dirName and cutoff must be "YYYY-MM-DD"
  if (dirName.size() != 10) return false;
  return dirName < cutoff; // lexicographic compare works for fixed format
}

static std::vector<std::string> collectDayFolders(const fs::path& base, const std::string& cutoff) {
  std::vector<std::string> days;
  if (!fs::exists(base) || !fs::is_directory(base)) return days;
  for (const auto& e : fs::directory_iterator(base)) {
    if (!e.is_directory()) continue;
    std::string name = e.path().filename().string();
    if (dateDirIsBefore(name, cutoff)) days.push_back(name);
  }
  std::sort(days.begin(), days.end());
  return days;
}

// Parse ts_ms from filename stem: "1755719995946" (ms) OR "sec_ns" e.g., "1754955539_206530777"
static bool parseTsMsFromStem(const std::string& stem, long long& ts_ms_out) {
  if (!stem.empty() && std::all_of(stem.begin(), stem.end(), ::isdigit)) {
    try { ts_ms_out = std::stoll(stem); return true; } catch (...) { return false; }
  }
  size_t us = stem.find('_');
  if (us != std::string::npos) {
    std::string sec = stem.substr(0, us);
    std::string ns  = stem.substr(us + 1);
    if (!sec.empty() && !ns.empty() &&
        std::all_of(sec.begin(), sec.end(), ::isdigit) &&
        std::all_of(ns.begin(),  ns.end(),  ::isdigit)) {
      try {
        long long s = std::stoll(sec);
        long long nsll = std::stoll(ns);
        ts_ms_out = s * 1000LL + (nsll / 1000000LL);
        return true;
      } catch (...) { return false; }
    }
  }
  return false;
}

struct FileEntry {
  fs::path path;      // absolute file path
  long long ts_ms{0};
};

// Enumerate files with extension ".ext" under <base>/<day>
static std::vector<FileEntry> listDayFiles(const fs::path& base, const std::string& day, const std::string& extDot) {
  std::vector<FileEntry> out;
  fs::path dayDir = base / day;
  if (!fs::exists(dayDir) || !fs::is_directory(dayDir)) return out;
  for (const auto& e : fs::directory_iterator(dayDir)) {
    if (!e.is_regular_file()) continue;
    if (e.path().extension() == extDot) {
      long long ts{0};
      if (parseTsMsFromStem(e.path().stem().string(), ts)) {
        out.push_back({ fs::absolute(e.path()), ts });
      }
    }
  }
  std::sort(out.begin(), out.end(), [](const FileEntry& a, const FileEntry& b){ return a.ts_ms < b.ts_ms; });
  return out;
}

static bool ensureDir(const fs::path& p) {
  std::error_code ec;
  fs::create_directories(p, ec);
  return !ec;
}

static bool rmrf(const fs::path& p) {
  std::error_code ec;
  fs::remove_all(p, ec);
  return !ec;
}

static bool renameAtomic(const fs::path& tmp, const fs::path& dst) {
  std::error_code ec;
  fs::rename(tmp, dst, ec);
  return !ec;
}

// ----------------------- libarchive tar writer -----------------------
static bool writeTarFromFileList(const fs::path& ssdBase,
                                 const std::string& day,
                                 const std::vector<FileEntry>& files,
                                 const fs::path& destTarTmp,
                                 std::string& err) {
  if (files.empty()) { err = "no files to archive"; return false; }

  if (!ensureDir(destTarTmp.parent_path())) {
    err = "failed to create parent dir: " + destTarTmp.parent_path().string();
    return false;
  }

  struct archive* a = archive_write_new();
  if (!a) { err = "archive_write_new failed"; return false; }

  // Plain tar; no compression (fast, HDD-friendly, minimal CPU on Pi 5)
  archive_write_set_format_pax_restricted(a);
  // Optional: set block size; default is fine. For large files you could tune:
  // archive_write_set_bytes_per_block(a, 1024 * 1024);

  if (archive_write_open_filename(a, destTarTmp.string().c_str()) != ARCHIVE_OK) {
    err = std::string("archive_open: ") + archive_error_string(a);
    archive_write_free(a);
    return false;
  }

  constexpr size_t kBufSize = 1 << 20; // 1 MiB
  std::vector<char> buf(kBufSize);

  for (const auto& fe : files) {
    const fs::path& p = fe.path;
    std::error_code fec;
    uintmax_t fsize = fs::file_size(p, fec);
    if (fec) {
      err = "stat failed: " + p.string();
      archive_write_close(a);
      archive_write_free(a);
      return false;
    }

    // path in tar: "YYYY-MM-DD/<filename>"
    std::string tarPath = day + "/" + p.filename().string();

    archive_entry* entry = archive_entry_new();
    archive_entry_set_pathname(entry, tarPath.c_str());
    archive_entry_set_filetype(entry, AE_IFREG);
    archive_entry_set_perm(entry, 0644);
    archive_entry_set_size(entry, static_cast<la_int64_t>(fsize));

    if (archive_write_header(a, entry) < ARCHIVE_OK) {
      err = std::string("write_header failed: ") + archive_error_string(a);
      archive_entry_free(entry);
      archive_write_close(a);
      archive_write_free(a);
      return false;
    }

    std::ifstream ifs(p, std::ios::binary);
    if (!ifs) {
      err = "open failed: " + p.string();
      archive_entry_free(entry);
      archive_write_close(a);
      archive_write_free(a);
      return false;
    }

    while (ifs) {
      ifs.read(buf.data(), static_cast<std::streamsize>(buf.size()));
      std::streamsize got = ifs.gcount();
      if (got > 0) {
        la_ssize_t w = archive_write_data(a, buf.data(), static_cast<size_t>(got));
        if (w < 0) {
          err = std::string("write_data failed: ") + archive_error_string(a);
          archive_entry_free(entry);
          archive_write_close(a);
          archive_write_free(a);
          return false;
        }
      }
    }

    archive_entry_free(entry);
  }

  if (archive_write_close(a) != ARCHIVE_OK) {
    err = std::string("archive_close failed: ") + archive_error_string(a);
    archive_write_free(a);
    return false;
  }
  archive_write_free(a);
  return true;
}

// ----------------------- Day processing -----------------------
struct WorkStats { size_t days_considered{0}, days_archived{0}, files_archived{0}; };

static bool processOneDay(
    const std::string& kind,               // "images"/"lidar"
    const fs::path& ssdBase,               // SSD base dir
    const fs::path& hddBase,               // HDD base dir
    const std::string& extDot,             // ".jpg" / ".png" / ".laz"
    const std::string& sensor_id,          // from YAML topic
    const std::string& data_type,          // format, e.g., "jpg"/"png"/"laz"
    const std::string& day,                // YYYY-MM-DD
    bool dryRun,
    avs::AvsDb& hotDb,                     // opened hot DB
    avs::AvsDb& archiveDb,                 // opened archive DB
    WorkStats& stats)
{
  stats.days_considered++;

  auto files = listDayFiles(ssdBase, day, extDot);
  if (files.empty()) {
    std::cout << "[" << kind << "] " << day << ": no " << extDot << " files; skip.\n";
    return true;
  }
  const int tar_file_count = static_cast<int>(files.size());
  std::cout << "[" << kind << "] " << day << ": " << tar_file_count << " files.\n";

  fs::path dayDir    = ssdBase / day;
  fs::path hddTarDir = hddBase; // tar lives directly under HDD base
  fs::path hddTar    = hddTarDir / (day + ".tar");
  fs::path hddTarTmp = hddTarDir / (day + ".tar.tmp");

  if (fs::exists(hddTar)) {
    std::cout << "  Tar already exists: " << hddTar << " (idempotent).\n";
  }

  if (!dryRun && !fs::exists(hddTar)) {
    std::string werr;
    if (!writeTarFromFileList(ssdBase, day, files, hddTarTmp, werr)) {
      std::cerr << "  ERROR: tar write failed: " << werr << "\n";
      return false;
    }
    if (!renameAtomic(hddTarTmp, hddTar)) {
      std::cerr << "  ERROR: rename tmp -> final failed\n";
      return false;
    }
  } else if (dryRun) {
    std::cout << "  [dry-run] would write tar " << hddTar << "\n";
  }

  // DB updates
  const long long archived_at = nowMs();

  if (!dryRun) {
    if (!archiveDb.beginTx()) { std::cerr << "  ERROR: archive beginTx\n"; return false; }
    bool ok = true;
    for (const auto& f : files) {
      avs::AvsArchRow ar;
      ar.sensor_id      = sensor_id;
      ar.data_type      = data_type;
      ar.ts_ms          = f.ts_ms;
      ar.path           = hddTar.string();
      ar.archive_ts_ms  = archived_at;
      ar.tar_file_count = tar_file_count;
      if (!archiveDb.insertArchiveRow(ar)) { ok = false; break; }
    }
    if (ok) ok = archiveDb.commitTx(); else archiveDb.rollbackTx();
    if (!ok) { std::cerr << "  ERROR: archive insert failed\n"; return false; }

    if (!hotDb.beginTx()) { std::cerr << "  ERROR: hot beginTx\n"; return false; }
    ok = true;
    for (const auto& f : files) {
      if (!hotDb.deleteRow(sensor_id, data_type, f.ts_ms)) { ok = false; break; }
    }
    if (ok) ok = hotDb.commitTx(); else hotDb.rollbackTx();
    if (!ok) { std::cerr << "  ERROR: hot delete failed; SSD left intact.\n"; return false; }
  } else {
    std::cout << "  [dry-run] would insert " << files.size() << " archive rows and delete hot rows\n";
  }

  // Remove SSD day folder last
  if (!dryRun) {
    if (!rmrf(dayDir)) {
      std::cerr << "  WARNING: failed to remove SSD dir " << dayDir << "\n";
    }
  } else {
    std::cout << "  [dry-run] would remove " << dayDir << "\n";
  }

  stats.days_archived++;
  stats.files_archived += files.size();
  return true;
}

// ----------------------- main -----------------------
int main(int argc, char** argv) {
  Args args;
  if (!parseArgs(argc, argv, args)) { usage(); return 2; }

  Config cfg;
  std::string err;
  if (!loadConfig(args.cfg_path, cfg, err)) {
    std::cerr << "ERROR: " << err << "\n";
    return 1;
  }

  // Derive DB paths (single db_dir hosting hot + archive DBs)
  const fs::path dbdir(cfg.db_dir);
  const fs::path imageHotDbPath = dbdir / "avs_image.sqlite3";
  const fs::path lidarHotDbPath = dbdir / "avs_lidar.sqlite3";
  const fs::path archiveDbPath  = dbdir / "avs_archive.sqlite3";

  // Ensure HDD target dirs
  if (!ensureDir(cfg.img_hdd_dir) || !ensureDir(cfg.lidar_hdd_dir) || !ensureDir(cfg.db_dir)) {
    std::cerr << "ERROR: failed to ensure output directories\n";
    return 1;
  }

  // Open DBs
  avs::AvsDb archiveDb, imageHotDb, lidarHotDb;
  if (!archiveDb.openArchive(archiveDbPath.string(), &err)) { std::cerr << "ERROR: openArchive: " << err << "\n"; return 1; }
  if (!imageHotDb.open(imageHotDbPath.string(), &err))      { std::cerr << "ERROR: open image DB: " << err << "\n"; return 1; }
  if (!lidarHotDb.open(lidarHotDbPath.string(), &err))      { std::cerr << "ERROR: open lidar DB: " << err << "\n"; return 1; }

  // Build extension strings with dot
  const std::string imgExtDot   = "." + cfg.img_format;      // ".jpg" or ".png"
  const std::string lidarExtDot = "." + cfg.lidar_format;    // ".laz"

  // Collect days
  auto imgDays   = collectDayFolders(cfg.img_ssd_dir,   args.before_date);
  auto lidarDays = collectDayFolders(cfg.lidar_ssd_dir, args.before_date);

  WorkStats wsImg, wsLidar;

  // Images first
  for (const auto& day : imgDays) {
    bool ok = processOneDay("images",
                            cfg.img_ssd_dir,
                            cfg.img_hdd_dir,
                            imgExtDot,
                            cfg.img_topic,
                            cfg.img_format,
                            day,
                            args.dry_run,
                            imageHotDb,
                            archiveDb,
                            wsImg);
    if (!ok) std::cerr << "ERROR: failed archiving images " << day << " (continuing)\n";
  }

  // Lidar next
  for (const auto& day : lidarDays) {
    bool ok = processOneDay("lidar",
                            cfg.lidar_ssd_dir,
                            cfg.lidar_hdd_dir,
                            lidarExtDot,
                            cfg.lidar_topic,
                            cfg.lidar_format,
                            day,
                            args.dry_run,
                            lidarHotDb,
                            archiveDb,
                            wsLidar);
    if (!ok) std::cerr << "ERROR: failed archiving lidar " << day << " (continuing)\n";
  }

  std::cout << "\n=== Summary ===\n";
  std::cout << "Images: days considered=" << wsImg.days_considered
            << ", archived=" << wsImg.days_archived
            << ", files=" << wsImg.files_archived << "\n";
  std::cout << "Lidar : days considered=" << wsLidar.days_considered
            << ", archived=" << wsLidar.days_archived
            << ", files=" << wsLidar.files_archived << "\n";
  std::cout << (args.dry_run ? "[DRY-RUN]\n" : "Done.\n");
  return 0;
}
