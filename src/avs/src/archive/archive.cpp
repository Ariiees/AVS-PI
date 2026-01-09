// archive.cpp
// AVS archive utility
// Usage
//   archive <cutoff_day_YYYY-MM-DD> [topic]
// Behavior
//   Archives all (topic, day) pairs in SSD global sqlite3 with day < cutoff_day.
//   If topic is provided, only archives that topic.
// Notes
//   This program writes one tar per (topic, day) on HDD and a small tar index
//   file alongside the tar. The filesystem does not interpret tar.
//   AVS uses the tar index to locate trip logs and trip indices by byte offset.
//
// Output format
//   ARCHIVE_PER topic=... folder=... day=... tar_bytes=... wall_s=... tar_MBps=... ssd_removed_bytes=... hdd_written_bytes=...
//   ARCHIVE_SUM pairs=... ssd_removed_bytes=... hdd_written_bytes=... tar_bytes=...

#include <sqlite3.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include "avs/topic_map.h"

namespace fs = std::filesystem;

namespace avs {

static constexpr size_t kTarBlock = 512;

static std::string NormalizeDayToDash(const std::string& in) {
  // Accept only YYYY-MM-DD or YYYY_MM_DD, return YYYY-MM-DD
  if (in.size() != 10) throw std::runtime_error("bad day format: " + in);

  auto is_digit = [](char c) { return c >= '0' && c <= '9'; };
  if (!is_digit(in[0]) || !is_digit(in[1]) || !is_digit(in[2]) || !is_digit(in[3]) ||
      !is_digit(in[5]) || !is_digit(in[6]) || !is_digit(in[8]) || !is_digit(in[9])) {
    throw std::runtime_error("bad day digits: " + in);
  }

  if (!((in[4] == '-' || in[4] == '_') && (in[7] == '-' || in[7] == '_'))) {
    throw std::runtime_error("bad day separators: " + in);
  }

  std::string out = in;
  out[4] = '-';
  out[7] = '-';
  return out;
}

static void ThrowErrno(const std::string& what) {
  std::ostringstream oss;
  oss << what << " errno=" << errno << " msg=" << std::strerror(errno);
  throw std::runtime_error(oss.str());
}

static void CheckedWriteAll(int fd, const void* buf, size_t n) {
  const uint8_t* p = static_cast<const uint8_t*>(buf);
  size_t left = n;
  while (left > 0) {
    ssize_t w = ::write(fd, p, left);
    if (w < 0) {
      if (errno == EINTR) continue;
      ThrowErrno("write");
    }
    p += static_cast<size_t>(w);
    left -= static_cast<size_t>(w);
  }
}

static void CheckedFsync(int fd) {
  for (;;) {
    if (::fsync(fd) == 0) return;
    if (errno == EINTR) continue;
    ThrowErrno("fsync");
  }
}

static uint64_t RoundUp512(uint64_t n) {
  return (n + (kTarBlock - 1)) & ~(static_cast<uint64_t>(kTarBlock - 1));
}

// Minimal ustar header
struct TarHeader {
  char name[100];
  char mode[8];
  char uid[8];
  char gid[8];
  char size[12];
  char mtime[12];
  char chksum[8];
  char typeflag;
  char linkname[100];
  char magic[6];
  char version[2];
  char uname[32];
  char gname[32];
  char devmajor[8];
  char devminor[8];
  char prefix[155];
  char pad[12];
};

static void OctalEncode(char* out, size_t out_len, uint64_t value) {
  std::snprintf(out, out_len, "%0*lo", static_cast<int>(out_len - 1),
                static_cast<unsigned long>(value));
}

static void FillTarHeader(TarHeader& h,
                          const std::string& tar_path,
                          uint64_t file_size,
                          uint64_t mtime,
                          uint32_t mode) {
  std::memset(&h, 0, sizeof(h));

  if (tar_path.size() <= sizeof(h.name) - 1) {
    std::snprintf(h.name, sizeof(h.name), "%s", tar_path.c_str());
  } else if (tar_path.size() <= sizeof(h.prefix) + sizeof(h.name) - 1) {
    size_t split = tar_path.rfind('/');
    if (split == std::string::npos) {
      throw std::runtime_error("tar path too long and no slash to split");
    }
    std::string prefix = tar_path.substr(0, split);
    std::string name = tar_path.substr(split + 1);
    if (prefix.size() > sizeof(h.prefix) - 1 || name.size() > sizeof(h.name) - 1) {
      throw std::runtime_error("tar path split still too long");
    }
    std::snprintf(h.prefix, sizeof(h.prefix), "%s", prefix.c_str());
    std::snprintf(h.name, sizeof(h.name), "%s", name.c_str());
  } else {
    throw std::runtime_error("tar path too long");
  }

  OctalEncode(h.mode, sizeof(h.mode), mode & 0777U);
  OctalEncode(h.uid, sizeof(h.uid), 0);
  OctalEncode(h.gid, sizeof(h.gid), 0);
  OctalEncode(h.size, sizeof(h.size), file_size);
  OctalEncode(h.mtime, sizeof(h.mtime), mtime);

  std::memset(h.chksum, ' ', sizeof(h.chksum));
  h.typeflag = '0';
  std::snprintf(h.magic, sizeof(h.magic), "ustar");
  std::snprintf(h.version, sizeof(h.version), "00");
  std::snprintf(h.uname, sizeof(h.uname), "avs");
  std::snprintf(h.gname, sizeof(h.gname), "avs");

  const unsigned char* bytes = reinterpret_cast<const unsigned char*>(&h);
  unsigned int sum = 0;
  for (size_t i = 0; i < sizeof(TarHeader); ++i) sum += bytes[i];

  std::snprintf(h.chksum, sizeof(h.chksum), "%06o", sum);
  h.chksum[6] = '\0';
  h.chksum[7] = ' ';
}

struct TarIndexEntry {
  std::string name;
  uint64_t data_offset = 0;
  uint64_t size = 0;
};

static void WriteTarIndexFile(const fs::path& idx_path_tmp,
                              const std::vector<TarIndexEntry>& entries) {
  std::ofstream ofs(idx_path_tmp, std::ios::binary | std::ios::trunc);
  if (!ofs) throw std::runtime_error("cannot open tar index for write");
  for (const auto& e : entries) {
    ofs << e.name << "\t" << e.data_offset << "\t" << e.size << "\n";
  }
  ofs.flush();
  if (!ofs) throw std::runtime_error("tar index write failed");
}

static void EnsureDir(const fs::path& p) {
  std::error_code ec;
  fs::create_directories(p, ec);
  if (ec) throw std::runtime_error("create_directories failed: " + p.string() + " " + ec.message());
}

static uint64_t FileMtimeSeconds(const fs::path& p) {
  std::error_code ec;
  auto ftime = fs::last_write_time(p, ec);
  if (ec) return 0;
  auto s = std::chrono::time_point_cast<std::chrono::seconds>(ftime);
  return static_cast<uint64_t>(s.time_since_epoch().count());
}

static uint64_t FileSize(const fs::path& p) {
  std::error_code ec;
  auto sz = fs::file_size(p, ec);
  if (ec) throw std::runtime_error("file_size failed: " + p.string() + " " + ec.message());
  return static_cast<uint64_t>(sz);
}

static uint64_t DirTotalSize(const fs::path& dir) {
  uint64_t sum = 0;
  std::error_code ec;
  if (!fs::exists(dir, ec)) return 0;
  for (auto it = fs::recursive_directory_iterator(dir, ec);
       !ec && it != fs::recursive_directory_iterator();
       it.increment(ec)) {
    if (ec) break;
    if (it->is_regular_file()) sum += static_cast<uint64_t>(it->file_size());
  }
  return sum;
}

static void RemoveAll(const fs::path& p) {
  std::error_code ec;
  fs::remove_all(p, ec);
  if (ec) throw std::runtime_error("remove_all failed: " + p.string() + " " + ec.message());
}

// sqlite helpers
class SqliteDb {
public:
  explicit SqliteDb(const fs::path& path) {
    if (sqlite3_open(path.string().c_str(), &db_) != SQLITE_OK) {
      std::string msg = sqlite3_errmsg(db_);
      sqlite3_close(db_);
      db_ = nullptr;
      throw std::runtime_error("sqlite open failed: " + msg);
    }
  }
  ~SqliteDb() {
    if (db_) sqlite3_close(db_);
  }
  sqlite3* get() const { return db_; }

  void Exec(const std::string& sql) {
    char* err = nullptr;
    if (sqlite3_exec(db_, sql.c_str(), nullptr, nullptr, &err) != SQLITE_OK) {
      std::string msg = err ? err : "";
      sqlite3_free(err);
      throw std::runtime_error("sqlite exec failed: " + msg);
    }
  }

private:
  sqlite3* db_ = nullptr;
};

static void EnsureGlobalSchema(SqliteDb& db) {
  const char* sql =
    "CREATE TABLE IF NOT EXISTS global ("
    "  sensor_topic TEXT NOT NULL,"
    "  day TEXT NOT NULL,"
    "  trip_id INTEGER NOT NULL,"
    "  start_ts_ns TEXT NOT NULL,"
    "  end_ts_ns TEXT NOT NULL,"
    "  PRIMARY KEY(sensor_topic, day, trip_id)"
    ");";
  db.Exec(sql);
  db.Exec("PRAGMA journal_mode=WAL;");
  db.Exec("PRAGMA synchronous=NORMAL;");
}

struct TopicDay {
  std::string topic;
  std::string day;  // may be legacy, normalize at use sites
  bool operator<(const TopicDay& o) const {
    if (topic != o.topic) return topic < o.topic;
    return day < o.day;
  }
};

static std::vector<TopicDay> QueryCandidates(SqliteDb& ssd_db,
                                             const std::string& cutoff_day_norm_dash,
                                             const std::optional<std::string>& topic_filter) {
  std::vector<TopicDay> out;

  // Robust: normalize day in DB by replacing '_' with '-', compare against normalized cutoff
  std::string sql =
    "SELECT DISTINCT sensor_topic, day "
    "FROM global "
    "WHERE REPLACE(day, '_', '-') < ?1";
  if (topic_filter) sql += " AND sensor_topic = ?2";
  sql += " ORDER BY sensor_topic, day;";

  sqlite3_stmt* stmt = nullptr;
  if (sqlite3_prepare_v2(ssd_db.get(), sql.c_str(), -1, &stmt, nullptr) != SQLITE_OK) {
    throw std::runtime_error("sqlite prepare failed");
  }
  auto stmt_guard = std::unique_ptr<sqlite3_stmt, decltype(&sqlite3_finalize)>(stmt, &sqlite3_finalize);

  sqlite3_bind_text(stmt, 1, cutoff_day_norm_dash.c_str(), -1, SQLITE_TRANSIENT);
  if (topic_filter) sqlite3_bind_text(stmt, 2, topic_filter->c_str(), -1, SQLITE_TRANSIENT);

  for (;;) {
    int rc = sqlite3_step(stmt);
    if (rc == SQLITE_ROW) {
      const unsigned char* t = sqlite3_column_text(stmt, 0);
      const unsigned char* d = sqlite3_column_text(stmt, 1);
      if (t && d) out.push_back({reinterpret_cast<const char*>(t),
                                 reinterpret_cast<const char*>(d)});
    } else if (rc == SQLITE_DONE) {
      break;
    } else {
      throw std::runtime_error("sqlite step failed");
    }
  }

  return out;
}

static void CopyRowsTopicDay(SqliteDb& src, SqliteDb& dst,
                             const std::string& topic, const std::string& day_exact) {
  dst.Exec("BEGIN IMMEDIATE TRANSACTION;");

  sqlite3_stmt* sel = nullptr;
  const char* sel_sql =
    "SELECT sensor_topic, day, trip_id, start_ts_ns, end_ts_ns "
    "FROM global WHERE sensor_topic = ?1 AND day = ?2;";
  if (sqlite3_prepare_v2(src.get(), sel_sql, -1, &sel, nullptr) != SQLITE_OK) {
    dst.Exec("ROLLBACK;");
    throw std::runtime_error("sqlite prepare select failed");
  }
  auto sel_guard = std::unique_ptr<sqlite3_stmt, decltype(&sqlite3_finalize)>(sel, &sqlite3_finalize);

  sqlite3_bind_text(sel, 1, topic.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(sel, 2, day_exact.c_str(), -1, SQLITE_TRANSIENT);

  sqlite3_stmt* ins = nullptr;
  const char* ins_sql =
    "INSERT OR REPLACE INTO global(sensor_topic, day, trip_id, start_ts_ns, end_ts_ns) "
    "VALUES(?1, ?2, ?3, ?4, ?5);";
  if (sqlite3_prepare_v2(dst.get(), ins_sql, -1, &ins, nullptr) != SQLITE_OK) {
    dst.Exec("ROLLBACK;");
    throw std::runtime_error("sqlite prepare insert failed");
  }
  auto ins_guard = std::unique_ptr<sqlite3_stmt, decltype(&sqlite3_finalize)>(ins, &sqlite3_finalize);

  for (;;) {
    int rc = sqlite3_step(sel);
    if (rc == SQLITE_ROW) {
      const unsigned char* t = sqlite3_column_text(sel, 0);
      const unsigned char* d = sqlite3_column_text(sel, 1);
      int trip_id = sqlite3_column_int(sel, 2);
      const unsigned char* s = sqlite3_column_text(sel, 3);
      const unsigned char* e = sqlite3_column_text(sel, 4);

      sqlite3_reset(ins);
      sqlite3_clear_bindings(ins);

      sqlite3_bind_text(ins, 1, reinterpret_cast<const char*>(t), -1, SQLITE_TRANSIENT);
      sqlite3_bind_text(ins, 2, reinterpret_cast<const char*>(d), -1, SQLITE_TRANSIENT);
      sqlite3_bind_int(ins, 3, trip_id);
      sqlite3_bind_text(ins, 4, reinterpret_cast<const char*>(s), -1, SQLITE_TRANSIENT);
      sqlite3_bind_text(ins, 5, reinterpret_cast<const char*>(e), -1, SQLITE_TRANSIENT);

      int irc = sqlite3_step(ins);
      if (irc != SQLITE_DONE) {
        dst.Exec("ROLLBACK;");
        throw std::runtime_error("sqlite insert failed");
      }
    } else if (rc == SQLITE_DONE) {
      break;
    } else {
      dst.Exec("ROLLBACK;");
      throw std::runtime_error("sqlite select step failed");
    }
  }

  dst.Exec("COMMIT;");
}

static void DeleteRowsTopicDay(SqliteDb& db, const std::string& topic, const std::string& day_exact) {
  sqlite3_stmt* del = nullptr;
  const char* del_sql = "DELETE FROM global WHERE sensor_topic = ?1 AND day = ?2;";
  if (sqlite3_prepare_v2(db.get(), del_sql, -1, &del, nullptr) != SQLITE_OK) {
    throw std::runtime_error("sqlite prepare delete failed");
  }
  auto del_guard = std::unique_ptr<sqlite3_stmt, decltype(&sqlite3_finalize)>(del, &sqlite3_finalize);

  sqlite3_bind_text(del, 1, topic.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(del, 2, day_exact.c_str(), -1, SQLITE_TRANSIENT);

  int rc = sqlite3_step(del);
  if (rc != SQLITE_DONE) throw std::runtime_error("sqlite delete failed");
}

static std::pair<std::string, std::string> YearMonthFromDayDash(const std::string& day_dash) {
  // day_dash must be YYYY-MM-DD
  if (day_dash.size() != 10 || day_dash[4] != '-' || day_dash[7] != '-') {
    throw std::runtime_error("bad day string: " + day_dash);
  }
  return {day_dash.substr(0, 4), day_dash.substr(5, 2)};
}

static std::vector<fs::path> ListTripFiles(const fs::path& day_dir) {
  std::vector<fs::path> files;
  std::error_code ec;
  if (!fs::exists(day_dir, ec)) return files;

  for (auto& ent : fs::directory_iterator(day_dir, ec)) {
    if (ec) break;
    if (!ent.is_regular_file()) continue;

    auto name = ent.path().filename().string();

    if (name.rfind("trip_", 0) == 0) {
      if (name.find(".log") != std::string::npos || name.find(".idx") != std::string::npos) {
        files.push_back(ent.path());
      }
    }

    if (name == "day.idx" || name == "day.log") {
      files.push_back(ent.path());
    }
  }

  std::sort(files.begin(), files.end());
  return files;
}

static void WriteFileIntoTar(int tar_fd,
                             const fs::path& src_path,
                             const std::string& tar_member_path,
                             uint64_t& tar_offset_inout,
                             std::vector<TarIndexEntry>& tar_index) {
  const uint64_t sz = FileSize(src_path);
  const uint64_t mtime = FileMtimeSeconds(src_path);

  TarHeader h;
  FillTarHeader(h, tar_member_path, sz, mtime, 0644);

  CheckedWriteAll(tar_fd, &h, sizeof(h));
  tar_offset_inout += kTarBlock;

  TarIndexEntry idx;
  idx.name = tar_member_path;
  idx.data_offset = tar_offset_inout;
  idx.size = sz;
  tar_index.push_back(idx);

  int in_fd = ::open(src_path.string().c_str(), O_RDONLY | O_CLOEXEC);
  if (in_fd < 0) ThrowErrno("open input file");

  std::vector<uint8_t> buf(1024 * 1024);
  uint64_t left = sz;

  while (left > 0) {
    size_t chunk = static_cast<size_t>(std::min<uint64_t>(left, buf.size()));
    ssize_t r = ::read(in_fd, buf.data(), chunk);
    if (r < 0) {
      if (errno == EINTR) continue;
      ::close(in_fd);
      ThrowErrno("read input file");
    }
    if (r == 0) {
      ::close(in_fd);
      throw std::runtime_error("unexpected eof while reading input");
    }
    CheckedWriteAll(tar_fd, buf.data(), static_cast<size_t>(r));
    left -= static_cast<uint64_t>(r);
    tar_offset_inout += static_cast<uint64_t>(r);
  }

  ::close(in_fd);

  uint64_t padded = RoundUp512(sz);
  uint64_t pad = padded - sz;
  if (pad) {
    std::vector<uint8_t> zeros(static_cast<size_t>(pad), 0);
    CheckedWriteAll(tar_fd, zeros.data(), zeros.size());
    tar_offset_inout += pad;
  }
}

struct ArchiveStats {
  uint64_t ssd_bytes_removed = 0;
  uint64_t hdd_bytes_written = 0;
  uint64_t tar_bytes = 0;
  double wall_s = 0.0;
  std::string folder;
  std::string day_dash;
};

static std::string ResolveFolderOrThrow(const TopicMap& topic_map, const std::string& topic) {
  std::string folder = GetTopicFolder(topic_map, topic);
  if (folder.empty()) throw std::runtime_error("topic not found in topics yaml: " + topic);
  return folder;
}

static fs::path ResolveSsdDayDirPreferDash(const fs::path& ssd_root,
                                          const std::string& folder,
                                          const std::string& day_dash) {
  // Canonical storage is YYYY-MM-DD, but tolerate legacy YYYY_MM_DD directories.
  fs::path p = ssd_root / folder / day_dash;
  if (fs::exists(p)) return p;

  std::string legacy = day_dash;
  legacy[4] = '_';
  legacy[7] = '_';
  fs::path alt = ssd_root / folder / legacy;
  if (fs::exists(alt)) return alt;

  return p;  // will fail later with clear message
}

static ArchiveStats ArchiveOneTopicDay(const fs::path& ssd_root,
                                       const fs::path& hdd_root,
                                       SqliteDb& ssd_db,
                                       SqliteDb& hdd_db,
                                       const TopicMap& topic_map,
                                       const std::string& topic,
                                       const std::string& day_from_db_exact) {
  ArchiveStats st;

  st.folder = ResolveFolderOrThrow(topic_map, topic);

  // Canonical day for path naming and reporting
  st.day_dash = NormalizeDayToDash(day_from_db_exact);

  fs::path ssd_day_dir = ResolveSsdDayDirPreferDash(ssd_root, st.folder, st.day_dash);
  st.ssd_bytes_removed = DirTotalSize(ssd_day_dir);

  auto [yyyy, mm] = YearMonthFromDayDash(st.day_dash);
  fs::path hdd_topic_dir = hdd_root / st.folder / yyyy / mm;
  EnsureDir(hdd_topic_dir);

  fs::path tar_final = hdd_topic_dir / (st.day_dash + ".tar");
  fs::path tar_tmp   = hdd_topic_dir / (st.day_dash + ".tar.tmp");
  fs::path idx_final = hdd_topic_dir / (st.day_dash + ".tar.idx");
  fs::path idx_tmp   = hdd_topic_dir / (st.day_dash + ".tar.idx.tmp");

  if (fs::exists(tar_final)) {
    throw std::runtime_error("tar already exists: " + tar_final.string());
  }

  int tar_fd = ::open(tar_tmp.string().c_str(), O_CREAT | O_TRUNC | O_WRONLY | O_CLOEXEC, 0644);
  if (tar_fd < 0) ThrowErrno("open tar tmp");

  std::vector<TarIndexEntry> tar_index;
  uint64_t tar_offset = 0;

  auto files = ListTripFiles(ssd_day_dir);
  if (files.empty()) {
    ::close(tar_fd);
    throw std::runtime_error("no trip files found in " + ssd_day_dir.string());
  }

  for (const auto& f : files) {
    std::string member = st.day_dash + "/" + f.filename().string();
    WriteFileIntoTar(tar_fd, f, member, tar_offset, tar_index);
  }

  std::array<uint8_t, kTarBlock> zero{};
  CheckedWriteAll(tar_fd, zero.data(), zero.size());
  CheckedWriteAll(tar_fd, zero.data(), zero.size());
  tar_offset += 2 * kTarBlock;

  CheckedFsync(tar_fd);
  ::close(tar_fd);

  WriteTarIndexFile(idx_tmp, tar_index);

  {
    int idx_fd = ::open(idx_tmp.string().c_str(), O_RDONLY | O_CLOEXEC);
    if (idx_fd >= 0) {
      CheckedFsync(idx_fd);
      ::close(idx_fd);
    }
  }

  fs::rename(tar_tmp, tar_final);
  fs::rename(idx_tmp, idx_final);

  st.tar_bytes = FileSize(tar_final);
  st.hdd_bytes_written = st.tar_bytes + FileSize(idx_final);

  // Copy and delete rows using exact day string as stored in DB
  CopyRowsTopicDay(ssd_db, hdd_db, topic, day_from_db_exact);
  DeleteRowsTopicDay(ssd_db, topic, day_from_db_exact);

  RemoveAll(ssd_day_dir);

  return st;
}

static void PrintUsage() {
  std::cerr << "usage: archive <cutoff_day_YYYY-MM-DD> [topic]\n";
}

}  // namespace avs

int main(int argc, char** argv) {
  try {
    if (argc < 2 || argc > 3) {
      avs::PrintUsage();
      return 2;
    }

    fs::path ssd_root("/home/avs/DATA/SSD");
    fs::path hdd_root("/home/avs/DATA/HDD");

    fs::path topic_map_path("/home/avs/AVS-PI/src/avs/config/topics.yaml");
    if (const char* env = std::getenv("AVS_TOPIC_MAP_PATH")) topic_map_path = fs::path(env);
    avs::TopicMap topic_map = avs::LoadTopicMap(topic_map_path.string());

    const std::string cutoff_day = avs::NormalizeDayToDash(argv[1]);

    std::optional<std::string> topic_filter;
    if (argc == 3) topic_filter = std::string(argv[2]);

    fs::path ssd_db_path = ssd_root / "global.sqlite3";
    fs::path hdd_db_path = hdd_root / "global.sqlite3";

    avs::SqliteDb ssd_db(ssd_db_path);
    avs::SqliteDb hdd_db(hdd_db_path);
    avs::EnsureGlobalSchema(ssd_db);
    avs::EnsureGlobalSchema(hdd_db);

    auto candidates = avs::QueryCandidates(ssd_db, cutoff_day, topic_filter);
    if (candidates.empty()) {
      std::cout << "ARCHIVE_SUM pairs=0 ssd_removed_bytes=0 hdd_written_bytes=0 tar_bytes=0\n";
      return 0;
    }

    uint64_t total_ssd_removed = 0;
    uint64_t total_hdd_written = 0;
    uint64_t total_tar_bytes = 0;
    uint64_t archived_pairs = 0;

    for (const auto& td : candidates) {
      auto t0 = std::chrono::steady_clock::now();
      auto st = avs::ArchiveOneTopicDay(ssd_root, hdd_root, ssd_db, hdd_db, topic_map, td.topic, td.day);
      auto t1 = std::chrono::steady_clock::now();
      st.wall_s = std::chrono::duration<double>(t1 - t0).count();

      total_ssd_removed += st.ssd_bytes_removed;
      total_hdd_written += st.hdd_bytes_written;
      total_tar_bytes += st.tar_bytes;
      archived_pairs += 1;

      double mb = static_cast<double>(st.tar_bytes) / (1024.0 * 1024.0);
      double mbps = st.wall_s > 0 ? mb / st.wall_s : 0;

      std::cout << "ARCHIVE_PER"
                << " topic=" << td.topic
                << " folder=" << st.folder
                << " day=" << st.day_dash
                << " tar_bytes=" << st.tar_bytes
                << " wall_s=" << st.wall_s
                << " tar_MBps=" << mbps
                << " ssd_removed_bytes=" << st.ssd_bytes_removed
                << " hdd_written_bytes=" << st.hdd_bytes_written
                << "\n";
    }

    std::cout << "ARCHIVE_SUM"
              << " pairs=" << archived_pairs
              << " ssd_removed_bytes=" << total_ssd_removed
              << " hdd_written_bytes=" << total_hdd_written
              << " tar_bytes=" << total_tar_bytes
              << "\n";
    return 0;

  } catch (const std::exception& e) {
    std::cerr << "error " << e.what() << "\n";
    return 1;
  }
}
