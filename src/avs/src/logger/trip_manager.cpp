#include "avs/trip_manager.h"

#include <filesystem>
#include <sstream>
#include <cctype>
#include <sqlite3.h>
#include <cstdlib>

namespace fs = std::filesystem;

namespace avs {

static bool IsTripLogFile(const fs::path& p, int& id_out)
{
  if (!fs::is_regular_file(p)) return false;
  if (p.extension() != ".log") return false;

  const std::string stem = p.stem().string();  // trip_00 from trip_00.log
  if (stem.rfind("trip_", 0) != 0) return false;
  if (stem.size() <= 5) return false;

  for (size_t i = 5; i < stem.size(); ++i) {
    if (!std::isdigit(static_cast<unsigned char>(stem[i]))) return false;
  }

  try {
    id_out = std::stoi(stem.substr(5));
    return true;
  } catch (...) {
    return false;
  }
}

int TripManager::CountTripLogs(const std::string& day_path)
{
  fs::path root(day_path);
  if (!fs::exists(root) || !fs::is_directory(root)) {
    return 0;
  }

  int count = 0;
  for (const auto& e : fs::directory_iterator(root)) {
    int id = 0;
    if (IsTripLogFile(e.path(), id)) {
      ++count;
    }
  }
  return count;
}

int TripManager::GetTripId(const std::string& day_path)
{
  std::lock_guard<std::mutex> lock(mutex_);

  auto it = cached_next_id_.find(day_path);
  if (it != cached_next_id_.end()) {
    int id = it->second;
    it->second = id + 1;
    return id;
  }

  // // Resume only when explicitly requested (e.g., power-loss recovery).
  // const char* resume_env = std::getenv("AVS_RESUME_TRIP");
  // if (resume_env && std::string(resume_env) == "1") {
  //   try {
  //   fs::path dayp(day_path);
  //   if (dayp.has_parent_path()) {
  //     std::string day = dayp.filename().string();
  //     std::string topic_folder = dayp.parent_path().filename().string();
  //     fs::path ssd_root = dayp.parent_path().parent_path();
  //     fs::path db_path = ssd_root / "global.sqlite3";
  //     if (fs::exists(db_path)) {
  //       sqlite3* db = nullptr;
  //       if (sqlite3_open(db_path.string().c_str(), &db) == SQLITE_OK) {
  //         const char* sql =
  //             "SELECT trip_id, end_ts_ns, number_of_records "
  //             "FROM global WHERE topic_folder = ? AND day = ? "
  //             "ORDER BY trip_id DESC LIMIT 1;";
  //         sqlite3_stmt* stmt = nullptr;
  //         if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) == SQLITE_OK) {
  //           sqlite3_bind_text(stmt, 1, topic_folder.c_str(), -1, SQLITE_TRANSIENT);
  //           sqlite3_bind_text(stmt, 2, day.c_str(), -1, SQLITE_TRANSIENT);
  //           if (sqlite3_step(stmt) == SQLITE_ROW) {
  //             int trip_id = sqlite3_column_int(stmt, 0);
  //             const unsigned char* end_ts = sqlite3_column_text(stmt, 1);
  //             long long end_ts_ns = end_ts ? std::atoll(reinterpret_cast<const char*>(end_ts)) : 0;
  //             int num_records = sqlite3_column_int(stmt, 2);
  //             if (end_ts_ns == 0 || num_records == 0) {
  //               sqlite3_finalize(stmt);
  //               sqlite3_close(db);
  //               cached_next_id_[day_path] = trip_id + 1;
  //               return trip_id;
  //             }
  //           }
  //         }
  //         if (stmt) sqlite3_finalize(stmt);
  //         sqlite3_close(db);
  //       }
  //     }
  //   }
  //   } catch (...) {
  //     // fall through to default behavior
  //   }
  // }

  const int next_id = CountTripLogs(day_path);
  cached_next_id_[day_path] = next_id + 1;
  return next_id;
}

std::string TripManager::FormatTripId2(int trip_id)
{
  std::ostringstream oss;
  oss.width(2);
  oss.fill('0');
  oss << trip_id;
  return oss.str();
}

}  // namespace avs
