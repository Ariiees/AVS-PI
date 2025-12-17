#include "avs/trip_manager.h"

#include <filesystem>
#include <sstream>
#include <cctype>

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
