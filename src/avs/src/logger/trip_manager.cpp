#include "avs/trip_manager.h"

#include <filesystem>
#include <sstream>
#include <cctype>

namespace fs = std::filesystem;

namespace avs {

TripManager::TripManager(const std::string& ssd_root)
    : ssd_root_(ssd_root) {}

static bool ParseTripIdFromFilename(const std::string& filename, int& trip_id_out) {
  const std::string prefix = "trip_";
  if (filename.rfind(prefix, 0) != 0) {
    return false;
  }

  std::size_t pos = prefix.size();
  std::size_t start = pos;
  while (pos < filename.size() && std::isdigit(static_cast<unsigned char>(filename[pos]))) {
    ++pos;
  }
  if (pos == start) {
    return false;
  }

  trip_id_out = std::stoi(filename.substr(start, pos - start));
  return true;
}

int TripManager::ScanExistingTrips(const std::string& day) {
  fs::path day_path = fs::path(ssd_root_) / day;
  int max_trip_id = -1;

  if (!fs::exists(day_path) || !fs::is_directory(day_path)) {
    return 0;
  }

  for (const auto& entry : fs::directory_iterator(day_path)) {
    if (!entry.is_regular_file()) {
      continue;
    }
    int id = -1;
    if (ParseTripIdFromFilename(entry.path().filename().string(), id)) {
      if (id > max_trip_id) {
        max_trip_id = id;
      }
    }
  }

  if (max_trip_id < 0) {
    return 0;
  }
  return max_trip_id + 1;
}

int TripManager::GetTripId(const std::string& day) {
  std::lock_guard<std::mutex> lock(mutex_);

  auto it = day_next_trip_id_.find(day);
  if (it != day_next_trip_id_.end()) {
    int id = it->second;
    it->second = id + 1;
    return id;
  }

  int next_id = ScanExistingTrips(day);
  day_next_trip_id_[day] = next_id + 1;
  return next_id;
}

std::string TripManager::FormatTripId(int trip_id) {
  std::ostringstream oss;
  oss.width(2);
  oss.fill('0');
  oss << trip_id;
  return oss.str();
}

}  // namespace avs
