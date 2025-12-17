#pragma once

#include <string>
#include <unordered_map>
#include <mutex>

namespace avs {

class TripManager {
public:
  TripManager() = default;

  // day_path is the exact directory SSD topic day
  // returns next trip id based on how many trip log files exist
  int GetTripId(const std::string& day_path);

  // 0 becomes 00, 1 becomes 01
  static std::string FormatTripId2(int trip_id);

private:
  static int CountTripLogs(const std::string& day_path);

private:
  std::mutex mutex_;
  std::unordered_map<std::string, int> cached_next_id_;
};

}  // namespace avs
