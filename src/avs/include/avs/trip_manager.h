#pragma once

#include <string>
#include <unordered_map>
#include <mutex>

namespace avs {

class TripManager {
public:
  explicit TripManager(const std::string& ssd_root);

  // day format: "YYYY-MM-DD"
  int GetTripId(const std::string& day);

  static std::string FormatTripId(int trip_id);

private:
  std::string ssd_root_;
  std::unordered_map<std::string, int> day_next_trip_id_;
  std::mutex mutex_;

  int ScanExistingTrips(const std::string& day);
};

}  // namespace avs
