#pragma once

#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

namespace avs {

class StorageLogger {
 public:
  virtual ~StorageLogger() = default;

  virtual void startTrip(const std::string& day,
                         const std::string& topic_folder,
                         int trip_id,
                         std::uint64_t start_ts_ns) = 0;
  virtual void appendRecord(std::uint64_t ts_ns,
                            const std::vector<std::uint8_t>& payload) = 0;
  virtual void endTrip(std::uint64_t end_ts_ns) = 0;
};

std::string NormalizeStorageBackend(const std::string& backend);
bool IsSupportedStorageBackend(const std::string& backend);

std::filesystem::path StorageBackendRoot(const std::string& backend,
                                         const std::filesystem::path& ssd_root);
std::filesystem::path StorageTopicDayDir(const std::string& backend,
                                         const std::filesystem::path& ssd_root,
                                         const std::string& topic_folder,
                                         const std::string& day);

std::shared_ptr<StorageLogger> CreateStorageLogger(const std::string& backend,
                                                   const std::string& ssd_root,
                                                   const std::string& topic);

}  // namespace avs
