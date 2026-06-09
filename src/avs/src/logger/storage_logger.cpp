#include "avs/storage_logger.h"

#include <algorithm>
#include <cctype>
#include <stdexcept>

#include "avs/append_logger.h"
#include "avs/rocksdb_logger.h"

namespace avs {
namespace {

std::string ToLower(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return value;
}

}  // namespace

std::string NormalizeStorageBackend(const std::string& backend) {
  std::string normalized = ToLower(backend);
  if (normalized.empty()) {
    return "append";
  }
  if (normalized == "append-only") {
    return "append";
  }
  if (normalized == "rocks") {
    return "rocksdb";
  }
  return normalized;
}

bool IsSupportedStorageBackend(const std::string& backend) {
  const std::string normalized = NormalizeStorageBackend(backend);
  return normalized == "append" || normalized == "rocksdb";
}

std::filesystem::path StorageBackendRoot(const std::string& backend,
                                         const std::filesystem::path& ssd_root) {
  const std::string normalized = NormalizeStorageBackend(backend);
  if (normalized == "append") {
    return ssd_root;
  }
  if (normalized == "rocksdb") {
    return ssd_root / "rocksdb";
  }
  throw std::invalid_argument("Unsupported storage backend: " + backend);
}

std::filesystem::path StorageTopicDayDir(const std::string& backend,
                                         const std::filesystem::path& ssd_root,
                                         const std::string& topic_folder,
                                         const std::string& day) {
  return StorageBackendRoot(backend, ssd_root) / topic_folder / day;
}

std::shared_ptr<StorageLogger> CreateStorageLogger(const std::string& backend,
                                                   const std::string& ssd_root,
                                                   const std::string& topic) {
  const std::string normalized = NormalizeStorageBackend(backend);
  if (normalized == "append") {
    return std::make_shared<AppendLogger>(ssd_root, topic);
  }
  if (normalized == "rocksdb") {
    return std::make_shared<RocksDbLogger>(ssd_root, topic);
  }
  throw std::invalid_argument("Unsupported storage backend: " + backend);
}

}  // namespace avs
