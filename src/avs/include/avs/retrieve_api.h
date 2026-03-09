#pragma once

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace avs {

struct DataRef {
  std::string sensor_topic;
  std::string day;
  std::string topic_folder;
  int         trip_id = 0;
  std::uint64_t ts_ns = 0;

  std::filesystem::path log_path;
  std::uint64_t payload_offset = 0;
  std::uint32_t payload_size   = 0;
};

class RetrieveAPI {
 public:
  explicit RetrieveAPI(std::filesystem::path ssd_root = std::filesystem::path("/home/avs/DATA/SSD"));

  const std::filesystem::path& ssd_root() const { return ssd_root_; }

  std::vector<DataRef> QueryRefs(const std::string& topic,
                                 std::uint64_t start_ns,
                                 std::uint64_t end_ns,
                                 std::string* err = nullptr) const;

  bool LoadPayload(const DataRef& ref,
                   std::vector<std::uint8_t>& out_payload,
                   std::string* err = nullptr) const;

  bool LoadPayloadAt(const std::filesystem::path& log_path,
                     std::uint64_t payload_offset,
                     std::uint32_t payload_size,
                     std::vector<std::uint8_t>& out_payload,
                     std::string* err = nullptr) const;

 private:
  std::filesystem::path ssd_root_;
};

}  // namespace avs
