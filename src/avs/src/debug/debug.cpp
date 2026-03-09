// debug.cpp
// Query a topic in a time range and report average payload size
//
// Example
//   ros2 run avs debug --ssd_root /home/avs/DATA/SSD --topic /my_topic --start_ns 1767982505538546849 --end_ns 1767982515538546849
//
// Notes
//   This computes average from DataRef.payload_size, so it does not read payload bytes from disk.
//   That keeps the debug path fast and low overhead.

#include "avs/retrieve_api.h"

#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>

namespace fs = std::filesystem;

static bool ParseU64(const std::string& s, std::uint64_t* out) {
  if (!out) return false;
  if (s.empty()) return false;
  char* endp = nullptr;
  errno = 0;
  unsigned long long v = std::strtoull(s.c_str(), &endp, 10);
  if (errno != 0) return false;
  if (endp == nullptr || *endp != '\0') return false;
  if (v > std::numeric_limits<std::uint64_t>::max()) return false;
  *out = static_cast<std::uint64_t>(v);
  return true;
}

static void PrintUsage(const char* argv0) {
  std::cerr
      << "usage\n"
      << "  " << (argv0 ? argv0 : "debug")
      << " --topic TOPIC --start_ns U64 --end_ns U64\n";
}

int main(int argc, char** argv) {
  fs::path ssd_root("/home/avs/DATA/SSD");
  std::string topic;
  std::string start_s;
  std::string end_s;

  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];

    auto need_value = [&](const std::string& key) -> const char* {
      if (i + 1 >= argc) {
        std::cerr << "missing value for " << key << "\n";
        return nullptr;
      }
      return argv[++i];
    };

    if (a == "--topic") {
      const char* v = need_value(a);
      if (!v) return 2;
      topic = v;
    } else if (a == "--start_ns") {
      const char* v = need_value(a);
      if (!v) return 2;
      start_s = v;
    } else if (a == "--end_ns") {
      const char* v = need_value(a);
      if (!v) return 2;
      end_s = v;
    } else if (a == "--help" || a == "-h") {
      PrintUsage(argv[0]);
      return 0;
    } else {
      std::cerr << "unknown arg " << a << "\n";
      PrintUsage(argv[0]);
      return 2;
    }
  }

  if (topic.empty() || start_s.empty() || end_s.empty()) {
    PrintUsage(argv[0]);
    return 2;
  }

  std::uint64_t start_ns = 0;
  std::uint64_t end_ns = 0;
  if (!ParseU64(start_s, &start_ns) || !ParseU64(end_s, &end_ns)) {
    std::cerr << "invalid start_ns or end_ns\n";
    return 2;
  }
  if (start_ns == 0 || end_ns == 0 || end_ns < start_ns) {
    std::cerr << "invalid time range\n";
    return 2;
  }

  avs::RetrieveAPI api(ssd_root);

  std::string err;
  auto refs = api.QueryRefs(topic, start_ns, end_ns, &err);
  if (!err.empty()) {
    std::cerr << "query error " << err << "\n";
  }

  std::uint64_t count = 0;
  std::uint64_t total_bytes = 0;
  std::uint64_t min_bytes = std::numeric_limits<std::uint64_t>::max();
  std::uint64_t max_bytes = 0;

  for (const auto& r : refs) {
    const std::uint64_t sz = static_cast<std::uint64_t>(r.payload_size);
    total_bytes += sz;
    count += 1;
    if (sz < min_bytes) min_bytes = sz;
    if (sz > max_bytes) max_bytes = sz;
  }

  double avg = 0.0;
  if (count > 0) {
    avg = static_cast<double>(total_bytes) / static_cast<double>(count);
  } else {
    min_bytes = 0;
    max_bytes = 0;
  }

  std::cout << "DEBUG_AVG_SIZE\n";
  std::cout << "ssd_root " << ssd_root << "\n";
  std::cout << "topic " << topic << "\n";
  std::cout << "start_ns " << start_ns << "\n";
  std::cout << "end_ns " << end_ns << "\n";
  std::cout << "records " << count << "\n";
  std::cout << "total_bytes " << total_bytes << "\n";
  std::cout << std::fixed << std::setprecision(2);
  std::cout << "avg_bytes " << avg << "\n";
  std::cout.unsetf(std::ios::floatfield);
  std::cout << "min_bytes " << min_bytes << "\n";
  std::cout << "max_bytes " << max_bytes << "\n";

  return (count > 0) ? 0 : 1;
}
