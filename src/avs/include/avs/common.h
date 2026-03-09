#ifndef AVS_COMMON_H
#define AVS_COMMON_H

#include <string>
#include <chrono>
#include <filesystem>
#include <regex>
#include <climits>
#include <fstream>
#include <openssl/evp.h>
namespace avs
{
// --------------------------------Used in Prototype --------------------------
// Ensure a directory exists; create if not.
// Returns true if the directory exists (either already or after creation).
inline bool ensureDirectory(const std::string &path, std::error_code *ec_out = nullptr) {
    namespace fs = std::filesystem;
    std::error_code ec;

    if (fs::exists(path, ec)) {
        if (ec_out) *ec_out = ec;
        return !ec && fs::is_directory(path, ec);
    }

    fs::create_directories(path, ec);
    if (ec_out) *ec_out = ec;
    return !ec;
}

// Return today's folder name in "YYYY-MM-DD" format (local time)
inline std::string getCurrentDayFolder()
{
  using namespace std::chrono;
  const auto now = system_clock::now();
  const std::time_t tt = system_clock::to_time_t(now);

  std::tm tm{};
#if defined(_WIN32)
  localtime_s(&tm, &tt);
#else
  localtime_r(&tt, &tm);
#endif

  std::ostringstream oss;
  oss << std::setfill('0')
      << std::setw(4) << (tm.tm_year + 1900) << "-"
      << std::setw(2) << (tm.tm_mon + 1)     << "-"
      << std::setw(2) << tm.tm_mday;
  return oss.str();
}


// --------------------------------Used in Benchmark --------------------------
inline std::string getTimestampFilename(const std::string& output_dir, const std::string& extension)
{
  namespace fs = std::filesystem;
  auto now = std::chrono::system_clock::now().time_since_epoch();
  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now).count();

  fs::path dir(output_dir);
  fs::path filename = std::to_string(ms) + extension;
  return (dir / filename).string();
}

} // namespace avs

#endif // AVS_COMMON_H