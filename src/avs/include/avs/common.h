#ifndef AVS_COMMON_H
#define AVS_COMMON_H

#include <string>
#include <chrono>
#include <filesystem>

namespace avs
{

// Generate a timestamped filename with the given extension and output directory.
// Example output: /home/avs/DATA/SSD/lidar_laz/1722971225678.laz
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
