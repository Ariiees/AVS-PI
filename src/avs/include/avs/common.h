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

// Extract ("YYYY","MM") from "YYYY-MM-DD"
inline std::pair<std::string,std::string> yearMonthFromDay(const std::string& ymd) {
  return { ymd.substr(0,4), ymd.substr(5,2) };
}


// SHA256 of file (hex)
inline bool sha256File(const std::string& path, std::string& out_hex) {
  std::ifstream ifs(path, std::ios::binary);
  if (!ifs) return false;

  // OpenSSL 3.0+ (for no deprecation warnings)
  EVP_MD_CTX* ctx = EVP_MD_CTX_new();
  if (!ctx) return false;

  const EVP_MD* md = EVP_sha256();
  if (EVP_DigestInit_ex(ctx, md, nullptr) != 1) {
    EVP_MD_CTX_free(ctx);
    return false;
  }

  char buf[1 << 16];
  while (ifs.good()) {
    ifs.read(buf, sizeof(buf));
    std::streamsize n = ifs.gcount();
    if (n > 0) {
      if (EVP_DigestUpdate(ctx, buf, static_cast<size_t>(n)) != 1) {
        EVP_MD_CTX_free(ctx);
        return false;
      }
    }
  }

  unsigned char digest[EVP_MAX_MD_SIZE];
  unsigned int len = 0;
  if (EVP_DigestFinal_ex(ctx, digest, &len) != 1) {
    EVP_MD_CTX_free(ctx);
    return false;
  }
  EVP_MD_CTX_free(ctx);

  static const char* HEX = "0123456789abcdef";
  out_hex.assign(len * 2, '0');
  for (unsigned int i = 0; i < len; ++i) {
    out_hex[i*2]   = HEX[(digest[i] >> 4) & 0xF];
    out_hex[i*2+1] = HEX[(digest[i])      & 0xF];
  }
  return true;
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

// Parse a wall-clock RANGE in strict format "YYYY-M-D_HH-MM" (seconds = 00)
// into epoch milliseconds (local time). Returns true on success.
// On failure, if 'err' is provided, it will contain a short reason.
inline bool wallRangeYmdHmToEpochMsLocal(const std::string& start_wall,
                                         const std::string& end_wall,
                                         long long* start_ms,
                                         long long* end_ms,
                                         std::string* err = nullptr)
{
  if (!start_ms || !end_ms) {
    if (err) *err = "start_ms or end_ms is null";
    return false;
  }

  auto parse_one = [](const std::string& wall, long long* out_ms, std::string* err) -> bool {
    // Expect exactly one '_' separating date and time.
    const size_t us = wall.find('_');
    if (us == std::string::npos || wall.find('_', us + 1) != std::string::npos) {
      if (err) *err = "Expected format YYYY-M-D_HH-MM";
      return false;
    }
    const std::string date = wall.substr(0, us);
    const std::string time = wall.substr(us + 1);

    // Date: YYYY-M-D  (M/D may be 1-2 digits)
    const size_t d1 = date.find('-');
    const size_t d2 = (d1 == std::string::npos) ? std::string::npos : date.find('-', d1 + 1);
    if (d1 == std::string::npos || d2 == std::string::npos || date.find('-', d2 + 1) != std::string::npos) {
      if (err) *err = "Date must be YYYY-M-D";
      return false;
    }
    const std::string yS = date.substr(0, d1);
    const std::string mS = date.substr(d1 + 1, d2 - d1 - 1);
    const std::string dS = date.substr(d2 + 1);

    // Time: HH-MM
    const size_t t1 = time.find('-');
    if (t1 == std::string::npos || time.find('-', t1 + 1) != std::string::npos) {
      if (err) *err = "Time must be HH-MM";
      return false;
    }
    const std::string hS = time.substr(0, t1);
    const std::string minS = time.substr(t1 + 1);

    auto is_digits = [](const std::string& s) {
      if (s.empty()) return false;
      for (char c : s) if (c < '0' || c > '9') return false;
      return true;
    };

    if (!is_digits(yS) || !is_digits(mS) || !is_digits(dS) || !is_digits(hS) || !is_digits(minS)) {
      if (err) *err = "Non-numeric date/time field";
      return false;
    }

    int Y = std::stoi(yS);
    int M = std::stoi(mS);
    int D = std::stoi(dS);
    int h = std::stoi(hS);
    int m = std::stoi(minS);

    // Range checks (seconds fixed to 0)
    if (M < 1 || M > 12 || D < 1 || D > 31 || h < 0 || h > 23 || m < 0 || m > 59) {
      if (err) *err = "Out-of-range date/time field";
      return false;
    }

    std::tm tm{};
    tm.tm_year = Y - 1900;
    tm.tm_mon  = M - 1;
    tm.tm_mday = D;
    tm.tm_hour = h;
    tm.tm_min  = m;
    tm.tm_sec  = 0;
    tm.tm_isdst = -1;  // let libc determine DST

    std::time_t tloc = std::mktime(&tm); // interpret as local time
    if (tloc == static_cast<std::time_t>(-1)) {
      if (err) *err = "mktime failed (invalid/ambiguous local time?)";
      return false;
    }

    *out_ms = static_cast<long long>(tloc) * 1000LL;
    return true;
  };

  std::string why;
  if (!parse_one(start_wall, start_ms, err ? err : &why)) return false;
  if (!parse_one(end_wall,   end_ms,   err ? err : &why)) return false;

  if (*end_ms < *start_ms) {
    if (err) *err = "end time earlier than start time";
    return false;
  }
  return true;
}

// --------------------------------Used in Append Logger --------------------------


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
