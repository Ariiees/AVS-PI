#include "avs/retrieve_api.h"

#include <filesystem>
#include <cctype>
#include <opencv2/imgcodecs.hpp>
#include <laszip_api.h>

namespace fs = std::filesystem;

namespace avs {

// --------- helpers ---------
bool RetrieveAPI::endsWithInsensitive(const std::string& s, const std::string& suf) {
  if (s.size() < suf.size()) return false;
  for (size_t i = 0; i < suf.size(); ++i) {
    char a = std::tolower(s[s.size() - suf.size() + i]);
    char b = std::tolower(suf[i]);
    if (a != b) return false;
  }
  return true;
}

AvsRecord::Kind RetrieveAPI::detectKind(const std::string& path) {
  if (endsWithInsensitive(path, ".jpg")) return AvsRecord::Kind::IMAGE_JPG;
  if (endsWithInsensitive(path, ".laz")) return AvsRecord::Kind::LIDAR_LAZ;
  return AvsRecord::Kind::OTHER;
}

std::string RetrieveAPI::resolvePath(const std::string& db_path) const {
  if (root_dir_.empty()) return db_path;
  fs::path p(db_path);
  if (p.is_absolute()) return db_path;
  return (fs::path(root_dir_) / p).string();
}

// --------- ctor/dtor ---------
RetrieveAPI::RetrieveAPI(const std::string& db_path, const std::string& root_dir)
  : root_dir_(root_dir) {
  if (!db_.open(db_path, &err_)) {
    is_open_ = false;
  } else {
    is_open_ = true;
  }
}

RetrieveAPI::~RetrieveAPI() = default;

// --------- list() ---------
bool RetrieveAPI::list(const std::string& sensor_id,
                       const std::string& data_type,
                       const std::string& start_wall,
                       const std::string& end_wall,
                       std::vector<AvsRecord>* out) {
  if (!is_open_) { err_ = "DB not open"; return false; }
  if (!out) { err_ = "out is null"; return false; }

  out->clear();
  std::string qerr;
  auto cb = [&](const AvsRow& r) {
    AvsRecord rec;
    rec.sensor_id = r.sensor_id;
    rec.data_type = r.data_type;
    rec.ts_ms     = r.ts_ms;
    rec.path      = resolvePath(r.path);
    rec.kind      = detectKind(rec.path);
    out->push_back(std::move(rec));
  };

  if (!db_.queryByWallRange(sensor_id, data_type, start_wall, end_wall, cb, &qerr)) {
    err_ = qerr;
    return false;
  }
  return true;
}

// --------- image load ---------
bool RetrieveAPI::loadImage(const AvsRecord& rec, cv::Mat* out_bgr) {
  return loadImagePath(rec.path, out_bgr);
}

bool RetrieveAPI::loadImagePath(const std::string& path, cv::Mat* out_bgr) {
  if (!out_bgr) { err_ = "out_bgr is null"; return false; }
  *out_bgr = cv::imread(path, cv::IMREAD_COLOR);
  if (out_bgr->empty()) {
    err_ = "Failed to read image: " + path;
    return false;
  }
  return true;
}

// --------- LAZ load ---------
bool RetrieveAPI::loadLaz(const AvsRecord& rec, pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud) {
  return loadLazPath(rec.path, out_cloud);
}

bool RetrieveAPI::loadLazPath(const std::string& path, pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud) {
  if (!out_cloud) { err_ = "out_cloud is null"; return false; }
  out_cloud->clear();

  // Quick sanity: file exists & regular
  std::error_code ec;
  if (!fs::exists(path, ec) || !fs::is_regular_file(path, ec)) {
    err_ = "LAZ path not found or not a file: " + path;
    return false;
  }

  laszip_POINTER reader = nullptr;
  if (laszip_create(&reader) != 0 || !reader) {
    err_ = "laszip_create failed";
    return false;
  }

  bool ok = true;
  laszip_BOOL is_compressed = 0;

  auto guard = [&](){
    if (reader) { laszip_close_reader(reader); laszip_destroy(reader); }
  };

  // Accept both .laz and .las (laszip can read either; compression flag returned in is_compressed)
  if (laszip_open_reader(reader, path.c_str(), &is_compressed) != 0) {
    err_ = "laszip_open_reader failed: " + path;
    guard(); return false;
  }

  laszip_header* header = nullptr;
  if (laszip_get_header_pointer(reader, &header) != 0 || !header) {
    err_ = "laszip_get_header_pointer failed";
    guard(); return false;
  }

  laszip_point* point = nullptr;
  if (laszip_get_point_pointer(reader, &point) != 0 || !point) {
    err_ = "laszip_get_point_pointer failed";
    guard(); return false;
  }

  const laszip_U64 npts = header->number_of_point_records;
  try {
    out_cloud->reserve(static_cast<std::size_t>(npts));
  } catch (...) {
    // reserve may throw; continue without reserve
  }

  for (laszip_U64 i = 0; i < npts; ++i) {
    if (laszip_read_point(reader) != 0) {
      ok = false; err_ = "laszip_read_point failed at point " + std::to_string(static_cast<unsigned long long>(i));
      break;
    }

    // Apply scale/offset properly
    const double x = header->x_offset + header->x_scale_factor * static_cast<double>(point->X);
    const double y = header->y_offset + header->y_scale_factor * static_cast<double>(point->Y);
    const double z = header->z_offset + header->z_scale_factor * static_cast<double>(point->Z);

    pcl::PointXYZI pt;
    pt.x = static_cast<float>(x);
    pt.y = static_cast<float>(y);
    pt.z = static_cast<float>(z);
    // LAS intensity is usually uint16; store as float
    pt.intensity = static_cast<float>(point->intensity);
    out_cloud->push_back(pt);
  }

  if (laszip_close_reader(reader) != 0) {
    ok = false; err_ = "laszip_close_reader failed";
  }
  laszip_destroy(reader);
  return ok;
}

} // namespace avs
