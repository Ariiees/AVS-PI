#pragma once
#include <string>
#include <vector>
#include <opencv2/core.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "avs/db_operation.h"  // AvsDb, AvsRow

namespace avs {

// One row from AVS joined with a resolved absolute path and a simple kind tag.
struct AvsRecord {
  std::string sensor_id;
  std::string data_type;
  long long   ts_ms{0};
  std::string path;   // resolved path (root_dir + DB path if DB path is relative)

  enum class Kind { IMAGE_JPG, LIDAR_LAZ, OTHER } kind{Kind::OTHER};
};

// Lightweight retrieval/loader API.
// - Construct with DB path and optional root_dir (prefix for relative DB paths).
// - list(...) collects matching records into a vector.
// - loadImage/loadLaz read files into cv::Mat / PCL cloud for the caller.
class RetrieveAPI {
public:
  RetrieveAPI(const std::string& db_path, const std::string& root_dir = "");
  ~RetrieveAPI();

  bool isOpen() const { return is_open_; }
  const std::string& lastError() const { return err_; }

  // Query by wall-clock range, e.g., "2025-8-20_13-20" .. "2025-8-20_14-00"
  // Uses AvsDb::queryByWallRange under the hood.
  bool list(const std::string& sensor_id,
            const std::string& data_type,
            const std::string& start_wall,
            const std::string& end_wall,
            std::vector<AvsRecord>* out);

  // Loading helpers by record (uses 'path' inside record).
  bool loadImage(const AvsRecord& rec, cv::Mat* out_bgr);
  bool loadLaz(const AvsRecord& rec, pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud);

  // Loading helpers by explicit path (if you just want the I/O bits).
  bool loadImagePath(const std::string& path, cv::Mat* out_bgr);
  bool loadLazPath(const std::string& path, pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud);

private:
  AvsDb db_;
  bool is_open_{false};
  std::string root_dir_;  // may be empty; used to resolve relative DB paths
  std::string err_;

  static bool endsWithInsensitive(const std::string& s, const std::string& suf);
  static AvsRecord::Kind detectKind(const std::string& path);
  std::string resolvePath(const std::string& db_path) const;
};

} // namespace avs
