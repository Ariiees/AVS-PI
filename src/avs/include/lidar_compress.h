#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>

namespace avs {

class LidarCompressor {
public:
  explicit LidarCompressor(const std::string& output_dir);

  void saveAsBin(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud);
  void saveAsLAZ(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud);

private:
  std::string output_dir_;

  std::string getTimestampFilename(const std::string& extension);
};

}  // namespace avs
