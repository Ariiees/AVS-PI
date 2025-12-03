#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>


namespace avs {

class LidarCompressor {
public:
  explicit LidarCompressor(const std::string& output_dir);

  void saveAsBin(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud, const std::string &filename);
  void saveAsLAZ(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud, const std::string &filename);
  void getLAZ(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud, std::vector<uint8_t> &out_bytes);

private:
  std::string output_dir_;
};

}  // namespace avs
