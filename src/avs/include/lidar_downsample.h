#pragma once

#include <string>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace avs
{

class LidarDownsampler
{
public:
  explicit LidarDownsampler(const std::string& config_path);

  pcl::PointCloud<pcl::PointXYZI>::Ptr downsample(
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);

private:
  void loadConfig(const std::string& config_path);
  float leaf_size_;
};

} // namespace avs
