#include "avs/lidar_downsample.h"
#include <pcl/filters/voxel_grid.h>
#include <yaml-cpp/yaml.h>
#include <iostream>

namespace avs
{

LidarDownsampler::LidarDownsampler(const std::string& config_path)
: leaf_size_(0.2f) // default
{
  loadConfig(config_path);
}

void LidarDownsampler::loadConfig(const std::string& config_path)
{
  YAML::Node root = YAML::LoadFile(config_path);
  YAML::Node config = root["lidar_downsample"];

  leaf_size_ = config["leaf_size"] ? config["leaf_size"].as<float>() : 0.2f;

  if (leaf_size_ <= 0.0f) {
    std::cerr << "[LidarDownsampler] Invalid leaf_size in config. Using default 0.2\n";
    leaf_size_ = 0.2f;
  }

  std::cout << "[LidarDownsampler] Loaded config: leaf_size = " << leaf_size_ << "\n";
}

pcl::PointCloud<pcl::PointXYZI>::Ptr LidarDownsampler::downsample(
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
  pcl::VoxelGrid<pcl::PointXYZI> voxelGrid;
  voxelGrid.setInputCloud(cloud);
  voxelGrid.setLeafSize(leaf_size_, leaf_size_, leaf_size_);

  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>());
  voxelGrid.filter(*filtered);
  return filtered;
}

} // namespace avs
