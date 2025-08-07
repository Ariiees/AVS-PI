#include "avs/lidar_compress.h"
#include "avs/common.h"
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <laszip/laszip_api.h>
#include <yaml-cpp/yaml.h>

namespace fs = std::filesystem;

namespace avs {

LidarCompressor::LidarCompressor(const std::string& output_dir)
: output_dir_(output_dir)
{
  fs::create_directories(output_dir_);
}

void LidarCompressor::saveAsBin(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud, const std::string &filename)
{
  std::ofstream ofs(filename, std::ios::binary);
  for (const auto& pt : cloud->points) {
    ofs.write(reinterpret_cast<const char*>(&pt.x), sizeof(float));
    ofs.write(reinterpret_cast<const char*>(&pt.y), sizeof(float));
    ofs.write(reinterpret_cast<const char*>(&pt.z), sizeof(float));
    ofs.write(reinterpret_cast<const char*>(&pt.intensity), sizeof(float));
  }
  ofs.close();
}

void LidarCompressor::saveAsLAZ(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud, const std::string &filename)
{
  laszip_POINTER writer;
  laszip_create(&writer);

  laszip_header* header;
  laszip_get_header_pointer(writer, &header);
  header->point_data_format = 3;
  header->point_data_record_length = 34;
  header->number_of_point_records = static_cast<laszip_U32>(cloud->size());

  laszip_point* point;
  laszip_get_point_pointer(writer, &point);

  laszip_open_writer(writer, filename.c_str(), true);

  for (const auto& pt : cloud->points) {
    point->X = static_cast<laszip_I32>(pt.x * 1000);
    point->Y = static_cast<laszip_I32>(pt.y * 1000);
    point->Z = static_cast<laszip_I32>(pt.z * 1000);
    point->intensity = static_cast<laszip_U16>(pt.intensity);
    laszip_write_point(writer);
    laszip_update_inventory(writer);
  }

  laszip_close_writer(writer);
  laszip_destroy(writer);
}

}  // namespace avs
