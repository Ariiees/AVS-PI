#pragma once

#include "rosidl_runtime_c/message_initialization.h"
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <bitset>

namespace avs
{

class ImgDeduplicator
{
public:
  ImgDeduplicator(const std::string& output_dir, const std::string& config_path);
  bool isUniqueAndStore(const sensor_msgs::msg::Image& img_msg, const std::string &filename);
  bool isUnique(const sensor_msgs::msg::Image& img_msg);

private:
  std::string output_dir_;
  std::string img_format_;
  int img_quality_;
  int hamming_threshold_;
  bool first_image_;
  std::bitset<64> last_hash_;
  std::vector<int> write_params_;

  void loadConfig(const std::string& config_path);
  cv::Mat rosImgToCvMat(const sensor_msgs::msg::Image& img_msg);
  std::bitset<64> computePhash(const cv::Mat& img);
  int hammingDistance(const std::bitset<64>& h1, const std::bitset<64>& h2);
  std::string getTimestampedFilename();
};

} // namespace avs
