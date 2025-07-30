#pragma once

#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace avs
{

class ImgDeduplicator
{
public:
  ImgDeduplicator(const std::string& output_dir, const std::string& config_path);
  bool isUniqueAndStore(const sensor_msgs::msg::Image& img_msg);

private:
  std::string output_dir_;
  std::string img_format_;
  std::string extension_;
  int img_quality_;
  int hamming_threshold_;
  bool first_image_;
  std::vector<bool> last_hash_;
  std::vector<int> write_params_;

  void loadConfig(const std::string& config_path);
  cv::Mat rosImgToCvMat(const sensor_msgs::msg::Image& img_msg);
  std::vector<bool> computePhash(const cv::Mat& img);
  int hammingDistance(const std::vector<bool>& hash1, const std::vector<bool>& hash2);
  std::string getTimestampedFilename();
};

} // namespace avs
