#pragma once

#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <filesystem>

namespace avs {

class VideoCompressor {
public:
  VideoCompressor(const std::string& output_dir, const std::string& config_path);
  void addImage(const cv::Mat& image);
  void finalize();

private:
  void createSegment();
  void loadConfig(const std::string& config_path);

  std::filesystem::path output_dir_;
  int fps_;
  int segment_duration_sec_;
  int max_images_per_segment_;
  int segment_index_;
  std::vector<cv::Mat> current_images_;
};

} // namespace avs
