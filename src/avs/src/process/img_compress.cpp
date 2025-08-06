#include "avs/img_compress.h"
#include <yaml-cpp/yaml.h>
#include <iostream>

namespace fs = std::filesystem;

namespace avs {

VideoCompressor::VideoCompressor(const std::string& output_dir, const std::string& config_path)
  : output_dir_(output_dir), segment_index_(0)
{
  fs::create_directories(output_dir_);
  loadConfig(config_path);
  max_images_per_segment_ = fps_ * segment_duration_sec_;
}

void VideoCompressor::loadConfig(const std::string& config_path)
{
  YAML::Node root = YAML::LoadFile(config_path);

  YAML::Node config = root["image_compress"];

  fps_ = config["fps"] ? config["fps"].as<int>() : 10;
  segment_duration_sec_ = config["segment_duration_sec"] ? config["segment_duration_sec"].as<int>() : 5;

  if (fps_ <= 0 || segment_duration_sec_ <= 0) {
    std::cerr << "[VideoCompressor] Invalid FPS or segment duration in config. Using defaults.\n";
    fps_ = 10;
    segment_duration_sec_ = 5;
  }

  std::cout << "[VideoCompressor] Loaded config: fps = " << fps_
            << ", segment_duration_sec = " << segment_duration_sec_ << "\n";
}

void VideoCompressor::addImage(const cv::Mat& image)
{
  if (image.empty()) return;

  current_images_.push_back(image.clone());

  if (current_images_.size() >= max_images_per_segment_)
  {
    createSegment();
    current_images_.clear();
  }
}

void VideoCompressor::finalize()
{
  if (!current_images_.empty())
  {
    createSegment();
    current_images_.clear();
  }
}

void VideoCompressor::createSegment()
{
  std::string filename = "segment_" + std::to_string(segment_index_) + ".mp4";
  fs::path path = output_dir_ / filename;

  int width = current_images_[0].cols;
  int height = current_images_[0].rows;

  int codec = cv::VideoWriter::fourcc('a', 'v', 'c', '1');
  cv::VideoWriter writer(path.string(), codec, fps_, cv::Size(width, height));

  if (!writer.isOpened()) {
    std::cerr << "[VideoCompressor] Failed to open video writer with H.264 codec, trying MJPG fallback\n";
    codec = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
    writer.open(path.string(), codec, fps_, cv::Size(width, height));
  }

  if (!writer.isOpened()) {
    std::cerr << "[VideoCompressor] Could not open video writer for: " << path << "\n";
    return;
  }

  for (const auto& img : current_images_) {
    writer.write(img);
  }

  writer.release();
  std::cout << "[VideoCompressor] Wrote segment: " << path << "\n";
  ++segment_index_;
}

} // namespace avs
