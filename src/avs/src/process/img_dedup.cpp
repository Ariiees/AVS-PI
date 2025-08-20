#include "avs/img_dedup.h"
#include "avs/common.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <numeric>
#include <chrono>
#include <bitset>


namespace fs = std::filesystem;

namespace avs
{

ImgDeduplicator::ImgDeduplicator(const std::string& output_dir, const std::string& config_path)
: output_dir_(output_dir), first_image_(true)
{
  fs::create_directories(output_dir_);
  loadConfig(config_path);
}

void ImgDeduplicator::loadConfig(const std::string& config_path)
{
  YAML::Node root = YAML::LoadFile(config_path);
  YAML::Node config = root["image_dedup"];

  hamming_threshold_ = config["hamming_threshold"] ? config["hamming_threshold"].as<int>() : 2;
  img_format_ = config["img_format"] ? config["img_format"].as<std::string>() : "jpg";
  img_quality_ = config["img_quality"] ? config["img_quality"].as<int>() : 95;

  if (img_format_ == "jpg" || img_format_ == "jpeg")
  {
    write_params_ = {cv::IMWRITE_JPEG_QUALITY, img_quality_};
  }
  else if (img_format_ == "png")
  {
    write_params_ = {cv::IMWRITE_PNG_COMPRESSION, img_quality_};
  }
  else
  {
    RCLCPP_WARN(rclcpp::get_logger("img_dedup"), "Unsupported format '%s', defaulting to jpg", img_format_.c_str());
    write_params_ = {cv::IMWRITE_JPEG_QUALITY, 95};
  }

  std::cout << "[VideoCompressor] Loaded config: image formace = " << img_format_
            << ", image quality = " << img_quality_ << "\n";
}

cv::Mat ImgDeduplicator::rosImgToCvMat(const sensor_msgs::msg::Image& img_msg)
{
  return cv_bridge::toCvCopy(img_msg, "bgr8")->image;
}

std::bitset<64> ImgDeduplicator::computePhash(const cv::Mat& img)
{
  cv::Mat gray;
  cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
  cv::resize(gray, gray, cv::Size(32, 32));
  gray.convertTo(gray, CV_32F);

  cv::Mat dct_image;
  cv::dct(gray, dct_image);

  // Extract top-left 8x8 block
  cv::Mat dct_block = dct_image(cv::Rect(0, 0, 8, 8)).clone();

  // Flatten the block
  std::vector<float> vals;
  vals.reserve(64);
  for (int i = 0; i < 8; ++i)
    for (int j = 0; j < 8; ++j)
      vals.push_back(dct_block.at<float>(i, j));

  // Compute mean excluding the DC term
  float mean = std::accumulate(vals.begin() + 1, vals.end(), 0.0f) / 63.0f;

  // Compute hash bits
  std::bitset<64> hash;
  for (size_t i = 1; i < vals.size(); ++i)
    hash[i] = vals[i] >= mean;
  hash[0] = 0; // DC component

  return hash;
}

int ImgDeduplicator::hammingDistance(const std::bitset<64>& h1, const std::bitset<64>& h2)
{
  return (h1 ^ h2).count();
}

bool ImgDeduplicator::isUniqueAndStore(const sensor_msgs::msg::Image& img_msg, const std::string &filename)
{
  cv::Mat img = rosImgToCvMat(img_msg);
  auto hash = computePhash(img);

  if (first_image_ || hammingDistance(last_hash_, hash) > hamming_threshold_)
  {
    cv::imwrite(filename, img, write_params_);
    last_hash_ = hash;
    first_image_ = false;
    return true;
  }
  return false;
}

} // namespace avs
