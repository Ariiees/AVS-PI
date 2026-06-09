#include <openssl/evp.h>
#include <openssl/rand.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "avs/append_logger.h"
#include "avs/trip_manager.h"

namespace fs = std::filesystem;

namespace {

constexpr std::size_t kKeyBytes = 32;
constexpr std::size_t kIvBytes = 12;
constexpr std::size_t kTagBytes = 16;

struct Options {
  fs::path input_dir;
  fs::path output_root = "/tmp/avs_encryption_report";
  std::size_t max_frames = 271;
  std::string topic = "/benchmark/encrypted_image";
};

void Usage(const char* program) {
  std::cerr
      << "Usage: " << program
      << " --input-dir <image_directory>"
         " [--output-root <path>]"
         " [--max-frames <n>]"
         " [--topic <sensor_topic>]\n";
}

bool ParseArgs(int argc, char** argv, Options* options) {
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto next = [&](const char* name) -> std::string {
      if (i + 1 >= argc) throw std::runtime_error(std::string("missing value for ") + name);
      return argv[++i];
    };

    if (arg == "--input-dir") {
      options->input_dir = next("--input-dir");
    } else if (arg == "--output-root") {
      options->output_root = next("--output-root");
    } else if (arg == "--max-frames") {
      options->max_frames = static_cast<std::size_t>(std::stoull(next("--max-frames")));
    } else if (arg == "--topic") {
      options->topic = next("--topic");
    } else if (arg == "--help" || arg == "-h") {
      return false;
    } else {
      throw std::runtime_error("unknown argument: " + arg);
    }
  }

  if (options->input_dir.empty()) throw std::runtime_error("--input-dir is required");
  if (options->max_frames == 0) throw std::runtime_error("--max-frames must be positive");
  if (options->topic.empty()) throw std::runtime_error("--topic must not be empty");
  return true;
}

bool IsImageFile(const fs::path& path) {
  std::string ext = path.extension().string();
  std::transform(ext.begin(), ext.end(), ext.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return ext == ".jpg" || ext == ".jpeg" || ext == ".png" || ext == ".bmp" ||
         ext == ".tiff" || ext == ".ppm";
}

std::vector<fs::path> ListImages(const fs::path& input_dir, std::size_t max_frames) {
  if (!fs::is_directory(input_dir)) {
    throw std::runtime_error("input directory does not exist: " + input_dir.string());
  }

  std::vector<fs::path> images;
  for (const auto& entry : fs::directory_iterator(input_dir)) {
    if (entry.is_regular_file() && IsImageFile(entry.path())) images.push_back(entry.path());
  }
  std::sort(images.begin(), images.end());
  if (images.size() > max_frames) images.resize(max_frames);
  return images;
}

std::vector<std::uint8_t> ReadFile(const fs::path& path) {
  std::ifstream input(path, std::ios::binary | std::ios::ate);
  if (!input) throw std::runtime_error("failed to open input: " + path.string());

  const std::streamsize size = input.tellg();
  if (size < 0) throw std::runtime_error("failed to get input size: " + path.string());
  input.seekg(0, std::ios::beg);

  std::vector<std::uint8_t> data(static_cast<std::size_t>(size));
  if (size > 0 && !input.read(reinterpret_cast<char*>(data.data()), size)) {
    throw std::runtime_error("failed to read input: " + path.string());
  }
  return data;
}

class Aes256Gcm {
 public:
  Aes256Gcm() : key_(kKeyBytes) {
    if (RAND_bytes(key_.data(), static_cast<int>(key_.size())) != 1) {
      throw std::runtime_error("failed to generate AES-256 key");
    }
  }

  std::vector<std::uint8_t> Encrypt(const std::vector<std::uint8_t>& plain) const {
    std::vector<std::uint8_t> iv(kIvBytes);
    std::vector<std::uint8_t> tag(kTagBytes);
    std::vector<std::uint8_t> cipher(plain.size() + EVP_MAX_BLOCK_LENGTH);

    if (RAND_bytes(iv.data(), static_cast<int>(iv.size())) != 1) {
      throw std::runtime_error("failed to generate AES-GCM IV");
    }

    EVP_CIPHER_CTX* raw_ctx = EVP_CIPHER_CTX_new();
    if (!raw_ctx) throw std::runtime_error("EVP_CIPHER_CTX_new failed");

    int output_len = 0;
    int final_len = 0;
    bool ok =
        EVP_EncryptInit_ex(raw_ctx, EVP_aes_256_gcm(), nullptr, nullptr, nullptr) == 1 &&
        EVP_CIPHER_CTX_ctrl(raw_ctx, EVP_CTRL_GCM_SET_IVLEN, static_cast<int>(iv.size()), nullptr) ==
            1 &&
        EVP_EncryptInit_ex(raw_ctx, nullptr, nullptr, key_.data(), iv.data()) == 1 &&
        EVP_EncryptUpdate(raw_ctx, cipher.data(), &output_len, plain.data(),
                          static_cast<int>(plain.size())) == 1 &&
        EVP_EncryptFinal_ex(raw_ctx, cipher.data() + output_len, &final_len) == 1 &&
        EVP_CIPHER_CTX_ctrl(raw_ctx, EVP_CTRL_GCM_GET_TAG, static_cast<int>(tag.size()), tag.data()) ==
            1;

    EVP_CIPHER_CTX_free(raw_ctx);
    if (!ok) throw std::runtime_error("AES-256-GCM encryption failed");

    cipher.resize(static_cast<std::size_t>(output_len + final_len));
    std::vector<std::uint8_t> packed;
    packed.reserve(iv.size() + tag.size() + cipher.size());
    packed.insert(packed.end(), iv.begin(), iv.end());
    packed.insert(packed.end(), tag.begin(), tag.end());
    packed.insert(packed.end(), cipher.begin(), cipher.end());
    return packed;
  }

  std::vector<std::uint8_t> Decrypt(const std::vector<std::uint8_t>& packed) const {
    if (packed.size() < kIvBytes + kTagBytes) {
      throw std::runtime_error("encrypted record is too short");
    }

    const std::uint8_t* iv = packed.data();
    const std::uint8_t* tag = packed.data() + kIvBytes;
    const std::uint8_t* cipher = packed.data() + kIvBytes + kTagBytes;
    const std::size_t cipher_size = packed.size() - kIvBytes - kTagBytes;
    std::vector<std::uint8_t> plain(cipher_size);

    EVP_CIPHER_CTX* raw_ctx = EVP_CIPHER_CTX_new();
    if (!raw_ctx) throw std::runtime_error("EVP_CIPHER_CTX_new failed");

    int output_len = 0;
    int final_len = 0;
    bool ok =
        EVP_DecryptInit_ex(raw_ctx, EVP_aes_256_gcm(), nullptr, nullptr, nullptr) == 1 &&
        EVP_CIPHER_CTX_ctrl(raw_ctx, EVP_CTRL_GCM_SET_IVLEN, static_cast<int>(kIvBytes), nullptr) ==
            1 &&
        EVP_DecryptInit_ex(raw_ctx, nullptr, nullptr, key_.data(), iv) == 1 &&
        EVP_DecryptUpdate(raw_ctx, plain.data(), &output_len, cipher,
                          static_cast<int>(cipher_size)) == 1 &&
        EVP_CIPHER_CTX_ctrl(raw_ctx, EVP_CTRL_GCM_SET_TAG, static_cast<int>(kTagBytes),
                            const_cast<std::uint8_t*>(tag)) == 1 &&
        EVP_DecryptFinal_ex(raw_ctx, plain.data() + output_len, &final_len) == 1;

    EVP_CIPHER_CTX_free(raw_ctx);
    if (!ok) throw std::runtime_error("AES-256-GCM authentication failed");

    plain.resize(static_cast<std::size_t>(output_len + final_len));
    return plain;
  }

 private:
  std::vector<std::uint8_t> key_;
};

double Percentile(std::vector<double> values, double percentile) {
  if (values.empty()) return 0.0;
  std::sort(values.begin(), values.end());
  const double position = percentile * static_cast<double>(values.size() - 1);
  const std::size_t lower = static_cast<std::size_t>(position);
  const std::size_t upper = std::min(lower + 1, values.size() - 1);
  const double fraction = position - static_cast<double>(lower);
  return values[lower] + fraction * (values[upper] - values[lower]);
}

std::uint64_t NowSystemNs() {
  return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                        std::chrono::system_clock::now().time_since_epoch())
                                        .count());
}

}  // namespace

int main(int argc, char** argv) {
  try {
    Options options;
    if (!ParseArgs(argc, argv, &options)) {
      Usage(argv[0]);
      return argc > 1 ? 0 : 2;
    }

    const std::vector<fs::path> images = ListImages(options.input_dir, options.max_frames);
    if (images.empty()) throw std::runtime_error("no supported image files found");

    const std::string topic_folder = "encryption_benchmark";
    const std::string day = "benchmark";
    const fs::path day_path = options.output_root / topic_folder / day;
    fs::create_directories(day_path);

    avs::TripManager trip_manager;
    const int trip_id = trip_manager.GetTripId(day_path.string());
    avs::AppendLogger logger(options.output_root.string(), options.topic);
    logger.startTrip(day, topic_folder, trip_id, NowSystemNs());

    Aes256Gcm encryptor;
    std::vector<double> latencies_ms;
    latencies_ms.reserve(images.size());
    std::uint64_t plain_bytes = 0;
    std::uint64_t encrypted_bytes = 0;

    for (const fs::path& image : images) {
      const std::vector<std::uint8_t> plain = ReadFile(image);
      const auto start = std::chrono::steady_clock::now();
      const std::vector<std::uint8_t> packed = encryptor.Encrypt(plain);
      logger.appendRecord(NowSystemNs(), packed);
      const auto end = std::chrono::steady_clock::now();

      if (encryptor.Decrypt(packed) != plain) {
        throw std::runtime_error("decrypted payload mismatch: " + image.string());
      }

      latencies_ms.push_back(std::chrono::duration<double, std::milli>(end - start).count());
      plain_bytes += plain.size();
      encrypted_bytes += packed.size();
    }

    logger.endTrip(NowSystemNs());

    double total_ms = 0.0;
    for (double latency : latencies_ms) total_ms += latency;
    const double average_ms = total_ms / static_cast<double>(latencies_ms.size());
    const double max_ms = *std::max_element(latencies_ms.begin(), latencies_ms.end());

    std::cout << std::fixed << std::setprecision(3)
              << "ENCRYPTION_REPORT"
              << " frames=" << latencies_ms.size()
              << " plain_bytes=" << plain_bytes
              << " encrypted_bytes=" << encrypted_bytes
              << " avg_ms=" << average_ms
              << " p50_ms=" << Percentile(latencies_ms, 0.50)
              << " p95_ms=" << Percentile(latencies_ms, 0.95)
              << " p99_ms=" << Percentile(latencies_ms, 0.99)
              << " max_ms=" << max_ms
              << " output_root=" << options.output_root.string() << '\n';
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "encryption_report error: " << error.what() << '\n';
    Usage(argv[0]);
    return 2;
  }
}
