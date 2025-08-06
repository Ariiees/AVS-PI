// fs_benchmark_logger.h
#pragma once

#include <fstream>
#include <string>
#include <chrono>
#include <thread>
#include <sys/resource.h>
#include <unistd.h>

class FSBenchmarkLogger
{
public:
  explicit FSBenchmarkLogger(const std::string &csv_path)
    : csv_path_(csv_path)
  {
    std::ofstream file(csv_path_, std::ios::out | std::ios::trunc);
    file << "timestamp_ms,callback_duration_ms,fsync_latency_ms,write_throughput_MBps,write_amplification,cpu_usage_percent\n";
    file.close();
    last_cpu_time_ = get_process_cpu_time();
    last_wall_time_ = std::chrono::steady_clock::now();
  }

  void log(double callback_duration_ms,
           double fsync_latency_ms,
           double write_throughput_MBps,
           double write_amplification,
           double cpu_usage)
  {
    std::ofstream file(csv_path_, std::ios::out | std::ios::app);
    auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now().time_since_epoch()).count();
    file << now_ms << ","
         << callback_duration_ms << ","
         << fsync_latency_ms << ","
         << write_throughput_MBps << ","
         << write_amplification << ","
         << cpu_usage << "\n";
    file.close();
  }

  double get_cpu_usage()
  {
    auto now = std::chrono::steady_clock::now();
    auto now_cpu_time = get_process_cpu_time();

    double cpu_diff = static_cast<double>(now_cpu_time - last_cpu_time_);
    double wall_diff = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_wall_time_).count();

    last_cpu_time_ = now_cpu_time;
    last_wall_time_ = now;

    return (cpu_diff / (wall_diff * sysconf(_SC_NPROCESSORS_ONLN))) * 100.0;
  }

private:
  std::string csv_path_;
  std::chrono::steady_clock::time_point last_wall_time_;
  long last_cpu_time_;

  long get_process_cpu_time()
  {
    struct rusage usage;
    getrusage(RUSAGE_SELF, &usage);
    return (usage.ru_utime.tv_sec + usage.ru_stime.tv_sec) * 1000 +
           (usage.ru_utime.tv_usec + usage.ru_stime.tv_usec) / 1000;
  }
};
