// offload_sender_tcp_cloud_batch.cpp
// Build
//   g++ -O2 -std=c++17 offload_sender_tcp_cloud_batch.cpp -o offload_sender_tcp_cloud_batch
//
// Run
//   ros2 run avs offload <dst_ip> <dst_port> <start_ts_ns> <end_ts_ns> <topics_csv> [ssd_root] [max_records] [runs] [batch_records]
//
// Behavior changes
//   Topic is sent once in Start, per record topic bytes removed
//   Records are batched into one kMsgRec frame with many entries
//   One writev per batch to reduce syscall overhead
//
// Printed metrics remain compatible with your current parser output fields

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/resource.h>
#include <sys/uio.h>
#include <unistd.h>

#include <chrono>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>

#include "avs/retrieve_api.h"

namespace fs = std::filesystem;

static inline std::uint64_t mono_ns() {
  using namespace std::chrono;
  return (std::uint64_t)duration_cast<std::chrono::nanoseconds>(
           std::chrono::steady_clock::now().time_since_epoch())
    .count();
}

static bool send_all(int fd, const void* buf, std::size_t n) {
  const std::uint8_t* p = static_cast<const std::uint8_t*>(buf);
  std::size_t off = 0;
  while (off < n) {
    ssize_t rc = ::send(fd, p + off, n - off, 0);
    if (rc <= 0) return false;
    off += (std::size_t)rc;
  }
  return true;
}

static bool recv_all(int fd, void* buf, std::size_t n) {
  std::uint8_t* p = static_cast<std::uint8_t*>(buf);
  std::size_t off = 0;
  while (off < n) {
    ssize_t rc = ::recv(fd, p + off, n - off, 0);
    if (rc <= 0) return false;
    off += (std::size_t)rc;
  }
  return true;
}

static std::uint32_t crc32_update(std::uint32_t crc, const std::uint8_t* data, std::size_t n) {
  static std::uint32_t table[256];
  static bool init = false;
  if (!init) {
    for (std::uint32_t i = 0; i < 256; ++i) {
      std::uint32_t c = i;
      for (int k = 0; k < 8; ++k) c = (c & 1) ? (0xEDB88320U ^ (c >> 1)) : (c >> 1);
      table[i] = c;
    }
    init = true;
  }
  std::uint32_t c = crc;
  for (std::size_t i = 0; i < n; ++i) c = table[(c ^ data[i]) & 0xFFU] ^ (c >> 8);
  return c;
}

static std::uint32_t crc32_full(const std::uint8_t* data, std::size_t n) {
  std::uint32_t c = 0xFFFFFFFFU;
  c = crc32_update(c, data, n);
  return c ^ 0xFFFFFFFFU;
}

static double ns_to_ms(std::uint64_t ns) { return (double)ns / 1e6; }
static double ns_to_s(std::uint64_t ns) { return (double)ns / 1e9; }

static std::vector<std::string> split_csv(const std::string& s) {
  std::vector<std::string> out;
  std::string cur;
  for (char ch : s) {
    if (ch == ',') {
      if (!cur.empty()) out.push_back(cur);
      cur.clear();
    } else {
      cur.push_back(ch);
    }
  }
  if (!cur.empty()) out.push_back(cur);
  return out;
}

static double mean(const std::vector<double>& v) {
  if (v.empty()) return 0.0;
  double s = 0.0;
  for (double x : v) s += x;
  return s / (double)v.size();
}

static double stdev_sample(const std::vector<double>& v) {
  if (v.size() < 2) return 0.0;
  double m = mean(v);
  double s2 = 0.0;
  for (double x : v) {
    double d = x - m;
    s2 += d * d;
  }
  return std::sqrt(s2 / (double)(v.size() - 1));
}

static double t95(int df) {
  static const double tab[31] = {
    0.0,
    12.706, 4.303, 3.182, 2.776, 2.571,
    2.447, 2.365, 2.306, 2.262, 2.228,
    2.201, 2.179, 2.160, 2.145, 2.131,
    2.120, 2.110, 2.101, 2.093, 2.086,
    2.080, 2.074, 2.069, 2.064, 2.060,
    2.056, 2.052, 2.048, 2.045, 2.042
  };
  if (df <= 0) return 0.0;
  if (df < 31) return tab[df];
  return 1.96;
}

static double ci_halfwidth95(const std::vector<double>& v) {
  int n = (int)v.size();
  if (n < 2) return 0.0;
  double sd = stdev_sample(v);
  double tc = t95(n - 1);
  return tc * sd / std::sqrt((double)n);
}

static double rusage_cpu_seconds(const rusage& ru) {
  double u = (double)ru.ru_utime.tv_sec + (double)ru.ru_utime.tv_usec / 1e6;
  double s = (double)ru.ru_stime.tv_sec + (double)ru.ru_stime.tv_usec / 1e6;
  return u + s;
}

static constexpr std::uint32_t kMagic = 0x34535641U;
static constexpr std::uint16_t kVer = 5;

enum : std::uint16_t {
  kMsgStart = 1,
  kMsgRec   = 2,
  kMsgEnd   = 3,
  kMsgAck   = 4
};

#pragma pack(push, 1)
struct MsgHdr {
  std::uint32_t magic;
  std::uint16_t ver;
  std::uint16_t type;
  std::uint64_t request_id;
  std::uint64_t seq;
  std::uint32_t body_len;
};

// Start body now includes topic once
struct StartBodyV2 {
  std::uint64_t planned_records;
  std::uint64_t sender_first_ns;
  std::uint32_t topic_len;
  std::uint32_t reserved0;
  // followed by topic bytes topic_len
};

// Each record entry inside a batch frame
struct RecEntryHdr {
  std::uint64_t ts_ns;
  std::uint32_t payload_len;
  std::uint32_t crc32;
  // followed by payload bytes payload_len
};

struct EndBody {
  std::uint64_t sender_last_ns;
  std::uint64_t sent_records_ok;
  std::uint64_t sent_payload_bytes_ok;
  std::uint64_t sent_frames;
  std::uint64_t sent_bytes_total;
  std::uint64_t last_seq_plus1;
};

struct AckBody {
  std::uint64_t sender_first_ns;
  std::uint64_t server_first_recv_ns;
  std::uint64_t server_last_recv_ns;
  std::uint64_t server_ack_send_ns;

  std::uint64_t recv_records_ok;
  std::uint64_t recv_records_crc_fail;
  std::uint64_t recv_payload_bytes_ok;
  std::uint64_t recv_bytes_total;

  std::uint32_t status;
  std::uint32_t reserved0;
};
#pragma pack(pop)

static void usage(const char* p) {
  std::cerr
    << "Usage:\n"
    << "  " << p << " dst_ip dst_port start_ts_ns end_ts_ns topics_csv max_records [runs] [batch_records]\n";
  std::exit(2);
}

int main(int argc, char** argv) {
  if (argc < 7) usage(argv[0]);

  std::string dst_ip = argv[1];
  int dst_port = std::stoi(argv[2]);
  std::uint64_t start_ns = std::stoull(argv[3]);
  std::uint64_t end_ns = std::stoull(argv[4]);
  std::string topics_csv = argv[5];
  std::uint64_t max_records = argv[6] ? std::stoull(argv[6]) : 0;

  fs::path ssd_root("/home/avs/DATA/SSD");
  int runs = 10;
  std::uint32_t batch_records = 128;

  if (argc >= 8) runs = std::stoi(argv[8]);
  if (argc >= 9) batch_records = (std::uint32_t)std::stoul(argv[9]);

  if (dst_ip.empty() || dst_port <= 0 || start_ns == 0 || end_ns == 0 || end_ns < start_ns || topics_csv.empty()) {
    usage(argv[0]);
  }
  if (runs <= 0) runs = 1;
  if (batch_records == 0) batch_records = 1;

  std::vector<std::string> topics = split_csv(topics_csv);
  if (topics.size() != 1) {
    std::cerr << "error this optimized sender currently expects exactly one topic per run\n";
    return 2;
  }
  const std::string topic = topics[0];
  const std::uint32_t topic_len = (std::uint32_t)topic.size();

  avs::RetrieveAPI api(ssd_root);

  std::string qerr;
  auto refs = api.QueryRefs(topic, start_ns, end_ns, &qerr);
  if (refs.empty()) {
    std::cerr << "warn QueryRefs empty topic=" << topic << " err=" << qerr << "\n";
  }

  std::uint64_t planned_total = refs.size();
  if (max_records > 0 && planned_total > max_records) planned_total = max_records;

  std::vector<double> v_latency_ms;
  std::vector<double> v_goodput_ratio;
  std::vector<double> v_cpu_pct;

  v_latency_ms.reserve((std::size_t)runs);
  v_goodput_ratio.reserve((std::size_t)runs);
  v_cpu_pct.reserve((std::size_t)runs);

  for (int run_i = 0; run_i < runs; ++run_i) {
    std::uint64_t request_id = mono_ns() ^ (std::uint64_t(::getpid()) << 32) ^ (std::uint64_t)run_i;
    std::uint64_t seq = 0;

    std::uint64_t bytes_sent_total = 0;
    std::uint64_t frames_sent = 0;
    std::uint64_t records_sent_ok = 0;
    std::uint64_t payload_bytes_sent_ok = 0;

    int s = ::socket(AF_INET, SOCK_STREAM, 0);
    if (s < 0) { std::perror("socket"); return 1; }

    int sndbuf = 8 * 1024 * 1024;
    (void)setsockopt(s, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));

    sockaddr_in dst{};
    dst.sin_family = AF_INET;
    dst.sin_port = htons((std::uint16_t)dst_port);
    if (::inet_pton(AF_INET, dst_ip.c_str(), &dst.sin_addr) != 1) {
      std::cerr << "inet_pton failed\n";
      ::close(s);
      return 1;
    }

    if (::connect(s, (sockaddr*)&dst, sizeof(dst)) != 0) {
      std::perror("connect");
      ::close(s);
      return 1;
    }

    auto send_frame_simple = [&](std::uint16_t type, const void* body, std::uint32_t body_len) -> bool {
      MsgHdr h{};
      h.magic = kMagic;
      h.ver = kVer;
      h.type = type;
      h.request_id = request_id;
      h.seq = seq;
      h.body_len = body_len;

      if (!send_all(s, &h, sizeof(h))) return false;
      if (body_len && !send_all(s, body, body_len)) return false;

      frames_sent += 1;
      bytes_sent_total += (std::uint64_t)sizeof(h) + (std::uint64_t)body_len;
      seq += 1;
      return true;
    };

    std::uint64_t sender_first_ns = mono_ns();

    // Start with topic included once
    StartBodyV2 sb{};
    sb.planned_records = planned_total;
    sb.sender_first_ns = sender_first_ns;
    sb.topic_len = topic_len;
    sb.reserved0 = 0;

    {
      MsgHdr h{};
      h.magic = kMagic;
      h.ver = kVer;
      h.type = kMsgStart;
      h.request_id = request_id;
      h.seq = seq;
      h.body_len = (std::uint32_t)(sizeof(StartBodyV2) + topic_len);

      iovec iov[2];
      iov[0].iov_base = &h;
      iov[0].iov_len = sizeof(h);
      iov[1].iov_base = (void*)(&sb);
      iov[1].iov_len = sizeof(sb);

      if (::writev(s, iov, 2) != (ssize_t)(sizeof(h) + sizeof(sb))) {
        std::cerr << "send start fixed part failed\n";
        ::close(s);
        return 1;
      }
      if (topic_len && !send_all(s, topic.data(), topic_len)) {
        std::cerr << "send start topic failed\n";
        ::close(s);
        return 1;
      }

      frames_sent += 1;
      bytes_sent_total += (std::uint64_t)sizeof(h) + (std::uint64_t)sizeof(sb) + (std::uint64_t)topic_len;
      seq += 1;
    }

    std::vector<std::uint8_t> payload;
    std::vector<std::uint8_t> batch_buf;
    batch_buf.reserve(4 * 1024 * 1024);

    std::uint64_t sent_records = 0;

    rusage ru0{};
    rusage ru1{};
    (void)::getrusage(RUSAGE_SELF, &ru0);

    std::uint64_t send_wall_begin = mono_ns();

    auto flush_batch = [&](bool force) -> bool {
      if (!force && batch_buf.empty()) return true;
      if (batch_buf.empty()) return true;

      MsgHdr h{};
      h.magic = kMagic;
      h.ver = kVer;
      h.type = kMsgRec;
      h.request_id = request_id;
      h.seq = seq;
      h.body_len = (std::uint32_t)batch_buf.size();

      iovec iov[2];
      iov[0].iov_base = &h;
      iov[0].iov_len = sizeof(h);
      iov[1].iov_base = batch_buf.data();
      iov[1].iov_len = batch_buf.size();

      ssize_t need = (ssize_t)(sizeof(h) + batch_buf.size());
      ssize_t got = ::writev(s, iov, 2);
      if (got != need) return false;

      frames_sent += 1;
      bytes_sent_total += (std::uint64_t)need;
      seq += 1;

      batch_buf.clear();
      return true;
    };

    std::uint32_t in_batch = 0;

    for (const auto& r : refs) {
      if (max_records > 0 && sent_records >= max_records) break;

      std::string perr;
      bool ok = api.LoadPayload(r, payload, &perr);
      if (!ok) continue;

      std::uint32_t crc = 0;
      if (!payload.empty()) crc = crc32_full(payload.data(), payload.size());

      RecEntryHdr eh{};
      eh.ts_ns = r.ts_ns;
      eh.payload_len = (std::uint32_t)payload.size();
      eh.crc32 = crc;

      std::size_t need = sizeof(RecEntryHdr) + payload.size();
      if (need > (16u * 1024u * 1024u)) {
        std::cerr << "warn payload too large to batch\n";
        continue;
      }

      batch_buf.insert(batch_buf.end(),
                       (std::uint8_t*)&eh,
                       (std::uint8_t*)&eh + sizeof(eh));
      if (!payload.empty()) {
        batch_buf.insert(batch_buf.end(), payload.begin(), payload.end());
      }

      records_sent_ok += 1;
      payload_bytes_sent_ok += (std::uint64_t)payload.size();
      sent_records += 1;

      in_batch += 1;
      if (in_batch >= batch_records) {
        if (!flush_batch(true)) {
          std::cerr << "send batch failed\n";
          break;
        }
        in_batch = 0;
      }
    }

    if (!flush_batch(true)) {
      std::cerr << "send final batch failed\n";
      ::close(s);
      return 1;
    }

    std::uint64_t send_wall_end = mono_ns();
    (void)::getrusage(RUSAGE_SELF, &ru1);

    EndBody eb{};
    eb.sender_last_ns = mono_ns();
    eb.sent_records_ok = records_sent_ok;
    eb.sent_payload_bytes_ok = payload_bytes_sent_ok;
    eb.sent_frames = frames_sent;
    eb.sent_bytes_total = bytes_sent_total;
    eb.last_seq_plus1 = seq;

    (void)send_frame_simple(kMsgEnd, &eb, sizeof(eb));

    MsgHdr ah{};
    if (!recv_all(s, &ah, sizeof(ah))) {
      std::cerr << "recv ack hdr failed\n";
      ::close(s);
      return 1;
    }
    if (ah.magic != kMagic || ah.ver != kVer || ah.type != kMsgAck || ah.request_id != request_id || ah.body_len != sizeof(AckBody)) {
      std::cerr << "bad ack hdr\n";
      ::close(s);
      return 1;
    }

    AckBody ack{};
    if (!recv_all(s, &ack, sizeof(ack))) {
      std::cerr << "recv ack body failed\n";
      ::close(s);
      return 1;
    }

    std::uint64_t ack_recv_ns = mono_ns();

    (void)::shutdown(s, SHUT_RDWR);
    ::close(s);

    double send_wall_s = ns_to_s(send_wall_end - send_wall_begin);
    double cpu_s = rusage_cpu_seconds(ru1) - rusage_cpu_seconds(ru0);
    double cpu_pct = (send_wall_s > 0.0) ? (100.0 * cpu_s / send_wall_s) : 0.0;

    double latency_ms = ns_to_ms(ack_recv_ns - sender_first_ns);
    double goodput_ratio = (bytes_sent_total > 0) ? (100.0 * (double)payload_bytes_sent_ok / (double)bytes_sent_total) : 0.0;

    if (ack.status == 0) {
      v_latency_ms.push_back(latency_ms);
      v_goodput_ratio.push_back(goodput_ratio);
      v_cpu_pct.push_back(cpu_pct);
    } else {
      std::cerr << "warn run failed run=" << run_i << " ack_status=" << ack.status << "\n";
    }

    std::cout
      << "RUN"
      << " i=" << run_i
      << " ack_status=" << ack.status
      << " latency_ms=" << latency_ms
      << " cpu_pct=" << cpu_pct
      << " records_sent_ok=" << records_sent_ok
      << " total_bytes_sent=" << bytes_sent_total
      << " recv_records_ok=" << ack.recv_records_ok
      << " crc_fail=" << ack.recv_records_crc_fail
      << "\n";
  }

  std::cout
    << "OFFLOAD_SUM"
    << " runs_requested=" << runs
    << " runs_ok=" << v_latency_ms.size()
    << " goodput_ratio_avg=" << mean(v_goodput_ratio)
    << " goodput_ratio_ci95=" << ci_halfwidth95(v_goodput_ratio)
    << " latency_ms_avg=" << mean(v_latency_ms)
    << " latency_ms_ci95=" << ci_halfwidth95(v_latency_ms)
    << " cpu_pct_avg=" << mean(v_cpu_pct)
    << " cpu_pct_ci95=" << ci_halfwidth95(v_cpu_pct)
    << "\n";

  return 0;
}
