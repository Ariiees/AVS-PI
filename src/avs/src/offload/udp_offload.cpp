// offload_sender.cpp
// Build
//   g++ -O2 -std=c++17 offload_sender.cpp -o offload_sender -lpthread
//
// Run example
//   ./offload_sender --dst_ip 192.168.1.10 --dst_port 9000 --start_ts_ns 1 --end_ts_ns 2 --topics /cam,/lidar
//
// Protocol
//   UDP uses dst_port
//   TCP uses dst_port + 1 unless --tcp_port is set
//
// Output
//   One line per run plus a final summary with averages and 95 percent CI for wall_s and delivery_ratio.

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <numeric>
#include <string>
#include <thread>
#include <vector>
#include <math.h>

#include "avs/retrieve_api.h"

namespace fs = std::filesystem;

static inline std::uint64_t monotonic_ns() {
  using namespace std::chrono;
  return duration_cast<nanoseconds>(steady_clock::now().time_since_epoch()).count();
}

static inline std::uint64_t now_wall_ns() {
  using namespace std::chrono;
  return duration_cast<nanoseconds>(system_clock::now().time_since_epoch()).count();
}

static std::uint32_t crc32_u(const std::uint8_t* data, std::size_t n) {
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
  std::uint32_t c = 0xFFFFFFFFU;
  for (std::size_t i = 0; i < n; ++i) c = table[(c ^ data[i]) & 0xFFU] ^ (c >> 8);
  return c ^ 0xFFFFFFFFU;
}

static bool recv_all(int fd, void* buf, std::size_t n) {
  std::uint8_t* p = static_cast<std::uint8_t*>(buf);
  std::size_t off = 0;
  while (off < n) {
    ssize_t rc = ::recv(fd, p + off, n - off, 0);
    if (rc <= 0) return false;
    off += static_cast<std::size_t>(rc);
  }
  return true;
}

static bool send_all(int fd, const void* buf, std::size_t n) {
  const std::uint8_t* p = static_cast<const std::uint8_t*>(buf);
  std::size_t off = 0;
  while (off < n) {
    ssize_t rc = ::send(fd, p + off, n - off, 0);
    if (rc <= 0) return false;
    off += static_cast<std::size_t>(rc);
  }
  return true;
}

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
  return s / double(v.size());
}

static double stdev_sample(const std::vector<double>& v) {
  if (v.size() < 2) return 0.0;
  double m = mean(v);
  double s2 = 0.0;
  for (double x : v) {
    double d = x - m;
    s2 += d * d;
  }
  return std::sqrt(s2 / double(v.size() - 1));
}

// t critical values for 95 percent CI, df 1..30, index is df
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
  int n = int(v.size());
  if (n < 2) return 0.0;
  double sd = stdev_sample(v);
  double tc = t95(n - 1);
  return tc * sd / std::sqrt(double(n));
}

static constexpr std::uint32_t kMagic = 0x31535641U;  // AVS1 little endian
static constexpr std::uint16_t kVer = 1;

enum : std::uint16_t {
  kMsgStart = 1,
  kMsgRec = 2,
  kMsgEnd = 3,
  kMsgAck = 4
};

#pragma pack(push, 1)
struct MsgHdr {
  std::uint32_t magic;
  std::uint16_t ver;
  std::uint16_t type;
  std::uint64_t request_id;
  std::uint64_t seq;
};

struct StartBody {
  std::uint32_t sender_ip_be;
  std::uint16_t sender_ack_port_be;
  std::uint16_t reserved0;
  std::uint64_t expected_records;
  std::uint64_t expected_payload_bytes;
  std::uint32_t mtu_payload;
  std::uint32_t reserved1;
};

struct RecBody {
  std::uint64_t ts_ns;
  std::uint32_t topic_len;
  std::uint32_t payload_len;
  std::uint32_t crc32;
  std::uint16_t frag_idx;
  std::uint16_t frag_cnt;
  std::uint32_t frag_payload_len;
  // followed by: topic bytes, then fragment payload bytes
};

struct EndBody {
  std::uint64_t sender_first_mono_ns;
  std::uint64_t sender_last_mono_ns;
};

struct AckBody {
  std::uint64_t received_records_ok;
  std::uint64_t received_payload_bytes_ok;
  std::uint64_t received_packets;
  std::uint64_t wall_ns;
};
#pragma pack(pop)

struct RunResult {
  std::uint64_t expected_records = 0;
  std::uint64_t expected_payload_bytes = 0;

  std::uint64_t sent_bytes_total = 0;
  std::uint64_t sent_packets = 0;
  double wall_s = 0.0;

  double delivery_ratio = 0.0;
};

static bool udp_recv_ack(int ack_sock, std::uint64_t request_id, AckBody* out, int timeout_ms) {
  fd_set rfds;
  FD_ZERO(&rfds);
  FD_SET(ack_sock, &rfds);
  timeval tv{};
  tv.tv_sec = timeout_ms / 1000;
  tv.tv_usec = (timeout_ms % 1000) * 1000;

  int rc = ::select(ack_sock + 1, &rfds, nullptr, nullptr, &tv);
  if (rc <= 0) return false;

  std::vector<std::uint8_t> buf(2048);
  sockaddr_in src{};
  socklen_t sl = sizeof(src);
  ssize_t n = ::recvfrom(ack_sock, buf.data(), buf.size(), 0, (sockaddr*)&src, &sl);
  if (n < (ssize_t)(sizeof(MsgHdr) + sizeof(AckBody))) return false;

  MsgHdr h{};
  std::memcpy(&h, buf.data(), sizeof(h));
  if (h.magic != kMagic || h.ver != kVer || h.type != kMsgAck) return false;
  if (h.request_id != request_id) return false;

  AckBody a{};
  std::memcpy(&a, buf.data() + sizeof(MsgHdr), sizeof(a));
  *out = a;
  return true;
}

static RunResult run_udp(const std::string& dst_ip,
                         int udp_port,
                         const std::vector<std::pair<std::string, std::vector<avs::DataRef>>>& work,
                         avs::RetrieveAPI& api,
                         const fs::path& ssd_root,
                         std::uint64_t max_records,
                         std::size_t mtu_payload,
                         int ack_timeout_ms) {
  (void)ssd_root;

  RunResult rr{};

  std::uint64_t total_refs = 0;
  for (const auto& t : work) total_refs += t.second.size();
  if (max_records > 0 && total_refs > max_records) total_refs = max_records;
  rr.expected_records = total_refs;

  // Compute expected payload bytes by loading payload once
  std::vector<std::uint8_t> payload;
  std::uint64_t expected_payload_bytes = 0;
  std::uint64_t count = 0;
  for (const auto& t : work) {
    for (const auto& r : t.second) {
      if (max_records > 0 && count >= max_records) break;
      std::string perr;
      if (!api.LoadPayload(r, payload, &perr)) continue;
      expected_payload_bytes += payload.size();
      count += 1;
    }
    if (max_records > 0 && count >= max_records) break;
  }
  rr.expected_payload_bytes = expected_payload_bytes;

  int sock = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (sock < 0) { std::perror("udp socket"); return rr; }

  sockaddr_in dst{};
  dst.sin_family = AF_INET;
  dst.sin_port = htons((std::uint16_t)udp_port);
  if (::inet_pton(AF_INET, dst_ip.c_str(), &dst.sin_addr) != 1) {
    std::cerr << "udp inet_pton failed\n";
    ::close(sock);
    return rr;
  }

  int ack_sock = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (ack_sock < 0) { std::perror("ack socket"); ::close(sock); return rr; }

  sockaddr_in ack_bind{};
  ack_bind.sin_family = AF_INET;
  ack_bind.sin_addr.s_addr = htonl(INADDR_ANY);
  ack_bind.sin_port = htons(0);
  if (::bind(ack_sock, (sockaddr*)&ack_bind, sizeof(ack_bind)) != 0) {
    std::perror("ack bind");
    ::close(ack_sock);
    ::close(sock);
    return rr;
  }

  sockaddr_in ack_name{};
  socklen_t ack_len = sizeof(ack_name);
  if (::getsockname(ack_sock, (sockaddr*)&ack_name, &ack_len) != 0) {
    std::perror("getsockname");
    ::close(ack_sock);
    ::close(sock);
    return rr;
  }

  std::uint32_t sender_ip_be = 0;
  {
    in_addr tmp{};
    if (::inet_pton(AF_INET, "0.0.0.0", &tmp) == 1) sender_ip_be = tmp.s_addr;
  }

  std::uint64_t request_id = monotonic_ns() ^ (now_wall_ns() << 1);
  std::uint64_t seq = 0;

  auto send_udp = [&](std::uint16_t type, const void* body, std::size_t body_n) -> bool {
    MsgHdr h{};
    h.magic = kMagic;
    h.ver = kVer;
    h.type = type;
    h.request_id = request_id;
    h.seq = seq;

    std::vector<std::uint8_t> buf(sizeof(MsgHdr) + body_n);
    std::memcpy(buf.data(), &h, sizeof(h));
    if (body_n) std::memcpy(buf.data() + sizeof(h), body, body_n);

    ssize_t rc = ::sendto(sock, buf.data(), buf.size(), 0, (sockaddr*)&dst, sizeof(dst));
    if (rc < 0) return false;
    rr.sent_packets += 1;
    rr.sent_bytes_total += (std::uint64_t)buf.size();
    seq += 1;
    return true;
  };

  StartBody sb{};
  sb.sender_ip_be = sender_ip_be;
  sb.sender_ack_port_be = ack_name.sin_port;
  sb.reserved0 = 0;
  sb.expected_records = rr.expected_records;
  sb.expected_payload_bytes = rr.expected_payload_bytes;
  sb.mtu_payload = (std::uint32_t)mtu_payload;
  sb.reserved1 = 0;

  std::uint64_t t0 = monotonic_ns();
  if (!send_udp(kMsgStart, &sb, sizeof(sb))) {
    ::close(ack_sock);
    ::close(sock);
    return rr;
  }

  std::uint64_t sent_records = 0;

  for (const auto& tw : work) {
    const std::string& topic = tw.first;
    for (const auto& r : tw.second) {
      if (max_records > 0 && sent_records >= max_records) break;

      std::string perr;
      if (!api.LoadPayload(r, payload, &perr)) continue;

      std::uint32_t crc = crc32_u(payload.data(), payload.size());
      std::uint32_t topic_len = (std::uint32_t)topic.size();
      const std::size_t hdr_over = sizeof(RecBody) + topic.size();

      // Fragment model: fixed slice size except last
      std::size_t max_frag = 1;
      if (mtu_payload > hdr_over) max_frag = mtu_payload - hdr_over;

      std::uint16_t frag_cnt = (std::uint16_t)((payload.size() + max_frag - 1) / max_frag);
      if (frag_cnt == 0) frag_cnt = 1;

      for (std::uint16_t fi = 0; fi < frag_cnt; ++fi) {
        std::size_t off = (std::size_t)fi * max_frag;
        std::size_t len = std::min(max_frag, payload.size() - off);

        RecBody rb{};
        rb.ts_ns = r.ts_ns;
        rb.topic_len = topic_len;
        rb.payload_len = (std::uint32_t)payload.size();
        rb.crc32 = crc;
        rb.frag_idx = fi;
        rb.frag_cnt = frag_cnt;
        rb.frag_payload_len = (std::uint32_t)len;

        std::vector<std::uint8_t> body;
        body.resize(sizeof(RecBody) + topic.size() + len);
        std::memcpy(body.data(), &rb, sizeof(rb));
        std::memcpy(body.data() + sizeof(rb), topic.data(), topic.size());
        std::memcpy(body.data() + sizeof(rb) + topic.size(), payload.data() + off, len);

        if (!send_udp(kMsgRec, body.data(), body.size())) break;
      }

      sent_records += 1;
    }
    if (max_records > 0 && sent_records >= max_records) break;
  }

  EndBody eb{};
  eb.sender_first_mono_ns = t0;
  eb.sender_last_mono_ns = monotonic_ns();
  (void)send_udp(kMsgEnd, &eb, sizeof(eb));

  AckBody ack{};
  bool got = udp_recv_ack(ack_sock, request_id, &ack, ack_timeout_ms);
  std::uint64_t t1 = monotonic_ns();

  rr.wall_s = double(t1 - t0) / 1e9;
  if (got && rr.expected_records > 0) {
    rr.delivery_ratio = double(ack.received_records_ok) / double(rr.expected_records);
  } else {
    rr.delivery_ratio = 0.0;
  }

  ::close(ack_sock);
  ::close(sock);
  return rr;
}

static RunResult run_tcp(const std::string& dst_ip,
                         int tcp_port,
                         const std::vector<std::pair<std::string, std::vector<avs::DataRef>>>& work,
                         avs::RetrieveAPI& api,
                         std::uint64_t max_records,
                         std::size_t mtu_payload,
                         int ack_timeout_ms) {
  (void)mtu_payload;

  RunResult rr{};

  std::uint64_t total_refs = 0;
  for (const auto& t : work) total_refs += t.second.size();
  if (max_records > 0 && total_refs > max_records) total_refs = max_records;
  rr.expected_records = total_refs;

  std::vector<std::uint8_t> payload;
  std::uint64_t expected_payload_bytes = 0;
  std::uint64_t count = 0;
  for (const auto& t : work) {
    for (const auto& r : t.second) {
      if (max_records > 0 && count >= max_records) break;
      std::string perr;
      if (!api.LoadPayload(r, payload, &perr)) continue;
      expected_payload_bytes += payload.size();
      count += 1;
    }
    if (max_records > 0 && count >= max_records) break;
  }
  rr.expected_payload_bytes = expected_payload_bytes;

  int s = ::socket(AF_INET, SOCK_STREAM, 0);
  if (s < 0) { std::perror("tcp socket"); return rr; }

  sockaddr_in dst{};
  dst.sin_family = AF_INET;
  dst.sin_port = htons((std::uint16_t)tcp_port);
  if (::inet_pton(AF_INET, dst_ip.c_str(), &dst.sin_addr) != 1) {
    std::cerr << "tcp inet_pton failed\n";
    ::close(s);
    return rr;
  }

  if (::connect(s, (sockaddr*)&dst, sizeof(dst)) != 0) {
    std::perror("tcp connect");
    ::close(s);
    return rr;
  }

  std::uint64_t request_id = monotonic_ns() ^ (now_wall_ns() << 1);
  std::uint64_t seq = 0;

  auto send_msg = [&](std::uint16_t type, const void* body, std::size_t body_n) -> bool {
    MsgHdr h{};
    h.magic = kMagic;
    h.ver = kVer;
    h.type = type;
    h.request_id = request_id;
    h.seq = seq;

    if (!send_all(s, &h, sizeof(h))) return false;
    if (body_n && !send_all(s, body, body_n)) return false;

    rr.sent_packets += 1;
    rr.sent_bytes_total += (std::uint64_t)(sizeof(h) + body_n);
    seq += 1;
    return true;
  };

  StartBody sb{};
  sb.sender_ip_be = 0;
  sb.sender_ack_port_be = 0;
  sb.reserved0 = 0;
  sb.expected_records = rr.expected_records;
  sb.expected_payload_bytes = rr.expected_payload_bytes;
  sb.mtu_payload = (std::uint32_t)mtu_payload;
  sb.reserved1 = 0;

  std::uint64_t t0 = monotonic_ns();
  if (!send_msg(kMsgStart, &sb, sizeof(sb))) { ::close(s); return rr; }

  std::uint64_t sent_records = 0;

  for (const auto& tw : work) {
    const std::string& topic = tw.first;
    for (const auto& r : tw.second) {
      if (max_records > 0 && sent_records >= max_records) break;

      std::string perr;
      if (!api.LoadPayload(r, payload, &perr)) continue;

      std::uint32_t crc = crc32_u(payload.data(), payload.size());
      std::uint32_t topic_len = (std::uint32_t)topic.size();

      RecBody rb{};
      rb.ts_ns = r.ts_ns;
      rb.topic_len = topic_len;
      rb.payload_len = (std::uint32_t)payload.size();
      rb.crc32 = crc;
      rb.frag_idx = 0;
      rb.frag_cnt = 1;
      rb.frag_payload_len = (std::uint32_t)payload.size();

      if (!send_msg(kMsgRec, &rb, sizeof(rb))) break;
      if (topic_len && !send_all(s, topic.data(), topic.size())) break;
      if (!send_all(s, payload.data(), payload.size())) break;

      rr.sent_bytes_total += (std::uint64_t)(topic.size() + payload.size());
      sent_records += 1;
    }
    if (max_records > 0 && sent_records >= max_records) break;
  }

  EndBody eb{};
  eb.sender_first_mono_ns = t0;
  eb.sender_last_mono_ns = monotonic_ns();
  (void)send_msg(kMsgEnd, &eb, sizeof(eb));

  // Expect ACK on same TCP connection
  // Apply a simple timeout via SO_RCVTIMEO
  timeval tv{};
  tv.tv_sec = ack_timeout_ms / 1000;
  tv.tv_usec = (ack_timeout_ms % 1000) * 1000;
  (void)setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  MsgHdr ah{};
  AckBody ack{};
  bool got = false;
  if (recv_all(s, &ah, sizeof(ah))) {
    if (ah.magic == kMagic && ah.ver == kVer && ah.type == kMsgAck && ah.request_id == request_id) {
      if (recv_all(s, &ack, sizeof(ack))) got = true;
    }
  }

  std::uint64_t t1 = monotonic_ns();
  rr.wall_s = double(t1 - t0) / 1e9;
  if (got && rr.expected_records > 0) {
    rr.delivery_ratio = double(ack.received_records_ok) / double(rr.expected_records);
  } else {
    rr.delivery_ratio = 0.0;
  }

  ::close(s);
  return rr;
}

static void usage(const char* p) {
  std::cerr
    << "Usage: " << p
    << " --dst_ip <ip> --dst_port <port_base> --start_ts_ns <n> --end_ts_ns <n>"
       " [--topics <csv>] [--ssd_root <path>] [--max_records <n>]"
       " [--mtu_payload <bytes>] [--runs <n>] [--tcp_port <p>] [--ack_timeout_ms <ms>]\n";
  std::exit(2);
}

int main(int argc, char** argv) {
  std::string dst_ip;
  int dst_port = 0;
  int tcp_port = 0;

  std::uint64_t start_ns = 0;
  std::uint64_t end_ns = 0;

  std::string topics_csv;
  fs::path ssd_root("/home/avs/DATA/SSD");

  std::uint64_t max_records = 0;
  std::size_t mtu_payload = 1200;
  int runs = 10;
  int ack_timeout_ms = 10000;

  for (int i = 1; i < argc; ++i) {
    std::string a(argv[i]);
    if (a == "--dst_ip" && i + 1 < argc) { dst_ip = argv[++i]; continue; }
    if (a == "--dst_port" && i + 1 < argc) { dst_port = std::stoi(argv[++i]); continue; }
    if (a == "--tcp_port" && i + 1 < argc) { tcp_port = std::stoi(argv[++i]); continue; }
    if (a == "--start_ts_ns" && i + 1 < argc) { start_ns = std::stoull(argv[++i]); continue; }
    if (a == "--end_ts_ns" && i + 1 < argc) { end_ns = std::stoull(argv[++i]); continue; }
    if (a == "--topics" && i + 1 < argc) { topics_csv = argv[++i]; continue; }
    if (a == "--ssd_root" && i + 1 < argc) { ssd_root = fs::path(argv[++i]); continue; }
    if (a == "--max_records" && i + 1 < argc) { max_records = std::stoull(argv[++i]); continue; }
    if (a == "--mtu_payload" && i + 1 < argc) { mtu_payload = (std::size_t)std::stoull(argv[++i]); continue; }
    if (a == "--runs" && i + 1 < argc) { runs = std::stoi(argv[++i]); continue; }
    if (a == "--ack_timeout_ms" && i + 1 < argc) { ack_timeout_ms = std::stoi(argv[++i]); continue; }
    usage(argv[0]);
  }

  if (dst_ip.empty() || dst_port <= 0 || start_ns == 0 || end_ns == 0 || end_ns < start_ns) usage(argv[0]);
  if (tcp_port <= 0) tcp_port = dst_port + 1;

  std::vector<std::string> topics;
  if (!topics_csv.empty()) topics = split_csv(topics_csv);

  if (topics.empty()) {
    std::cerr << "error: --topics is required for this sender\n";
    return 2;
  }

  avs::RetrieveAPI api(ssd_root);

  // Pre query references once for determinism across runs
  std::vector<std::pair<std::string, std::vector<avs::DataRef>>> work;
  work.reserve(topics.size());

  for (const auto& t : topics) {
    std::string qerr;
    auto refs = api.QueryRefs(t, start_ns, end_ns, &qerr);
    if (refs.empty()) {
      std::cerr << "warn: QueryRefs empty topic=" << t << " err=" << qerr << "\n";
    }
    work.push_back({t, std::move(refs)});
  }

  std::vector<double> tcp_wall_s, tcp_dr;
  std::vector<double> udp_wall_s, udp_dr;

  std::uint64_t last_sent_bytes_tcp = 0, last_sent_packets_tcp = 0;
  std::uint64_t last_sent_bytes_udp = 0, last_sent_packets_udp = 0;

  for (int i = 0; i < runs; ++i) {
    RunResult tr = run_tcp(dst_ip, tcp_port, work, api, max_records, mtu_payload, ack_timeout_ms);
    RunResult ur = run_udp(dst_ip, dst_port, work, api, ssd_root, max_records, mtu_payload, ack_timeout_ms);

    tcp_wall_s.push_back(tr.wall_s);
    tcp_dr.push_back(tr.delivery_ratio);
    udp_wall_s.push_back(ur.wall_s);
    udp_dr.push_back(ur.delivery_ratio);

    last_sent_bytes_tcp = tr.sent_bytes_total;
    last_sent_packets_tcp = tr.sent_packets;
    last_sent_bytes_udp = ur.sent_bytes_total;
    last_sent_packets_udp = ur.sent_packets;

    std::cout
      << "RUN i=" << i
      << " TCP_WALL_S=" << tr.wall_s
      << " TCP_delivery_ratio=" << tr.delivery_ratio
      << " UDP_WALL_S=" << ur.wall_s
      << " UDP_delivery_ratio=" << ur.delivery_ratio
      << " TCP_sent_bytes=" << tr.sent_bytes_total
      << " TCP_packets=" << tr.sent_packets
      << " UDP_sent_bytes=" << ur.sent_bytes_total
      << " UDP_packets=" << ur.sent_packets
      << "\n";
  }

  double tcp_wall_m = mean(tcp_wall_s);
  double tcp_wall_ci = ci_halfwidth95(tcp_wall_s);
  double tcp_dr_m = mean(tcp_dr);
  double tcp_dr_ci = ci_halfwidth95(tcp_dr);

  double udp_wall_m = mean(udp_wall_s);
  double udp_wall_ci = ci_halfwidth95(udp_wall_s);
  double udp_dr_m = mean(udp_dr);
  double udp_dr_ci = ci_halfwidth95(udp_dr);

  std::cout
    << "OFFLOAD_SUM"
    << " runs=" << runs
    << " TCP_sent_bytes=" << last_sent_bytes_tcp
    << " TCP_packets=" << last_sent_packets_tcp
    << " UDP_sent_bytes=" << last_sent_bytes_udp
    << " UDP_packets=" << last_sent_packets_udp
    << " TCP_WALL_S_avg=" << tcp_wall_m
    << " TCP_WALL_S_ci95=" << tcp_wall_ci
    << " TCP_delivery_ratio_avg=" << tcp_dr_m
    << " TCP_delivery_ratio_ci95=" << tcp_dr_ci
    << " UDP_WALL_S_avg=" << udp_wall_m
    << " UDP_WALL_S_ci95=" << udp_wall_ci
    << " UDP_delivery_ratio_avg=" << udp_dr_m
    << " UDP_delivery_ratio_ci95=" << udp_dr_ci
    << "\n";

  return 0;
}
