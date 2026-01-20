// offload_sender_latency_v3.cpp
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <math.h>
#include <numeric>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "avs/retrieve_api.h"

namespace fs = std::filesystem;

static inline std::uint64_t monotonic_ns() {
  using namespace std::chrono;
  return (std::uint64_t)duration_cast<std::chrono::nanoseconds>(
           std::chrono::steady_clock::now().time_since_epoch())
    .count();
}

static std::vector<std::string> split_csv(const std::string& s) {
  std::vector<std::string> out;
  std::string cur;
  for (char ch : s) {
    if (ch == ',') {
      if (!cur.empty()) out.push_back(cur);
      cur.clear();
    } else cur.push_back(ch);
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

static constexpr std::uint32_t kMagic = 0x33535641U;  // AVS3 little endian
static constexpr std::uint16_t kVer = 3;

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
  std::uint64_t seq;       // packet seq
  std::uint32_t body_len;
};

struct StartBody {
  std::uint64_t planned_records;
  std::uint32_t mtu_payload;
  std::uint32_t sender_udp_ack_port;  // sender listen port for UDP Ack
  std::uint64_t sender_first_ns;
};

struct RecBody {
  std::uint64_t record_id;        // stable record id for reassembly
  std::uint64_t ts_ns;
  std::uint32_t topic_len;
  std::uint32_t payload_len;
  std::uint32_t crc32;
  std::uint16_t frag_idx;
  std::uint16_t frag_cnt;
  std::uint32_t frag_off;         // byte offset within payload
  std::uint32_t frag_payload_len; // bytes in this fragment
};

struct EndBody {
  std::uint64_t sender_last_ns;
  std::uint64_t sent_records_ok;
  std::uint64_t sent_payload_bytes_ok;
  std::uint64_t sent_packets;
  std::uint64_t sent_bytes_total;
  std::uint64_t last_seq_plus1;
};

struct AckBody {
  std::uint64_t sender_first_ns;      // echoed
  std::uint64_t server_ack_send_ns;   // server time when Ack is sent
  std::uint64_t recv_records_ok;
  std::uint64_t recv_payload_bytes_ok;
  std::uint64_t recv_packets;
  std::uint64_t recv_bytes_total;
  std::uint64_t last_seq_plus1;       // echoed from End
  std::uint32_t status;               // 0 ok, nonzero error
  std::uint32_t reserved0;
};
#pragma pack(pop)

struct RunResult {
  std::uint64_t bytes_sent_total = 0;
  std::uint64_t packets_sent = 0;
  std::uint64_t records_sent_ok = 0;
  std::uint64_t payload_bytes_sent_ok = 0;

  std::uint64_t sender_first_ns = 0;
  std::uint64_t ack_recv_ns = 0;
  std::uint64_t transfer_latency_ns = 0;

  std::uint32_t ack_status = 0xFFFFFFFFU;
  std::uint64_t ack_recv_records_ok = 0;
  std::uint64_t ack_recv_payload_bytes_ok = 0;
};

static bool udp_wait_ack(int ack_sock,
                         std::uint64_t request_id,
                         AckBody* out_ack,
                         int timeout_ms) {
  fd_set rfds;
  FD_ZERO(&rfds);
  FD_SET(ack_sock, &rfds);

  timeval tv{};
  tv.tv_sec = timeout_ms / 1000;
  tv.tv_usec = (timeout_ms % 1000) * 1000;

  int rc = ::select(ack_sock + 1, &rfds, nullptr, nullptr, &tv);
  if (rc <= 0) return false;

  std::uint8_t buf[sizeof(MsgHdr) + sizeof(AckBody)];
  sockaddr_in src{};
  socklen_t sl = sizeof(src);
  ssize_t n = ::recvfrom(ack_sock, buf, sizeof(buf), 0, (sockaddr*)&src, &sl);
  if (n < (ssize_t)(sizeof(MsgHdr) + sizeof(AckBody))) return false;

  MsgHdr h{};
  std::memcpy(&h, buf, sizeof(h));
  if (h.magic != kMagic || h.ver != kVer || h.type != kMsgAck) return false;
  if (h.request_id != request_id) return false;
  if (h.body_len != sizeof(AckBody)) return false;

  std::memcpy(out_ack, buf + sizeof(MsgHdr), sizeof(AckBody));
  return true;
}

static RunResult run_udp(const std::string& dst_ip,
                         int udp_port,
                         const std::vector<std::pair<std::string, std::vector<avs::DataRef>>>& work,
                         avs::RetrieveAPI& api,
                         std::uint64_t max_records,
                         std::size_t mtu_payload,
                         int ack_timeout_ms) {
  RunResult rr{};

  int sock = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (sock < 0) { std::perror("udp socket"); return rr; }

  int sndbuf = 8 * 1024 * 1024;
  (void)setsockopt(sock, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));

  int ack_sock = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (ack_sock < 0) { std::perror("udp ack socket"); ::close(sock); return rr; }

  sockaddr_in ack_bind{};
  ack_bind.sin_family = AF_INET;
  ack_bind.sin_port = htons(0);               // ephemeral
  ack_bind.sin_addr.s_addr = htonl(INADDR_ANY);
  if (::bind(ack_sock, (sockaddr*)&ack_bind, sizeof(ack_bind)) != 0) {
    std::perror("udp ack bind");
    ::close(ack_sock);
    ::close(sock);
    return rr;
  }

  sockaddr_in ack_name{};
  socklen_t ack_name_len = sizeof(ack_name);
  if (::getsockname(ack_sock, (sockaddr*)&ack_name, &ack_name_len) != 0) {
    std::perror("udp ack getsockname");
    ::close(ack_sock);
    ::close(sock);
    return rr;
  }
  int sender_ack_port = (int)ntohs(ack_name.sin_port);

  sockaddr_in dst{};
  dst.sin_family = AF_INET;
  dst.sin_port = htons((std::uint16_t)udp_port);
  if (::inet_pton(AF_INET, dst_ip.c_str(), &dst.sin_addr) != 1) {
    std::cerr << "udp inet_pton failed\n";
    ::close(ack_sock);
    ::close(sock);
    return rr;
  }

  std::uint64_t planned = 0;
  for (const auto& t : work) planned += t.second.size();
  if (max_records > 0 && planned > max_records) planned = max_records;

  std::uint64_t request_id = monotonic_ns() ^ (std::uint64_t(::getpid()) << 32);
  std::uint64_t seq = 0;
  std::uint64_t record_id = 0;

  std::vector<std::uint8_t> pkt;
  pkt.resize(sizeof(MsgHdr) + mtu_payload);

  auto send_datagram = [&](std::uint16_t type, const void* body, std::uint32_t body_len) -> bool {
    MsgHdr h{};
    h.magic = kMagic;
    h.ver = kVer;
    h.type = type;
    h.request_id = request_id;
    h.seq = seq;
    h.body_len = body_len;

    std::memcpy(pkt.data(), &h, sizeof(h));
    if (body_len) std::memcpy(pkt.data() + sizeof(h), body, body_len);

    const std::size_t out_n = sizeof(MsgHdr) + (std::size_t)body_len;
    ssize_t rc = ::sendto(sock, pkt.data(), out_n, 0, (sockaddr*)&dst, sizeof(dst));
    if (rc < 0) return false;

    rr.packets_sent += 1;
    rr.bytes_sent_total += (std::uint64_t)out_n;
    seq += 1;
    return true;
  };

  rr.sender_first_ns = monotonic_ns();

  StartBody sb{};
  sb.planned_records = planned;
  sb.mtu_payload = (std::uint32_t)mtu_payload;
  sb.sender_udp_ack_port = (std::uint32_t)sender_ack_port;
  sb.sender_first_ns = rr.sender_first_ns;

  if (!send_datagram(kMsgStart, &sb, sizeof(sb))) {
    ::close(ack_sock);
    ::close(sock);
    return rr;
  }

  std::vector<std::uint8_t> payload;
  std::uint64_t sent_records = 0;

  for (const auto& tw : work) {
    const std::string& topic = tw.first;
    const std::uint32_t topic_len = (std::uint32_t)topic.size();
    const std::size_t hdr_over = sizeof(RecBody) + (std::size_t)topic_len;

    for (const auto& r : tw.second) {
      if (max_records > 0 && sent_records >= max_records) break;

      std::string perr;
      if (!api.LoadPayload(r, payload, &perr)) continue;

      rr.records_sent_ok += 1;
      rr.payload_bytes_sent_ok += payload.size();

      const std::uint32_t crc = crc32_u(payload.data(), payload.size());

      std::size_t max_frag_payload = 1;
      if (mtu_payload > hdr_over) max_frag_payload = mtu_payload - hdr_over;

      std::uint16_t frag_cnt = (std::uint16_t)((payload.size() + max_frag_payload - 1) / max_frag_payload);
      if (frag_cnt == 0) frag_cnt = 1;

      for (std::uint16_t fi = 0; fi < frag_cnt; ++fi) {
        const std::size_t off = (std::size_t)fi * max_frag_payload;
        const std::size_t len = std::min(max_frag_payload, payload.size() - off);

        RecBody rb{};
        rb.record_id = record_id;
        rb.ts_ns = r.ts_ns;
        rb.topic_len = topic_len;
        rb.payload_len = (std::uint32_t)payload.size();
        rb.crc32 = crc;
        rb.frag_idx = fi;
        rb.frag_cnt = frag_cnt;
        rb.frag_off = (std::uint32_t)off;
        rb.frag_payload_len = (std::uint32_t)len;

        const std::uint32_t body_len =
          (std::uint32_t)(sizeof(RecBody) + (std::size_t)topic_len + len);
        if ((std::size_t)body_len > mtu_payload) continue;

        MsgHdr h{};
        h.magic = kMagic;
        h.ver = kVer;
        h.type = kMsgRec;
        h.request_id = request_id;
        h.seq = seq;
        h.body_len = body_len;

        std::uint8_t* p = pkt.data();
        std::memcpy(p, &h, sizeof(h));
        p += sizeof(h);

        std::memcpy(p, &rb, sizeof(rb));
        p += sizeof(rb);

        if (topic_len) {
          std::memcpy(p, topic.data(), topic_len);
          p += topic_len;
        }

        if (len) {
          std::memcpy(p, payload.data() + off, len);
          p += len;
        }

        const std::size_t out_n = sizeof(MsgHdr) + (std::size_t)body_len;
        ssize_t rc = ::sendto(sock, pkt.data(), out_n, 0, (sockaddr*)&dst, sizeof(dst));
        if (rc < 0) break;

        rr.packets_sent += 1;
        rr.bytes_sent_total += (std::uint64_t)out_n;
        seq += 1;
      }

      sent_records += 1;
      record_id += 1;
    }
    if (max_records > 0 && sent_records >= max_records) break;
  }

  EndBody eb{};
  eb.sender_last_ns = monotonic_ns();
  eb.sent_records_ok = rr.records_sent_ok;
  eb.sent_payload_bytes_ok = rr.payload_bytes_sent_ok;
  eb.sent_packets = rr.packets_sent;
  eb.sent_bytes_total = rr.bytes_sent_total;
  eb.last_seq_plus1 = seq;

  (void)send_datagram(kMsgEnd, &eb, sizeof(eb));

  AckBody ack{};
  bool got_ack = udp_wait_ack(ack_sock, request_id, &ack, ack_timeout_ms);
  rr.ack_recv_ns = monotonic_ns();

  if (got_ack) {
    rr.transfer_latency_ns = rr.ack_recv_ns - rr.sender_first_ns;
    rr.ack_status = ack.status;
    rr.ack_recv_records_ok = ack.recv_records_ok;
    rr.ack_recv_payload_bytes_ok = ack.recv_payload_bytes_ok;
  } else {
    rr.transfer_latency_ns = 0;
    rr.ack_status = 0xFFFFFFFFU;
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
                         std::size_t mtu_payload) {
  (void)mtu_payload;
  RunResult rr{};

  int s = ::socket(AF_INET, SOCK_STREAM, 0);
  if (s < 0) { std::perror("tcp socket"); return rr; }

  int sndbuf = 8 * 1024 * 1024;
  (void)setsockopt(s, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));

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

  std::uint64_t planned = 0;
  for (const auto& t : work) planned += t.second.size();
  if (max_records > 0 && planned > max_records) planned = max_records;

  std::uint64_t request_id = monotonic_ns() ^ (std::uint64_t(::getpid()) << 32);
  std::uint64_t seq = 0;
  std::uint64_t record_id = 0;

  auto send_frame = [&](std::uint16_t type, const void* body, std::uint32_t body_len) -> bool {
    MsgHdr h{};
    h.magic = kMagic;
    h.ver = kVer;
    h.type = type;
    h.request_id = request_id;
    h.seq = seq;
    h.body_len = body_len;

    if (!send_all(s, &h, sizeof(h))) return false;
    if (body_len && !send_all(s, body, body_len)) return false;

    rr.packets_sent += 1;
    rr.bytes_sent_total += sizeof(h) + (std::uint64_t)body_len;
    seq += 1;
    return true;
  };

  rr.sender_first_ns = monotonic_ns();

  StartBody sb{};
  sb.planned_records = planned;
  sb.mtu_payload = (std::uint32_t)mtu_payload;
  sb.sender_udp_ack_port = 0;
  sb.sender_first_ns = rr.sender_first_ns;

  if (!send_frame(kMsgStart, &sb, sizeof(sb))) { ::close(s); return rr; }

  std::vector<std::uint8_t> payload;
  std::uint64_t sent_records = 0;

  for (const auto& tw : work) {
    const std::string& topic = tw.first;
    const std::uint32_t topic_len = (std::uint32_t)topic.size();

    for (const auto& r : tw.second) {
      if (max_records > 0 && sent_records >= max_records) break;

      std::string perr;
      if (!api.LoadPayload(r, payload, &perr)) continue;

      rr.records_sent_ok += 1;
      rr.payload_bytes_sent_ok += payload.size();

      const std::uint32_t crc = crc32_u(payload.data(), payload.size());

      RecBody rb{};
      rb.record_id = record_id;
      rb.ts_ns = r.ts_ns;
      rb.topic_len = topic_len;
      rb.payload_len = (std::uint32_t)payload.size();
      rb.crc32 = crc;
      rb.frag_idx = 0;
      rb.frag_cnt = 1;
      rb.frag_off = 0;
      rb.frag_payload_len = (std::uint32_t)payload.size();

      std::vector<std::uint8_t> body;
      body.resize(sizeof(RecBody) + (std::size_t)topic_len + payload.size());
      std::memcpy(body.data(), &rb, sizeof(rb));
      if (topic_len) std::memcpy(body.data() + sizeof(rb), topic.data(), topic_len);
      if (!payload.empty()) {
        std::memcpy(body.data() + sizeof(rb) + (std::size_t)topic_len, payload.data(), payload.size());
      }

      if (!send_frame(kMsgRec, body.data(), (std::uint32_t)body.size())) break;

      sent_records += 1;
      record_id += 1;
    }
    if (max_records > 0 && sent_records >= max_records) break;
  }

  EndBody eb{};
  eb.sender_last_ns = monotonic_ns();
  eb.sent_records_ok = rr.records_sent_ok;
  eb.sent_payload_bytes_ok = rr.payload_bytes_sent_ok;
  eb.sent_packets = rr.packets_sent;
  eb.sent_bytes_total = rr.bytes_sent_total;
  eb.last_seq_plus1 = seq;

  (void)send_frame(kMsgEnd, &eb, sizeof(eb));

  MsgHdr ah{};
  if (!recv_all(s, &ah, sizeof(ah))) { ::close(s); return rr; }
  if (ah.magic != kMagic || ah.ver != kVer || ah.type != kMsgAck) { ::close(s); return rr; }
  if (ah.request_id != request_id || ah.body_len != sizeof(AckBody)) { ::close(s); return rr; }

  AckBody ack{};
  if (!recv_all(s, &ack, sizeof(ack))) { ::close(s); return rr; }

  rr.ack_recv_ns = monotonic_ns();
  rr.transfer_latency_ns = rr.ack_recv_ns - rr.sender_first_ns;
  rr.ack_status = ack.status;
  rr.ack_recv_records_ok = ack.recv_records_ok;
  rr.ack_recv_payload_bytes_ok = ack.recv_payload_bytes_ok;

  (void)::shutdown(s, SHUT_RDWR);
  ::close(s);
  return rr;
}

static void usage(const char* p) {
  std::cerr
    << "Usage: " << p
    << " --dst_ip <ip> --dst_port <port> --start_ts_ns <n> --end_ts_ns <n> --topics <csv>"
    << " [--ssd_root <path>] [--max_records <n>] [--mtu_payload <bytes>] [--runs <n>]"
    << " [--tcp_port <p>] [--ack_timeout_ms <ms>]\n";
  std::exit(2);
}

int main(int argc, char** argv) {
  std::string dst_ip;
  int dst_port = 0;
  int tcp_port = 0;
  int ack_timeout_ms = 30000;

  std::uint64_t start_ns = 0;
  std::uint64_t end_ns = 0;
  std::string topics_csv;

  fs::path ssd_root("/home/avs/DATA/SSD");
  std::uint64_t max_records = 0;
  std::size_t mtu_payload = 1200;
  int runs = 10;

  for (int i = 1; i < argc; ++i) {
    std::string a(argv[i]);
    if (a == "--dst_ip" && i + 1 < argc) { dst_ip = argv[++i]; continue; }
    if (a == "--dst_port" && i + 1 < argc) { dst_port = std::stoi(argv[++i]); continue; }
    if (a == "--tcp_port" && i + 1 < argc) { tcp_port = std::stoi(argv[++i]); continue; }
    if (a == "--ack_timeout_ms" && i + 1 < argc) { ack_timeout_ms = std::stoi(argv[++i]); continue; }
    if (a == "--start_ts_ns" && i + 1 < argc) { start_ns = std::stoull(argv[++i]); continue; }
    if (a == "--end_ts_ns" && i + 1 < argc) { end_ns = std::stoull(argv[++i]); continue; }
    if (a == "--topics" && i + 1 < argc) { topics_csv = argv[++i]; continue; }
    if (a == "--ssd_root" && i + 1 < argc) { ssd_root = fs::path(argv[++i]); continue; }
    if (a == "--max_records" && i + 1 < argc) { max_records = std::stoull(argv[++i]); continue; }
    if (a == "--mtu_payload" && i + 1 < argc) { mtu_payload = (std::size_t)std::stoull(argv[++i]); continue; }
    if (a == "--runs" && i + 1 < argc) { runs = std::stoi(argv[++i]); continue; }
    usage(argv[0]);
  }

  if (dst_ip.empty() || dst_port <= 0 || start_ns == 0 || end_ns == 0 || end_ns < start_ns || topics_csv.empty()) usage(argv[0]);
  if (tcp_port <= 0) tcp_port = dst_port + 1;

  std::vector<std::string> topics = split_csv(topics_csv);
  avs::RetrieveAPI api(ssd_root);

  std::vector<std::pair<std::string, std::vector<avs::DataRef>>> work;
  work.reserve(topics.size());

  for (const auto& t : topics) {
    std::string qerr;
    auto refs = api.QueryRefs(t, start_ns, end_ns, &qerr);
    if (refs.empty()) std::cerr << "warn: QueryRefs empty topic=" << t << " err=" << qerr << "\n";
    work.push_back({t, std::move(refs)});
  }

  auto ns_to_ms = [](std::uint64_t ns) -> double { return double(ns) / 1e6; };

  std::vector<double> tcp_lat_ms, udp_lat_ms;
  std::uint64_t last_tcp_bytes = 0, last_udp_bytes = 0;

  for (int i = 0; i < runs; ++i) {
    RunResult tr = run_tcp(dst_ip, tcp_port, work, api, max_records, mtu_payload);
    RunResult ur = run_udp(dst_ip, dst_port, work, api, max_records, mtu_payload, ack_timeout_ms);

    double tr_ms = tr.transfer_latency_ns ? ns_to_ms(tr.transfer_latency_ns) : 0.0;
    double ur_ms = ur.transfer_latency_ns ? ns_to_ms(ur.transfer_latency_ns) : 0.0;

    if (tr.transfer_latency_ns) tcp_lat_ms.push_back(tr_ms);
    if (ur.transfer_latency_ns) udp_lat_ms.push_back(ur_ms);

    last_tcp_bytes = tr.bytes_sent_total;
    last_udp_bytes = ur.bytes_sent_total;

    std::cout
      << "RUN i=" << i
      << " TCP_transfer_latency_ns=" << tr.transfer_latency_ns
      << " TCP_transfer_latency_ms=" << tr_ms
      << " TCP_ack_status=" << tr.ack_status
      << " TCP_bytes_sent_total=" << tr.bytes_sent_total
      << " TCP_records_sent_ok=" << tr.records_sent_ok
      << " TCP_ack_recv_records_ok=" << tr.ack_recv_records_ok
      << " TCP_ack_recv_payload_bytes_ok=" << tr.ack_recv_payload_bytes_ok
      << " UDP_transfer_latency_ns=" << ur.transfer_latency_ns
      << " UDP_transfer_latency_ms=" << ur_ms
      << " UDP_ack_status=" << ur.ack_status
      << " UDP_bytes_sent_total=" << ur.bytes_sent_total
      << " UDP_packets_sent=" << ur.packets_sent
      << " UDP_records_sent_ok=" << ur.records_sent_ok
      << " UDP_ack_recv_records_ok=" << ur.ack_recv_records_ok
      << " UDP_ack_recv_payload_bytes_ok=" << ur.ack_recv_payload_bytes_ok
      << "\n";
  }

  std::cout
    << "OFFLOAD_SENDER_LAT_SUM"
    << " runs=" << runs
    << " TCP_latency_ms_avg=" << mean(tcp_lat_ms)
    << " TCP_latency_ms_ci95=" << ci_halfwidth95(tcp_lat_ms)
    << " UDP_latency_ms_avg=" << mean(udp_lat_ms)
    << " UDP_latency_ms_ci95=" << ci_halfwidth95(udp_lat_ms)
    << " TCP_last_bytes_sent_total=" << last_tcp_bytes
    << " UDP_last_bytes_sent_total=" << last_udp_bytes
    << "\n";

  return 0;
}
