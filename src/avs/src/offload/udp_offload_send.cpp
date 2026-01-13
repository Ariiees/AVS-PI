#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

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
  // Portable table based CRC32. Fast enough for evaluation.
  static std::uint32_t table[256];
  static bool init = false;
  if (!init) {
    for (std::uint32_t i = 0; i < 256; ++i) {
      std::uint32_t c = i;
      for (int k = 0; k < 8; ++k) {
        c = (c & 1) ? (0xEDB88320U ^ (c >> 1)) : (c >> 1);
      }
      table[i] = c;
    }
    init = true;
  }
  std::uint32_t c = 0xFFFFFFFFU;
  for (std::size_t i = 0; i < n; ++i) {
    c = table[(c ^ data[i]) & 0xFFU] ^ (c >> 8);
  }
  return c ^ 0xFFFFFFFFU;
}

struct ProcStats {
  double cpu_avg_percent = 0.0;
  double cpu_max_percent = 0.0;
  std::uint64_t rss_avg_bytes = 0;
  std::uint64_t rss_max_bytes = 0;
};

static bool read_self_stat(double* cpu_s_out) {
  FILE* f = std::fopen("/proc/self/stat", "r");
  if (!f) return false;
  // We only need fields 14 and 15, but comm may contain spaces within parentheses.
  // Simplest: read entire line then split.
  char buf[4096];
  if (!std::fgets(buf, sizeof(buf), f)) {
    std::fclose(f);
    return false;
  }
  std::fclose(f);

  std::string s(buf);
  // Find last ')', then split from there.
  auto rp = s.rfind(')');
  if (rp == std::string::npos) return false;
  std::string tail = s.substr(rp + 2);  // skip ") "
  std::vector<std::string> parts;
  parts.reserve(64);
  std::string cur;
  for (char ch : tail) {
    if (ch == ' ' || ch == '\n') {
      if (!cur.empty()) {
        parts.push_back(cur);
        cur.clear();
      }
    } else {
      cur.push_back(ch);
    }
  }
  if (!cur.empty()) parts.push_back(cur);

  // In /proc/self/stat, utime is field 14, stime is field 15.
  // In tail after comm, state is field 3, so utime is index 11, stime is index 12 within tail split.
  if (parts.size() < 13) return false;
  long long utime_ticks = std::stoll(parts[11]);
  long long stime_ticks = std::stoll(parts[12]);
  long clk = sysconf(_SC_CLK_TCK);
  *cpu_s_out = double(utime_ticks + stime_ticks) / double(clk);
  return true;
}

static bool read_self_rss(std::uint64_t* rss_bytes_out) {
  FILE* f = std::fopen("/proc/self/status", "r");
  if (!f) return false;
  char line[512];
  std::uint64_t rss_kb = 0;
  while (std::fgets(line, sizeof(line), f)) {
    if (std::strncmp(line, "VmRSS:", 6) == 0) {
      // VmRSS:  12345 kB
      std::uint64_t v = 0;
      std::sscanf(line + 6, "%llu", (unsigned long long*)&v);
      rss_kb = v;
      break;
    }
  }
  std::fclose(f);
  *rss_bytes_out = rss_kb * 1024ULL;
  return true;
}

class Sampler {
 public:
  explicit Sampler(int interval_ms) : interval_ms_(interval_ms) {}
  void start() {
    stop_ = false;
    th_ = std::thread([this]() { run(); });
  }
  void stop() {
    stop_ = true;
    if (th_.joinable()) th_.join();
  }
  ProcStats stats() const {
    ProcStats out;
    if (samples_.size() < 2) return out;

    std::vector<double> cpu_p;
    cpu_p.reserve(samples_.size() - 1);
    for (std::size_t i = 1; i < samples_.size(); ++i) {
      const auto& a = samples_[i - 1];
      const auto& b = samples_[i];
      double dt = double(b.t_ns - a.t_ns) / 1e9;
      double dc = b.cpu_s - a.cpu_s;
      if (dt > 0) cpu_p.push_back(100.0 * dc / dt);
    }
    if (!cpu_p.empty()) {
      double sum = 0.0;
      double mx = cpu_p[0];
      for (double v : cpu_p) {
        sum += v;
        if (v > mx) mx = v;
      }
      out.cpu_avg_percent = sum / double(cpu_p.size());
      out.cpu_max_percent = mx;
    }

    std::uint64_t rss_sum = 0;
    std::uint64_t rss_mx = 0;
    for (const auto& s : samples_) {
      rss_sum += s.rss_bytes;
      if (s.rss_bytes > rss_mx) rss_mx = s.rss_bytes;
    }
    out.rss_avg_bytes = rss_sum / samples_.size();
    out.rss_max_bytes = rss_mx;
    return out;
  }

 private:
  struct Sample {
    std::uint64_t t_ns = 0;
    double cpu_s = 0.0;
    std::uint64_t rss_bytes = 0;
  };

  void run() {
    while (!stop_) {
      Sample s;
      s.t_ns = monotonic_ns();
      read_self_stat(&s.cpu_s);
      read_self_rss(&s.rss_bytes);
      samples_.push_back(s);
      std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms_));
    }
  }

  int interval_ms_;
  volatile bool stop_ = false;
  std::thread th_;
  std::vector<Sample> samples_;
};

// Wire format
static constexpr std::uint32_t kMagic = 0x31535641U;  // "AVS1" little endian
static constexpr std::uint16_t kVersion = 1;

static constexpr std::uint16_t kFlagStart = 1;
static constexpr std::uint16_t kFlagEnd   = 2;
static constexpr std::uint16_t kFlagData  = 4;

#pragma pack(push, 1)
struct UdpHdr {
  std::uint32_t magic;
  std::uint16_t version;
  std::uint16_t flags;
  std::uint64_t request_id;
  std::uint64_t pkt_seq;
  std::uint64_t sender_mono_ns;
  std::uint16_t recs_in_pkt;
  std::uint16_t reserved;
};

struct EndFooter {
  std::uint64_t total_records;
  std::uint64_t total_payload_bytes;
  std::uint64_t total_data_packets;
  std::uint64_t first_send_mono_ns;
  std::uint64_t last_send_mono_ns;
};

struct RecHdr {
  std::uint64_t ts_ns;
  std::uint32_t payload_len;
  std::uint32_t crc32;
};
#pragma pack(pop)

static void append_bytes(std::vector<std::uint8_t>& b, const void* p, std::size_t n) {
  const auto* u = static_cast<const std::uint8_t*>(p);
  b.insert(b.end(), u, u + n);
}

static void usage(const char* p) {
  std::cerr << "Usage: " << p
            << " --dst_ip <ip> --dst_port <port> --topic <sensor_topic> --start <ts_ns> --end <ts_ns>"
            << " [--ssd_root <path>] [--max_records <n>] [--mtu_payload <bytes>] [--batch_us <us>]\n";
  std::exit(1);
}

int main(int argc, char** argv) {
  std::string dst_ip;
  int dst_port = 0;
  std::string topic;
  std::uint64_t start_ns = 0;
  std::uint64_t end_ns = 0;
  fs::path ssd_root("/home/avs/DATA/SSD");
  std::uint64_t max_records = 0;
  std::size_t mtu_payload = 1200;
  int batch_us = 0;  // 0 means flush only by size

  for (int i = 1; i < argc; ++i) {
    std::string a(argv[i]);
    if (a == "--dst_ip" && i + 1 < argc) { dst_ip = argv[++i]; continue; }
    if (a == "--dst_port" && i + 1 < argc) { dst_port = std::stoi(argv[++i]); continue; }
    if (a == "--topic" && i + 1 < argc) { topic = argv[++i]; continue; }
    if (a == "--start" && i + 1 < argc) { start_ns = std::stoull(argv[++i]); continue; }
    if (a == "--end" && i + 1 < argc) { end_ns = std::stoull(argv[++i]); continue; }
    if (a == "--ssd_root" && i + 1 < argc) { ssd_root = fs::path(argv[++i]); continue; }
    if (a == "--max_records" && i + 1 < argc) { max_records = std::stoull(argv[++i]); continue; }
    if (a == "--mtu_payload" && i + 1 < argc) { mtu_payload = static_cast<std::size_t>(std::stoull(argv[++i])); continue; }
    if (a == "--batch_us" && i + 1 < argc) { batch_us = std::stoi(argv[++i]); continue; }
    usage(argv[0]);
  }

  if (dst_ip.empty() || dst_port <= 0 || topic.empty() || start_ns == 0 || end_ns == 0 || end_ns < start_ns) {
    usage(argv[0]);
  }

  int sock = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (sock < 0) {
    std::perror("socket");
    return 2;
  }

  sockaddr_in dst{};
  dst.sin_family = AF_INET;
  dst.sin_port = htons(static_cast<std::uint16_t>(dst_port));
  if (::inet_pton(AF_INET, dst_ip.c_str(), &dst.sin_addr) != 1) {
    std::cerr << "inet_pton failed for dst_ip " << dst_ip << "\n";
    ::close(sock);
    return 2;
  }

  avs::RetrieveAPI api(ssd_root);
  std::string qerr;
  std::vector<avs::DataRef> refs = api.QueryRefs(topic, start_ns, end_ns, &qerr);
  if (refs.empty()) {
    std::cerr << "QueryRefs empty. " << qerr << "\n";
    ::close(sock);
    return 0;
  }

  const std::uint64_t request_id = monotonic_ns() ^ (now_wall_ns() << 1);
  std::uint64_t pkt_seq = 0;

  Sampler sampler(200);
  sampler.start();

  std::uint64_t total_records = 0;
  std::uint64_t total_payload_bytes = 0;
  std::uint64_t total_data_packets = 0;
  std::uint64_t first_send_mono_ns = 0;
  std::uint64_t last_send_mono_ns = 0;

  auto send_packet = [&](std::uint16_t flags, std::uint16_t recs, const std::vector<std::uint8_t>& payload) -> bool {
    UdpHdr h{};
    h.magic = kMagic;
    h.version = kVersion;
    h.flags = flags;
    h.request_id = request_id;
    h.pkt_seq = pkt_seq;
    h.sender_mono_ns = monotonic_ns();
    h.recs_in_pkt = recs;
    h.reserved = 0;

    std::vector<std::uint8_t> buf;
    buf.reserve(sizeof(UdpHdr) + payload.size());
    append_bytes(buf, &h, sizeof(h));
    append_bytes(buf, payload.data(), payload.size());

    ssize_t rc = ::sendto(sock, buf.data(), buf.size(), 0,
                          reinterpret_cast<sockaddr*>(&dst), sizeof(dst));
    if (rc < 0) {
      std::perror("sendto");
      return false;
    }
    pkt_seq += 1;
    return true;
  };

  // Start packet
  {
    std::vector<std::uint8_t> empty;
    if (!send_packet(kFlagStart, 0, empty)) {
      ::close(sock);
      sampler.stop();
      return 2;
    }
  }

  // Batching
  std::vector<std::uint8_t> batch_payload;
  batch_payload.reserve(mtu_payload);
  std::uint16_t batch_recs = 0;

  auto flush_batch = [&]() -> bool {
    if (batch_recs == 0) return true;
    if (!send_packet(kFlagData, batch_recs, batch_payload)) return false;
    total_data_packets += 1;
    if (first_send_mono_ns == 0) first_send_mono_ns = last_send_mono_ns = monotonic_ns();
    last_send_mono_ns = monotonic_ns();
    batch_payload.clear();
    batch_recs = 0;
    return true;
  };

  const std::uint64_t batch_deadline_ns = static_cast<std::uint64_t>(batch_us) * 1000ULL;
  std::uint64_t batch_start_ns = monotonic_ns();

  std::vector<std::uint8_t> payload;
  std::uint64_t sent_records = 0;

  for (const auto& r : refs) {
    if (max_records > 0 && sent_records >= max_records) break;

    std::string perr;
    if (!api.LoadPayload(r, payload, &perr)) {
      continue;
    }

    RecHdr rh{};
    rh.ts_ns = r.ts_ns;
    rh.payload_len = static_cast<std::uint32_t>(payload.size());
    rh.crc32 = crc32_u(payload.data(), payload.size());

    const std::size_t rec_bytes = sizeof(RecHdr) + payload.size();
    const std::size_t header_room = 0;  // batch_payload holds only record entries

    if (rec_bytes + header_room > mtu_payload) {
      // Oversize record. Flush current batch, then segment payload into mtu sized chunks.
      if (!flush_batch()) break;

      const std::size_t max_seg = (mtu_payload > sizeof(RecHdr)) ? (mtu_payload - sizeof(RecHdr)) : 1;
      std::size_t off = 0;
      while (off < payload.size()) {
        std::size_t seg_len = std::min(max_seg, payload.size() - off);
        RecHdr srh{};
        srh.ts_ns = r.ts_ns;
        srh.payload_len = static_cast<std::uint32_t>(seg_len);
        srh.crc32 = crc32_u(payload.data() + off, seg_len);

        std::vector<std::uint8_t> one;
        one.reserve(sizeof(RecHdr) + seg_len);
        append_bytes(one, &srh, sizeof(srh));
        append_bytes(one, payload.data() + off, seg_len);

        // Each segment counts as one record unit for transport metrics.
        if (!send_packet(kFlagData, 1, one)) {
          off = payload.size();
          break;
        }
        total_data_packets += 1;
        total_records += 1;
        total_payload_bytes += seg_len;
        sent_records += 1;
        if (first_send_mono_ns == 0) first_send_mono_ns = last_send_mono_ns = monotonic_ns();
        last_send_mono_ns = monotonic_ns();

        off += seg_len;
      }
      continue;
    }

    // Size based flush
    if (!batch_payload.empty() && (batch_payload.size() + rec_bytes) > mtu_payload) {
      if (!flush_batch()) break;
      batch_start_ns = monotonic_ns();
    }

    // Append record entry
    append_bytes(batch_payload, &rh, sizeof(rh));
    append_bytes(batch_payload, payload.data(), payload.size());
    batch_recs += 1;

    total_records += 1;
    total_payload_bytes += payload.size();
    sent_records += 1;

    // Latency bound flush
    if (batch_us > 0) {
      std::uint64_t now = monotonic_ns();
      if ((now - batch_start_ns) >= batch_deadline_ns) {
        if (!flush_batch()) break;
        batch_start_ns = monotonic_ns();
      }
    }
  }

  flush_batch();

  // End packet
  {
    EndFooter ef{};
    ef.total_records = total_records;
    ef.total_payload_bytes = total_payload_bytes;
    ef.total_data_packets = total_data_packets;
    ef.first_send_mono_ns = first_send_mono_ns;
    ef.last_send_mono_ns = last_send_mono_ns;

    std::vector<std::uint8_t> endp;
    endp.reserve(sizeof(EndFooter));
    append_bytes(endp, &ef, sizeof(ef));

    send_packet(kFlagEnd, 0, endp);
  }

  sampler.stop();
  ProcStats ps = sampler.stats();

  std::cout << "\nUDP_OFFLOAD_SENDER_SUMMARY\n";
  std::cout << "dst_ip " << dst_ip << "\n";
  std::cout << "dst_port " << dst_port << "\n";
  std::cout << "request_id " << request_id << "\n";
  std::cout << "topic " << topic << "\n";
  std::cout << "start_ts_ns " << start_ns << "\n";
  std::cout << "end_ts_ns " << end_ns << "\n";
  std::cout << "mtu_payload_bytes " << mtu_payload << "\n";
  std::cout << "batch_us " << batch_us << "\n";
  std::cout << "records_sent " << total_records << "\n";
  std::cout << "bytes_payload_sent " << total_payload_bytes << "\n";
  std::cout << "data_packets_sent " << total_data_packets << "\n";
  std::cout << "cpu_avg_percent " << ps.cpu_avg_percent << "\n";
  std::cout << "cpu_max_percent " << ps.cpu_max_percent << "\n";
  std::cout << "rss_avg_bytes " << ps.rss_avg_bytes << "\n";
  std::cout << "rss_max_bytes " << ps.rss_max_bytes << "\n\n";

  ::close(sock);
  return 0;
}
