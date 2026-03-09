#include "avs/retrieve_api.h"

#include <fstream>
#include <sstream>
#include <tuple>
#include <utility>

#include <sqlite3.h>

#include "avs/append_logger.h"

namespace fs = std::filesystem;

namespace avs {
namespace {

static bool OpenDb(const fs::path& dbpath, sqlite3** out, std::string* err) {
  *out = nullptr;
  int rc = sqlite3_open(dbpath.c_str(), out);
  if (rc != SQLITE_OK) {
    if (err) {
      std::ostringstream ss;
      ss << "sqlite open failed " << dbpath << " msg " << sqlite3_errmsg(*out);
      *err = ss.str();
    }
    if (*out) sqlite3_close(*out);
    *out = nullptr;
    return false;
  }
  return true;
}

static std::vector<std::tuple<std::string, std::string, std::string, int, std::uint64_t, std::uint64_t>>
FindMatchingTrips(sqlite3* db,
                  const std::string& topic,
                  std::uint64_t start_ns,
                  std::uint64_t end_ns,
                  std::string* err) {
  std::vector<std::tuple<std::string, std::string, std::string, int, std::uint64_t, std::uint64_t>> out;

  const char* sql =
      "SELECT sensor_topic, topic_folder, day, trip_id, start_ts_ns, end_ts_ns "
      "FROM global "
      "WHERE sensor_topic = ? "
      "  AND CAST(start_ts_ns AS INTEGER) <= ? "
      "  AND (end_ts_ns = '0' OR CAST(end_ts_ns AS INTEGER) >= ?);";

  sqlite3_stmt* stmt = nullptr;
  if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK) {
    if (err) {
      std::ostringstream ss;
      ss << "sqlite prepare failed msg " << sqlite3_errmsg(db);
      *err = ss.str();
    }
    return out;
  }

  sqlite3_bind_text(stmt, 1, topic.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_int64(stmt, 2, static_cast<sqlite3_int64>(end_ns));
  sqlite3_bind_int64(stmt, 3, static_cast<sqlite3_int64>(start_ns));

  while (sqlite3_step(stmt) == SQLITE_ROW) {
    const unsigned char* s_topic        = sqlite3_column_text(stmt, 0);
    const unsigned char* s_topic_folder = sqlite3_column_text(stmt, 1);
    const unsigned char* s_day          = sqlite3_column_text(stmt, 2);
    int trip_id                         = sqlite3_column_int(stmt, 3);

    sqlite3_int64 s_start = sqlite3_column_int64(stmt, 4);
    sqlite3_int64 s_end   = sqlite3_column_int64(stmt, 5);

    std::uint64_t t_start = static_cast<std::uint64_t>(s_start);
    std::uint64_t t_end   = static_cast<std::uint64_t>(s_end);

    out.emplace_back(
        s_topic ? reinterpret_cast<const char*>(s_topic) : "",
        s_topic_folder ? reinterpret_cast<const char*>(s_topic_folder) : "",
        s_day ? reinterpret_cast<const char*>(s_day) : "",
        trip_id,
        t_start,
        t_end);
  }

  sqlite3_finalize(stmt);
  return out;
}

static bool LoadPayloadInternal(const fs::path& log_path,
                                std::uint64_t payload_offset,
                                std::uint32_t payload_size,
                                std::vector<std::uint8_t>& out_payload,
                                std::string* err) {
  out_payload.clear();

  std::ifstream lf(log_path, std::ios::binary);
  if (!lf.is_open()) {
    if (err) {
      std::ostringstream ss;
      ss << "cannot open log " << log_path;
      *err = ss.str();
    }
    return false;
  }

  lf.seekg(static_cast<std::streamoff>(payload_offset), std::ios::beg);
  out_payload.resize(payload_size);
  lf.read(reinterpret_cast<char*>(out_payload.data()), payload_size);
  if (!lf) {
    out_payload.clear();
    if (err) {
      std::ostringstream ss;
      ss << "payload read failed log " << log_path
         << " off " << payload_offset
         << " size " << payload_size;
      *err = ss.str();
    }
    return false;
  }

  return true;
}

}  // namespace

RetrieveAPI::RetrieveAPI(std::filesystem::path ssd_root) : ssd_root_(std::move(ssd_root)) {}

std::vector<DataRef> RetrieveAPI::QueryRefs(const std::string& topic,
                                            std::uint64_t start_ns,
                                            std::uint64_t end_ns,
                                            std::string* err) const {
  std::vector<DataRef> refs;

  if (topic.empty()) {
    if (err) *err = "topic is empty";
    return refs;
  }
  if (start_ns == 0 || end_ns == 0 || end_ns < start_ns) {
    if (err) *err = "invalid time range";
    return refs;
  }

  sqlite3* db = nullptr;
  fs::path dbpath = fs::path(ssd_root_) / "global.sqlite3";
  if (!OpenDb(dbpath, &db, err)) {
    return refs;
  }

  std::string local_err;
  auto trips = FindMatchingTrips(db, topic, start_ns, end_ns, &local_err);
  sqlite3_close(db);

  if (trips.empty()) {
    if (err) {
      if (!local_err.empty()) *err = local_err;
      else *err = "no matching trips";
    }
    return refs;
  }

  for (const auto& t : trips) {
    const std::string sensor       = std::get<0>(t);
    const std::string topic_folder = std::get<1>(t);
    const std::string day          = std::get<2>(t);
    int trip_id                    = std::get<3>(t);

    fs::path daydir = fs::path(ssd_root_) / topic_folder / day;

    char tb[64];
    std::snprintf(tb, sizeof(tb), "trip_%02d.idx", trip_id);
    fs::path idxp = daydir / tb;
    std::snprintf(tb, sizeof(tb), "trip_%02d.log", trip_id);
    fs::path logp = daydir / tb;

    std::ifstream idxf(idxp, std::ios::binary);
    if (!idxf.is_open()) {
      continue;
    }

    std::ifstream lf(logp, std::ios::binary);
    if (!lf.is_open()) {
      continue;
    }

    avs::TripIndexEntry ent;
    while (idxf.read(reinterpret_cast<char*>(&ent), sizeof(ent))) {
      if (ent.end_ts_ns < static_cast<std::int64_t>(start_ns) ||
          ent.start_ts_ns > static_cast<std::int64_t>(end_ns)) {
        continue;
      }

      const std::uint64_t chunk_data_start =
          static_cast<std::uint64_t>(ent.file_offset) + sizeof(avs::ChunkHeader);
      const std::uint64_t chunk_data_end =
          chunk_data_start + static_cast<std::uint64_t>(ent.chunk_size_bytes);

      std::uint64_t pos = chunk_data_start;

      for (std::uint32_t rec = 0;
           rec < ent.record_count &&
           pos + sizeof(avs::RecordHeader) <= chunk_data_end;
           ++rec) {
        avs::RecordHeader rh;

        lf.seekg(static_cast<std::streamoff>(pos), std::ios::beg);
        lf.read(reinterpret_cast<char*>(&rh), sizeof(rh));
        if (!lf) {
          lf.clear();
          break;
        }

        std::uint64_t next_pos =
            pos + sizeof(avs::RecordHeader) +
            static_cast<std::uint64_t>(rh.payload_size);
        if (next_pos > chunk_data_end) {
          break;
        }

        std::uint64_t rts = static_cast<std::uint64_t>(rh.ts_ns);
        if (rts >= start_ns && rts <= end_ns) {
          DataRef ref;
          ref.sensor_topic   = sensor;
          ref.topic_folder   = topic_folder;
          ref.day            = day;
          ref.trip_id        = trip_id;
          ref.ts_ns          = rts;
          ref.log_path       = logp;
          ref.payload_offset = pos + sizeof(avs::RecordHeader);
          ref.payload_size   = rh.payload_size;
          refs.emplace_back(std::move(ref));
        }

        pos = next_pos;
      }
    }
  }

  return refs;
}

bool RetrieveAPI::LoadPayload(const DataRef& ref,
                              std::vector<std::uint8_t>& out_payload,
                              std::string* err) const {
  return LoadPayloadInternal(ref.log_path, ref.payload_offset, ref.payload_size, out_payload, err);
}

bool RetrieveAPI::LoadPayloadAt(const std::filesystem::path& log_path,
                                std::uint64_t payload_offset,
                                std::uint32_t payload_size,
                                std::vector<std::uint8_t>& out_payload,
                                std::string* err) const {
  return LoadPayloadInternal(log_path, payload_offset, payload_size, out_payload, err);
}

}  // namespace avs
