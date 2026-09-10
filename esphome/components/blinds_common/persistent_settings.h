#pragma once

#include "esphome.h"
#include <Arduino.h>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <string>
#include <type_traits>

#if defined(USE_ESP32)
#include <dirent.h>
#endif

namespace blind_settings {

static constexpr const char *TAG = "blind_settings";
static constexpr const char *FS_BASE_PATH = "/littlefs";
static constexpr const char *FS_PARTITION_LABEL = "littlefs";
extern bool ready;

class FsFile {
 public:
  FsFile() = default;
  explicit FsFile(FILE *fp);
  ~FsFile();
  FsFile(const FsFile &) = delete;
  FsFile &operator=(const FsFile &) = delete;
  FsFile(FsFile &&other) noexcept;
  FsFile &operator=(FsFile &&other) noexcept;
  explicit operator bool() const;
  size_t size();
  int available();
  size_t read(uint8_t *buffer, size_t length);
  size_t write(const uint8_t *buffer, size_t length);
  void flush();
  void close();
 private:
  FILE *fp_{nullptr};
};

struct FsInfo {
  size_t totalBytes{0};
  size_t usedBytes{0};
};

class FsDir {
 public:
  FsDir() = default;
  explicit FsDir(DIR *dir);
  ~FsDir();
  FsDir(const FsDir &) = delete;
  FsDir &operator=(const FsDir &) = delete;
  FsDir(FsDir &&other) noexcept;
  FsDir &operator=(FsDir &&other) noexcept;
  bool next();
  String fileName() const;
  FsFile openFile(const char *mode) const;
  explicit operator bool() const;
  void close();
 private:
  DIR *dir_{nullptr};
  String current_name_;
};

std::string vfs_path_for(const String &logical_path);
const char *normalized_fopen_mode(const char *mode);
FsFile fs_open(const String &logical_path, const char *mode);
bool fs_remove(const String &logical_path);
bool fs_rename(const String &from, const String &to);
bool fs_info(FsInfo &info);
FsDir fs_open_dir(const char *logical_path);
bool begin();

// Permanent LittleFS schema. Never change or recycle an assigned key.
// New persistent items get the next sequential ID.
static constexpr uint32_t K_SITE_LATITUDE            = 0xB1000001;
static constexpr uint32_t K_SITE_LONGITUDE           = 0xB1000002;
static constexpr uint32_t K_SUN_HORIZON              = 0xB1000003;
static constexpr uint32_t K_LDR_OPEN_V                = 0xB1000004;
static constexpr uint32_t K_LDR_CLOSED_V              = 0xB1000005;
static constexpr uint32_t K_AUTO_OPEN_POS             = 0xB1000006;
static constexpr uint32_t K_AUTO_CLOSED_POS           = 0xB1000007;
static constexpr uint32_t K_LDR_SAMPLES               = 0xB1000008;
static constexpr uint32_t K_MAX_SPEED                 = 0xB1000009;
static constexpr uint32_t K_ACCEL                     = 0xB100000A;
static constexpr uint32_t K_DECEL                     = 0xB100000B;
static constexpr uint32_t K_DRIVER_IDLE               = 0xB100000C;
static constexpr uint32_t K_RUN_CURRENT               = 0xB100000D;
static constexpr uint32_t K_HOLD_CURRENT              = 0xB100000E;
static constexpr uint32_t K_STALLGUARD                = 0xB100000F;
static constexpr uint32_t K_OVERCURRENT               = 0xB1000010;
static constexpr uint32_t K_FULL_RANGE                = 0xB1000011;
static constexpr uint32_t K_CAL_POSITION              = 0xB1000012;
static constexpr uint32_t K_LDR_PRESET                = 0xB1000013;
static constexpr uint32_t K_AUTO_OPEN_MODE            = 0xB1000014;
static constexpr uint32_t K_TIMEZONE                  = 0xB1000015;
static constexpr uint32_t K_NTP1                      = 0xB1000016;
static constexpr uint32_t K_NTP2                      = 0xB1000017;
static constexpr uint32_t K_NTP3                      = 0xB1000018;
static constexpr uint32_t K_OPEN_TIME                 = 0xB1000019;
static constexpr uint32_t K_CLOSE_TIME                = 0xB100001A;
static constexpr uint32_t K_USE_CUSTOM_NTP            = 0xB100001B;
static constexpr uint32_t K_AUTOMATIC_PROGRAM         = 0xB100001C;
static constexpr uint32_t K_INVERT_TILT               = 0xB100001D;
static constexpr uint32_t K_BLIND_TILT_POSITION       = 0xB100001E;
static constexpr uint32_t K_REINITIALIZE_NEXT_BOOT    = 0xB100001F;
static constexpr uint32_t K_STEPPER_MOVE_COUNT        = 0xB1000020;
static constexpr uint32_t K_BLOCKED_MOVE_COUNT        = 0xB1000021;
static constexpr uint32_t K_OVERCURRENT_FAULT_COUNT   = 0xB1000022;
static constexpr uint32_t K_MIGRATION_MARKER          = 0xB1000023;
static constexpr uint32_t K_AS5600_ZERO_RAW           = 0xB1000024;
static constexpr uint32_t K_AS5600_HUNDRED_RAW        = 0xB1000025;
static constexpr uint32_t K_AS5600_REVERSE            = 0xB1000026;
static constexpr uint32_t K_WEBHOOK_URL               = 0xB1000027;
static constexpr uint32_t K_WEBHOOK_ENABLED           = 0xB1000028;

String path_for(uint32_t key);

static constexpr uint16_t HISTORY_MAX_ENTRIES = 32;
static constexpr uint16_t HISTORY_MAX_VALUE_LENGTH = 1024;

String metadata_path_for(uint32_t key);
String history_path_for(uint32_t key);

struct UpdateMetadata {
  uint32_t update_count;
  uint32_t last_updated;
};

struct HistoryRecordHeader {
  uint32_t timestamp;
  uint16_t old_length;
  uint16_t new_length;
};

static_assert(sizeof(HistoryRecordHeader) == 8, "Unexpected HistoryRecordHeader size");

bool load_metadata(uint32_t key, UpdateMetadata &meta);
bool save_metadata(uint32_t key, const UpdateMetadata &meta);
bool read_history_record(FsFile &f, HistoryRecordHeader &header, std::string &old_value, std::string &new_value);
bool write_history_record(FsFile &f, uint32_t timestamp, const std::string &old_value, const std::string &new_value);
uint16_t history_record_count(uint32_t key);
bool append_history(uint32_t key, uint32_t timestamp, const std::string &old_value, const std::string &new_value);
void record_change(uint32_t key, const std::string &old_value, const std::string &new_value);
void log_flash_layout();
bool load_string(uint32_t key, std::string &value, size_t max_len = 253);
bool save_string(uint32_t key, const std::string &value);

template<typename T>
inline std::string history_value_to_string(const T &value) {
  char buffer[64];

  if constexpr (std::is_same<T, bool>::value) {
    return value ? std::string("true") : std::string("false");
  } else if constexpr (std::is_floating_point<T>::value) {
    snprintf(buffer, sizeof(buffer), "%.9g", static_cast<double>(value));
    return std::string(buffer);
  } else if constexpr (std::is_integral<T>::value && std::is_signed<T>::value) {
    snprintf(buffer, sizeof(buffer), "%lld", static_cast<long long>(value));
    return std::string(buffer);
  } else if constexpr (std::is_integral<T>::value && std::is_unsigned<T>::value) {
    snprintf(buffer, sizeof(buffer), "%llu", static_cast<unsigned long long>(value));
    return std::string(buffer);
  } else {
    const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&value);
    std::string result = "HEX:";
    char byte_text[4];

    for (size_t i = 0; i < sizeof(T); i++) {
      snprintf(byte_text, sizeof(byte_text), "%02X", bytes[i]);
      result += byte_text;
      if (i + 1 < sizeof(T)) result += ' ';
    }

    return result;
  }
}

template<typename T>
inline bool load(uint32_t key, T &value) {
  static_assert(std::is_trivially_copyable<T>::value, "POD required");
  if (!ready && !begin()) return false;
  FsFile f = fs_open(path_for(key), "r");
  if (!f || f.size() != sizeof(T)) { if (f) f.close(); return false; }
  const size_t n = f.read(reinterpret_cast<uint8_t *>(&value), sizeof(T));
  f.close();
  return n == sizeof(T);
}

template<typename T>
inline bool save(uint32_t key, const T &value) {
  static_assert(std::is_trivially_copyable<T>::value, "blind_settings::save requires POD type");

  if (!ready && !begin()) return false;

  T old_value{};
  const bool had_old_value = load(key, old_value);

  if (had_old_value && memcmp(&old_value, &value, sizeof(T)) == 0) {
    return true;
  }

  const String final_path = path_for(key);
  const String temp_path = final_path + ".tmp";

  FsFile f = fs_open(temp_path, "w");
  if (!f) return false;

  const size_t n = f.write(reinterpret_cast<const uint8_t *>(&value), sizeof(T));
  f.flush();
  f.close();

  if (n != sizeof(T)) {
    fs_remove(temp_path);
    return false;
  }

  fs_remove(final_path);
  if (!fs_rename(temp_path, final_path)) return false;

  const std::string old_text = had_old_value ? history_value_to_string(old_value) : std::string("<unset>");
  const std::string new_text = history_value_to_string(value);
  record_change(key, old_text, new_text);
  return true;
}

}  // namespace blind_settings
