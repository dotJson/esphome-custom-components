#include "persistent_settings.h"

#if defined(USE_ESP32)
#include <esp_err.h>
#include <esp_littlefs.h>
#include <esp_system.h>
#endif

#include <cerrno>
#include <climits>
#include <ctime>
#include <unistd.h>

namespace blind_settings {

bool ready = false;

std::string vfs_path_for(const String &logical_path) {
  const char *p = logical_path.c_str();
  if (p == nullptr || *p == '\0') return std::string(FS_BASE_PATH);
  if (*p == '/') return std::string(FS_BASE_PATH) + p;
  return std::string(FS_BASE_PATH) + "/" + p;
}

FsFile::FsFile(FILE *fp) : fp_(fp) {}
FsFile::~FsFile() { close(); }
FsFile::FsFile(FsFile &&other) noexcept : fp_(other.fp_) { other.fp_ = nullptr; }
FsFile &FsFile::operator=(FsFile &&other) noexcept {
  if (this != &other) {
    close();
    fp_ = other.fp_;
    other.fp_ = nullptr;
  }
  return *this;
}
FsFile::operator bool() const { return fp_ != nullptr; }
size_t FsFile::size() {
  if (fp_ == nullptr) return 0;
  const long current = ftell(fp_);
  if (current < 0 || fseek(fp_, 0, SEEK_END) != 0) return 0;
  const long end = ftell(fp_);
  (void) fseek(fp_, current, SEEK_SET);
  return end < 0 ? 0U : static_cast<size_t>(end);
}
int FsFile::available() {
  if (fp_ == nullptr) return 0;
  const long current = ftell(fp_);
  if (current < 0) return 0;
  const size_t end = size();
  if (end <= static_cast<size_t>(current)) return 0;
  const size_t remaining = end - static_cast<size_t>(current);
  return remaining > static_cast<size_t>(INT_MAX) ? INT_MAX : static_cast<int>(remaining);
}
size_t FsFile::read(uint8_t *buffer, size_t length) {
  if (fp_ == nullptr || buffer == nullptr || length == 0) return 0;
  return fread(buffer, 1, length, fp_);
}
size_t FsFile::write(const uint8_t *buffer, size_t length) {
  if (fp_ == nullptr || buffer == nullptr || length == 0) return 0;
  return fwrite(buffer, 1, length, fp_);
}
void FsFile::flush() {
  if (fp_ != nullptr) (void) fflush(fp_);
}
void FsFile::close() {
  if (fp_ != nullptr) {
    (void) fclose(fp_);
    fp_ = nullptr;
  }
}

const char *normalized_fopen_mode(const char *mode) {
  if (mode == nullptr) return "rb";
  if (strcmp(mode, "r") == 0) return "rb";
  if (strcmp(mode, "w") == 0) return "wb";
  if (strcmp(mode, "a") == 0) return "ab";
  return mode;
}

FsFile fs_open(const String &logical_path, const char *mode) {
  const std::string path = vfs_path_for(logical_path);
  return FsFile(fopen(path.c_str(), normalized_fopen_mode(mode)));
}

bool fs_remove(const String &logical_path) {
  const std::string path = vfs_path_for(logical_path);
  if (unlink(path.c_str()) == 0) return true;
  return errno == ENOENT;
}

bool fs_rename(const String &from, const String &to) {
  const std::string source = vfs_path_for(from);
  const std::string target = vfs_path_for(to);
  return rename(source.c_str(), target.c_str()) == 0;
}

bool fs_info(FsInfo &info) {
#if defined(USE_ESP32)
  size_t total = 0;
  size_t used = 0;
  const esp_err_t err = esp_littlefs_info(FS_PARTITION_LABEL, &total, &used);
  if (err != ESP_OK) return false;
  info.totalBytes = total;
  info.usedBytes = used;
  return true;
#else
  (void) info;
  return false;
#endif
}

FsDir::FsDir(DIR *dir) : dir_(dir) {}
FsDir::~FsDir() { close(); }
FsDir::FsDir(FsDir &&other) noexcept : dir_(other.dir_), current_name_(other.current_name_) {
  other.dir_ = nullptr;
}
FsDir &FsDir::operator=(FsDir &&other) noexcept {
  if (this != &other) {
    close();
    dir_ = other.dir_;
    current_name_ = other.current_name_;
    other.dir_ = nullptr;
  }
  return *this;
}
bool FsDir::next() {
  if (dir_ == nullptr) return false;
  for (;;) {
    errno = 0;
    struct dirent *entry = readdir(dir_);
    if (entry == nullptr) {
      current_name_ = "";
      return false;
    }
    if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0) continue;
    current_name_ = String(entry->d_name);
    return true;
  }
}
String FsDir::fileName() const { return current_name_; }
FsFile FsDir::openFile(const char *mode) const {
  if (current_name_.length() == 0) return FsFile();
  return fs_open(String("/") + current_name_, mode);
}
FsDir::operator bool() const { return dir_ != nullptr; }
void FsDir::close() {
  if (dir_ != nullptr) {
    (void) closedir(dir_);
    dir_ = nullptr;
  }
  current_name_ = "";
}

FsDir fs_open_dir(const char *logical_path) {
  const String logical = logical_path == nullptr ? String("/") : String(logical_path);
  const std::string path = vfs_path_for(logical);
  return FsDir(opendir(path.c_str()));
}

String path_for(uint32_t key) {
  char path[24];
  snprintf(path, sizeof(path), "/cfg_%08lX.bin", static_cast<unsigned long>(key));
  return String(path);
}

String metadata_path_for(uint32_t key) {
  char path[24];
  snprintf(path, sizeof(path), "/meta_%08lX.bin", static_cast<unsigned long>(key));
  return String(path);
}

String history_path_for(uint32_t key) {
  char path[24];
  snprintf(path, sizeof(path), "/hist_%08lX.bin", static_cast<unsigned long>(key));
  return String(path);
}

bool load_metadata(uint32_t key, UpdateMetadata &meta) {
  meta.update_count = 0;
  meta.last_updated = 0;
  if (!ready && !begin()) return false;

  FsFile f = fs_open(metadata_path_for(key), "r");
  if (!f || f.size() != sizeof(UpdateMetadata)) {
    if (f) f.close();
    return false;
  }

  const size_t n = f.read(reinterpret_cast<uint8_t *>(&meta), sizeof(UpdateMetadata));
  f.close();
  return n == sizeof(UpdateMetadata);
}

bool save_metadata(uint32_t key, const UpdateMetadata &meta) {
  if (!ready && !begin()) return false;

  const String final_path = metadata_path_for(key);
  const String temp_path = final_path + ".tmp";
  FsFile f = fs_open(temp_path, "w");
  if (!f) return false;

  const size_t n = f.write(reinterpret_cast<const uint8_t *>(&meta), sizeof(UpdateMetadata));
  f.flush();
  f.close();

  if (n != sizeof(UpdateMetadata)) {
    fs_remove(temp_path);
    return false;
  }

  fs_remove(final_path);
  return fs_rename(temp_path, final_path);
}

bool read_history_record(
  FsFile &f,
  HistoryRecordHeader &header,
  std::string &old_value,
  std::string &new_value
) {
  if (f.available() < static_cast<int>(sizeof(HistoryRecordHeader))) return false;

  const size_t header_read = f.read(reinterpret_cast<uint8_t *>(&header), sizeof(header));
  if (header_read != sizeof(header)) return false;

  if (header.old_length > HISTORY_MAX_VALUE_LENGTH || header.new_length > HISTORY_MAX_VALUE_LENGTH) {
    ESP_LOGW(TAG, "Invalid history record length");
    return false;
  }

  old_value.resize(header.old_length);
  new_value.resize(header.new_length);

  if (header.old_length > 0) {
    const size_t n = f.read(reinterpret_cast<uint8_t *>(&old_value[0]), header.old_length);
    if (n != header.old_length) return false;
  }

  if (header.new_length > 0) {
    const size_t n = f.read(reinterpret_cast<uint8_t *>(&new_value[0]), header.new_length);
    if (n != header.new_length) return false;
  }

  return true;
}

bool write_history_record(
  FsFile &f,
  uint32_t timestamp,
  const std::string &old_value,
  const std::string &new_value
) {
  if (old_value.size() > HISTORY_MAX_VALUE_LENGTH || new_value.size() > HISTORY_MAX_VALUE_LENGTH) {
    ESP_LOGW(TAG, "History value too long; record not written");
    return false;
  }

  HistoryRecordHeader header{};
  header.timestamp = timestamp;
  header.old_length = static_cast<uint16_t>(old_value.size());
  header.new_length = static_cast<uint16_t>(new_value.size());

  if (f.write(reinterpret_cast<const uint8_t *>(&header), sizeof(header)) != sizeof(header)) return false;
  if (!old_value.empty() &&
      f.write(reinterpret_cast<const uint8_t *>(old_value.data()), old_value.size()) != old_value.size()) return false;
  if (!new_value.empty() &&
      f.write(reinterpret_cast<const uint8_t *>(new_value.data()), new_value.size()) != new_value.size()) return false;

  return true;
}

uint16_t history_record_count(uint32_t key) {
  if (!ready && !begin()) return 0;

  FsFile f = fs_open(history_path_for(key), "r");
  if (!f) return 0;

  uint16_t count = 0;
  HistoryRecordHeader header{};
  std::string old_value;
  std::string new_value;

  while (read_history_record(f, header, old_value, new_value)) {
    count++;
    if (count > 1024) break;
  }

  f.close();
  return count;
}

bool append_history(
  uint32_t key,
  uint32_t timestamp,
  const std::string &old_value,
  const std::string &new_value
) {
  if (!ready && !begin()) return false;

  const String final_path = history_path_for(key);
  const uint16_t count = history_record_count(key);

  if (count < HISTORY_MAX_ENTRIES) {
    FsFile f = fs_open(final_path, "a");
    if (!f) return false;
    const bool ok = write_history_record(f, timestamp, old_value, new_value);
    f.flush();
    f.close();
    return ok;
  }

  const String temp_path = final_path + ".tmp";
  FsFile source = fs_open(final_path, "r");
  if (!source) return false;

  FsFile dest = fs_open(temp_path, "w");
  if (!dest) {
    source.close();
    return false;
  }

  const uint16_t records_to_skip = count - (HISTORY_MAX_ENTRIES - 1);
  uint16_t record_index = 0;
  bool ok = true;
  HistoryRecordHeader header{};
  std::string existing_old;
  std::string existing_new;

  while (read_history_record(source, header, existing_old, existing_new)) {
    if (record_index >= records_to_skip &&
        !write_history_record(dest, header.timestamp, existing_old, existing_new)) {
      ok = false;
      break;
    }
    record_index++;
  }

  if (ok) ok = write_history_record(dest, timestamp, old_value, new_value);

  dest.flush();
  source.close();
  dest.close();

  if (!ok) {
    fs_remove(temp_path);
    return false;
  }

  fs_remove(final_path);
  if (!fs_rename(temp_path, final_path)) {
    fs_remove(temp_path);
    return false;
  }

  return true;
}

void record_change(uint32_t key, const std::string &old_value, const std::string &new_value) {
  uint32_t timestamp = 0;
  const time_t now = ::time(nullptr);
  if (now >= 1704067200) timestamp = static_cast<uint32_t>(now);

  UpdateMetadata meta{};
  load_metadata(key, meta);
  meta.update_count++;
  if (timestamp != 0) meta.last_updated = timestamp;

  if (!save_metadata(key, meta)) {
    ESP_LOGW(TAG, "Could not save metadata for key %08lX", static_cast<unsigned long>(key));
  }

  if (!append_history(key, timestamp, old_value, new_value)) {
    ESP_LOGW(TAG, "Could not save history for key %08lX", static_cast<unsigned long>(key));
  }
}

void log_flash_layout() {
#if defined(USE_ESP32)
  FsInfo info{};
  const bool have_info = fs_info(info);
  const size_t fs_size = have_info ? info.totalBytes : 0;
  const size_t fs_used = have_info ? info.usedBytes : 0;

  ESP_LOGI(TAG, "=========== ESP32 FLASH / LITTLEFS LAYOUT ===========");
  ESP_LOGI(TAG, "Flash configured size: %lu bytes", static_cast<unsigned long>(ESP.getFlashChipSize()));
  ESP_LOGI(TAG, "Sketch size:           %lu bytes", static_cast<unsigned long>(ESP.getSketchSize()));
  ESP_LOGI(TAG, "Free sketch space:     %lu bytes", static_cast<unsigned long>(ESP.getFreeSketchSpace()));
  ESP_LOGI(TAG, "LittleFS partition:    %s", FS_PARTITION_LABEL);
  ESP_LOGI(TAG, "LittleFS mount point:  %s", FS_BASE_PATH);
  ESP_LOGI(TAG, "LittleFS total:        %lu bytes", static_cast<unsigned long>(fs_size));
  ESP_LOGI(TAG, "LittleFS used:         %lu bytes", static_cast<unsigned long>(fs_used));
  ESP_LOGI(TAG, "LittleFS free:         %lu bytes",
           static_cast<unsigned long>(fs_size >= fs_used ? fs_size - fs_used : 0));

  if (!have_info || fs_size == 0) {
    ESP_LOGE(TAG, "DIAG RESULT: LittleFS partition unavailable or not mounted");
  } else {
    ESP_LOGI(TAG, "DIAG RESULT: LittleFS partition EXISTS");
  }
#else
  ESP_LOGI(TAG, "=============== LITTLEFS LAYOUT =====================");
  ESP_LOGW(TAG, "Filesystem diagnostics are not implemented for this platform");
#endif
  ESP_LOGI(TAG, "=====================================================");
}

bool begin() {
  if (ready) return true;

#if defined(USE_ESP32)
  if (esp_littlefs_mounted(FS_PARTITION_LABEL)) {
    ready = true;
    return true;
  }

  esp_vfs_littlefs_conf_t conf{};
  conf.base_path = FS_BASE_PATH;
  conf.partition_label = FS_PARTITION_LABEL;
  conf.partition = nullptr;
  conf.format_if_mount_failed = true;
  conf.read_only = false;
  conf.dont_mount = false;
  conf.grow_on_mount = true;

  const esp_err_t err = esp_vfs_littlefs_register(&conf);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "LittleFS mount failed: %s (0x%X)",
             esp_err_to_name(err), static_cast<unsigned int>(err));
    return false;
  }

  ready = true;
  ESP_LOGI(TAG, "LittleFS mounted: partition=%s path=%s", FS_PARTITION_LABEL, FS_BASE_PATH);
  return true;
#else
  ESP_LOGE(TAG, "LittleFS backend is implemented for ESP32 only");
  return false;
#endif
}

bool load_string(uint32_t key, std::string &value, size_t max_len) {
  if (!ready && !begin()) return false;
  FsFile f = fs_open(path_for(key), "r");
  if (!f) return false;

  const size_t len = f.size();
  if (len > max_len) {
    f.close();
    return false;
  }

  value.resize(len);
  const size_t n = len ? f.read(reinterpret_cast<uint8_t *>(&value[0]), len) : 0;
  f.close();
  return n == len;
}

bool save_string(uint32_t key, const std::string &value) {
  if (!ready && !begin()) return false;

  std::string old_value;
  const bool had_old_value = load_string(key, old_value, 1024);
  if (had_old_value && old_value == value) return true;

  const String final_path = path_for(key);
  const String temp_path = final_path + ".tmp";
  FsFile f = fs_open(temp_path, "w");
  if (!f) return false;

  const size_t n = value.empty()
    ? 0
    : f.write(reinterpret_cast<const uint8_t *>(value.data()), value.size());

  f.flush();
  f.close();

  if (n != value.size()) {
    fs_remove(temp_path);
    return false;
  }

  fs_remove(final_path);
  if (!fs_rename(temp_path, final_path)) return false;

  record_change(key, had_old_value ? old_value : std::string("<unset>"), value);
  return true;
}

}  // namespace blind_settings
