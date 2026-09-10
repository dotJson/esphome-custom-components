#include "storage_diagnostics.h"
#include "persistent_settings.h"
#include "esphome/components/web_server_base/web_server_base.h"

#if defined(USE_ESP32) && defined(USE_WEBSERVER)

#include <array>
#include <atomic>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <time.h>

#include <esp_heap_caps.h>
#include <esp_http_server.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace littlefs_web_dump {
namespace {

static constexpr const char *TAG = "littlefs_web_dump";
static constexpr const char *DUMP_PATH = "/littlefs";
static constexpr size_t SNAPSHOT_CAPACITY = 1024U * 1024U;
static constexpr size_t HTTP_CHUNK_SIZE = 4096U;

std::atomic<MotionQuery> motion_query{nullptr};
std::atomic<bool> snapshot_building{false};
std::atomic<bool> snapshot_ready{false};
std::atomic<bool> snapshot_serving{false};
char *snapshot_buffer = nullptr;
size_t snapshot_length = 0;
bool snapshot_truncated = false;

bool motor_is_moving() {
  const MotionQuery query = motion_query.load(std::memory_order_acquire);
  return query != nullptr && query();
}

void wait_for_motor_idle() {
  while (motor_is_moving()) vTaskDelay(pdMS_TO_TICKS(50));
}

class SnapshotWriter {
 public:
  SnapshotWriter(char *buffer, size_t capacity) : buffer_(buffer), capacity_(capacity) {
    if (buffer_ != nullptr && capacity_ > 0) buffer_[0] = '\0';
  }

  bool ok() const { return ok_; }
  size_t size() const { return length_; }
  bool truncated() const { return truncated_; }

  bool write(const char *text) {
    if (text == nullptr) return false;
    return append_(text, strlen(text));
  }

  bool write(const std::string &text) { return append_(text.data(), text.size()); }

  bool line(const char *text = "") {
    if (!write(text)) return false;
    return write("\n");
  }

  bool linef(const char *format, ...) {
    if (!ok_ || format == nullptr) return false;

    va_list args;
    va_start(args, format);
    const int n = vsnprintf(line_buffer_.data(), line_buffer_.size(), format, args);
    va_end(args);

    if (n < 0) {
      ok_ = false;
      return false;
    }
    if (static_cast<size_t>(n) >= line_buffer_.size()) {
      return line("[line exceeded diagnostics formatter buffer]");
    }
    if (!append_(line_buffer_.data(), static_cast<size_t>(n))) return false;
    return write("\n");
  }

 private:
  bool append_(const char *data, size_t length) {
    if (!ok_ || buffer_ == nullptr || capacity_ == 0) return false;
    if (length == 0) return true;

    const size_t available = capacity_ - 1U - length_;
    if (length > available) {
      static constexpr const char *TRUNCATED =
        "\n[STORAGE DIAGNOSTICS TRUNCATED: 1 MiB snapshot limit reached]\n";
      const size_t marker_len = strlen(TRUNCATED);
      if (available >= marker_len) {
        memcpy(buffer_ + length_, TRUNCATED, marker_len);
        length_ += marker_len;
      }
      buffer_[length_] = '\0';
      truncated_ = true;
      ok_ = false;
      return false;
    }

    memcpy(buffer_ + length_, data, length);
    length_ += length;
    buffer_[length_] = '\0';
    return true;
  }

  char *buffer_{nullptr};
  size_t capacity_{0};
  size_t length_{0};
  bool ok_{true};
  bool truncated_{false};
  std::array<char, 2304> line_buffer_{};
};

const char *key_name(uint32_t key) {
  using namespace blind_settings;
  switch (key) {
    case K_SITE_LATITUDE: return "Site Latitude";
    case K_SITE_LONGITUDE: return "Site Longitude";
    case K_SUN_HORIZON: return "Sun Horizon Elevation";
    case K_LDR_OPEN_V: return "LDR Open Voltage";
    case K_LDR_CLOSED_V: return "LDR Closed Voltage";
    case K_AUTO_OPEN_POS: return "Auto Open Position";
    case K_AUTO_CLOSED_POS: return "Auto Closed Position";
    case K_LDR_SAMPLES: return "LDR Samples";
    case K_MAX_SPEED: return "Stepper Maximum Speed";
    case K_ACCEL: return "Stepper Acceleration";
    case K_DECEL: return "Stepper Deceleration";
    case K_DRIVER_IDLE: return "Driver Idle Timeout";
    case K_RUN_CURRENT: return "TMC2209 Run Current";
    case K_HOLD_CURRENT: return "TMC2209 Hold Current";
    case K_STALLGUARD: return "TMC2209 StallGuard Threshold";
    case K_OVERCURRENT: return "Overcurrent Threshold";
    case K_FULL_RANGE: return "Stepper Full Range Steps";
    case K_CAL_POSITION: return "Stepper Calibration Position";
    case K_LDR_PRESET: return "LDR Resistor Preset";
    case K_AUTO_OPEN_MODE: return "Auto Open Mode";
    case K_TIMEZONE: return "Runtime Timezone";
    case K_NTP1: return "Custom NTP Server 1";
    case K_NTP2: return "Custom NTP Server 2";
    case K_NTP3: return "Custom NTP Server 3";
    case K_OPEN_TIME: return "Auto Open Time";
    case K_CLOSE_TIME: return "Auto Close Time";
    case K_USE_CUSTOM_NTP: return "Use Custom NTP";
    case K_AUTOMATIC_PROGRAM: return "Automatic Program";
    case K_INVERT_TILT: return "Invert Tilt";
    case K_BLIND_TILT_POSITION: return "Blind Tilt Position";
    case K_REINITIALIZE_NEXT_BOOT: return "Reinitialize At Next Boot";
    case K_STEPPER_MOVE_COUNT: return "Stepper Move Count";
    case K_BLOCKED_MOVE_COUNT: return "Blocked Move Count";
    case K_OVERCURRENT_FAULT_COUNT: return "Overcurrent Fault Count";
    case K_MIGRATION_MARKER: return "LittleFS Migration Marker";
    case K_AS5600_ZERO_RAW: return "AS5600 0% Raw Position";
    case K_AS5600_HUNDRED_RAW: return "AS5600 100% Raw Position";
    case K_AS5600_REVERSE: return "AS5600 Reverse Direction";
    case K_WEBHOOK_URL: return "Activity Webhook URL";
    case K_WEBHOOK_ENABLED: return "Activity Webhook Enable";
    default: return "Unknown Setting";
  }
}

bool is_string_key(uint32_t key) {
  using namespace blind_settings;
  switch (key) {
    case K_LDR_PRESET:
    case K_AUTO_OPEN_MODE:
    case K_TIMEZONE:
    case K_NTP1:
    case K_NTP2:
    case K_NTP3:
    case K_WEBHOOK_URL:
      return true;
    default:
      return false;
  }
}

bool is_bool_key(uint32_t key) {
  using namespace blind_settings;
  switch (key) {
    case K_USE_CUSTOM_NTP:
    case K_AUTOMATIC_PROGRAM:
    case K_INVERT_TILT:
    case K_REINITIALIZE_NEXT_BOOT:
    case K_AS5600_REVERSE:
    case K_WEBHOOK_ENABLED:
      return true;
    default:
      return false;
  }
}

bool is_uint32_key(uint32_t key) {
  using namespace blind_settings;
  switch (key) {
    case K_OPEN_TIME:
    case K_CLOSE_TIME:
    case K_STEPPER_MOVE_COUNT:
    case K_BLOCKED_MOVE_COUNT:
    case K_OVERCURRENT_FAULT_COUNT:
    case K_MIGRATION_MARKER:
    case K_AS5600_ZERO_RAW:
    case K_AS5600_HUNDRED_RAW:
      return true;
    default:
      return false;
  }
}

bool is_time_key(uint32_t key) {
  using namespace blind_settings;
  return key == K_OPEN_TIME || key == K_CLOSE_TIME;
}

bool is_float_key(uint32_t key) {
  using namespace blind_settings;
  switch (key) {
    case K_SITE_LATITUDE:
    case K_SITE_LONGITUDE:
    case K_SUN_HORIZON:
    case K_LDR_OPEN_V:
    case K_LDR_CLOSED_V:
    case K_AUTO_OPEN_POS:
    case K_AUTO_CLOSED_POS:
    case K_LDR_SAMPLES:
    case K_MAX_SPEED:
    case K_ACCEL:
    case K_DECEL:
    case K_DRIVER_IDLE:
    case K_RUN_CURRENT:
    case K_HOLD_CURRENT:
    case K_STALLGUARD:
    case K_OVERCURRENT:
    case K_FULL_RANGE:
    case K_CAL_POSITION:
    case K_BLIND_TILT_POSITION:
      return true;
    default:
      return false;
  }
}

std::string format_timestamp(uint32_t timestamp) {
  if (timestamp == 0) return std::string("TIME NOT AVAILABLE");
  const time_t raw = static_cast<time_t>(timestamp);
  struct tm t{};
  localtime_r(&raw, &t);
  char text[48];
  strftime(text, sizeof(text), "%Y-%m-%d %H:%M:%S %Z %z", &t);
  return std::string(text);
}

void raw_hex_dump(SnapshotWriter &out, const String &path) {
  using namespace blind_settings;
  FsFile f = fs_open(path, "r");
  if (!f) {
    out.linef("Could not open %s", path.c_str());
    return;
  }

  out.line("RAW CONTENT:");
  uint32_t offset = 0;
  uint8_t buffer[16];

  while (out.ok() && f.available()) {
    wait_for_motor_idle();
    const size_t n = f.read(buffer, sizeof(buffer));
    char line[80];
    int pos = snprintf(line, sizeof(line), "  %04lX : ", static_cast<unsigned long>(offset));
    for (size_t i = 0; i < n && pos > 0 && static_cast<size_t>(pos) < sizeof(line); i++) {
      pos += snprintf(line + pos, sizeof(line) - static_cast<size_t>(pos), "%02X ", buffer[i]);
    }
    out.line(line);
    offset += n;
  }

  if (offset == 0 && out.ok()) out.line("  <empty>");
  f.close();
}

void dump_current_value(SnapshotWriter &out, uint32_t key) {
  using namespace blind_settings;
  out.linef("SETTING: %s", key_name(key));

  if (is_string_key(key)) {
    std::string value;
    if (load_string(key, value, 1024)) out.linef("CURRENT: \"%s\"", value.c_str());
    else out.line("CURRENT: unable to decode string");
    return;
  }

  if (is_bool_key(key)) {
    bool value = false;
    if (load(key, value)) out.linef("CURRENT: %s", value ? "true" : "false");
    else out.line("CURRENT: unable to decode bool");
    return;
  }

  if (is_uint32_key(key)) {
    uint32_t value = 0;
    if (load(key, value)) {
      if (is_time_key(key)) {
        const uint8_t hour = static_cast<uint8_t>((value >> 16) & 0xFF);
        const uint8_t minute = static_cast<uint8_t>((value >> 8) & 0xFF);
        const uint8_t second = static_cast<uint8_t>(value & 0xFF);
        out.linef("CURRENT: %02u:%02u:%02u", hour, minute, second);
        out.linef("PACKED VALUE: %lu", static_cast<unsigned long>(value));
      } else {
        out.linef("CURRENT: %lu", static_cast<unsigned long>(value));
      }
    } else {
      out.line("CURRENT: unable to decode uint32");
    }
    return;
  }

  if (is_float_key(key)) {
    float value = 0.0f;
    if (load(key, value)) out.linef("CURRENT: %.9g", static_cast<double>(value));
    else out.line("CURRENT: unable to decode float");
    return;
  }

  out.line("CURRENT: unknown data type");
}

void dump_metadata(SnapshotWriter &out, uint32_t key) {
  using namespace blind_settings;
  UpdateMetadata meta{};
  if (!load_metadata(key, meta)) {
    out.line("UPDATE COUNT: 0 / metadata not yet established");
    out.line("LAST UPDATED: unknown");
    return;
  }

  out.linef("UPDATE COUNT: %lu", static_cast<unsigned long>(meta.update_count));
  if (meta.last_updated == 0) {
    out.line("LAST UPDATED: TIME NOT AVAILABLE");
    return;
  }

  const std::string when = format_timestamp(meta.last_updated);
  out.linef("LAST UPDATED: %s", when.c_str());
  out.linef("LAST UPDATED EPOCH: %lu", static_cast<unsigned long>(meta.last_updated));
}

void dump_history(SnapshotWriter &out, uint32_t key) {
  using namespace blind_settings;
  const String path = history_path_for(key);
  FsFile f = fs_open(path, "r");
  if (!f) {
    out.line("HISTORY: none");
    return;
  }

  out.linef("HISTORY FILE: %s", path.c_str());
  out.linef("HISTORY SIZE: %u bytes", static_cast<unsigned int>(f.size()));

  uint16_t record_number = 0;
  HistoryRecordHeader header{};
  std::string old_value;
  std::string new_value;

  while (out.ok()) {
    wait_for_motor_idle();
    if (!read_history_record(f, header, old_value, new_value)) break;
    record_number++;
    const std::string when = format_timestamp(header.timestamp);
    out.linef("  CHANGE %u", record_number);
    out.linef("    DATE: %s", when.c_str());
    if (header.timestamp != 0) out.linef("    EPOCH: %lu", static_cast<unsigned long>(header.timestamp));
    out.linef("    OLD: \"%s\"", old_value.c_str());
    out.linef("    NEW: \"%s\"", new_value.c_str());
    out.linef("    CHANGE: \"%s\" -> \"%s\"", old_value.c_str(), new_value.c_str());
  }

  f.close();
  if (out.ok()) out.linef("HISTORY RECORDS RETAINED: %u / %u", record_number, HISTORY_MAX_ENTRIES);
}

void stream_dump(SnapshotWriter &out) {
  using namespace blind_settings;
  out.line("============================================================");
  out.line("LITTLEFS SETTINGS / METADATA / HISTORY DIAGNOSTICS");
  out.line("============================================================");

  if (!ready) {
    out.line("LittleFS is not mounted.");
    out.line("Diagnostics aborted rather than formatting or modifying filesystem.");
    return;
  }

  wait_for_motor_idle();
  FsInfo fs_stats{};
  if (fs_info(fs_stats)) {
    const size_t free_bytes = fs_stats.totalBytes >= fs_stats.usedBytes
      ? fs_stats.totalBytes - fs_stats.usedBytes : 0;
    out.linef("FILESYSTEM TOTAL: %u bytes", static_cast<unsigned int>(fs_stats.totalBytes));
    out.linef("FILESYSTEM USED:  %u bytes", static_cast<unsigned int>(fs_stats.usedBytes));
    out.linef("FILESYSTEM FREE:  %u bytes", static_cast<unsigned int>(free_bytes));
    out.line("FILESYSTEM BACKEND: ESP-IDF LittleFS VFS");
  } else {
    out.line("Could not obtain LittleFS usage information");
  }

  uint16_t total_files = 0;
  uint16_t cfg_files = 0;
  uint16_t meta_files = 0;
  uint16_t hist_files = 0;
  uint16_t other_files = 0;

  out.line("------------------------------------------------------------");
  out.line("PHYSICAL FILE INVENTORY");
  out.line("------------------------------------------------------------");

  wait_for_motor_idle();
  FsDir dir = fs_open_dir("/");
  while (out.ok() && dir.next()) {
    wait_for_motor_idle();
    total_files++;
    const String name = dir.fileName();
    FsFile f = dir.openFile("r");
    const size_t size = f ? f.size() : 0;
    if (f) f.close();

    out.linef("FILE: %s", name.c_str());
    out.linef("SIZE: %u bytes", static_cast<unsigned int>(size));

    if (name.startsWith("cfg_")) {
      cfg_files++;
      raw_hex_dump(out, name);
    } else if (name.startsWith("meta_")) {
      meta_files++;
      raw_hex_dump(out, name);
    } else if (name.startsWith("hist_")) {
      hist_files++;
    } else {
      other_files++;
      raw_hex_dump(out, name);
    }

    out.line();
    vTaskDelay(pdMS_TO_TICKS(5));
  }
  dir.close();
  if (!out.ok()) return;

  out.line("============================================================");
  out.line("DECODED SETTINGS");
  out.line("============================================================");

  wait_for_motor_idle();
  FsDir cfg_dir = fs_open_dir("/");
  uint16_t decoded_settings = 0;

  while (out.ok() && cfg_dir.next()) {
    wait_for_motor_idle();
    const String name = cfg_dir.fileName();
    if (!name.startsWith("cfg_")) continue;

    const char *hex_start = name.c_str() + 4;
    char key_text[9];
    memset(key_text, 0, sizeof(key_text));
    strncpy(key_text, hex_start, 8);
    const uint32_t key = static_cast<uint32_t>(strtoul(key_text, nullptr, 16));
    decoded_settings++;

    out.line("------------------------------------------------------------");
    out.linef("KEY: 0x%08lX", static_cast<unsigned long>(key));
    out.linef("CFG FILE: %s", name.c_str());
    dump_current_value(out, key);
    dump_metadata(out, key);
    dump_history(out, key);
    vTaskDelay(pdMS_TO_TICKS(5));
  }
  cfg_dir.close();
  if (!out.ok()) return;

  out.line("============================================================");
  out.line("DIAGNOSTICS SUMMARY");
  out.line("============================================================");
  out.linef("TOTAL FILES:        %u", total_files);
  out.linef("CFG FILES:          %u", cfg_files);
  out.linef("METADATA FILES:     %u", meta_files);
  out.linef("HISTORY FILES:      %u", hist_files);
  out.linef("OTHER FILES:        %u", other_files);
  out.linef("DECODED SETTINGS:   %u", decoded_settings);
  out.linef("HISTORY LIMIT:      %u changes per setting", HISTORY_MAX_ENTRIES);
  out.line("============================================================");
  out.line("END STORAGE DIAGNOSTICS");
  out.line("============================================================");
}

void snapshot_task(void *parameter) {
  (void) parameter;
  ESP_LOGI(TAG, "Building LittleFS diagnostics snapshot in low-priority task");
  wait_for_motor_idle();

  char *new_buffer = static_cast<char *>(
    heap_caps_malloc(SNAPSHOT_CAPACITY, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));

  if (new_buffer == nullptr) {
    ESP_LOGE(TAG, "Could not allocate 1 MiB PSRAM diagnostics snapshot buffer");
    snapshot_building.store(false, std::memory_order_release);
    vTaskDelete(nullptr);
    return;
  }

  SnapshotWriter out(new_buffer, SNAPSHOT_CAPACITY);
  stream_dump(out);
  const size_t new_length = out.size();
  const bool new_truncated = out.truncated();

  char *old_buffer = snapshot_buffer;
  snapshot_buffer = new_buffer;
  snapshot_length = new_length;
  snapshot_truncated = new_truncated;
  snapshot_ready.store(true, std::memory_order_release);
  snapshot_building.store(false, std::memory_order_release);

  if (old_buffer != nullptr) heap_caps_free(old_buffer);

  ESP_LOGI(TAG, "LittleFS diagnostics snapshot ready: %u bytes%s",
           static_cast<unsigned int>(new_length), new_truncated ? " (TRUNCATED)" : "");
  vTaskDelete(nullptr);
}

bool start_snapshot_build() {
  bool expected = false;
  if (!snapshot_building.compare_exchange_strong(expected, true, std::memory_order_acq_rel)) return false;

  TaskHandle_t task_handle = nullptr;
  const BaseType_t result = xTaskCreate(
    snapshot_task, "storage_diagnostics", 12288, nullptr, 1, &task_handle);

  if (result != pdPASS) {
    snapshot_building.store(false, std::memory_order_release);
    ESP_LOGE(TAG, "Could not start LittleFS diagnostics snapshot task");
    return false;
  }
  return true;
}

const char *waiting_page() {
  return "<!doctype html><html><head>"
         "<meta charset=\"utf-8\">"
         "<meta http-equiv=\"refresh\" content=\"1\">"
         "<title>Storage Diagnostics</title></head><body><pre>"
         "Storage diagnostics are waiting for motor travel to finish.\n"
         "No filesystem work will run while the motor is moving.\n"
         "This page will refresh automatically."
         "</pre></body></html>";
}

const char *building_page() {
  return "<!doctype html><html><head>"
         "<meta charset=\"utf-8\">"
         "<meta http-equiv=\"refresh\" content=\"1\">"
         "<title>Storage Diagnostics</title></head><body><pre>"
         "Storage diagnostics are being prepared in a low-priority background task.\n"
         "This page will refresh automatically."
         "</pre></body></html>";
}

class StorageDiagnosticsHandler : public AsyncWebHandler {
 public:
  bool canHandle(AsyncWebServerRequest *request) const override {
    if (request == nullptr || request->method() != HTTP_GET) return false;
    char url_buffer[AsyncWebServerRequest::URL_BUF_SIZE];
    return request->url_to(url_buffer) == DUMP_PATH;
  }

  void handleRequest(AsyncWebServerRequest *request) override {
    if (request == nullptr) return;
    httpd_req_t *raw_request = static_cast<httpd_req_t *>(*request);
    if (raw_request == nullptr) return;

    if (motor_is_moving()) {
      httpd_resp_set_status(raw_request, "202 Accepted");
      httpd_resp_set_type(raw_request, "text/html; charset=utf-8");
      httpd_resp_set_hdr(raw_request, "Cache-Control", "no-store");
      (void) httpd_resp_send(raw_request, waiting_page(), HTTPD_RESP_USE_STRLEN);
      return;
    }

    if (!snapshot_ready.load(std::memory_order_acquire)) {
      const bool started = start_snapshot_build();
      httpd_resp_set_status(raw_request, "202 Accepted");
      httpd_resp_set_type(raw_request, "text/html; charset=utf-8");
      httpd_resp_set_hdr(raw_request, "Cache-Control", "no-store");
      (void) httpd_resp_send(raw_request, building_page(), HTTPD_RESP_USE_STRLEN);
      if (started) ESP_LOGI(TAG, "Started nonblocking LittleFS diagnostics snapshot");
      return;
    }

    bool expected_serving = false;
    if (!snapshot_serving.compare_exchange_strong(expected_serving, true, std::memory_order_acq_rel)) {
      httpd_resp_set_status(raw_request, "503 Service Unavailable");
      httpd_resp_set_type(raw_request, "text/plain; charset=utf-8");
      (void) httpd_resp_send(raw_request, "Storage diagnostics snapshot is already being served.\n", HTTPD_RESP_USE_STRLEN);
      return;
    }

    if (motor_is_moving()) {
      snapshot_serving.store(false, std::memory_order_release);
      httpd_resp_set_status(raw_request, "202 Accepted");
      httpd_resp_set_type(raw_request, "text/html; charset=utf-8");
      httpd_resp_set_hdr(raw_request, "Cache-Control", "no-store");
      (void) httpd_resp_send(raw_request, waiting_page(), HTTPD_RESP_USE_STRLEN);
      return;
    }

    const char *data = snapshot_buffer;
    const size_t length = snapshot_length;
    if (data == nullptr) {
      snapshot_serving.store(false, std::memory_order_release);
      snapshot_ready.store(false, std::memory_order_release);
      httpd_resp_send_err(raw_request, HTTPD_500_INTERNAL_SERVER_ERROR, "Snapshot unavailable");
      return;
    }

    httpd_resp_set_status(raw_request, HTTPD_200);
    httpd_resp_set_type(raw_request, "text/plain; charset=utf-8");
    httpd_resp_set_hdr(raw_request, "Cache-Control", "no-store");
    httpd_resp_set_hdr(raw_request, "Content-Disposition", "inline; filename=\"storage-diagnostics.txt\"");

    size_t offset = 0;
    while (offset < length) {
      wait_for_motor_idle();
      const size_t remaining = length - offset;
      const size_t n = remaining > HTTP_CHUNK_SIZE ? HTTP_CHUNK_SIZE : remaining;
      const esp_err_t err = httpd_resp_send_chunk(raw_request, data + offset, n);
      if (err != ESP_OK) {
        ESP_LOGW(TAG, "Storage diagnostics client disconnected: %s", esp_err_to_name(err));
        snapshot_serving.store(false, std::memory_order_release);
        return;
      }
      offset += n;
      vTaskDelay(1);
    }

    (void) httpd_resp_send_chunk(raw_request, nullptr, 0);
    ESP_LOGI(TAG, "Served LittleFS diagnostics RAM snapshot: %u bytes", static_cast<unsigned int>(length));

    snapshot_ready.store(false, std::memory_order_release);
    snapshot_buffer = nullptr;
    snapshot_length = 0;
    heap_caps_free(const_cast<char *>(data));
    snapshot_serving.store(false, std::memory_order_release);
  }
};

StorageDiagnosticsHandler handler;
bool handler_registered = false;

}  // namespace

void set_motion_query(MotionQuery query) {
  motion_query.store(query, std::memory_order_release);
}

bool register_handler() {
  if (handler_registered) return true;

  auto *base = esphome::web_server_base::global_web_server_base;
  if (base == nullptr) {
    ESP_LOGE(TAG, "Web server base unavailable; /littlefs not registered");
    return false;
  }

  base->add_handler(&handler);
  handler_registered = true;
  ESP_LOGI(TAG, "Registered storage diagnostics endpoint at %s", DUMP_PATH);
  return true;
}

}  // namespace littlefs_web_dump

#endif  // USE_ESP32 && USE_WEBSERVER
