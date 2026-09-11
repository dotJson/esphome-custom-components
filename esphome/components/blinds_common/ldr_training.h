#pragma once

#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <time.h>

#include "esphome/core/log.h"

namespace ldr_training {

static constexpr float ADC_MAX_V = 3.3f * 0.3125f;
static constexpr float PRIMARY_OHMS = 18000.0f;
static constexpr uint16_t BIN_COUNT = 256;
static constexpr uint32_t MAGIC = 0x4C445231U;  // LDR1
static constexpr uint16_t VERSION = 1;
static constexpr const char *CHECKPOINT = "/littlefs/ldr_learn.chk";
static constexpr const char *CHECKPOINT_TMP = "/littlefs/ldr_learn.tmp";
static constexpr const char *SUMMARY = "/littlefs/ldr_learn_summary.txt";
static constexpr const char *SUMMARY_TMP = "/littlefs/ldr_learn_summary.tmp";

struct __attribute__((packed)) Checkpoint {
  uint32_t magic;
  uint16_t version;
  uint16_t bins;
  uint32_t duration_seconds;
  uint32_t elapsed_seconds;
  uint32_t sample_count;
  uint32_t histogram[BIN_COUNT];
  uint32_t crc32;
};

struct Result {
  bool valid{false};
  float bright_v{NAN};
  float dark_v{NAN};
  float span_v{NAN};
  float closed_v{NAN};
  float open_v{NAN};
  float recommended_parallel_ohms{NAN};
  float predicted_bright_v{NAN};
  float predicted_dark_v{NAN};
  bool no_secondary{true};
};

class Trainer {
 public:
  void begin() { restore(); }

  void start(uint32_t duration_hours) {
    histogram_.fill(0);
    sample_count_ = 0;
    elapsed_before_boot_s_ = 0;
    started_ms_ = millis();
    last_checkpoint_ms_ = started_ms_;
    duration_s_ = duration_hours * 3600U;
    active_ = true;
    complete_ = false;
    result_ = Result{};
    std::remove(CHECKPOINT);
    ESP_LOGI("ldr_training", "Learning started for %u hour(s)", duration_hours);
  }

  void cancel() {
    active_ = false;
    complete_ = false;
    histogram_.fill(0);
    sample_count_ = 0;
    std::remove(CHECKPOINT);
    ESP_LOGI("ldr_training", "Learning cancelled; checkpoint removed");
  }

  void add_sample(float volts) {
    if (!active_ || !std::isfinite(volts) || volts < 0.0f || volts > ADC_MAX_V * 1.02f) return;
    float clipped = volts < 0.0f ? 0.0f : (volts > ADC_MAX_V ? ADC_MAX_V : volts);
    uint16_t bin = static_cast<uint16_t>(floorf((clipped / ADC_MAX_V) * BIN_COUNT));
    if (bin >= BIN_COUNT) bin = BIN_COUNT - 1;
    if (histogram_[bin] != UINT32_MAX) histogram_[bin]++;
    if (sample_count_ != UINT32_MAX) sample_count_++;
    if (elapsed_seconds() >= duration_s_) finish();
  }

  bool checkpoint_if_due(bool motor_idle) {
    if (!active_ || !motor_idle || (uint32_t)(millis() - last_checkpoint_ms_) < 3600000U) return false;
    return checkpoint();
  }

  bool checkpoint() {
    if (!active_) return false;
    Checkpoint cp{};
    cp.magic = MAGIC; cp.version = VERSION; cp.bins = BIN_COUNT;
    cp.duration_seconds = duration_s_; cp.elapsed_seconds = elapsed_seconds(); cp.sample_count = sample_count_;
    memcpy(cp.histogram, histogram_.data(), sizeof(cp.histogram));
    cp.crc32 = crc32(reinterpret_cast<const uint8_t *>(&cp), sizeof(cp) - sizeof(cp.crc32));
    FILE *f = fopen(CHECKPOINT_TMP, "wb");
    if (f == nullptr) return false;
    bool ok = fwrite(&cp, 1, sizeof(cp), f) == sizeof(cp) && fflush(f) == 0;
    fclose(f);
    if (ok) { std::remove(CHECKPOINT); ok = std::rename(CHECKPOINT_TMP, CHECKPOINT) == 0; }
    if (!ok) std::remove(CHECKPOINT_TMP);
    if (ok) last_checkpoint_ms_ = millis();
    return ok;
  }

  bool restore() {
    FILE *f = fopen(CHECKPOINT, "rb");
    if (f == nullptr) return false;
    Checkpoint cp{};
    bool ok = fread(&cp, 1, sizeof(cp), f) == sizeof(cp);
    fclose(f);
    uint32_t expected = crc32(reinterpret_cast<const uint8_t *>(&cp), sizeof(cp) - sizeof(cp.crc32));
    if (!ok || cp.magic != MAGIC || cp.version != VERSION || cp.bins != BIN_COUNT || cp.crc32 != expected || cp.elapsed_seconds >= cp.duration_seconds) {
      ESP_LOGW("ldr_training", "Ignoring invalid or expired checkpoint");
      return false;
    }
    duration_s_ = cp.duration_seconds; elapsed_before_boot_s_ = cp.elapsed_seconds;
    sample_count_ = cp.sample_count; memcpy(histogram_.data(), cp.histogram, sizeof(cp.histogram));
    started_ms_ = millis(); last_checkpoint_ms_ = started_ms_; active_ = true; complete_ = false;
    ESP_LOGI("ldr_training", "Restored %u samples at %.1f%%", sample_count_, progress_percent());
    return true;
  }

  void finish() {
    if (!active_) return;
    active_ = false;
    result_ = calculate();
    complete_ = result_.valid;
    std::remove(CHECKPOINT);
    write_summary();
  }

  Result calculate() const {
    Result r{};
    if (sample_count_ < 120) return r;
    r.bright_v = percentile(0.02f); r.dark_v = percentile(0.98f);
    r.span_v = r.dark_v - r.bright_v;
    if (!(r.span_v > 0.01f) || r.bright_v <= 0.0f || r.dark_v >= ADC_MAX_V) return r;
    float margin = fmaxf(0.005f, r.span_v * 0.05f);
    r.closed_v = r.bright_v + margin; r.open_v = r.dark_v - margin;
    const float ldr_bright = PRIMARY_OHMS * r.bright_v / (ADC_MAX_V - r.bright_v);
    const float ldr_dark = PRIMARY_OHMS * r.dark_v / (ADC_MAX_V - r.dark_v);
    const float kit[] = {1000,1800,3000,4700,5100,7500,10000,12000,18000,33000,43000,51000,75000,100000,200000,300000,390000,470000,680000,1000000};
    float best_score = score(PRIMARY_OHMS, ldr_bright, ldr_dark);
    float best_eff = PRIMARY_OHMS;
    for (float parallel : kit) {
      float eff = PRIMARY_OHMS * parallel / (PRIMARY_OHMS + parallel);
      float s = score(eff, ldr_bright, ldr_dark);
      if (s > best_score) { best_score = s; best_eff = eff; r.recommended_parallel_ohms = parallel; r.no_secondary = false; }
    }
    r.predicted_bright_v = ADC_MAX_V * ldr_bright / (best_eff + ldr_bright);
    r.predicted_dark_v = ADC_MAX_V * ldr_dark / (best_eff + ldr_dark);
    r.valid = true;
    return r;
  }

  bool active() const { return active_; }
  bool complete() const { return complete_; }
  uint32_t samples() const { return sample_count_; }
  uint32_t duration_hours() const { return duration_s_ / 3600U; }
  float progress_percent() const { return duration_s_ ? fminf(100.0f, 100.0f * elapsed_seconds() / duration_s_) : 0.0f; }
  const Result &result() const { return result_; }

  std::string status() const {
    char b[96];
    if (active_) snprintf(b, sizeof(b), "LEARNING %.1f%% (%u samples)", progress_percent(), sample_count_);
    else if (complete_) snprintf(b, sizeof(b), "READY TO APPLY (%u samples)", sample_count_);
    else snprintf(b, sizeof(b), "IDLE");
    return b;
  }

  std::string recommendation() const {
    if (!result_.valid) return "NOT AVAILABLE";
    if (result_.no_secondary) return "NONE - LEAVE R-ADJUST OPEN";
    char b[96]; snprintf(b, sizeof(b), "ADD %.1f kOhm PARALLEL (R-effective %.1f kOhm)", result_.recommended_parallel_ohms / 1000.0f,
      (PRIMARY_OHMS * result_.recommended_parallel_ohms / (PRIMARY_OHMS + result_.recommended_parallel_ohms)) / 1000.0f);
    return b;
  }

 private:
  uint32_t elapsed_seconds() const { return elapsed_before_boot_s_ + ((uint32_t)(millis() - started_ms_) / 1000U); }
  float percentile(float p) const {
    uint32_t target = static_cast<uint32_t>(ceilf(sample_count_ * p)); if (target < 1) target = 1;
    uint32_t cumulative = 0;
    for (uint16_t i = 0; i < BIN_COUNT; i++) { cumulative += histogram_[i]; if (cumulative >= target) return (i + 0.5f) * ADC_MAX_V / BIN_COUNT; }
    return ADC_MAX_V;
  }
  static float score(float top, float rb, float rd) {
    float vb = ADC_MAX_V * rb / (top + rb), vd = ADC_MAX_V * rd / (top + rd);
    float span = vd - vb, midpoint = (vd + vb) * 0.5f;
    float score = span - 0.35f * fabsf(midpoint - ADC_MAX_V * 0.5f);
    if (vb < 0.10f) score -= (0.10f - vb) * 3.0f;
    if (vd > 0.93f) score -= (vd - 0.93f) * 3.0f;
    return score;
  }
  static uint32_t crc32(const uint8_t *data, size_t len) {
    uint32_t crc = 0xFFFFFFFFU;
    while (len--) { crc ^= *data++; for (uint8_t i = 0; i < 8; i++) crc = (crc >> 1) ^ (0xEDB88320U & (0U - (crc & 1U))); }
    return ~crc;
  }
  void write_summary() const {
    if (!result_.valid) return;
    FILE *f = fopen(SUMMARY_TMP, "w"); if (f == nullptr) return;
    fprintf(f, "version=%u\nprimary_ohms=18000\nsamples=%u\nbright_v=%.5f\ndark_v=%.5f\nspan_v=%.5f\nclosed_v=%.5f\nopen_v=%.5f\nrecommendation=%s\npredicted_bright_v=%.5f\npredicted_dark_v=%.5f\n",
      VERSION, sample_count_, result_.bright_v, result_.dark_v, result_.span_v, result_.closed_v, result_.open_v,
      recommendation().c_str(), result_.predicted_bright_v, result_.predicted_dark_v);
    bool ok = fflush(f) == 0; fclose(f);
    if (ok) { std::remove(SUMMARY); std::rename(SUMMARY_TMP, SUMMARY); } else std::remove(SUMMARY_TMP);
  }

  std::array<uint32_t, BIN_COUNT> histogram_{};
  uint32_t sample_count_{0}, duration_s_{0}, elapsed_before_boot_s_{0}, started_ms_{0}, last_checkpoint_ms_{0};
  bool active_{false}, complete_{false};
  Result result_{};
};

inline Trainer trainer;

}  // namespace ldr_training
