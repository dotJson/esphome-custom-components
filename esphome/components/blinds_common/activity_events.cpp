#include "activity_events.h"

namespace blind_webhook {

uint32_t boot_id = 0;
uint32_t event_sequence = 0;
bool boot_event_queued = false;
bool active = false;
bool active_publish = false;
bool post_in_progress = false;
Event active_event{};
std::deque<Event> queue{};
static Verbosity configured_verbosity = Verbosity::ACTIVITY;

const char *verbosity_name(Verbosity verbosity) {
  switch (verbosity) {
    case Verbosity::ESSENTIAL: return "essential";
    case Verbosity::ACTIVITY: return "activity";
    case Verbosity::CHANGES: return "changes";
    case Verbosity::DIAGNOSTIC: return "diagnostic";
    default: return "unknown";
  }
}

bool verbosity_allows(Verbosity configured, Verbosity event) {
  return static_cast<uint8_t>(event) <= static_cast<uint8_t>(configured);
}

void set_verbosity(Verbosity verbosity) { configured_verbosity = verbosity; }
Verbosity get_verbosity() { return configured_verbosity; }

void copy_text(char *dest, size_t dest_size, const std::string &value) {
  if (dest_size == 0) return;
  const size_t n = value.size() < (dest_size - 1) ? value.size() : (dest_size - 1);
  memcpy(dest, value.data(), n);
  dest[n] = '\0';
}

uint32_t valid_epoch_now() {
  const time_t now = ::time(nullptr);
  return now >= 1704067200 ? static_cast<uint32_t>(now) : 0U;
}

void init_boot_session() {
  boot_id = esphome::random_uint32();
  if (boot_id == 0) boot_id = 1;
  // Reserve event 1 for the explicit boot event.
  event_sequence = 1;
  boot_event_queued = false;
  active = false;
  active_publish = false;
  post_in_progress = false;
  queue.clear();
}

uint32_t next_event_id() {
  event_sequence++;
  if (event_sequence == 0) event_sequence = 2;
  return event_sequence;
}

void enqueue(const Event &event) {
  if (queue.size() >= MAX_QUEUED_EVENTS) {
    ESP_LOGW(TAG, "Webhook queue full; dropping oldest event");
    queue.pop_front();
  }
  queue.push_back(event);
}

void clear_queue() { queue.clear(); }
bool has_queued_event() { return !queue.empty(); }
Event &front_event() { return queue.front(); }
void pop_front() { if (!queue.empty()) queue.pop_front(); }

bool queue_event(
  Verbosity verbosity,
  const std::string &event_type,
  const std::string &category,
  const std::string &source,
  const std::string &subject,
  const std::string &result,
  const std::string &previous_value,
  const std::string &value,
  const std::string &detail,
  bool publish_now,
  const std::string &error
) {
  if (!publish_now || !verbosity_allows(configured_verbosity, verbosity)) return false;

  Event event{};
  event.boot_id = boot_id;
  event.event_id = next_event_id();
  event.verbosity = verbosity;
  copy_text(event.event_type, sizeof(event.event_type), event_type);
  copy_text(event.category, sizeof(event.category), category);
  copy_text(event.source, sizeof(event.source), source);
  copy_text(event.subject, sizeof(event.subject), subject);
  copy_text(event.result, sizeof(event.result), result);
  copy_text(event.previous_value, sizeof(event.previous_value), previous_value);
  copy_text(event.value, sizeof(event.value), value);
  copy_text(event.detail, sizeof(event.detail), detail);
  copy_text(event.error, sizeof(event.error), error);
  event.started_epoch = valid_epoch_now();
  event.completed_epoch = event.started_epoch;
  event.started_ms = millis();
  enqueue(event);
  return true;
}

void update_motion_aggregates(Event &event, float current_a, float power_w, float supply_voltage) {
  if (!std::isnan(current_a)) {
    if (std::isnan(event.peak_current_a) || current_a > event.peak_current_a) event.peak_current_a = current_a;
    event.current_sum += current_a;
  }
  if (!std::isnan(power_w)) {
    if (std::isnan(event.peak_power_w) || power_w > event.peak_power_w) event.peak_power_w = power_w;
    event.power_sum += power_w;
  }
  if (!std::isnan(supply_voltage)) {
    if (std::isnan(event.minimum_supply_voltage) || supply_voltage < event.minimum_supply_voltage) event.minimum_supply_voltage = supply_voltage;
    event.voltage_sum += supply_voltage;
  }
  event.motion_samples++;
}

void sample_motion(float current_a, float power_w, float supply_voltage) {
  if (!active) return;
  update_motion_aggregates(active_event, current_a, power_w, supply_voltage);
}

void finish_active(
  const std::string &result,
  const std::string &error,
  bool publish_now,
  float final_position,
  int32_t final_steps,
  bool final_encoder_valid,
  float final_encoder_position,
  float final_ldr_voltage,
  float final_supply_voltage,
  float final_current_a,
  float final_power_w,
  uint32_t superseded_by_event_id
) {
  if (!active) return;

  active_event.completed_epoch = valid_epoch_now();
  active_event.duration_ms = static_cast<uint32_t>(millis() - active_event.started_ms);
  active_event.final_position = final_position;
  active_event.final_steps = final_steps;
  active_event.steps_completed = final_steps - active_event.start_steps;
  active_event.final_encoder_valid = final_encoder_valid;
  active_event.final_encoder_position = final_encoder_position;
  active_event.final_ldr_voltage = final_ldr_voltage;
  active_event.final_supply_voltage = final_supply_voltage;
  active_event.final_current_a = final_current_a;
  active_event.final_power_w = final_power_w;
  active_event.superseded_by_event_id = superseded_by_event_id;
  copy_text(active_event.result, sizeof(active_event.result), result);
  copy_text(active_event.error, sizeof(active_event.error), error);

  if (active_event.motion_samples > 0) {
    active_event.average_current_a = static_cast<float>(active_event.current_sum / active_event.motion_samples);
    active_event.average_power_w = static_cast<float>(active_event.power_sum / active_event.motion_samples);
    active_event.average_supply_voltage = static_cast<float>(active_event.voltage_sum / active_event.motion_samples);
  }

  if (active_publish && publish_now) enqueue(active_event);
  active = false;
  active_publish = false;
}

uint32_t begin_move(
  const std::string &source,
  bool publish_now,
  float start_position,
  float target_position,
  int32_t start_steps,
  int32_t target_steps,
  bool encoder_valid,
  float encoder_position,
  float ldr_voltage,
  float supply_voltage,
  float current_a,
  float power_w
) {
  const uint32_t new_event_id = next_event_id();
  uint32_t supersedes = 0;

  if (active) {
    supersedes = active_event.event_id;
    finish_active("superseded", "", publish_now, start_position, start_steps,
                  encoder_valid, encoder_position, ldr_voltage, supply_voltage,
                  current_a, power_w, new_event_id);
  }

  active_event = Event{};
  active_event.boot_id = boot_id;
  active_event.event_id = new_event_id;
  active_event.supersedes_event_id = supersedes;
  copy_text(active_event.event_type, sizeof(active_event.event_type), "move");
  copy_text(active_event.source, sizeof(active_event.source), source);
  copy_text(active_event.result, sizeof(active_event.result), "in_progress");
  active_event.started_epoch = valid_epoch_now();
  active_event.started_ms = millis();
  active_event.verbosity = Verbosity::ACTIVITY;
  copy_text(active_event.category, sizeof(active_event.category), "motion");
  copy_text(active_event.subject, sizeof(active_event.subject), "blind_position");
  active_event.start_position = start_position;
  active_event.target_position = target_position;
  active_event.start_steps = start_steps;
  active_event.target_steps = target_steps;
  active_event.steps_commanded = target_steps - start_steps;
  active_event.start_encoder_valid = encoder_valid;
  active_event.start_encoder_position = encoder_position;
  active_event.start_ldr_voltage = ldr_voltage;
  active_event.start_supply_voltage = supply_voltage;
  active_event.start_current_a = current_a;
  active_event.start_power_w = power_w;
  active = true;
  active_publish = publish_now && verbosity_allows(configured_verbosity, Verbosity::ACTIVITY);
  return new_event_id;
}

void queue_boot_event(bool publish_now) {
  if (!publish_now || boot_event_queued) return;
  Event event{};
  event.boot_id = boot_id;
  event.event_id = 1;
  copy_text(event.event_type, sizeof(event.event_type), "boot");
  event.verbosity = Verbosity::ESSENTIAL;
  copy_text(event.category, sizeof(event.category), "system");
  copy_text(event.subject, sizeof(event.subject), "controller");
  copy_text(event.source, sizeof(event.source), "system");
  copy_text(event.result, sizeof(event.result), "success");
  event.started_epoch = valid_epoch_now();
  event.completed_epoch = event.started_epoch;
  event.started_ms = 0;
  event.duration_ms = 0;
  // Event 1 is the causal root of every session. Settings restoration may
  // discover the webhook after other events have already been staged, so put
  // Boot at the head rather than allowing its reserved ID to arrive late.
  if (queue.size() >= MAX_QUEUED_EVENTS) queue.pop_back();
  queue.push_front(event);
  boot_event_queued = true;
}

std::string format_epoch_local(uint32_t epoch) {
  if (epoch == 0) return std::string();
  time_t raw = static_cast<time_t>(epoch);
  struct tm t{};
  localtime_r(&raw, &t);
  char text[48];
  strftime(text, sizeof(text), "%Y-%m-%dT%H:%M:%S%z", &t);
  return std::string(text);
}

}  // namespace blind_webhook
