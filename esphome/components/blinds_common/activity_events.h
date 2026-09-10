#pragma once

// Runtime activity-event/session model. No persistent storage is performed here.

#include "esphome.h"
#include <Arduino.h>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <string>
#include <time.h>

namespace blind_webhook {

static constexpr const char *TAG = "blind_webhook";
static constexpr size_t MAX_QUEUED_EVENTS = 8;

struct Event {
  uint32_t boot_id{0};
  uint32_t event_id{0};
  uint32_t supersedes_event_id{0};
  uint32_t superseded_by_event_id{0};

  char event_type[24]{};
  char source[40]{};
  char result[40]{};
  char error[80]{};

  uint32_t started_epoch{0};
  uint32_t completed_epoch{0};
  uint32_t started_ms{0};
  uint32_t duration_ms{0};

  float start_position{NAN};
  float target_position{NAN};
  float final_position{NAN};

  int32_t start_steps{0};
  int32_t target_steps{0};
  int32_t final_steps{0};
  int32_t steps_commanded{0};
  int32_t steps_completed{0};

  bool start_encoder_valid{false};
  bool final_encoder_valid{false};
  float start_encoder_position{NAN};
  float final_encoder_position{NAN};

  float start_ldr_voltage{NAN};
  float final_ldr_voltage{NAN};

  float start_supply_voltage{NAN};
  float start_current_a{NAN};
  float start_power_w{NAN};

  float peak_current_a{NAN};
  float average_current_a{NAN};
  float peak_power_w{NAN};
  float average_power_w{NAN};
  float minimum_supply_voltage{NAN};
  float average_supply_voltage{NAN};

  float final_supply_voltage{NAN};
  float final_current_a{NAN};
  float final_power_w{NAN};

  uint32_t motion_samples{0};
  double current_sum{0.0};
  double power_sum{0.0};
  double voltage_sum{0.0};
};

extern uint32_t boot_id;
extern uint32_t event_sequence;
extern bool boot_event_queued;
extern bool active;
extern bool active_publish;
extern bool post_in_progress;
extern Event active_event;
extern std::deque<Event> queue;

void copy_text(char *dest, size_t dest_size, const std::string &value);
uint32_t valid_epoch_now();
void init_boot_session();
uint32_t next_event_id();
void enqueue(const Event &event);
void clear_queue();
bool has_queued_event();
Event &front_event();
void pop_front();
void update_motion_aggregates(Event &event, float current_a, float power_w, float supply_voltage);
void sample_motion(float current_a, float power_w, float supply_voltage);

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
  uint32_t superseded_by_event_id = 0
);

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
);

void queue_boot_event(bool publish_now);
std::string format_epoch_local(uint32_t epoch);

}  // namespace blind_webhook
