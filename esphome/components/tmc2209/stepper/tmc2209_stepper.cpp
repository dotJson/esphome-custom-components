#include "tmc2209_stepper.h"
#include "esphome/components/tmc2209/tmc2209_api_registers.h"

#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace tmc2209 {

void TMC2209Stepper::dump_config() {
  ESP_LOGCONFIG(TAG, "TMC2209 Stepper:");
  LOG_STEPPER(this);
  LOG_TMC2209(this);
}

void TMC2209Stepper::setup() {
  ESP_LOGCONFIG(TAG, "Setting up TMC2209 Stepper...");

  TMC2209Component::setup();

  // The base component marks itself failed when the TMC2209 does not
  // answer its startup VERSION probe. Do not continue issuing UART
  // configuration writes to hardware that is absent/unpowered.
  if (this->is_failed()) {
    ESP_LOGE(TAG, "TMC2209 unavailable - skipping stepper setup");
    return;
  }

  this->high_freq_.start();

  // Ensure UART velocity mode is stopped at startup.
  // If the stop cannot be confirmed, deliberately keep the software cache
  // non-zero so loop() will continue retrying VACTUAL=0.
  if (this->write_field(VACTUAL_FIELD, 0)) {
    this->vactual_ = 0;
  } else {
    this->vactual_ = 1;
    ESP_LOGE(TAG, "FAILED TO CONFIRM VACTUAL=0 AT STARTUP - STOP WILL RETRY");
  }

  if (this->control_method_ == ControlMethod::PULSES_CONTROL) {
    this->write_field(MULTISTEP_FILT_FIELD, false);
    this->write_field(DEDGE_FIELD, true);
  }

  if (this->control_method_ == ControlMethod::SERIAL_CONTROL) {
    this->write_field(DEDGE_FIELD, false);
    bool gconf_ok = false;

    for (uint8_t attempt = 1; attempt <= 5; attempt++) {
      int32_t current_gconf = this->read_register(GCONF);
      int32_t desired_gconf = current_gconf;
      desired_gconf &= ~(static_cast<int32_t>(1UL << 4));
      desired_gconf |= static_cast<int32_t>(1UL << 5);
      desired_gconf |= static_cast<int32_t>(1UL << 7);

      ESP_LOGI(TAG,
               "SERIAL GCONF attempt %u: current=0x%08lX desired=0x%08lX",
               attempt,
               static_cast<unsigned long>(static_cast<uint32_t>(current_gconf)),
               static_cast<unsigned long>(static_cast<uint32_t>(desired_gconf)));

      const bool write_ok = this->write_register(GCONF, desired_gconf);
      if (!write_ok) {
        ESP_LOGW(TAG, "SERIAL GCONF attempt %u write was not confirmed", attempt);
        continue;
      }

      const int32_t verified_gconf = this->read_register(GCONF);
      const bool index_step_ok =
          (verified_gconf & static_cast<int32_t>(1UL << 5)) != 0;
      const bool index_otpw_ok =
          (verified_gconf & static_cast<int32_t>(1UL << 4)) == 0;
      const bool mstep_reg_ok =
          (verified_gconf & static_cast<int32_t>(1UL << 7)) != 0;

      ESP_LOGI(TAG,
               "SERIAL GCONF verify: 0x%08lX INDEX_STEP=%u INDEX_OTPW=%u MSTEP_REG_SELECT=%u",
               static_cast<unsigned long>(static_cast<uint32_t>(verified_gconf)),
               index_step_ok, !index_otpw_ok, mstep_reg_ok);

      if (index_step_ok && index_otpw_ok && mstep_reg_ok) {
        gconf_ok = true;
        break;
      }

      ESP_LOGW(TAG, "SERIAL GCONF verification failed on attempt %u", attempt);
    }

    if (!gconf_ok) {
      ESP_LOGE(TAG, "FAILED TO CONFIGURE SERIAL INDEX FEEDBACK");
    } else {
      ESP_LOGI(TAG, "SERIAL INDEX FEEDBACK CONFIGURED");
    }

    this->ips_.current_position_ptr = &this->current_position;
    this->ips_.direction_ptr = &this->current_direction;
    this->index_pin_->attach_interrupt(IndexPulseStore::pulse_isr, &this->ips_,
                                       gpio::INTERRUPT_ANY_EDGE);
  }

  ESP_LOGCONFIG(TAG, "TMC2209 Stepper setup done.");
}

void TMC2209Stepper::on_shutdown() {
  this->stop();
}

void TMC2209Stepper::loop() {
  // Once setup has marked the driver failed, this component must become
  // completely inert for the rest of the boot. In particular, do not call
  // the base loop because it may perform UART health/status transactions.
  if (this->is_failed()) {
    this->current_direction = Direction::STANDSTILL;
    return;
  }

  TMC2209Component::loop();

  const time_t now = micros();
  this->calculate_speed_(now);

  const int32_t to_target = this->target_position - this->current_position;
  this->current_direction =
      (to_target != 0)
          ? static_cast<Direction>(to_target / abs(to_target))
          : Direction::STANDSTILL;

  if (this->control_method_ == ControlMethod::SERIAL_CONTROL) {
    if (!this->target_initialized_) {
      if (this->vactual_ != 0) {
        if (this->write_field(VACTUAL_FIELD, 0)) {
          this->vactual_ = 0;
        } else {
          this->vactual_ = 1;
          ESP_LOGE(TAG, "FAILED TO CONFIRM VACTUAL=0 BEFORE TARGET INITIALIZATION");
        }
      }
      this->current_direction = Direction::STANDSTILL;
      return;
    }

    int32_t requested_vactual = 0;
    if (this->current_direction != Direction::STANDSTILL &&
        this->current_speed_ > 0.0f) {
      requested_vactual = this->speed_to_vactual(
          static_cast<int32_t>(this->current_speed_));
      requested_vactual *= static_cast<int32_t>(this->current_direction);
    }

    if (this->vactual_ != requested_vactual) {
      const bool confirmed = this->write_field(VACTUAL_FIELD, requested_vactual);
      if (confirmed) {
        this->vactual_ = requested_vactual;
      } else {
        ESP_LOGE(TAG, "VACTUAL write failed: requested=%ld",
                 static_cast<long>(requested_vactual));
        if (requested_vactual == 0) {
          ESP_LOGE(TAG, "FAILED TO CONFIRM MOTOR STOP - RETRYING VACTUAL=0");
          if (this->vactual_ == 0) {
            this->vactual_ = 1;
          }
        }
      }
    }
  }

  if (this->control_method_ == ControlMethod::PULSES_CONTROL) {
    if (this->current_direction == Direction::STANDSTILL ||
        this->current_speed_ <= 0.0f) {
      return;
    }

    const float pulse_interval_us = 1000000.0f / this->current_speed_;
    const uint32_t dt = static_cast<uint32_t>(now - this->last_step_);

    if (static_cast<float>(dt) >= pulse_interval_us) {
      if (this->direction_ != this->current_direction) {
        this->dir_pin_->digital_write(
            this->current_direction == Direction::BACKWARD);
        this->direction_ = this->current_direction;
      }

      this->step_pin_->digital_write(this->step_state_);
      this->step_state_ = !this->step_state_;
      this->current_position += static_cast<int32_t>(this->current_direction);
      this->last_step_ = now;
    }
  }
}

void TMC2209Stepper::set_target(int32_t steps) {
  // set_target() can be called by ESPHome/YAML during startup even after this
  // component has been marked failed. Reject it before ANY register access.
  if (this->is_failed()) {
    this->current_direction = Direction::STANDSTILL;
    this->target_position = this->current_position;
    ESP_LOGW(TAG, "Ignoring target command because TMC2209 is unavailable");
    return;
  }

  if (this->control_method_ == ControlMethod::CONTROL_UNSET) {
    ESP_LOGE(TAG, "Control method not set!");
    return;
  }

  if (this->control_method_ == ControlMethod::SERIAL_CONTROL) {
    bool gconf_ok = false;

    for (uint8_t attempt = 1; attempt <= 5; attempt++) {
      const int32_t current_gconf = this->read_register(GCONF);
      int32_t desired_gconf = current_gconf;
      desired_gconf &= ~(static_cast<int32_t>(1UL << 4));
      desired_gconf |= static_cast<int32_t>(1UL << 5);
      desired_gconf |= static_cast<int32_t>(1UL << 7);

      if (desired_gconf != current_gconf) {
        ESP_LOGW(TAG,
                 "Repairing SERIAL GCONF before move: 0x%08lX -> 0x%08lX",
                 static_cast<unsigned long>(static_cast<uint32_t>(current_gconf)),
                 static_cast<unsigned long>(static_cast<uint32_t>(desired_gconf)));

        if (!this->write_register(GCONF, desired_gconf)) {
          ESP_LOGW(TAG,
                   "SERIAL GCONF repair attempt %u was not confirmed",
                   attempt);
          continue;
        }
      }

      const int32_t verified_gconf = this->read_register(GCONF);
      const bool index_step_ok =
          (verified_gconf & static_cast<int32_t>(1UL << 5)) != 0;
      const bool index_otpw_ok =
          (verified_gconf & static_cast<int32_t>(1UL << 4)) == 0;
      const bool mstep_reg_ok =
          (verified_gconf & static_cast<int32_t>(1UL << 7)) != 0;

      if (index_step_ok && index_otpw_ok && mstep_reg_ok) {
        gconf_ok = true;
        ESP_LOGI(TAG, "SERIAL GCONF ready for move: 0x%08lX",
                 static_cast<unsigned long>(static_cast<uint32_t>(verified_gconf)));
        break;
      }
    }

    if (!gconf_ok) {
      ESP_LOGE(TAG,
               "MOVE REJECTED - SERIAL INDEX CONFIGURATION COULD NOT BE VERIFIED");
      this->write_field(VACTUAL_FIELD, 0);
      this->vactual_ = 0;
      return;
    }
  }

  if (!this->is_enabled_) {
    this->enable(true);
  }

  this->target_initialized_ = true;
  Stepper::set_target(steps);
}

void TMC2209Stepper::stop() {
  Stepper::stop();

  // A failed/absent driver cannot be stopped over UART. Keep software state
  // stopped and return without touching the bus.
  if (this->is_failed()) {
    this->vactual_ = 0;
    this->current_direction = Direction::STANDSTILL;
    return;
  }

  if (this->control_method_ == ControlMethod::SERIAL_CONTROL) {
    const bool confirmed = this->write_field(VACTUAL_FIELD, 0);
    if (confirmed) {
      this->vactual_ = 0;
    } else {
      this->vactual_ = 1;
      ESP_LOGE(TAG, "FAILED TO CONFIRM VACTUAL=0 STOP - STOP WILL RETRY");
    }
  }
}

void TMC2209Stepper::enable(bool enable) {
  // Do not let output/config actions revive UART traffic after startup failure.
  if (this->is_failed()) {
    this->is_enabled_ = false;
    this->current_direction = Direction::STANDSTILL;
    return;
  }

  if (!enable) {
    this->stop();
  }
  TMC2209Component::enable(enable);
}

bool TMC2209Stepper::is_stalled() {
  if (this->is_failed()) {
    return false;
  }

  if (this->current_direction == Direction::STANDSTILL) {
    return false;
  }

  const int32_t sgthrs = this->read_register(SGTHRS);
  const int32_t sgresult = this->read_register(SG_RESULT);
  return (sgthrs << 1) > sgresult;
}

}  // namespace tmc2209
}  // namespace esphome
