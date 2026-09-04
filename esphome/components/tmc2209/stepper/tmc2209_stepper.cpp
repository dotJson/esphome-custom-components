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

  if (this->control_method_ ==
      ControlMethod::PULSES_CONTROL) {

    this->write_field(
        MULTISTEP_FILT_FIELD, false);

    /*
     * Count both STEP pin transitions.
     *
     * The pulse-control implementation toggles STEP for
     * every logical position increment, so DEDGE keeps the
     * TMC2209's physical motion synchronized with the
     * software position counter.
     */
    this->write_field(
        DEDGE_FIELD, true);
  }

  if (this->control_method_ ==
      ControlMethod::SERIAL_CONTROL) {

    /*
     * Configure INDEX for pulse feedback from the driver.
     * See figure 15.1 in TMC2209 datasheet rev 1.09.
     */
    this->write_field(
        DEDGE_FIELD, false);

    this->write_field(
        INDEX_OTPW_FIELD, false);

    this->write_field(
        INDEX_STEP_FIELD, true);

    this->ips_.current_position_ptr =
        &this->current_position;

    this->ips_.direction_ptr =
        &this->current_direction;

    this->index_pin_->attach_interrupt(
        IndexPulseStore::pulse_isr,
        &this->ips_,
        gpio::INTERRUPT_ANY_EDGE);
  }

  this->enable(true);

  ESP_LOGCONFIG(
      TAG,
      "TMC2209 Stepper setup done.");
}

void TMC2209Stepper::on_shutdown() {
  this->stop();
}

void TMC2209Stepper::loop() {
  TMC2209Component::loop();

  const time_t now = micros();

  /*
   * ESPHome Stepper handles acceleration/deceleration and
   * updates current_speed_.
   */
  this->calculate_speed_(now);

  const int32_t to_target =
      this->target_position -
      this->current_position;

  this->current_direction =
      (to_target != 0)
          ? static_cast<Direction>(
                to_target / abs(to_target))
          : Direction::STANDSTILL;

  /*
   * ------------------------------------------------------
   * UART / VACTUAL CONTROL
   * ------------------------------------------------------
   *
   * VACTUAL uses the TMC2209 clock-dependent conversion.
   * Keep speed_to_vactual() ONLY in this control mode.
   */
  if (this->control_method_ ==
      ControlMethod::SERIAL_CONTROL) {

    int32_t requested_vactual = 0;

    if (this->current_direction !=
            Direction::STANDSTILL &&
        this->current_speed_ > 0.0f) {

      requested_vactual =
          this->speed_to_vactual(
              static_cast<int32_t>(
                  this->current_speed_));

      requested_vactual *=
          static_cast<int32_t>(
              this->current_direction);
    }

    if (this->vactual_ !=
        requested_vactual) {

      const bool confirmed =
          this->write_field(
              VACTUAL_FIELD,
              requested_vactual);

      if (confirmed) {
        // Only mirror the hardware state after IFCNT confirmed the write.
        this->vactual_ =
            requested_vactual;
      } else {
        ESP_LOGE(
            TAG,
            "VACTUAL write failed: requested=%d",
            requested_vactual);

        // Most important case: target reached, but the motor-stop write
        // did not make it to the TMC2209. Do NOT cache a false zero.
        // Leaving vactual_ different from requested_vactual causes the
        // next loop iteration to retry the stop command.
        if (requested_vactual == 0) {
          ESP_LOGE(
              TAG,
              "FAILED TO CONFIRM MOTOR STOP - RETRYING VACTUAL=0");

          // If vactual_ is already zero because this firmware has just
          // started or stop() previously failed, force a mismatch so the
          // next loop cannot suppress the retry.
          if (this->vactual_ == 0) {
            this->vactual_ = 1;
          }
        }
      }
    }
  }

  /*
   * ------------------------------------------------------
   * STEP / DIR PULSE CONTROL
   * ------------------------------------------------------
   *
   * IMPORTANT:
   *
   * current_speed_ is already expressed by ESPHome in
   * steps/second.
   *
   * Do NOT pass it through speed_to_vactual().
   *
   * speed_to_vactual() is a conversion for the TMC2209
   * UART VACTUAL register and introduces the chip-clock
   * conversion factor. That conversion does not belong in
   * an externally generated STEP/DIR timing calculation.
   */
  if (this->control_method_ ==
      ControlMethod::PULSES_CONTROL) {

    /*
     * Nothing to generate if stopped, at target, or if the
     * acceleration profile currently requests zero speed.
     */
    if (this->current_direction ==
            Direction::STANDSTILL ||
        this->current_speed_ <= 0.0f) {

      return;
    }

    /*
     * In this implementation DEDGE=true, so each STEP pin
     * transition represents one logical step.
     *
     * Therefore the interval between transitions is simply:
     *
     *   1,000,000 / requested_steps_per_second
     */
    const float pulse_interval_us =
        1000000.0f /
        this->current_speed_;

    const uint32_t dt =
        static_cast<uint32_t>(
            now - this->last_step_);

    if (static_cast<float>(dt) >=
        pulse_interval_us) {

      /*
       * Update DIR before generating the next STEP edge.
       */
      if (this->direction_ !=
          this->current_direction) {

        this->dir_pin_->digital_write(
            this->current_direction ==
            Direction::BACKWARD);

        this->direction_ =
            this->current_direction;
      }

      /*
       * Generate exactly ONE STEP transition.
       *
       * Do not use a catch-up while-loop here. Multiple
       * back-to-back transitions produced the runaway-fast
       * behavior seen during our previous test.
       */
      this->step_pin_->digital_write(
          this->step_state_);

      this->step_state_ =
          !this->step_state_;

      /*
       * DEDGE=true means every transition is counted by
       * the TMC2209, matching one logical position unit.
       */
      this->current_position +=
          static_cast<int32_t>(
              this->current_direction);

      /*
       * Deliberately anchor timing to the actual emitted
       * edge.
       *
       * We are NOT doing catch-up bursts.
       */
      this->last_step_ = now;
    }
  }
}

void TMC2209Stepper::set_target(
    int32_t steps) {

  if (this->control_method_ ==
      ControlMethod::CONTROL_UNSET) {

    ESP_LOGE(
        TAG,
        "Control method not set!");

    return;
  }

  if (!this->is_enabled_) {
    this->enable(true);
  }

  Stepper::set_target(steps);
}

void TMC2209Stepper::stop() {
  Stepper::stop();

  if (this->control_method_ ==
      ControlMethod::SERIAL_CONTROL) {

    const bool confirmed =
        this->write_field(
            VACTUAL_FIELD, 0);

    if (confirmed) {
      this->vactual_ = 0;
    } else {
      // Never cache a false zero. Force loop() to retry VACTUAL=0.
      this->vactual_ = 1;

      ESP_LOGE(
          TAG,
          "FAILED TO CONFIRM VACTUAL=0 STOP - STOP WILL RETRY");
    }
  }
}

void TMC2209Stepper::enable(
    bool enable) {

  if (!enable) {
    this->stop();
  }

  TMC2209Component::enable(enable);
}

bool TMC2209Stepper::is_stalled() {
  if (this->current_direction ==
      Direction::STANDSTILL) {

    return false;
  }

  const int32_t sgthrs =
      this->read_register(SGTHRS);

  const int32_t sgresult =
      this->read_register(SG_RESULT);

  return (sgthrs << 1) >
         sgresult;
}

}  // namespace tmc2209
}  // namespace esphome
