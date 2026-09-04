#include "tmc2209_api_registers.h"
#include "tmc2209_api.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace tmc2209 {

uint8_t TMC2209API::crc8_(uint8_t *data, uint32_t bytes) {
  uint8_t result = 0;
  uint8_t *table;

  while (bytes--)
    result = tmc_crc_table_poly7_reflected[result ^ *data++];

  // Flip the result around
  // swap odd and even bits
  result = ((result >> 1) & 0x55) | ((result & 0x55) << 1);
  // swap consecutive pairs
  result = ((result >> 2) & 0x33) | ((result & 0x33) << 2);
  // swap nibbles ...
  result = ((result >> 4) & 0x0F) | ((result & 0x0F) << 4);

  return result;
}

void TMC2209API::set_dirty_bit_(uint8_t index, bool value) {
  if (index >= REGISTER_COUNT)
    return;

  uint8_t *tmp = &this->dirty_bits_[index / 8];
  uint8_t shift = (index % 8);
  uint8_t mask = 1 << shift;
  *tmp = (((*tmp) & (~(mask))) | (((value) << (shift)) & (mask)));
}

bool TMC2209API::get_dirty_bit_(uint8_t index) {
  if (index >= REGISTER_COUNT)
    return false;

  uint8_t *tmp = &this->dirty_bits_[index / 8];
  uint8_t shift = (index % 8);
  return ((*tmp) >> shift) & 1;
}

bool TMC2209API::cache_(CacheOperation operation, uint8_t address, uint32_t *value) {
  if (operation == CACHE_READ) {
    if (IS_READABLE(register_access_[address]))
      return false;

    // Grab the value from the cache
    *value = this->shadow_register_[address];
    return true;
  } else if (operation == CACHE_WRITE || operation == CACHE_FILL_DEFAULT) {
    // Fill the cache

    // Write to the shadow register.
    this->shadow_register_[address] = *value;
    // For write operations, mark the register dirty
    if (operation == CACHE_WRITE) {
      this->set_dirty_bit_(address, true);
    }

    return true;
  }
  return false;
}

bool TMC2209API::read_register_verified_(uint8_t address, int32_t *result) {
  ESP_LOGVV(TAG, "reading address 0x%x", address);

  uint32_t cached_value;

  // Read from cache for registers with write-only access.
  if (this->cache_(CACHE_READ, address, &cached_value)) {
    *result = static_cast<int32_t>(cached_value);
    return true;
  }

  address = address & ADDRESS_MASK;

  for (uint8_t attempt = 0; attempt < 3; attempt++) {
    std::array<uint8_t, 8> buffer = {0};

    buffer.at(0) = 0x05;
    buffer.at(1) = this->address_;
    buffer.at(2) = address;
    buffer.at(3) = this->crc8_(buffer.data(), 3);

    // Clear stale RX data before starting a new transaction.
    while (this->parent_->available()) {
      uint8_t discard;
      this->parent_->read_byte(&discard);
    }

    // Send the 4-byte read request.
    this->parent_->write_array(buffer.data(), 4);

    // One-wire UART echoes our transmitted request into RX.
    if (!this->parent_->read_array(buffer.data(), 4)) {
      ESP_LOGW(TAG, "Read 0x%02X attempt %u: missing TX echo",
               address, attempt + 1);
      delay(2);
      continue;
    }

    this->parent_->flush();

    // Read the 8-byte TMC2209 response.
    if (!this->parent_->read_array(buffer.data(), 8)) {
      ESP_LOGW(TAG, "Read 0x%02X attempt %u: response timeout",
               address, attempt + 1);
      delay(2);
      continue;
    }

    if (buffer.at(0) != 0x05) {
      ESP_LOGW(TAG, "Read 0x%02X attempt %u: bad sync 0x%02X",
               address, attempt + 1, buffer.at(0));
      delay(2);
      continue;
    }

    if (buffer.at(1) != 0xFF) {
      ESP_LOGW(TAG, "Read 0x%02X attempt %u: bad master 0x%02X",
               address, attempt + 1, buffer.at(1));
      delay(2);
      continue;
    }

    if (buffer.at(2) != address) {
      ESP_LOGW(TAG, "Read 0x%02X attempt %u: wrong register 0x%02X",
               address, attempt + 1, buffer.at(2));
      delay(2);
      continue;
    }

    if (buffer.at(7) != this->crc8_(buffer.data(), 7)) {
      ESP_LOGW(TAG, "Read 0x%02X attempt %u: CRC error",
               address, attempt + 1);
      delay(2);
      continue;
    }

    *result = static_cast<int32_t>(encode_uint32(
        buffer.at(3),
        buffer.at(4),
        buffer.at(5),
        buffer.at(6)));

    return true;
  }

  ESP_LOGE(TAG, "Read 0x%02X failed after 3 attempts", address);
  return false;
}

bool TMC2209API::write_register(uint8_t address, int32_t value) {
  ESP_LOGVV(TAG, "writing address 0x%x with value 0x%x (%d)",
            address, value, value);

  address = address & ADDRESS_MASK;

  for (uint8_t attempt = 0; attempt < 3; attempt++) {
    // IFCNT increments once for every successful UART write.
    // Both IFCNT reads must themselves be valid before a write can
    // be considered confirmed.
    int32_t ifcnt_before_raw = 0;

    if (!this->read_register_verified_(IFCNT, &ifcnt_before_raw)) {
      ESP_LOGW(TAG,
               "Write 0x%02X attempt %u: unable to read IFCNT before write",
               address, attempt + 1);
      delay(2);
      continue;
    }

    const uint8_t ifcnt_before =
        static_cast<uint8_t>(ifcnt_before_raw & 0xFF);

    std::array<uint8_t, 8> buffer = {0};

    buffer.at(0) = 0x05;
    buffer.at(1) = this->address_;
    buffer.at(2) = address | WRITE_BIT;
    buffer.at(3) = (value >> 24) & 0xFF;
    buffer.at(4) = (value >> 16) & 0xFF;
    buffer.at(5) = (value >> 8) & 0xFF;
    buffer.at(6) = value & 0xFF;
    buffer.at(7) = this->crc8_(buffer.data(), 7);

    // Remove anything stale before transmitting.
    while (this->parent_->available()) {
      uint8_t discard;
      this->parent_->read_byte(&discard);
    }

    // Send exactly one write transaction.
    this->parent_->write_array(buffer.data(), 8);

    // At 115200 baud the 8-byte echo takes less than 1 ms.
    // Allow it to arrive, then discard whatever was echoed.
    delay(1);

    while (this->parent_->available()) {
      uint8_t discard;
      this->parent_->read_byte(&discard);
    }

    this->parent_->flush();

    int32_t ifcnt_after_raw = 0;

    if (!this->read_register_verified_(IFCNT, &ifcnt_after_raw)) {
      ESP_LOGW(TAG,
               "Write 0x%02X attempt %u: unable to read IFCNT after write",
               address, attempt + 1);
      delay(2);
      continue;
    }

    const uint8_t ifcnt_after =
        static_cast<uint8_t>(ifcnt_after_raw & 0xFF);

    if (ifcnt_after == static_cast<uint8_t>(ifcnt_before + 1)) {
      // Only update our shadow copy after the chip confirmed the write.
      uint32_t cached_value = static_cast<uint32_t>(value);
      this->cache_(CACHE_WRITE, address, &cached_value);

      ESP_LOGD(TAG,
               "Write 0x%02X confirmed by IFCNT (%u -> %u)",
               address, ifcnt_before, ifcnt_after);
      return true;
    }

    ESP_LOGW(TAG,
             "Write 0x%02X attempt %u not confirmed "
             "(IFCNT %u -> %u)",
             address,
             attempt + 1,
             ifcnt_before,
             ifcnt_after);

    delay(2);
  }

  ESP_LOGE(TAG, "Write 0x%02X failed after 3 attempts", address);
  return false;
}

int32_t TMC2209API::read_register(uint8_t address) {
  int32_t value = 0;

  if (!this->read_register_verified_(address, &value)) {
    return 0;
  }

  return value;
}

uint32_t TMC2209API::update_field(uint32_t data, RegisterField field, uint32_t value) {
  return (data & (~field.mask)) | ((value << field.shift) & field.mask);
}

bool TMC2209API::write_field(RegisterField field, uint32_t value) {
  int32_t raw_register = 0;

  // Preserve read failure information. Never build a write from a bogus
  // zero value when a readable register could not be retrieved.
  if (!this->read_register_verified_(field.address, &raw_register)) {
    ESP_LOGE(TAG,
             "Write field 0x%02X failed: unable to obtain register value",
             field.address);
    return false;
  }

  uint32_t reg_value = static_cast<uint32_t>(raw_register);
  reg_value = this->update_field(reg_value, field, value);

  return this->write_register(field.address, static_cast<int32_t>(reg_value));
}

uint32_t TMC2209API::extract_field(uint32_t data, RegisterField field) {
  uint32_t value = (data & field.mask) >> field.shift;

  if (field.is_signed) {
    uint32_t base_mask = field.mask >> field.shift;
    uint32_t sign_mask = base_mask & (~base_mask >> 1);
    value = (value ^ sign_mask) - sign_mask;
  }

  return value;
}

uint32_t TMC2209API::read_field(RegisterField field) {
  uint32_t value = this->read_register(field.address);
  return this->extract_field(value, field);
}

}  // namespace tmc2209
}  // namespace esphome
