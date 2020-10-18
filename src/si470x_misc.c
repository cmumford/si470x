/*
 * Copyright 2020 Christopher Mumford
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "si470x_misc.h"

#include <stdint.h>

/**
 * Reset the Si470X device.
 *
 * Initialize the Si470X as per AN230 section 2.1.1.
 *
 * I2C must be disabled before calling this function.
 */
bool reset_device(struct si470x_port_t* port,
                  const gpio_pin_t si470x_rst_pin,
                  const gpio_pin_t si470x_gpio2_int_pin,
                  const gpio_pin_t i2c_sda_pin) {
  if (port_i2c_enabled(port))
    return false;
  if (!port_enable_gpio(port))
    return false;
  port_set_pin_mode(port, si470x_rst_pin, PIN_MODE_OUTPUT);
  port_set_pin_mode(port, i2c_sda_pin, PIN_MODE_OUTPUT);
  if (si470x_gpio2_int_pin != -1)
    port_set_pin_mode(port, si470x_gpio2_int_pin, PIN_MODE_OUTPUT);

  // Low SDIO = 2-wire interface.
  port_digital_write(port, i2c_sda_pin, TTL_LOW);
  if (si470x_gpio2_int_pin != -1) {
    // goes low on interrupt.
    port_digital_write(port, si470x_gpio2_int_pin, TTL_HIGH);
  }
  // Put Si470X into reset.
  port_digital_write(port, si470x_rst_pin, TTL_LOW);

  port_delay(port, 1);  // Allow pin to settle.

  // Bring Si470X out of reset with SDIO set to low and SEN pulled high
  // (if used) with on-board resistor.
  port_digital_write(port, si470x_rst_pin, TTL_HIGH);

  port_delay(port, 1);  // Allow Si470X to come out of reset.
  return true;
}

static uint32_t get_channel_space_hz(const struct si470x_t* device) {
  // See AN230 sect. 3.4.2
  switch ((device->shadow_reg[SYSCONFIG2] & CHAN_SPACE_MASK) >> 4) {
    case 0x0:
      return 200000;
    case 0x1:
      return 100000;
    case 0x2:
      return 50000;
    case 0x3:  // reserved.
    default:
      return 200000;  // Default to US.
  }
}

static uint32_t get_min_band_freq_hz(const struct si470x_t* device) {
  // See AN230 sect. 3.4.1
  switch ((device->shadow_reg[SYSCONFIG2] & BAND_MASK) >> 6) {
    case 0x0:
      return 87500000;
    case 0x1:
      // FALLTHROUGH
    case 0x2:
      return 76000000;
    case 0x3:  // reserved.
    default:
      return 87500000;  // Default to US.
  }
}

/**
 * Convert channel to frequency (in Hz).
 */
static uint32_t channel_to_frequency(uint16_t channel,
                                     const struct si470x_t* device) {
  return get_min_band_freq_hz(device) +
         (uint32_t)channel * get_channel_space_hz(device);
}

enum si470x_device_t get_device(const struct si470x_t* device) {
  // See AN230 Table 3 for value->device mappings.
  switch ((device->shadow_reg[CHIPID] & DEVICE_MASK) >> 6) {
    case 0b0000:
      return DEVICE_4700;
    case 0b0001:
      return DEVICE_4702;
    case 0b1000:
      return DEVICE_4701;
    case 0b1001:
      return DEVICE_4703;
  }
  return DEVICE_UNKNOWN;
}

bool get_stc_interrupts_enabled(const struct si470x_t* device) {
  return device->shadow_reg[SYSCONFIG1] & STCIEN;
}

bool get_supports_rds(const struct si470x_t* device) {
  const enum si470x_device_t dev = get_device(device);
  return dev == DEVICE_4701 || dev == DEVICE_4703;
}

bool get_supports_rds_int(const struct si470x_t* device) {
  return get_device(device) == DEVICE_4703;
}

bool get_rds_interrupts_enabled(const struct si470x_t* device) {
  return device->shadow_reg[SYSCONFIG1] & RDSIEN;
}

int get_signal_strength(const struct si470x_t* device) {
  return device->shadow_reg[STATUSRSSI] & RSSI_MASK;
}

uint16_t get_block_a_errors(const struct si470x_t* device) {
  return (device->shadow_reg[STATUSRSSI] & BLERA_MASK) >> 9;
}

uint16_t get_block_b_errors(const struct si470x_t* device) {
  return (device->shadow_reg[READCHAN] & BLERB_MASK) >> 14;
}

uint16_t get_block_c_errors(const struct si470x_t* device) {
  return (device->shadow_reg[READCHAN] & BLERC_MASK) >> 12;
}

uint16_t get_block_d_errors(const struct si470x_t* device) {
  return (device->shadow_reg[READCHAN] & BLERD_MASK) >> 10;
}

int get_channel(const struct si470x_t* device) {
  return device->shadow_reg[READCHAN] & CHANNEL_MASK;
}

uint32_t get_frequency(const struct si470x_t* device) {
  return channel_to_frequency(get_channel(device), device);
}

uint16_t get_manufacturer(const struct si470x_t* device) {
  return device->shadow_reg[DEVICEID] & MANUFACTURER_MASK;
}

uint16_t get_part(const struct si470x_t* device) {
  return (device->shadow_reg[DEVICEID] & PART_MASK) >> 12;
}

uint16_t get_firmware(const struct si470x_t* device) {
  return device->shadow_reg[CHIPID] & FIRMWARE_MASK;
}

uint16_t get_revision(const struct si470x_t* device) {
  return (device->shadow_reg[CHIPID] & REVISION_MASK) >> 10;
}

uint16_t frequency_to_channel(int frequency_hz, const struct si470x_t* device) {
  if (frequency_hz == 0)
    return 0;
  return (frequency_hz - get_min_band_freq_hz(device)) /
         get_channel_space_hz(device);
}
