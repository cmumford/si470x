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

static uint32_t get_channel_space_hz(const struct si470x* device) {
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

static uint32_t get_min_band_freq_hz(const struct si470x* device) {
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
                                     const struct si470x* device) {
  return get_min_band_freq_hz(device) +
         (uint32_t)channel * get_channel_space_hz(device);
}

enum si470x_device get_device(const struct si470x* device) {
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

bool get_stc_interrupts_enabled(const struct si470x* device) {
  return device->shadow_reg[SYSCONFIG1] & STCIEN;
}

bool get_supports_rds(const struct si470x* device) {
  const enum si470x_device dev = get_device(device);
  return dev == DEVICE_4701 || dev == DEVICE_4703;
}

bool get_supports_rds_int(const struct si470x* device) {
  return get_device(device) == DEVICE_4703;
}

bool get_rds_interrupts_enabled(const struct si470x* device) {
  return device->shadow_reg[SYSCONFIG1] & RDSIEN;
}

int get_signal_strength(const struct si470x* device) {
  return device->shadow_reg[STATUSRSSI] & RSSI_MASK;
}

uint16_t get_block_a_errors(const struct si470x* device) {
  return (device->shadow_reg[STATUSRSSI] & BLERA_MASK) >> 9;
}

uint16_t get_block_b_errors(const struct si470x* device) {
  return (device->shadow_reg[READCHAN] & BLERB_MASK) >> 14;
}

uint16_t get_block_c_errors(const struct si470x* device) {
  return (device->shadow_reg[READCHAN] & BLERC_MASK) >> 12;
}

uint16_t get_block_d_errors(const struct si470x* device) {
  return (device->shadow_reg[READCHAN] & BLERD_MASK) >> 10;
}

int get_channel(const struct si470x* device) {
  return device->shadow_reg[READCHAN] & CHANNEL_MASK;
}

uint32_t get_frequency(const struct si470x* device) {
  return channel_to_frequency(get_channel(device), device);
}

uint16_t get_manufacturer(const struct si470x* device) {
  return device->shadow_reg[DEVICEID] & MANUFACTURER_MASK;
}

uint16_t get_part(const struct si470x* device) {
  return (device->shadow_reg[DEVICEID] & PART_MASK) >> 12;
}

uint16_t get_firmware(const struct si470x* device) {
  return device->shadow_reg[CHIPID] & FIRMWARE_MASK;
}

uint16_t get_revision(const struct si470x* device) {
  return (device->shadow_reg[CHIPID] & REVISION_MASK) >> 10;
}

uint16_t frequency_to_channel(int frequency_hz, const struct si470x* device) {
  if (frequency_hz == 0)
    return 0;
  return (frequency_hz - get_min_band_freq_hz(device)) /
         get_channel_space_hz(device);
}