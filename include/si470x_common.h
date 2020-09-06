/**
 * @file
 *
 * @author Chris Mumford
 *
 * @license
 *
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

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include <rds_decoder.h>

#ifdef __cplusplus
extern "C" {
#else

#endif /* __cplusplus */

#if !defined(ARRAY_SIZE)
#define ARRAY_SIZE(ARRAY) (sizeof(ARRAY) / sizeof((ARRAY)[0]))
#endif

/**
 * The Si470X chip type.
 */
enum si470x_device {
  DEVICE_4700 = 0,     ///< Device is a Si4700 (or not yet enabled).
  DEVICE_4701 = 1,     ///< Device is a Si4701.
  DEVICE_4702 = 2,     ///< Device is a Si4702.
  DEVICE_4703 = 3,     ///< Device is a Si4703.
  DEVICE_UNKNOWN = 4,  ///< Unknown device, or error retrieving value.
};

/**
 * Definition of a function called, possibly on a different thread,
 * when the RDS data has changed.
 */
typedef void (*RDSChangedFunc)(void*);

/**
 * The broadcast region.
 */
enum si470x_region {
  REGION_US,         ///< United States.
  REGION_EUROPE,     ///< Europe.
  REGION_AUSTRALIA,  ///< Australia.
  REGION_JAPAN       ///< Japan.
};

/**
 * Configuration parameter when creating Si470X device connection.
 */
struct si470x_config {
  enum si470x_region region;  ///< The broadcast region.
  bool advanced_ps_decoding;  ///< Algorithm when decoding PS text.
  /**
   * The pin connected to the Si470X's GPIO2 interrupt pin.
   *
   * Set to -1 not use interrupts.
   */
  int gpio2_int_pin;

#if defined(MGOS)
  int i2c_bus_no;  ///< The I2C bus on which the Si470X is connected.
#else
  // These pin numbers are not GPIO - they are wiringPi values.
  // See http://wiringpi.com/pins/

  int reset_pin;  ///< The pin used to reset the tuner.
  int sdio_pin;   ///< The I2C SDA pin.
  int sclk_pin;   ///< The I2C SCK pin.
#endif  // !defined(MGOS)
};

/**
 * The state of the Si470X device.
 */
struct si470x_state {
  bool enabled;               ///< True if the device is currently enabled.
  uint16_t manufacturer;      ///< Chip manufacturer ID.
  int firmware;               ///< Chip firmware version.
  enum si470x_device device;  ///< Chip device model.
  char revision;              ///< The chip silicon revision {A, B, C, or D}.
  int frequency;              ///< Currently tuned frequency (in Hz).
  int channel;                ///< The channel (function of frequency).
  int volume;                 ///< Current volume [0..15].
  bool stereo;                ///< Received signal is in stereo.
  uint8_t rssi;               ///< Signal strength (in dB, 0..50+).
};

/**
 * The opaque structure representing a connection to a Si470X device.
 */
struct si470x;

#ifdef __cplusplus
}
#endif /* __cplusplus */
