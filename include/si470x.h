/**
 * @file
 *
 * Library for controlling a Silicon Labs Si470X radio tuner.
 *
 * These currently include:
 *
 *   - Si4700
 *   - Si4701
 *   - Si4702
 *   - Si4703
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
#include <si470x_port.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
 * The Si470X chip type.
 */
enum si470x_device_t {
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
enum si470x_region_t {
  REGION_US,         ///< United States.
  REGION_EUROPE,     ///< Europe.
  REGION_AUSTRALIA,  ///< Australia.
  REGION_JAPAN       ///< Japan.
};

/**
 * Configuration parameter when creating Si470X device connection.
 */
struct si470x_config_t {
  struct si470x_port_t* port;   ///< The platform porting layer.
  enum si470x_region_t region;  ///< The broadcast region.
  bool advanced_ps_decoding;    ///< Algorithm when decoding PS text.
  /**
   * The pin connected to the Si470X's GPIO2 interrupt pin.
   *
   * Set to -1 (GPIO_NUM_NC on Mongoose OS) not use interrupts.
   */
  gpio_pin_t gpio2_int_pin;
  gpio_pin_t reset_pin;            ///< The pin used to reset the tuner.
  struct si470x_i2c_params_t i2c;  ///< Device I2C settings.
};

/**
 * The state of the Si470X device.
 */
struct si470x_state_t {
  bool enabled;                 ///< True if the device is currently enabled.
  uint16_t manufacturer;        ///< Chip manufacturer ID.
  int firmware;                 ///< Chip firmware version.
  enum si470x_device_t device;  ///< Chip device model.
  char revision;                ///< The chip silicon revision {A, B, C, or D}.
  int frequency;                ///< Currently tuned frequency (in Hz).
  int channel;                  ///< The channel (function of frequency).
  int volume;                   ///< Current volume [0..15].
  bool stereo;                  ///< Received signal is in stereo.
  uint8_t rssi;                 ///< Signal strength (in dB, 0..50+).
};

struct si470x_port_t;

/**
 * Creates an opaque device structure (`struct si470x_t`) controlling the
 * Si470X tuner.
 *
 * Use si470x_power_on to turn on and begin using the tuner. When
 * finished delete the returned value using si470x_delete.
 *
 * @param config  Configuration parameters.
 *
 * Returns opaque handle (NULL if an error occurred).
 */
struct si470x_t* si470x_create(const struct si470x_config_t* config);

/**
 * Register a function to be called with new RDS data has been decoded.
 *
 * @param device              The tuner device.
 *
 * @param rds_changed_cb      Address of the callback function.
 *
 * @param rds_changed_cb_data A pointer to pass as the argument to
 *                            \p rds_changed_cb.
 */
void si470x_set_rds_callback(struct si470x_t* device,
                             RDSChangedFunc rds_changed_cb,
                             void* rds_changed_cb_data);

/**
 * Set the RDS ODA decoding callback functions.
 *
 * @param device    The tuner device.
 *
 * @param decode_cb Address of function responsible for decoding (and storing)
 *                  the RDS ODA block data.
 *
 * @param clear_cb  Address of function responsible for clearing any stored
 *                  decoded RDS ODA data. This is typically called when tuning
 *                  to a new channel.
 *
 * @param cb_data   Data to be passed to the callbacks when invoked.
 */
void si470x_set_oda_callbacks(struct si470x_t* device,
                              DecodeODAFunc decode_cb,
                              ClearODAFunc clear_cb,
                              void* cb_data);

/**
 * Delete the device.
 *
 * Power off the given device and free all memory allocated by that device.
 *
 * @param device The device to delete.
 */
void si470x_delete(struct si470x_t* device);

/**
 * Power on the tuner.
 *
 * @param device The tuner device returned by si470x_create.
 *
 * @return True if successfully powered on, false if not.
 */
bool si470x_power_on(struct si470x_t* device);

/**
 * Power off the tuner.
 *
 * @param device The device to power off.
 *
 * @return True if powered off, false if not.
 */
bool si470x_power_off(struct si470x_t* device);

/**
 * Is the device currently on?
 *
 * @param device The tuner device returned by si470x_create.
 *
 * @return True if on, false if off.
 */
bool si470x_is_on(struct si470x_t* device);

/**
 * Set the tuner frequency.
 *
 * @param device    The tuner device.
 * @param frequency The desired frequency (in Hz).
 *
 * @return True if successfully tuned to the specified frequency.
 */
bool si470x_set_frequency(struct si470x_t* device, int frequency);

/**
 * Seek up to the next frequency.
 *
 * @param device       The tuner device.
 * @param allow_wrap   Wrap to the other end of the band and continue
 *                     seeking if end of band is encountered during seek.
 * @param reached_sfbl Address of boolean value where true will be written if
 *                     the band limit is reached during seek or false if not.
 *
 * @return The found next frequency (in Hz), or -1 upon error.
 */
int si470x_seek_up(struct si470x_t* device,
                   bool allow_wrap,
                   bool* reached_sfbl);

/**
 * Seek down to the next frequency.
 *
 * @param device       The tuner device.
 * @param allow_wrap   Wrap to the other end of the band and continue
 *                     seeking if end of band is encountered during seek.
 * @param reached_sfbl Address of boolean value where true will be written if
 *                     the band limit is reached during seek or false if not.
 *
 * @return The found next frequency (in Hz), or -1 upon error.
 */
int si470x_seek_down(struct si470x_t* device,
                     bool allow_wrap,
                     bool* reached_sfbl);

/**
 * Set the volume.
 *
 * @param device The tuner device.
 * @param volume The desired volume. Allowable ranges are 0..15 inclusive.
 *
 * @returns True if successful, false if not.
 */
bool si470x_set_volume(struct si470x_t* device, int volume);

/**
 * Control the device mute state.
 *
 * @param device       The tuner device.
 * @param mute_enabled True to mute the device, false to un-mute.
 *
 * @returns True if successful, false if not.
 */
bool si470x_set_mute(struct si470x_t* device, bool mute_enabled);

/**
 * Control the device soft mute state.
 *
 * @param device       The tuner device.
 * @param mute_enabled True to mute the device, false to un-mute.
 *
 * @returns True if successful, false if not.
 */
bool si470x_set_soft_mute(struct si470x_t* device, bool mute_enabled);

/**
 * Retreive current state for the tuner device.
 *
 * @param device The device to retrieve the state for.
 * @param state Device state will be written to this structure (if successful).
 *
 * @return True if successful, false if not.
 */
bool si470x_get_state(struct si470x_t* device, struct si470x_state_t* state);

/**
 * Retreive current state for the tuner device.
 *
 * @param device   The device to retrieve the state for.
 * @param rds_data The current RDS data of the device.
 *
 * @return True if successful, false if not.
 */
bool si470x_get_rds_data(struct si470x_t* device, struct rds_data* rds_data);

#if defined(RDS_DEV)

/**
 * Power on the tuner and use test data to simulate received RDS data.
 *
 * This function is only for development purposes. If used, the RDS data
 * from the Si470X will be ignored and the supplied blocks will be processed
 * as if they had been received. One test block will be parsed every
 * `block_delay` msec.
 *
 * @param device          The tuner device returned by si470x_create.
 * @param test_blocks     A pointer to one or more test blocks. The data
 *                        referenced by this pointer must remain valid as long
 *                        as the device is powered on.
 * @param num_test_blocks The number of blocks pointed to by test_blocks.
 * @param block_delay_ms  The delay (in msec.) in between processing blocks.
 *
 * @return True if successfully powered on, false if not.
 */
bool si470x_power_on_test(struct si470x_t* device,
                          const struct rds_blocks* test_blocks,
                          uint16_t num_test_blocks,
                          uint16_t block_delay_ms);

/**
 * Are the test blocks still being emitted?
 */
bool si470x_rds_test_running(const struct si470x_t* device);

#endif  // defined(RDS_DEV)

#ifdef __cplusplus
}
#endif /* __cplusplus */
