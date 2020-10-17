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
 * @author Christopher Mumford
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

#include "si470x.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
 * Return the Si470X device for the specified I2C bus.
 */
struct si470x_t* mgos_si470x_get_device(int i2c_bus_no);

/**
 * Return the first Si470X tuner device.
 *
 * Equivalent to calling:
 *
 * ```c
 * mgos_si470x_get_device(0);
 * ```
 */
struct si470x_t* mgos_si470x_get_global(void);

/**
 * Creates an opaque device structure (`struct si470x_t`) controlling the
 * Si470X tuner.
 *
 * Use mgos_si470x_power_on to turn on and begin using the tuner. When
 * finished delete the returned value using mgos_si470x_delete.
 *
 * @param config Configuration parameters.
 *
 * Returns opaque handle (NULL if an error occurred).
 */
struct si470x_t* mgos_si470x_create(const struct si470x_config_t* config);

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
void mgos_si470x_set_rds_callback(struct si470x_t* device,
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
void mgos_si470x_set_oda_callbacks(struct si470x_t* device,
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
void mgos_si470x_delete(struct si470x_t* device);

/**
 * Power on the tuner.
 *
 * @param device The tuner device returned by mgos_si470x_create.
 *
 * @return True if successfully powered on, false if not.
 */
bool mgos_si470x_power_on(struct si470x_t* device);

/**
 * Power off the tuner.
 *
 * @param device The device to power off.
 *
 * @return True if powered off, false if not.
 */
bool mgos_si470x_power_off(struct si470x_t* device);

/**
 * Is the device currently on?
 *
 * @param device The tuner device returned by mgos_si470x_create.
 *
 * @return True if on, false if off.
 */
bool mgos_si470x_is_on(struct si470x_t* device);

/**
 * Set the tuner frequency.
 *
 * @param device    The tuner device.
 * @param frequency The desired frequency (in Hz).
 *
 * @return True if successfully tuned to the specified frequency.
 */
bool mgos_si470x_set_frequency(struct si470x_t* device, int frequency);

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
int mgos_si470x_seek_up(struct si470x_t* device,
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
int mgos_si470x_seek_down(struct si470x_t* device,
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
bool mgos_si470x_set_volume(struct si470x_t* device, int volume);

/**
 * Control the device mute state.
 *
 * @param device       The tuner device.
 * @param mute_enabled True to mute the device, false to un-mute.
 *
 * @returns True if successful, false if not.
 */
bool mgos_si470x_set_mute(struct si470x_t* device, bool mute_enabled);

/**
 * Control the device soft mute state.
 *
 * @param device       The tuner device.
 * @param mute_enabled True to mute the device, false to un-mute.
 *
 * @returns True if successful, false if not.
 */
bool mgos_si470x_set_soft_mute(struct si470x_t* device, bool mute_enabled);

/**
 * Retreive current state for the tuner device.
 *
 * @param device The device to retrieve the state for.
 * @param state Device state will be written to this structure (if successful).
 *
 * @return True if successful, false if not.
 */
bool mgos_si470x_get_state(struct si470x_t* device,
                           struct si470x_state_t* state);

/**
 * Retreive current RDS data for the tuner device.
 *
 * @param device   The device to retrieve the RDS data for.
 * @param rds_data The currently decoded RDS data.
 *
 * @return True if successful, false if not.
 */
bool mgos_si470x_get_rds_data(struct si470x_t* device,
                              struct rds_data* rds_data);

#ifdef __cplusplus
}
#endif /* __cplusplus */
