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

/**
 * @file
 *
 * Library for controlling a Silicon Labs Si470X radio tuner.
 */

#include <si470x.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "logging.h"
#include "rds_decoder.h"
#include "si470x_misc.h"

// https://www.silabs.com/documents/public/data-sheets/Si4702-03-C19.pdf

enum SeekDir { SEEK_UP, SEEK_DOWN };

static uint16_t SwapEndian(uint16_t val) {
  return (val >> 8) | (val << 8);
}

#if defined(HAVE_WIRING_PI)

/**
 * This device is a singleton on the Raspberry Pi.
 */
struct si470x_t* g_device;

#endif  // defined(HAVE_wIRING_PI)

static void lock_mutex(struct si470x_t* device) {
#if defined(USE_FREERTOS_SEMAPHORE)
  xSemaphoreTake(device->xSemaphore, portMAX_DELAY);
#elif defined(USE_PTHREADS)
  pthread_mutex_lock(&device->mutex);
#else
  UNUSED(device);
#endif
}

static void unlock_mutex(struct si470x_t* device) {
#if defined(USE_FREERTOS_SEMAPHORE)
  xSemaphoreGive(device->xSemaphore);
#elif defined(USE_PTHREADS)
  pthread_mutex_unlock(&device->mutex);
#else
  UNUSED(device);
#endif
}

/**
 * Read the entire register control set from the associated Si470X into
 * the `device` struct.
 */
static bool read_registers(struct si470x_t* device) {
#if defined(SUPPORT_TEST_DATA)
  if (device->test_blocks)
    return true;
#endif

  uint16_t registers[16];
  if (!port_i2c_read(device->port, &device->i2c, registers,
                     sizeof(registers))) {
    return false;
  }
  int reg = 0xA;
  for (size_t i = 0; i < ARRAY_SIZE(registers); i++) {
    device->shadow_reg[reg] = SwapEndian(registers[i]);
    if (reg == 0x0f)
      reg = 0x0;
    else
      reg++;
  }
  return true;
}

/**
 * Write the control registers (0x02..0x07) to the Si470X device.
 */
static bool update_registers(const struct si470x_t* device) {
#if defined(SUPPORT_TEST_DATA)
  if (device->test_blocks)
    return true;
#endif

  uint16_t registers[6];
  for (size_t i = 0, idx = 0x02; idx <= 0x07; i++, idx++)
    registers[i] = SwapEndian(device->shadow_reg[idx]);

  return port_i2c_write(device->port, &device->i2c, registers,
                        sizeof(registers));
}

static void clear_ephemeral_rds_data(struct si470x_t* device) {
  CLEAR_BITS(device->rds.valid_values,
             RDS_AF | RDS_CLOCK | RDS_EWS | RDS_FBT | RDS_MC | RDS_PIC |
                 RDS_PI_CODE | RDS_PTY | RDS_SLC | RDS_TDC | RDS_TA_CODE |
                 RDS_TP_CODE | RDS_MS | RDS_EON);
}

/**
 * The scanning/tune complete interrupt handler.
 *
 * This interrupt handler is called when scanning or tuning is complete
 * and also when RDS data is available.
 *
 * NOTE: On some platforms this may be called on a different thread.
 */
static void stc_interrupt_handler(void* user_data) {
#if defined(HAVE_WIRING_PI)
  struct si470x_t* device = g_device;
  UNUSED(user_data);
#else
  struct si470x_t* device = (struct si470x_t*)user_data;
#endif

  bool dirty = false;
  lock_mutex(device);

  if (get_stc_interrupts_enabled(device))
    SET_BITS(device->shadow_reg[STATUSRSSI], STC);

  if (!get_supports_rds(device)) {
    goto DONE;
  }

  if (!read_registers(device))
    return;

  // According to AN230 RDSR is only supported on the Si4701.
  if (get_device(device) == DEVICE_4701 &&
      !(device->shadow_reg[STATUSRSSI] & RDSR)) {
    goto DONE;
  }

  clear_ephemeral_rds_data(device);

  const struct rds_blocks blocks = {
      {device->shadow_reg[RDSA], get_block_a_errors(device)},
      {device->shadow_reg[RDSB], get_block_b_errors(device)},
      {device->shadow_reg[RDSC], get_block_c_errors(device)},
      {device->shadow_reg[RDSD], get_block_d_errors(device)}};
  rds_decoder_decode(device->decoder, &blocks);
  dirty = true;

DONE:
  unlock_mutex(device);

  if (dirty && device->rds_changed.func)
    device->rds_changed.func(device->rds_changed.arg);
}

#if defined(SUPPORT_TEST_DATA)
static void* rds_test_data_func(void* arg) {
  struct si470x_t* device = (struct si470x_t*)arg;

  while (device->run_test_thread) {
    port_delay(device->port, device->block_delay_ms);

    lock_mutex(device);
    rds_decoder_decode(device->decoder,
                       &device->test_blocks[device->test_block_idx]);
    unlock_mutex(device);

    if (++device->test_block_idx >= device->num_test_blocks)
      device->run_test_thread = false;

    if (device->rds_changed.func)
      device->rds_changed.func(device->rds_changed.arg);
  }

  return NULL;
}
#endif  // defined(SUPPORT_TEST_DATA)

static bool set_stc_interrupt_handler(struct si470x_t* device) {
  port_set_pin_mode(device->port, device->gpio2_int_pin, PIN_MODE_INPUT);

  if (!port_set_interrupt_handler(device->port, device->gpio2_int_pin,
                                  EDGE_TYPE_FALLING, &stc_interrupt_handler,
                                  device)) {
    return false;
  }
  return true;
}

static bool enable_i2c(struct si470x_t* device) {
#if defined(SUPPORT_TEST_DATA)
  if (device->test_blocks)
    return true;
#endif
  return port_enable_i2c(device->port, &device->i2c);
}

/**
 * Wait for the STC bit to be set.
 */
static bool wait_for_stc_bit_set(struct si470x_t* device) {
  // Checks every 5 msec. This avoids an infinite loop which results in a crash
  // due to the watchdog timer (WDT) if, for some reason, the tuner never sets
  // the STC bit.
  const uint16_t stc_check_interval_ms = 5;
  const size_t num_stc_iters = device->max_seek_tune_ms / stc_check_interval_ms;
  size_t iter;

  if (get_stc_interrupts_enabled(device)) {
    for (iter = 0; iter < num_stc_iters; iter++) {
      if (device->shadow_reg[STATUSRSSI] & STC)
        break;
      port_delay(device->port, stc_check_interval_ms);
    }
  } else {
    for (iter = 0; iter < num_stc_iters; iter++) {
      port_delay(device->port, stc_check_interval_ms);
      if (!read_registers(device))
        return false;
      if (device->shadow_reg[STATUSRSSI] & STC)
        break;
    }
  }
  return true;
}

/**
 * Wait for the STC bit to be cleared.
 */
static bool wait_for_stc_bit_clear(struct si470x_t* device) {
  // Checks every 5 msec. This avoids an infinite loop which results in a crash
  // due to the watchdog timer (WDT) if, for some reason, the tuner never clears
  // the STC bit.
  const uint16_t stc_check_interval_ms = 5;
  const size_t num_stc_iters = device->max_seek_tune_ms / stc_check_interval_ms;

  for (size_t iter = 0; iter < num_stc_iters; iter++) {
    port_delay(device->port, stc_check_interval_ms);
    if (!read_registers(device))
      return false;
    if (!(device->shadow_reg[STATUSRSSI] & STC))
      break;
  }
  return true;
}

/**
 * Seeks out the next available station.
 *
 * These steps are defined in AN23O, rev 0.9, sect. 3.6.3.
 *
 * @return The channel, if it made it, or -1 if failed.
 */
static int seek_device(struct si470x_t* device,
                       enum SeekDir direction,
                       bool allow_wrap,
                       bool* reached_sfbl) {
  *reached_sfbl = false;
  if (!read_registers(device))
    return -1;

  if (allow_wrap)
    CLEAR_BITS(device->shadow_reg[POWERCFG], SKMODE);
  else
    SET_BITS(device->shadow_reg[POWERCFG], SKMODE);
  if (direction == SEEK_DOWN)
    CLEAR_BITS(device->shadow_reg[POWERCFG], SEEKUP);
  else
    SET_BITS(device->shadow_reg[POWERCFG], SEEKUP);

  const int seek_threshold_db = 25;
  CLEAR_BITS(device->shadow_reg[SYSCONFIG2], SEEKTH_MASK);
  SET_BITS(device->shadow_reg[SYSCONFIG2], seek_threshold_db << 8);

  SET_BITS(device->shadow_reg[POWERCFG], SEEK);
  CLEAR_BITS(device->shadow_reg[STATUSRSSI], STC);
  if (!update_registers(device))  // Seeking should now start.
    return -1;

  rds_decoder_reset(device->decoder);

  if (!wait_for_stc_bit_set(device))
    return -1;

  if (!read_registers(device))
    return -1;

  *reached_sfbl = device->shadow_reg[STATUSRSSI] & SFBL;
  CLEAR_BITS(device->shadow_reg[POWERCFG], SEEK);
  if (!update_registers(device))
    return -1;

  if (!wait_for_stc_bit_clear(device))
    return -1;

  return get_frequency(device);
}

/**
 * Do the minimum required steps to power on the device.
 *
 * These are the required steps defined in AN230 rev. 0.9 pg. 12.
 */
static bool min_power_on(struct si470x_t* device) {
  memset(device->shadow_reg, 0, sizeof(device->shadow_reg));
  rds_decoder_reset(device->decoder);

  // Set the XOSCEN bit to power up the crystal.
  device->shadow_reg[TEST1] = XOSCEN | XO_UNKN;
  if (!update_registers(device)) {
    PERROR("Can't update registers");
    return false;
  }

  // Wait for crystal to power up. Minimum 500 ms.
  port_delay(device->port, 550);

  // Disable mute and set enable bit. memset above clears DISABLE bit.
  device->shadow_reg[POWERCFG] = DMUTE | DSMUTE | ENABLE;
  if (!update_registers(device)) {
    return false;
    PERROR("Can't update registers (2)");
  }

  // Wait for device powerup. This is from the Si4703 datasheet.
  port_delay(device->port, 110);
  return true;
}

static void set_channel_spacing(struct si470x_t* device) {
  CLEAR_BITS(device->shadow_reg[SYSCONFIG2], CHAN_SPACE_MASK);
  if (device->region == REGION_EUROPE) {
    SET_BITS(device->shadow_reg[SYSCONFIG2], CHAN_SPACE_100);
  } else {
    // 0x00 is USA, so no need to set any bits.
  }
}

static void set_de_emphasis(struct si470x_t* device) {
  if (device->region != REGION_EUROPE) {
    SET_BITS(device->shadow_reg[SYSCONFIG1], DE);
  } else {
    CLEAR_BITS(device->shadow_reg[SYSCONFIG1], DE);
  }
}

// Sequence defined in AN230 table 4.
bool power_off(struct si470x_t* device) {
#if defined(SUPPORT_TEST_DATA)
  if (device->run_test_thread) {
    lock_mutex(device);
    device->run_test_thread = false;
    unlock_mutex(device);
    pthread_join(device->sdr_test_thread, NULL);
  }
#endif  // defined(SUPPORT_TEST_DATA)
  lock_mutex(device);
  if (!read_registers(device)) {
    unlock_mutex(device);
    return false;
  }
  SET_BITS(device->shadow_reg[TEST1], AHIZEN);
  if (!update_registers(device)) {
    unlock_mutex(device);
    return false;
  }

  if (device->gpio2_int_pin != -1) {
    // Set pin to low to reduce power consumption
    port_digital_write(device->port, device->gpio2_int_pin, TTL_LOW);
  }

  CLEAR_BITS(device->shadow_reg[POWERCFG], DMUTE | ENABLE);
  SET_BITS(device->shadow_reg[POWERCFG], ENABLE | DISABLE);
  bool ok = update_registers(device);
  unlock_mutex(device);
  return ok;
}

/****************************************/
/*vvvvvvvvvv PUBLIC FUNCTIONS *vvvvvvvvv*/
/****************************************/

struct si470x_t* si470x_create(const struct si470x_config_t* config) {
  if (!config || !config->port)
    return NULL;
  struct si470x_t* device =
      (struct si470x_t*)calloc(1, sizeof(struct si470x_t));
  if (!device)
    return NULL;

  device->port = config->port;

#if !defined(MGOS)
  device->reset_pin = config->reset_pin;
#endif
  device->gpio2_int_pin = config->gpio2_int_pin;
  device->region = config->region;
  device->i2c = config->i2c;
  // The maximum seek/tune time in msec. This is specified in the datasheet
  // and may be device specific. 60 msec is for the Si4703.
  device->max_seek_tune_ms = 60;
  const struct rds_decoder_config decoder_config = {
      .advanced_ps_decoding = config->advanced_ps_decoding,
      .rds_data = &device->rds,
  };

#if defined(USE_FREERTOS_SEMAPHORE)
  device->xSemaphore = xSemaphoreCreateMutex();
#endif

  device->decoder = rds_decoder_create(&decoder_config);
  rds_decoder_reset(device->decoder);

#if !defined(MGOS)
  if (!reset_device(device->port, device->reset_pin, device->gpio2_int_pin,
                    device->i2c.sdio_pin)) {
    free(device);
    return NULL;
  }
#endif

  // Now that device is reset
  if (!enable_i2c(device)) {
    free(device);
    return NULL;
  }

#if defined(HAVE_WIRING_PI)
  g_device = device;
#endif  // defined(HAVE_WIRING_PI)

  return device;
}

void si470x_set_rds_callback(struct si470x_t* device,
                             RDSChangedFunc rds_changed_cb,
                             void* rds_changed_cb_data) {
  device->rds_changed.func = rds_changed_cb;
  device->rds_changed.arg = rds_changed_cb_data;
}

void si470x_set_oda_callbacks(struct si470x_t* device,
                              DecodeODAFunc decode_cb,
                              ClearODAFunc clear_cb,
                              void* cb_data) {
  rds_decoder_set_oda_callbacks(device->decoder, decode_cb, clear_cb, cb_data);
}

void si470x_delete(struct si470x_t* device) {
  if (device == NULL)
    return;

  power_off(device);
  rds_decoder_delete(device->decoder);
  free(device);
}

/**
 * Powers on the chip as per instructions in AN230 rev. 0.9, page 12.
 */
bool si470x_power_on(struct si470x_t* device) {
  lock_mutex(device);
  if (!min_power_on(device)) {
    unlock_mutex(device);
    return false;
  }

  if (!read_registers(device)) {
    unlock_mutex(device);
    return false;
  }

  if (get_supports_rds(device))
    SET_BITS(device->shadow_reg[SYSCONFIG1], RDS | RDSM);

  const bool use_tuning_interrupts = false;
  bool rds_interrupts_enabled = false;
  if (device->gpio2_int_pin != -1) {
    rds_interrupts_enabled = get_supports_rds_int(device)
#if defined(SUPPORT_TEST_DATA)
                             && !device->test_blocks
#endif
        ;
    // See AN230 sect. 3.2.5 for interrupt support.
    if (rds_interrupts_enabled)
      SET_BITS(device->shadow_reg[SYSCONFIG1], RDSIEN);

    if (use_tuning_interrupts)
      SET_BITS(device->shadow_reg[SYSCONFIG1], STCIEN);
    else
      CLEAR_BITS(device->shadow_reg[SYSCONFIG1], STCIEN);

    CLEAR_BITS(device->shadow_reg[SYSCONFIG1], GPIO2_MASK);
    if (get_rds_interrupts_enabled(device) || use_tuning_interrupts)
      SET_BITS(device->shadow_reg[SYSCONFIG1], RDS_STC_RDS_INT);
  }

  set_channel_spacing(device);
  set_de_emphasis(device);

  CLEAR_BITS(device->shadow_reg[SYSCONFIG2], VOLUME_MASK);
  SET_BITS(device->shadow_reg[SYSCONFIG2], 0x1);  // Set volume to min.

  if (!update_registers(device)) {
    unlock_mutex(device);
    PERROR("Can't update registers");
    return false;
  }

  if (!read_registers(device)) {
    unlock_mutex(device);
    PERROR("Can't read registers");
    return false;
  }

  port_delay(device->port,
             110);  // Max powerup time (ms), from datasheet page 13.

  // Once device is fully on, enable interrupt handler.
  if ((get_rds_interrupts_enabled(device) || use_tuning_interrupts) &&
      !set_stc_interrupt_handler(device)) {
    unlock_mutex(device);
    return false;
  }

  unlock_mutex(device);
  return true;
}

bool si470x_power_on_test(struct si470x_t* device,
                          const struct rds_blocks* test_blocks,
                          uint16_t num_test_blocks,
                          uint16_t block_delay_ms) {
#if defined(SUPPORT_TEST_DATA)
  device->test_blocks = test_blocks;
  device->num_test_blocks = num_test_blocks;
  device->test_block_idx = 0;
  device->run_test_thread = true;
  device->block_delay_ms = block_delay_ms;

  bool is_on = si470x_power_on(device);
  if (!is_on)
    return false;

  if (pthread_create(&device->sdr_test_thread,
                     /*attr=*/NULL, rds_test_data_func, device)) {
    PERROR("Can't create thread");
    return false;
  }

  return true;
#else
  UNUSED(device);
  UNUSED(test_blocks);
  UNUSED(num_test_blocks);
  UNUSED(block_delay_ms);
  return false;
#endif  // defined(SUPPORT_TEST_DATA)
}

// Sequence defined in AN230 table 4.
bool si470x_power_off(struct si470x_t* device) {
  return power_off(device);
}

bool si470x_is_on(struct si470x_t* device) {
  return device->shadow_reg[POWERCFG] & ENABLE;
}

bool si470x_set_frequency(struct si470x_t* device, int frequency) {
  const uint16_t channel = frequency_to_channel(frequency, device);

  lock_mutex(device);

  // These steps defined in AN230 rev 0.9 sect 3.7.
  if (!read_registers(device)) {
    unlock_mutex(device);
    return false;
  }
  CLEAR_BITS(device->shadow_reg[CHANNEL], CHANNEL_MASK);
  SET_BITS(device->shadow_reg[CHANNEL], TUNE | channel);
  CLEAR_BITS(device->shadow_reg[STATUSRSSI], STC);
  if (!update_registers(device)) {
    unlock_mutex(device);
    return false;
  }
  rds_decoder_reset(device->decoder);

  if (!wait_for_stc_bit_set(device)) {
    unlock_mutex(device);
    return false;
  }

  // Clear the TUNE bit to end the tuning operation.
  CLEAR_BITS(device->shadow_reg[CHANNEL], TUNE);
  if (!update_registers(device)) {
    unlock_mutex(device);
    return false;
  }

  if (!wait_for_stc_bit_clear(device)) {
    unlock_mutex(device);
    return false;
  }

  unlock_mutex(device);

  return true;
}

int si470x_seek_up(struct si470x_t* device,
                   bool allow_wrap,
                   bool* reached_sfbl) {
  return seek_device(device, SEEK_UP, allow_wrap, reached_sfbl);
}

int si470x_seek_down(struct si470x_t* device,
                     bool allow_wrap,
                     bool* reached_sfbl) {
  return seek_device(device, SEEK_DOWN, allow_wrap, reached_sfbl);
}

bool si470x_set_volume(struct si470x_t* device, int volume) {
  lock_mutex(device);
  if (!read_registers(device)) {
    unlock_mutex(device);
    return false;
  }

  // Clamp volume to valid values.
  if (volume < 0)
    volume = 0;
  else if (volume > 15)
    volume = 15;

  CLEAR_BITS(device->shadow_reg[SYSCONFIG2], VOLUME_MASK);
  SET_BITS(device->shadow_reg[SYSCONFIG2], volume);

  bool ok = update_registers(device);
  unlock_mutex(device);

  return ok;
}

bool si470x_set_mute(struct si470x_t* device, bool mute_enabled) {
  lock_mutex(device);
  if (mute_enabled)
    CLEAR_BITS(device->shadow_reg[POWERCFG], DMUTE);
  else
    SET_BITS(device->shadow_reg[POWERCFG], DMUTE);
  bool ok = update_registers(device);
  unlock_mutex(device);
  return ok;
}

bool si470x_set_soft_mute(struct si470x_t* device, bool mute_enabled) {
  lock_mutex(device);
  if (mute_enabled)
    CLEAR_BITS(device->shadow_reg[POWERCFG], DSMUTE);
  else
    SET_BITS(device->shadow_reg[POWERCFG], DSMUTE);
  bool ok = update_registers(device);
  unlock_mutex(device);
  return ok;
}

bool si470x_get_state(struct si470x_t* device, struct si470x_state_t* state) {
  lock_mutex(device);

  if (!read_registers(device)) {
    unlock_mutex(device);
    return false;
  }

  state->enabled = device->shadow_reg[POWERCFG] & ENABLE;
  state->manufacturer = get_manufacturer(device);
  state->firmware = device->shadow_reg[CHIPID] & FIRMWARE_MASK;
  state->device = get_device(device);
  switch (get_revision(device)) {
    case 1:
      state->revision = 'A';
      break;
    case 2:
      state->revision = 'B';
      break;
    case 3:
      state->revision = 'C';
      break;
    case 4:
      state->revision = 'D';
      break;
    default:
      state->revision = '?';
  }

  state->frequency = get_frequency(device);
  state->channel = get_channel(device);
  state->volume = device->shadow_reg[SYSCONFIG2] & VOLUME_MASK;
  state->stereo = device->shadow_reg[STATUSRSSI] & STEREO;
  state->rssi = get_signal_strength(device);

  unlock_mutex(device);

  return true;
}

bool si470x_get_rds_data(struct si470x_t* device, struct rds_data* rds_data) {
  lock_mutex(device);
  *rds_data = device->rds;
  unlock_mutex(device);
  return true;
}

#if defined(SUPPORT_TEST_DATA)

bool si470x_rds_test_running(const struct si470x_t* device) {
  return device->run_test_thread;
}

#endif  // defined(SUPPORT_TEST_DATA)
