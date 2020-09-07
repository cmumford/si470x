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
 *
 * https://www.silabs.com/documents/public/data-sheets/Si4702-03-C19.pdf
 */

#include <mgos_si470x.h>

#include <stdint.h>
#include <stdlib.h>

#include <mgos.h>
#include <mgos_i2c.h>

#include "mgos_rds_decoder.h"
#include "si470x_misc.h"

#ifndef MGOS_SYS_CONFIG_HAVE_I2C1
#define NUM_BUSES 1
#else
#ifndef MGOS_SYS_CONFIG_HAVE_I2C2
#define NUM_BUSES 2
#else
#define NUM_BUSES 3
#endif
#endif

enum SeekDir { SEEK_UP, SEEK_DOWN };

static bool g_library_initialized = false;
static struct si470x* s_devices[NUM_BUSES];

static uint16_t SwapEndian(uint16_t val) {
  return (val >> 8) | (val << 8);
}

/**
 * Read the entire register control set from the associated Si470X into
 * the `device` struct.
 */
static bool read_registers(struct si470x* device) {
  uint16_t registers[16];
  mgos_msleep(200);
  if (!mgos_i2c_read(device->i2c, device->i2caddr, registers, sizeof(registers),
                     /*stop=*/true)) {
    LOG(LL_ERROR, ("Can't read device registers"));
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
static bool update_registers(const struct si470x* device) {
  uint16_t registers[6];
  for (size_t i = 0, idx = 0x02; idx <= 0x07; i++, idx++)
    registers[i] = SwapEndian(device->shadow_reg[idx]);

  mgos_msleep(200);
  return mgos_i2c_write(device->i2c, device->i2caddr, registers,
                        sizeof(registers), /*stop=*/true);
}

/**
 * The scanning/tune complete interrupt handler.
 *
 * Also used for RDS interrupt handling.
 */
static void stc_interrupt_handler(int pin, void* arg) {
  UNUSED(pin);
  struct si470x* device = (struct si470x*)arg;

  if (get_stc_interrupts_enabled(device))
    SET_BITS(device->shadow_reg[STATUSRSSI], STC);

  if (!get_supports_rds(device))
    return;

  // According to AN230 RDSR is only supported on the Si4701.
  if (get_device(device) == DEVICE_4701 &&
      !(device->shadow_reg[STATUSRSSI] & RDSR)) {
    return;
  }

  const struct rds_blocks blocks = {
      {device->shadow_reg[RDSA], get_block_a_errors(device)},
      {device->shadow_reg[RDSB], get_block_b_errors(device)},
      {device->shadow_reg[RDSC], get_block_c_errors(device)},
      {device->shadow_reg[RDSD], get_block_d_errors(device)}};
  mgos_rds_decoder_decode(device->decoder, &blocks);

  if (!device->rds_changed.func)
    return;
  if (!mgos_invoke_cb(device->rds_changed.func, device->rds_changed.arg,
                      /*from_isr=*/false)) {
    LOG(LL_ERROR, ("Error invoking RDS callback."));
  }
}

/**
 * Reset the Si470X device.
 *
 * Initialize the Si470X as per AN230 section 2.1.1.
 *
 * This is using "busmode selection method 2" as described in the
 * Si4702-03-C19.pdf
 *
 * I2C must be disabled before calling this function.
 */
static bool reset_device(const int si470x_rst_pin,
                         const int si470x_gpio2_int_pin,
                         const int i2c_sda_pin) {
  if (mgos_i2c_get_global()) {
    LOG(LL_ERROR, ("I2C must be disabled"));
    return false;
  }
  mgos_gpio_set_mode(si470x_rst_pin, MGOS_GPIO_MODE_OUTPUT);
  mgos_gpio_set_mode(i2c_sda_pin, MGOS_GPIO_MODE_OUTPUT);
  if (si470x_gpio2_int_pin != -1)
    mgos_gpio_set_mode(si470x_gpio2_int_pin, MGOS_GPIO_MODE_OUTPUT);

  mgos_gpio_write(i2c_sda_pin, false);  // Low SDIO = 2-wire interface.
  if (si470x_gpio2_int_pin != -1)
    mgos_gpio_write(si470x_gpio2_int_pin, true);  // goes low on interrupt.
  mgos_gpio_write(si470x_rst_pin, false);         // Put Si470X into reset.

  mgos_msleep(1);  // Allow pin to settle.

  // Bring Si470X out of reset with SDIO set to low and SEN pulled high
  // (if used) with on-board resistor.
  mgos_gpio_write(si470x_rst_pin, true);

  mgos_msleep(1);  // Allow Si470X to come out of reset.

  return true;
}

static bool set_stc_interrupt_handler(struct si470x* device) {
  LOG(LL_DEBUG,
      ("Enabling GPIO2 interrupt handler on GPIO %d", device->gpio2_int_pin));
  if (!mgos_gpio_set_mode(device->gpio2_int_pin, MGOS_GPIO_MODE_INPUT)) {
    LOG(LL_ERROR, ("Error setting input on GPIO %d", device->gpio2_int_pin));
    return false;
  }
  if (!mgos_gpio_set_int_handler(device->gpio2_int_pin, MGOS_GPIO_INT_EDGE_NEG,
                                 &stc_interrupt_handler, device)) {
    LOG(LL_ERROR,
        ("Error setting interrupt handler on GPIO %d", device->gpio2_int_pin));
    return false;
  }
  if (!mgos_gpio_enable_int(device->gpio2_int_pin)) {
    LOG(LL_ERROR,
        ("Error enabling interrupt handler on GPIO %d", device->gpio2_int_pin));
    return false;
  }
  LOG(LL_DEBUG,
      ("Si470X STC interrupt handler set on GPIO %d.", device->gpio2_int_pin));
  return true;
}

/**
 * Wait for the STC bit to be set.
 */
static bool wait_for_stc_bit_set(struct si470x* device) {
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
      mgos_msleep(stc_check_interval_ms);
    }
  } else {
    for (iter = 0; iter < num_stc_iters; iter++) {
      mgos_msleep(stc_check_interval_ms);
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
static bool wait_for_stc_bit_clear(struct si470x* device) {
  // Checks every 5 msec. This avoids an infinite loop which results in a crash
  // due to the watchdog timer (WDT) if, for some reason, the tuner never clears
  // the STC bit.
  const uint16_t stc_check_interval_ms = 5;
  const size_t num_stc_iters = device->max_seek_tune_ms / stc_check_interval_ms;

  for (size_t iter = 0; iter < num_stc_iters; iter++) {
    mgos_msleep(stc_check_interval_ms);
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
static int seek_device(struct si470x* device,
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

  mgos_rds_decoder_reset(device->decoder);

  if (!wait_for_stc_bit_set(device))
    LOG(LL_WARN, ("Error waiting for STC bit set when seeking"));

  if (!read_registers(device))
    return -1;

  *reached_sfbl = device->shadow_reg[STATUSRSSI] & SFBL;
  CLEAR_BITS(device->shadow_reg[POWERCFG], SEEK);
  if (!update_registers(device))
    return -1;

  if (!wait_for_stc_bit_clear(device))
    LOG(LL_WARN, ("Error waiting for STC bit clear when seeking"));

  return get_frequency(device);
}

/**
 * Do the minimum required steps to power on the device.
 *
 * These are the required steps defined in AN230 rev. 0.9 pg. 12.
 */
static bool min_power_on(struct si470x* device) {
  memset(device->shadow_reg, 0, sizeof(device->shadow_reg));
  mgos_rds_decoder_reset(device->decoder);

  // Set the XOSCEN bit to power up the crystal.
  device->shadow_reg[TEST1] = XOSCEN | XO_UNKN;
  if (!update_registers(device))
    return false;

  // Wait for crystal to power up. Minimum 500 ms.
  mgos_msleep(550);

  // Disable mute and set enable bit. memset above clears DISABLE bit.
  device->shadow_reg[POWERCFG] = DMUTE | DSMUTE | ENABLE;
  if (!update_registers(device))
    return false;

  // Wait for device powerup. This is from the Si4703 datasheet.
  mgos_msleep(110);

  return true;
}

static void set_channel_spacing(struct si470x* device) {
  CLEAR_BITS(device->shadow_reg[SYSCONFIG2], CHAN_SPACE_MASK);
  if (device->region == REGION_EUROPE) {
    SET_BITS(device->shadow_reg[SYSCONFIG2], CHAN_SPACE_100);
  } else {
    // 0x00 is USA, so no need to set any bits.
  }
}

static void set_de_emphasis(struct si470x* device) {
  if (device->region != REGION_EUROPE) {
    SET_BITS(device->shadow_reg[SYSCONFIG1], DE);
  } else {
    CLEAR_BITS(device->shadow_reg[SYSCONFIG1], DE);
  }
}

enum si470x_region parse_region(const char* region) {
  if (!stricmp(region, "europe"))
    return REGION_EUROPE;
  if (!stricmp(region, "australia"))
    return REGION_AUSTRALIA;
  if (!stricmp(region, "japan"))
    return REGION_JAPAN;
  return REGION_US;
}

/****************************************/
/*vvvvvvvvvv PUBLIC FUNCTIONS *vvvvvvvvv*/
/****************************************/

/**
 * The library initialization function. Automatically called by MGOS.
 */
bool mgos_si470x_init(void) {
  if (mgos_i2c_get_global()) {
    LOG(LL_ERROR, ("si470x must be initialized before I2C."));
    return false;
  }

  if (mgos_sys_config_get_si470x_enable()) {
    const int reset_pin = mgos_sys_config_get_si470x_rst_gpio();
    if (reset_pin < 0) {
      LOG(LL_ERROR, ("si470x requires a reset pin."));
      return false;
    }

    if (!reset_device(reset_pin, mgos_sys_config_get_si470x_gpio2_int_gpio(),
                      mgos_sys_config_get_i2c_sda_gpio())) {
      LOG(LL_ERROR, ("Unable to reset the connected Si470X."));
      return false;
    }
  }

#if NUM_BUSES == 2
  if (mgos_sys_config_get_si470x1_enable()) {
    const int reset_pin = mgos_sys_config_get_si470x1_rst_gpio();
    if (reset_pin < 0) {
      LOG(LL_ERROR, ("si470x1 requires a reset pin."));
      return false;
    }

    if (!reset_device(reset_pin, mgos_sys_config_get_si470x1_gpio2_int_gpio(),
                      mgos_sys_config_get_i2c1_sda_gpio())) {
      LOG(LL_ERROR, ("Unable to reset the connected Si470X #2."));
      return false;
    }
  }
#endif  // NUM_BUSES == 2

  LOG(LL_INFO, ("si470x library initialized."));
  g_library_initialized = true;
  return true;
}

struct si470x* mgos_si470x_get_device(int i2c_bus_no) {
  if (i2c_bus_no < 0 || i2c_bus_no >= (int)ARRAY_SIZE(s_devices))
    return NULL;
  if (s_devices[i2c_bus_no] == NULL) {
    switch (i2c_bus_no) {
      case 0: {
        const struct si470x_config config = {
            .region = parse_region(mgos_sys_config_get_si470x_region()),
            .advanced_ps_decoding = mgos_sys_config_get_si470x_advanced_ps(),
            .gpio2_int_pin = mgos_sys_config_get_si470x_gpio2_int_gpio(),
            .i2c_bus_no = i2c_bus_no,
        };
        s_devices[i2c_bus_no] = mgos_si470x_create(&config);
      } break;
#if NUM_BUSES == 2
      case 1: {
        const struct si470x_config config = {
            .region = parse_region(mgos_sys_config_get_si470x1_region()),
            .advanced_ps_decoding = mgos_sys_config_get_si470x1_advanced_ps(),
            .gpio2_int_pin = mgos_sys_config_get_si470x1_gpio2_int_gpio(),
            .i2c_bus_no = i2c_bus_no,
        };
        s_devices[i2c_bus_no] = mgos_si470x_create(&config);
      } break;
#endif  // NUM_BUSES == 2
    }
  }
  return s_devices[i2c_bus_no];
}

struct si470x* mgos_si470x_get_global(void) {
  return mgos_si470x_get_device(0);
}

struct si470x* mgos_si470x_create(const struct si470x_config* config) {
  if (!g_library_initialized) {
    LOG(LL_ERROR, ("The si470x library was not initialized. Is it disabled?"));
    return NULL;
  }

  struct si470x* device = (struct si470x*)calloc(1, sizeof(struct si470x));
  if (device == NULL) {
    LOG(LL_ERROR, ("Could not allocate si470x structure."));
    return NULL;
  }

  device->i2c = mgos_i2c_get_bus(config->i2c_bus_no);
  if (!device->i2c) {
    LOG(LL_ERROR, ("I2C not initialized on bus %d.", config->i2c_bus_no));
    free(device);
    return false;
  }
  device->region = config->region;
  device->gpio2_int_pin = config->gpio2_int_pin;
  device->i2caddr = 0x10;
  // The maximum seek/tune time in msec. This is specified in the datasheet
  // and may be device specific. 60 msec is for the Si4703.
  device->max_seek_tune_ms = 60;
  const struct rds_decoder_config decoder_config = {
      .advanced_ps_decoding = config->advanced_ps_decoding,
      .rds_data = &device->rds,
  };
  device->decoder = mgos_rds_decoder_create(&decoder_config);
  if (!device->decoder) {
    free(device);
    return false;
  }
  mgos_rds_decoder_reset(device->decoder);

  LOG(LL_ERROR, ("si470x device successfully created."));
  return device;
}

void mgos_si470x_set_rds_callback(struct si470x* device,
                                  RDSChangedFunc rds_changed_cb,
                                  void* rds_changed_cb_data) {
  device->rds_changed.func = rds_changed_cb;
  device->rds_changed.arg = rds_changed_cb_data;
}

void mgos_si470x_set_oda_callbacks(struct si470x* device,
                                   DecodeODAFunc decode_cb,
                                   ClearODAFunc clear_cb,
                                   void* cb_data) {
  mgos_rds_decoder_set_oda_callbacks(device->decoder, decode_cb, clear_cb,
                                     cb_data);
}

void mgos_si470x_delete(struct si470x* device) {
  if (device == NULL)
    return;

  mgos_si470x_power_off(device);
  mgos_rds_decoder_delete(device->decoder);
  free(device);
}

/*
 * Powers on the chip as per instructions in AN230 rev. 0.9, page 12.
 */
bool mgos_si470x_power_on(struct si470x* device) {
  if (!min_power_on(device))
    return false;

  if (!read_registers(device))
    return false;

  if (get_supports_rds(device))
    SET_BITS(device->shadow_reg[SYSCONFIG1], RDS | RDSM);

  const bool use_tuning_interrupts = false;
  if (device->gpio2_int_pin != -1) {
    // See AN230 sect. 3.2.5 for interrupt support.
    if (get_supports_rds_int(device))
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

  if (!update_registers(device))
    return false;

  if (!read_registers(device))
    return false;

  mgos_msleep(110);  // Max powerup time, from datasheet page 13.

  // Once device is fully on enable interrupt handler.
  if ((get_rds_interrupts_enabled(device) || use_tuning_interrupts) &&
      !set_stc_interrupt_handler(device)) {
    LOG(LL_ERROR, ("Failure setting interrupt handler."));
    return false;
  }

  return true;
}

// Sequence defined in AN230 table 4.
bool mgos_si470x_power_off(struct si470x* device) {
  if (!read_registers(device))
    return false;
  SET_BITS(device->shadow_reg[TEST1], AHIZEN);
  if (!update_registers(device))
    return false;

  if (device->gpio2_int_pin != -1) {
    // Set pin to low to reduce power consumption
    mgos_gpio_write(device->gpio2_int_pin, false);
  }

  CLEAR_BITS(device->shadow_reg[POWERCFG], DMUTE | ENABLE);
  SET_BITS(device->shadow_reg[POWERCFG], ENABLE | DISABLE);
  if (!update_registers(device))
    return false;

  return true;
}

bool mgos_si470x_is_on(struct si470x* device) {
  return device->shadow_reg[POWERCFG] & ENABLE;
}

bool mgos_si470x_set_frequency(struct si470x* device, int frequency) {
  const uint16_t channel = frequency_to_channel(frequency, device);

  if (!read_registers(device))
    return false;

  // These steps defined in AN230 rev 0.9 sect 3.7.
  if (!read_registers(device))
    return false;
  CLEAR_BITS(device->shadow_reg[CHANNEL], CHANNEL_MASK);
  SET_BITS(device->shadow_reg[CHANNEL], TUNE | channel);
  CLEAR_BITS(device->shadow_reg[STATUSRSSI], STC);
  if (!update_registers(device))
    return false;
  mgos_rds_decoder_reset(device->decoder);

  if (!wait_for_stc_bit_set(device))
    LOG(LL_WARN, ("Error waiting for STC bit set when tuning"));

  // Clear the TUNE bit to end the tuning operation.
  CLEAR_BITS(device->shadow_reg[CHANNEL], TUNE);
  if (!update_registers(device))
    return false;

  if (!wait_for_stc_bit_clear(device))
    LOG(LL_WARN, ("Error waiting for STC bit clear when seeking"));

  return true;
}

int mgos_si470x_seek_up(struct si470x* device,
                        bool allow_wrap,
                        bool* reached_sfbl) {
  return seek_device(device, SEEK_UP, allow_wrap, reached_sfbl);
}

int mgos_si470x_seek_down(struct si470x* device,
                          bool allow_wrap,
                          bool* reached_sfbl) {
  return seek_device(device, SEEK_DOWN, allow_wrap, reached_sfbl);
}

bool mgos_si470x_set_volume(struct si470x* device, int volume) {
  if (!read_registers(device))
    return false;

  // Clamp volume to valid values.
  if (volume < 0)
    volume = 0;
  else if (volume > 15)
    volume = 15;

  CLEAR_BITS(device->shadow_reg[SYSCONFIG2], VOLUME_MASK);
  SET_BITS(device->shadow_reg[SYSCONFIG2], volume);

  if (!update_registers(device))
    return false;

  return true;
}

bool mgos_si470x_set_mute(struct si470x* device, bool mute_enabled) {
  if (mute_enabled)
    CLEAR_BITS(device->shadow_reg[POWERCFG], DMUTE);
  else
    SET_BITS(device->shadow_reg[POWERCFG], DMUTE);
  return update_registers(device);
}

bool mgos_si470x_set_soft_mute(struct si470x* device, bool mute_enabled) {
  if (mute_enabled)
    CLEAR_BITS(device->shadow_reg[POWERCFG], DSMUTE);
  else
    SET_BITS(device->shadow_reg[POWERCFG], DSMUTE);
  return update_registers(device);
}

bool mgos_si470x_get_state(struct si470x* device, struct si470x_state* state) {
  if (!read_registers(device))
    return false;

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

  return true;
}

bool mgos_si470x_get_rds_data(struct si470x* device,
                              struct rds_data* rds_data) {
  *rds_data = device->rds;
  return true;
}
