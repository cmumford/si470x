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

#include <mgos.h>
#include <mgos_i2c.h>

#include <si470x.h>
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

static bool g_library_initialized = false;
static struct si470x_t* s_devices[NUM_BUSES];

static enum si470x_region_t parse_region(const char* region) {
#if 0
  if (!stricmp(region, "europe"))
    return REGION_EUROPE;
  if (!stricmp(region, "australia"))
    return REGION_AUSTRALIA;
  if (!stricmp(region, "japan"))
    return REGION_JAPAN;
#endif
  return REGION_US;
}

/**
 * The library initialization function. Automatically called by MGOS.
 */
bool mgos_si470x_init(void) {
  if (mgos_i2c_get_global()) {
    LOG(LL_ERROR, ("si470x must be initialized before I2C."));
    return false;
  }

  struct si470x_port_t* port = port_create(/*noop=*/false);
  if (mgos_sys_config_get_si470x_enable()) {
    const int reset_pin = mgos_sys_config_get_si470x_rst_gpio();
    if (reset_pin < 0) {
      LOG(LL_ERROR, ("si470x requires a reset pin."));
      port_delete(port);
      return false;
    }

    if (!reset_device(port, reset_pin,
                      mgos_sys_config_get_si470x_gpio2_int_gpio(),
                      mgos_sys_config_get_i2c_sda_gpio())) {
      LOG(LL_ERROR, ("Unable to reset the connected Si470X."));
      port_delete(port);
      return false;
    }
  }

#if NUM_BUSES == 2
  if (mgos_sys_config_get_si470x1_enable()) {
    const int reset_pin = mgos_sys_config_get_si470x1_rst_gpio();
    if (reset_pin < 0) {
      LOG(LL_ERROR, ("si470x1 requires a reset pin."));
      port_delete(port);
      return false;
    }

    if (!reset_device(port, reset_pin,
                      mgos_sys_config_get_si470x1_gpio2_int_gpio(),
                      mgos_sys_config_get_i2c1_sda_gpio())) {
      LOG(LL_ERROR, ("Unable to reset the connected Si470X #2."));
      port_delete(port);
      return false;
    }
  }
#endif  // NUM_BUSES == 2

  port_delete(port);
  LOG(LL_INFO, ("si470x library initialized."));
  g_library_initialized = true;
  return true;
}

struct si470x_t* mgos_si470x_get_device(int i2c_bus_no) {
  if (i2c_bus_no < 0 || i2c_bus_no >= (int)ARRAY_SIZE(s_devices))
    return NULL;
  static struct si470x_port_t* port;
  if (!port) {
    // This leaks port, but embedded apps never quit.
    port = port_create(/*noop=*/false);
  }
  if (s_devices[i2c_bus_no] == NULL) {
    switch (i2c_bus_no) {
      case 0: {
        const struct si470x_config_t config = {
            .region = parse_region(mgos_sys_config_get_si470x_region()),
            .advanced_ps_decoding = mgos_sys_config_get_si470x_advanced_ps(),
            .gpio2_int_pin = mgos_sys_config_get_si470x_gpio2_int_gpio(),
            .reset_pin = mgos_sys_config_get_si470x_rst_gpio(),
            .i2c =
                {
                    .bus = i2c_bus_no,
                    .sdio_pin = -1,  // Configured in I2C settings
                    .sclk_pin = -1,  // Configured in I2C settings.
                    .slave_addr = 0x10,
                },
            .port = port,
        };
        s_devices[i2c_bus_no] = mgos_si470x_create(&config);
      } break;
#if NUM_BUSES == 2
      case 1: {
        const struct si470x_config_t config = {
            .region = parse_region(mgos_sys_config_get_si470x1_region()),
            .advanced_ps_decoding = mgos_sys_config_get_si470x1_advanced_ps(),
            .gpio2_int_pin = mgos_sys_config_get_si470x1_gpio2_int_gpio(),
            .reset_pin = mgos_sys_config_get_si470x_rst_gpio(),
            .i2c =
                {
                    .bus = i2c_bus_no,
                    .sdio_pin = -1,  // Configured in I2C settings
                    .sclk_pin = -1,  // Configured in I2C settings.
                    .slave_addr = 0x10,
                },
            .port = port,
        };
        s_devices[i2c_bus_no] = mgos_si470x_create(&config);
      } break;
#endif  // NUM_BUSES == 2
    }
  }
  return s_devices[i2c_bus_no];
}

struct si470x_t* mgos_si470x_get_global(void) {
  return mgos_si470x_get_device(0);
}

struct si470x_t* mgos_si470x_create(const struct si470x_config_t* config) {
  if (!g_library_initialized) {
    LOG(LL_ERROR,
        ("The si470x_t library was not initialized. Is it disabled?"));
    return NULL;
  }
  return si470x_create(config);
}

void mgos_si470x_set_rds_callback(struct si470x_t* device,
                                  RDSChangedFunc rds_changed_cb,
                                  void* rds_changed_cb_data) {
  si470x_set_rds_callback(device, rds_changed_cb, rds_changed_cb_data);
}

void mgos_si470x_set_oda_callbacks(struct si470x_t* device,
                                   DecodeODAFunc decode_cb,
                                   ClearODAFunc clear_cb,
                                   void* cb_data) {
  si470x_set_oda_callbacks(device, decode_cb, clear_cb, cb_data);
}

void mgos_si470x_delete(struct si470x_t* device) {
  si470x_delete(device);
}

/*
 * Powers on the chip as per instructions in AN230 rev. 0.9, page 12.
 */
bool mgos_si470x_power_on(struct si470x_t* device) {
  return si470x_power_on(device);
}

// Sequence defined in AN230 table 4.
bool mgos_si470x_power_off(struct si470x_t* device) {
  return si470x_power_off(device);
}

bool mgos_si470x_is_on(struct si470x_t* device) {
  return si470x_is_on(device);
}

bool mgos_si470x_set_frequency(struct si470x_t* device, int frequency) {
  return si470x_set_frequency(device, frequency);
}

int mgos_si470x_seek_up(struct si470x_t* device,
                        bool allow_wrap,
                        bool* reached_sfbl) {
  return si470x_seek_up(device, allow_wrap, reached_sfbl);
}

int mgos_si470x_seek_down(struct si470x_t* device,
                          bool allow_wrap,
                          bool* reached_sfbl) {
  return si470x_seek_down(device, allow_wrap, reached_sfbl);
}

bool mgos_si470x_set_volume(struct si470x_t* device, int volume) {
  return si470x_set_volume(device, volume);
}

bool mgos_si470x_set_mute(struct si470x_t* device, bool mute_enabled) {
  return si470x_set_mute(device, mute_enabled);
}

bool mgos_si470x_set_soft_mute(struct si470x_t* device, bool mute_enabled) {
  return si470x_set_soft_mute(device, mute_enabled);
}

bool mgos_si470x_get_state(struct si470x_t* device,
                           struct si470x_state_t* state) {
  return si470x_get_state(device, state);
}

bool mgos_si470x_get_rds_data(struct si470x_t* device,
                              struct rds_data* rds_data) {
  return si470x_get_rds_data(device, rds_data);
}
