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

#ifdef __cplusplus
extern "C" {
#endif

enum pin_mode { PIN_MODE_INPUT, PIN_MODE_OUTPUT };

enum ttl_level { TTL_HIGH, TTL_LOW };

enum edge_type {
  EDGE_TYPE_FALLING,
  EDGE_TYPE_RISING,
  EDGE_TYPE_BOTH,
  EDGE_TYPE_SETUP
};

typedef void (*InterruptHandler)(void);

/**
 * Pointers to functions which implement the port for the current platform.
 *
 * **All** Functions must be implemented.
 */
struct port {
  void (*delay)(uint16_t msec);
  bool (*enable_gpio)();
  void (*set_pin_mode)(uint16_t pin, enum pin_mode);
  void (*digital_write)(uint16_t pin, enum ttl_level);
  bool (*set_interrupt_handler)(uint16_t pin, enum edge_type, InterruptHandler);
  bool (*set_i2c_addr)(int i2c_fd, uint16_t addr);
  bool (*enable_i2c_packet_error_checking)(int i2c_fd);
};

#ifdef __cplusplus
}
#endif
