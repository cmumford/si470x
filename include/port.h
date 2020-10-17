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
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined(CONFIG_IDF_TARGET_ESP32)
typedef gpio_num_t gpio_pin_t;
#else
typedef uint16_t gpio_pin_t;
#endif

enum gpio_pin_mode_t { PIN_MODE_INPUT, PIN_MODE_OUTPUT };

enum gpio_ttl_level_t { TTL_HIGH, TTL_LOW };

enum gpio_edge_type_t { EDGE_TYPE_FALLING, EDGE_TYPE_RISING, EDGE_TYPE_BOTH };

// NOTE: On Raspberry Pi user_data is always NULL.
typedef void (*InterruptHandler)(void* user_data);

struct si470x_port;

/**
 * Create a port.
 *
 * @param noop Set to true for a no-op port, where all calls do nothing
 *             and never fail. This is for testing.
 */
struct si470x_port* port_create(bool noop);

/**
 * Delete the port.
 */
void port_delete(struct si470x_port* port);

/**
 * Does this port support GPIO?
 *
 * @param port The port object.
 */
bool port_supports_gpio(struct si470x_port* port);

/**
 * Does this port support I2C?
 *
 * @param port The port object.
 */
bool port_supports_i2c(struct si470x_port* port);

/**
 * Delay (sleep) the calling thread a specified amount of time.
 *
 * @param port The port object.
 * @param msec The number of milliseconds to delay.
 */
void port_delay(struct si470x_port* port, uint16_t msec);

/**
 * Enable GPIO for this port.
 *
 * @param port The port object.
 */
bool port_enable_gpio(struct si470x_port* port);

/**
 * Set the given pin mode.
 *
 * @param port The port object.
 * @param pin The GPIO pin.
 * @param mode The pin mode.
 */
void port_set_pin_mode(struct si470x_port* port,
                       gpio_pin_t pin,
                       enum gpio_pin_mode_t mode);

/**
 * Set the specified pin's TTL level.
 *
 * @param port  The port object.
 * @param pin   The pin to set.
 * @param level The port object.
 */
void port_digital_write(struct si470x_port* port,
                        gpio_pin_t pin,
                        enum gpio_ttl_level_t level);

/**
 * Set the interrupt handler for the specified pin
 *
 * @param port The port object.
 * @param pin The pin for which the handler will be called.
 * @param edge_type When to call the handler.
 * @param handler The ISR handler function pointer.
 */
bool port_set_interrupt_handler(struct si470x_port* port,
                                gpio_pin_t pin,
                                enum gpio_edge_type_t edge_type,
                                InterruptHandler handler,
                                void* handler_data);

/**
 * Enable I2C for the given port.
 *
 * @param port The port object.
 * @param i2c_bus The I2C bus number.
 * @param slave_addr The address of the slave.
 */
bool port_enable_i2c(struct si470x_port* port,
                     uint8_t i2c_bus,
                     uint16_t slave_addr);

/**
 * Is I2C enabled for the given port?
 *
 * @param port The port object.
 */
bool port_i2c_enabled(struct si470x_port* port);

/**
 * Write data to the I2C slave.
 *
 * @param port The port object.
 * @param data Pointer to the data to write.
 * @param len The number of bytes to write.
 */
bool port_i2c_write(struct si470x_port* port, const void* data, size_t len);

/**
 * Read data from the I2C slave.
 *
 * @param port The port object.
 * @param data Pointer to the data to read to.
 * @param len The number of bytes to read.
 */
bool port_i2c_read(struct si470x_port* port, void* data, size_t len);

#ifdef __cplusplus
}
#endif
