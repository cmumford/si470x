// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
// DEALINGS IN THE SOFTWARE.

/**
 * @file
 *
 * Abstraction implementation for Testing.
 */

#include "port.h"

#include <mgos.h>
#include <mgos_i2c.h>

#define UNUSED(expr) \
  do {               \
    (void)(expr);    \
  } while (0)

struct interrupt_handler_data {
  InterruptHandler handler;
  void* user_data;
};

struct port {
  bool noop;
  int i2c_slave_addr;
  uint8_t i2c_bus;
  struct mgos_i2c* i2c;
  struct interrupt_handler_data* int_handler;
};

static enum mgos_gpio_mode xlate_pin_mode(enum pin_mode mode) {
  switch (mode) {
    case PIN_MODE_INPUT:
      return MGOS_GPIO_MODE_INPUT;
    case PIN_MODE_OUTPUT:
      return MGOS_GPIO_MODE_OUTPUT;
  }
  return MGOS_GPIO_MODE_INPUT;
}

static bool xlate_ttl_level(enum ttl_level level) {
  switch (level) {
    case TTL_HIGH:
      return true;
    case TTL_LOW:
      return false;
  }
  return true;
}

static enum mgos_gpio_int_mode xlate_edge_type(enum edge_type type) {
  switch (type) {
    case EDGE_TYPE_FALLING:
      return MGOS_GPIO_INT_EDGE_NEG;
    case EDGE_TYPE_RISING:
      return MGOS_GPIO_INT_EDGE_POS;
    case EDGE_TYPE_BOTH:
      return MGOS_GPIO_INT_EDGE_ANY;
  }
  return MGOS_GPIO_INT_EDGE_NEG;
}

struct port* port_create(bool noop) {
  struct port* port = (struct port*)calloc(1, sizeof(struct port));

  port->noop = noop;

  return port;
}

void port_delete(struct port* port) {
  if (!port)
    return;
  if (port->int_handler)
    free(port->int_handler);
  free(port);
}

bool port_supports_gpio(struct port* port) {
  UNUSED(port);
  return true;
}

bool port_supports_i2c(struct port* port) {
  UNUSED(port);
  // I2C is always enabled.
  return true;
}

void port_delay(struct port* port, uint16_t msec) {
  UNUSED(port);
  mgos_msleep(msec);
}

bool port_enable_gpio(struct port* port) {
  UNUSED(port);
  // GPIO is always enabled.
  return true;
}

void port_set_pin_mode(struct port* port, uint16_t pin, enum pin_mode mode) {
  UNUSED(port);
  mgos_gpio_set_mode(pin, xlate_pin_mode(mode));
}

void port_digital_write(struct port* port, uint16_t pin, enum ttl_level level) {
  UNUSED(port);
  mgos_gpio_write(pin, xlate_ttl_level(level));
}

bool port_enable_i2c(struct port* port, uint8_t i2c_bus, uint16_t slave_addr) {
  port->i2c_slave_addr = slave_addr;
  port->i2c_bus = i2c_bus;

  // HACK: Don't really do anything - rely on MGOS's initialization
  //       system to enable I2C.
  return true;
}

bool port_i2c_enabled(struct port* port) {
  return mgos_i2c_get_bus(port->i2c_bus) != NULL;
}

static void local_interrupt_handler(int pin, void* user_data) {
  struct interrupt_handler_data* data =
      (struct interrupt_handler_data*)user_data;
  data->handler(data->user_data);
}

bool port_set_interrupt_handler(struct port* port,
                                uint16_t pin,
                                enum edge_type edge_type,
                                InterruptHandler handler,
                                void* user_data) {
  if (!port->int_handler) {
    port->int_handler = (struct interrupt_handler_data*)malloc(
        sizeof(struct interrupt_handler_data));
    port->int_handler->handler = handler;
    port->int_handler->user_data = user_data;
  }

  if (!mgos_gpio_set_int_handler(pin, xlate_edge_type(edge_type),
                                 &local_interrupt_handler, port->int_handler)) {
    LOG(LL_ERROR, ("Error setting interrupt handler on GPIO %d", pin));
    return false;
  }
  if (!mgos_gpio_enable_int(pin)) {
    LOG(LL_ERROR, ("Error enabling interrupt handler on GPIO %d", pin));
    return false;
  }
  LOG(LL_DEBUG, ("interrupt handler set on GPIO %d.", pin));
  return true;
}

bool port_i2c_write(struct port* port, const void* data, size_t len) {
  if (!port_i2c_enabled(port))
    return false;

  if (!port->i2c) {
    port->i2c = mgos_i2c_get_bus(port->i2c_bus);
    if (!port->i2c)
      return false;
  }

  if (!mgos_i2c_write(port->i2c, port->i2c_slave_addr, data, len,
                      /*stop=*/true)) {
    LOG(LL_ERROR, ("Can't write to device"));
    return false;
  }

  return true;
}

bool port_i2c_read(struct port* port, void* data, size_t len) {
  if (!port_i2c_enabled(port))
    return false;

  if (!port->i2c) {
    port->i2c = mgos_i2c_get_bus(port->i2c_bus);
    if (!port->i2c)
      return false;
  }

  if (!mgos_i2c_read(port->i2c, port->i2c_slave_addr, data, len,
                     /*stop=*/true)) {
    LOG(LL_ERROR, ("Can't read from device"));
    return false;
  }

  return true;
}