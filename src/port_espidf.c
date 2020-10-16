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

#include <driver/gpio.h>
#include <driver/i2c.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#define UNUSED(expr) \
  do {               \
    (void)(expr);    \
  } while (0)

#define ESP_INTR_FLAG_DEFAULT 0

const bool ACK_CHECK_EN = true;  ///< I2C master will check ack from slave.
const size_t I2C_MASTER_TX_BUF_DISABLE = 0;
const size_t I2C_MASTER_RX_BUF_DISABLE = 0;

struct port {
  bool noop;  ///< No-op setting.
  struct {
    i2c_config_t conf;    ///< I2C config params.
    i2c_port_t port;      ///< The I2C bus number.
    uint16_t slave_addr;  ///< The device I2C address.
    bool enabled;         ///< Has I2C been initialized?
    esp_err_t init_err;
  } i2c;
  struct {
    bool isr_service_installed;
    InterruptHandler isr_handler;
    void* isr_handler_arg;
    xQueueHandle event_queue;
  } gpio;
};

static struct port* g_port;

static gpio_mode_t xlate_pin_mode(enum pin_mode mode) {
  switch (mode) {
    case PIN_MODE_INPUT:
      return GPIO_MODE_INPUT;
    case PIN_MODE_OUTPUT:
      return GPIO_MODE_OUTPUT;
  }
  return GPIO_MODE_INPUT;
}

static int xlate_ttl_level(enum ttl_level level) {
  switch (level) {
    case TTL_HIGH:
      return 1;
    case TTL_LOW:
      return 0;
  }
  return 0;
}

static gpio_int_type_t xlate_edge_type(enum edge_type type) {
  switch (type) {
    case EDGE_TYPE_FALLING:
      return GPIO_INTR_NEGEDGE;
    case EDGE_TYPE_RISING:
      return GPIO_INTR_POSEDGE;
    case EDGE_TYPE_BOTH:
      return GPIO_INTR_ANYEDGE;
  }
  return GPIO_INTR_NEGEDGE;
}

static esp_err_t i2c_master_read_slave(struct port* port,
                                       uint8_t* data_rd,
                                       size_t size) {
  if (size == 0) {
    return ESP_OK;
  }
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (port->i2c.slave_addr << 1) | I2C_MASTER_READ,
                        ACK_CHECK_EN);
  if (size > 1) {
    i2c_master_read(cmd, data_rd, size - 1, I2C_MASTER_ACK);
  }
  i2c_master_read_byte(cmd, data_rd + size - 1, I2C_MASTER_NACK);
  i2c_master_stop(cmd);
  esp_err_t ret =
      i2c_master_cmd_begin(port->i2c.port, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  return ret;
}

static esp_err_t i2c_master_write_slave(struct port* port,
                                        const uint8_t* data_wr,
                                        size_t size) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (port->i2c.slave_addr << 1) | I2C_MASTER_WRITE,
                        ACK_CHECK_EN);
  // In newer IDF's data is const.
  i2c_master_write(cmd, (uint8_t*)(data_wr), size, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  esp_err_t ret =
      i2c_master_cmd_begin(port->i2c.port, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  return ret;
}

static esp_err_t i2c_master_init(struct port* port) {
  if (port->i2c.enabled)
    return port->i2c.init_err;

  i2c_param_config(port->i2c.port, &port->i2c.conf);
  port->i2c.init_err = i2c_driver_install(port->i2c.port, port->i2c.conf.mode,
                                          I2C_MASTER_RX_BUF_DISABLE,
                                          I2C_MASTER_TX_BUF_DISABLE, 0);
  port->i2c.enabled = true;
  return port->i2c.init_err;
}

static void gpio_task_handler(void* arg) {
  InterruptHandler handler = (InterruptHandler)arg;
  while (true) {
    uint32_t io_num;
    if (xQueueReceive(g_port->gpio.event_queue, &io_num, portMAX_DELAY)) {
      printf("GPIO[%d] intr, val: %d\n", io_num, gpio_get_level(io_num));
      handler(g_port->gpio.isr_handler_arg);
    }
  }
}

static void IRAM_ATTR gpio_isr_handler(void* arg) {
  // Post to queue to handle later.
  const uint32_t gpio_num = (uint32_t)arg;
  xQueueSendFromISR(g_port->gpio.event_queue, &gpio_num, NULL);
}

static esp_err_t install_isr_service(struct port* port,
                                     uint16_t pin,
                                     enum edge_type edge_type) {
  const uint32_t pin_mask = 1 << pin;

  const gpio_config_t io_conf = {
      .intr_type = xlate_edge_type(edge_type),
      .mode = GPIO_MODE_INPUT,
      .pin_bit_mask = pin_mask,
      .pull_down_en = 0,
      .pull_up_en = 1,
  };

  esp_err_t err;
  if ((err = gpio_config(&io_conf)) != ESP_OK)
    return err;

  port->gpio.event_queue = xQueueCreate(10, sizeof(uint32_t));

  xTaskCreate(gpio_task_handler, "gpio_task_isr", 2048, NULL, 10, NULL);

  return gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
}

struct port* port_create(bool noop) {
  struct port* port = (struct port*)calloc(1, sizeof(struct port));

  port->noop = noop;
  port->i2c.conf.mode = I2C_MODE_MASTER;
  // TODO: Why isn't clk_speed defined?
  // port->i2c_conf.clk_speed = 1000000;
  port->i2c.conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  port->i2c.conf.scl_pullup_en = GPIO_PULLUP_ENABLE;

  // TODO: Make this configurable.
  port->i2c.conf.sda_io_num = 5;
  port->i2c.conf.scl_io_num = 6;

  g_port = port;

  return port;
}

void port_delete(struct port* port) {
  if (!port)
    return;
  free(port);
}

bool port_supports_gpio(struct port* port) {
  UNUSED(port);
  return true;
}

bool port_supports_i2c(struct port* port) {
  UNUSED(port);
  return true;
}

void port_delay(struct port* port, uint16_t msec) {
  UNUSED(port);
  vTaskDelay(msec / portTICK_PERIOD_MS);
}

bool port_enable_gpio(struct port* port) {
  UNUSED(port);
  return true;
}

void port_set_pin_mode(struct port* port, uint16_t pin, enum pin_mode mode) {
  UNUSED(port);
  gpio_set_direction(pin, xlate_pin_mode(mode));
}

void port_digital_write(struct port* port, uint16_t pin, enum ttl_level level) {
  UNUSED(port);
  gpio_set_level(pin, xlate_ttl_level(level));
}

bool port_enable_i2c(struct port* port, uint8_t i2c_bus, uint16_t slave_addr) {
  port->i2c.port = i2c_bus;
  port->i2c.slave_addr = slave_addr;

  return i2c_master_init(port) == ESP_OK;
}

bool port_i2c_enabled(struct port* port) {
  return port->i2c.enabled && port->i2c.init_err == ESP_OK;
}

bool port_set_interrupt_handler(struct port* port,
                                uint16_t pin,
                                enum edge_type edge_type,
                                InterruptHandler handler,
                                void* user_data) {
  port->gpio.isr_handler = handler;
  port->gpio.isr_handler_arg = user_data;

  if (!port->gpio.isr_service_installed) {
    if (install_isr_service(port, pin, edge_type) != ESP_OK)
      return false;
    port->gpio.isr_service_installed = true;
  }

  return gpio_isr_handler_add(pin, gpio_isr_handler, handler) == ESP_OK;
}

bool port_i2c_write(struct port* port, const void* data, size_t len) {
  return i2c_master_write_slave(port, data, len) == ESP_OK;
}

bool port_i2c_read(struct port* port, void* data, size_t len) {
  return i2c_master_read_slave(port, data, len) == ESP_OK;
}
