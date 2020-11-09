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

#include <si470x_port.h>

#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG

#include <driver/gpio.h>
#include <driver/i2c.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include "si470x_misc.h"

#define ESP_INTR_FLAG_DEFAULT 0

const char TAG[] = "esp-idf port";
const bool ACK_CHECK_EN = true;  ///< I2C master will check ack from slave.
const size_t I2C_MASTER_TX_BUF_DISABLE = 0;
const size_t I2C_MASTER_RX_BUF_DISABLE = 0;

struct si470x_port_t {
  bool noop;  ///< No-op setting.
  SemaphoreHandle_t i2c_mutex;
  struct {
    bool init_called;    ///< Has I2C been initialized?
    esp_err_t init_err;  ///< The first error status when initializing I2C.
  } i2c;
  struct {
    bool isr_service_installed;
    InterruptHandler isr_handler;
    void* isr_handler_arg;
    xQueueHandle event_queue;
  } gpio;
};

static struct si470x_port_t* g_port = NULL;

static gpio_mode_t xlate_pin_mode(enum gpio_pin_mode_t mode) {
  switch (mode) {
    case PIN_MODE_INPUT:
      return GPIO_MODE_INPUT;
    case PIN_MODE_OUTPUT:
      return GPIO_MODE_OUTPUT;
  }
  return GPIO_MODE_INPUT;
}

static int xlate_ttl_level(enum gpio_ttl_level_t level) {
  switch (level) {
    case TTL_HIGH:
      return 1;
    case TTL_LOW:
      return 0;
  }
  return 0;
}

static gpio_int_type_t xlate_edge_type(enum gpio_edge_type_t type) {
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

static esp_err_t i2c_master_read_slave(
    struct si470x_port_t* port,
    const struct si470x_i2c_params_t* i2c_params,
    uint8_t* data_rd,
    size_t size) {
  ESP_LOGV(TAG, "reading %u bytes from slave 0x%x", size,
           i2c_params->slave_addr);
  if (size == 0)
    return ESP_OK;
  if (port->i2c_mutex)
    xSemaphoreTake(port->i2c_mutex, portMAX_DELAY);
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (i2c_params->slave_addr << 1) | I2C_MASTER_READ,
                        ACK_CHECK_EN);
  if (size > 1) {
    i2c_master_read(cmd, data_rd, size - 1, I2C_MASTER_ACK);
  }
  i2c_master_read_byte(cmd, data_rd + size - 1, I2C_MASTER_NACK);
  i2c_master_stop(cmd);
  esp_err_t ret =
      i2c_master_cmd_begin(i2c_params->bus, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  if (port->i2c_mutex)
    xSemaphoreGive(port->i2c_mutex);
  if (ret == ESP_OK)
    ESP_LOGV(TAG, "Successfully read %d bytes", size);
  else
    ESP_LOGE(TAG, "Failure reading %d bytes: %s", size, esp_err_to_name(ret));
  return ret;
}

static esp_err_t i2c_master_write_slave(
    struct si470x_port_t* port,
    const struct si470x_i2c_params_t* i2c_params,
    const uint8_t* data_wr,
    size_t size) {
  ESP_LOGV(TAG, "reading %u bytes from slave 0x%x", size,
           i2c_params->slave_addr);
  if (port->i2c_mutex)
    xSemaphoreTake(port->i2c_mutex, portMAX_DELAY);
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (i2c_params->slave_addr << 1) | I2C_MASTER_WRITE,
                        ACK_CHECK_EN);
  // In newer IDF's data is const.
  i2c_master_write(cmd, (uint8_t*)(data_wr), size, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  esp_err_t ret =
      i2c_master_cmd_begin(i2c_params->bus, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  if (port->i2c_mutex)
    xSemaphoreGive(port->i2c_mutex);
  return ret;
}

static esp_err_t i2c_master_init(struct si470x_port_t* port,
                                 const struct si470x_i2c_params_t* i2c_params) {
  ESP_LOGV(TAG, "Initializing I2C master");
  if (port->i2c.init_called) {
    ESP_LOGW(TAG, "I2C master already initialized");
    return port->i2c.init_err;
  }

  port->i2c.init_called = true;

  const i2c_config_t config = {
      .mode = I2C_MODE_MASTER,
      .sda_io_num = i2c_params->sdio_pin,
      .scl_io_num = i2c_params->sclk_pin,
      .sda_pullup_en = GPIO_PULLUP_DISABLE,
      .scl_pullup_en = GPIO_PULLUP_DISABLE,
      .master =
          {// Max of 1MHz recommended by:
           // https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/i2c.html#_CPPv4N12i2c_config_t9clk_speedE
           .clk_speed = 100000},
  };

  port->i2c.init_err = i2c_param_config(i2c_params->bus, &config);
  if (port->i2c.init_err != ESP_OK) {
    ESP_LOGE(TAG, "i2c_param_config: %s", esp_err_to_name(port->i2c.init_err));
    return port->i2c.init_err;
  }

  port->i2c.init_err = i2c_driver_install(i2c_params->bus, I2C_MODE_MASTER,
                                          I2C_MASTER_RX_BUF_DISABLE,
                                          I2C_MASTER_TX_BUF_DISABLE, 0);
  if (port->i2c.init_err == ESP_OK)
    ESP_LOGD(TAG, "I2C enabled on port %d", i2c_params->bus);
  else
    ESP_LOGE(TAG, "i2c_driver_install failed: %s",
             esp_err_to_name(port->i2c.init_err));

  return port->i2c.init_err;
}

static void gpio_task_handler(void* arg) {
  InterruptHandler handler = (InterruptHandler)arg;
  while (true) {
    uint32_t io_num;
    if (xQueueReceive(g_port->gpio.event_queue, &io_num, portMAX_DELAY)) {
      ESP_LOGV(TAG, "GPIO[%d] intr, val: %d", io_num, gpio_get_level(io_num));
      handler(g_port->gpio.isr_handler_arg);
    }
  }
}

static void IRAM_ATTR gpio_isr_handler(void* arg) {
  // Post to queue to handle later.
  const uint32_t gpio_num = (uint32_t)arg;
  xQueueSendFromISR(g_port->gpio.event_queue, &gpio_num, NULL);
}

static esp_err_t install_isr_service(struct si470x_port_t* port,
                                     gpio_pin_t pin,
                                     enum gpio_edge_type_t edge_type,
                                     InterruptHandler handler) {
  ESP_LOGI(TAG, "Installing ISR service");
  const uint32_t pin_mask = 1 << pin;

  const gpio_config_t io_conf = {
      .pin_bit_mask = pin_mask,
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_ENABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = xlate_edge_type(edge_type),
  };

  esp_err_t err;
  if ((err = gpio_config(&io_conf)) != ESP_OK) {
    ESP_LOGE(TAG, "Unable to config GPIO: %s", esp_err_to_name(err));
    return err;
  }

  port->gpio.event_queue = xQueueCreate(10, sizeof(uint32_t));

  BaseType_t task =
      xTaskCreate(gpio_task_handler, "gpio_task_isr", 2048, handler, 10, NULL);
  if (task != pdPASS)
    return ESP_FAIL;

  return gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
}

struct si470x_port_t* port_create(const struct si470x_port_params_t* config) {
  assert(g_port == g_port);

  g_port = (struct si470x_port_t*)calloc(1, sizeof(struct si470x_port_t));
  if (g_port == NULL)
    return NULL;

  g_port->noop = config->noop;
  g_port->i2c_mutex = config->i2c_mutex;
  g_port->i2c.init_err = ESP_FAIL;

  ESP_LOGD(TAG, "Created port");

  return g_port;
}

void port_delete(struct si470x_port_t* port) {
  if (!port)
    return;
  free(port);
}

bool port_supports_gpio(struct si470x_port_t* port) {
  UNUSED(port);
  return true;
}

bool port_supports_i2c(struct si470x_port_t* port) {
  UNUSED(port);
  return true;
}

void port_delay(struct si470x_port_t* port, uint16_t msec) {
  UNUSED(port);
  vTaskDelay(msec / portTICK_PERIOD_MS);
}

bool port_enable_gpio(struct si470x_port_t* port) {
  UNUSED(port);
  return true;
}

void port_set_pin_mode(struct si470x_port_t* port,
                       gpio_pin_t pin,
                       enum gpio_pin_mode_t mode) {
  UNUSED(port);
  esp_err_t err = gpio_set_direction(pin, xlate_pin_mode(mode));
  if (err == OK)
    ESP_LOGV(TAG, "Set mode for pin %d.", pin);
  else
    ESP_LOGE(TAG, "Can't set mode for pin %d.", pin);
}

void port_digital_write(struct si470x_port_t* port,
                        gpio_pin_t pin,
                        enum gpio_ttl_level_t level) {
  UNUSED(port);
  gpio_set_level(pin, xlate_ttl_level(level));
}

bool port_enable_i2c(struct si470x_port_t* port,
                     const struct si470x_i2c_params_t* i2c_params) {
  ESP_LOGI(TAG, "I2C enabled on port %d, SDA:%d, SCL:%d, slave:0x%x.",
           i2c_params->bus, i2c_params->sdio_pin, i2c_params->sclk_pin,
           i2c_params->slave_addr);

  return i2c_master_init(port, i2c_params) == ESP_OK;
}

bool port_i2c_enabled(struct si470x_port_t* port) {
  return port->i2c.init_called && port->i2c.init_err == ESP_OK;
}

bool port_set_interrupt_handler(struct si470x_port_t* port,
                                gpio_pin_t pin,
                                enum gpio_edge_type_t edge_type,
                                InterruptHandler handler,
                                void* user_data) {
  port->gpio.isr_handler = handler;
  port->gpio.isr_handler_arg = user_data;

  if (!port->gpio.isr_service_installed) {
    if (install_isr_service(port, pin, edge_type, handler) != ESP_OK)
      return false;
    port->gpio.isr_service_installed = true;
  }

  return gpio_isr_handler_add(pin, gpio_isr_handler, handler) == ESP_OK;
}

bool port_i2c_write(struct si470x_port_t* port,
                    const struct si470x_i2c_params_t* i2c_params,
                    const void* data,
                    size_t len) {
  return i2c_master_write_slave(port, i2c_params, data, len) == ESP_OK;
}

bool port_i2c_read(struct si470x_port_t* port,
                   const struct si470x_i2c_params_t* i2c_params,
                   void* data,
                   size_t len) {
  return i2c_master_read_slave(port, i2c_params, data, len) == ESP_OK;
}
