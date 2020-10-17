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

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#if defined(HAVE_NANOSLEEP)
#include <time.h>
#endif
#if defined(HAVE_I2C_DEV)
#include <linux/i2c-dev.h>
#endif
#if defined(HAVE_WIRING_PI)
#include <wiringPi.h>
#endif

#define UNUSED(expr) \
  do {               \
    (void)(expr);    \
  } while (0)

struct si470x_port_t {
  bool noop;
  struct {
    int fd;
    struct si470x_i2c_params_t conf;  ///< Si470X I2C config conf.
  } i2c;
};

#if defined(HAVE_WIRING_PI)

InterruptHandler g_interrupt_handler;
void* g_isr_user_data;

static int xlate_pin_mode(enum gpio_pin_mode_t mode) {
  switch (mode) {
    case PIN_MODE_INPUT:
      return INPUT;
    case PIN_MODE_OUTPUT:
      return OUTPUT;
  }
  return INPUT;
}

static int xlate_ttl_level(enum gpio_ttl_level_t level) {
  switch (level) {
    case TTL_HIGH:
      return HIGH;
    case TTL_LOW:
      return LOW;
  }
  return HIGH;
}

static int xlate_edge_type(enum gpio_edge_type_t type) {
  switch (type) {
    case EDGE_TYPE_FALLING:
      return INT_EDGE_FALLING;
    case EDGE_TYPE_RISING:
      return INT_EDGE_RISING;
    case EDGE_TYPE_BOTH:
      return INT_EDGE_BOTH;
  }
  return INT_EDGE_FALLING;
}

#endif  // defined(HAVE_WIRING_PI)

struct si470x_port_t* port_create(bool noop) {
  struct si470x_port_t* port =
      (struct si470x_port_t*)calloc(1, sizeof(struct si470x_port_t));

  port->noop = noop;
  port->i2c.fd = -1;

  return port;
}

void port_delete(struct si470x_port_t* port) {
  if (!port)
    return;
  free(port);
}

bool port_supports_gpio(struct si470x_port_t* port) {
  UNUSED(port);
#if defined(HAVE_WIRING_PI) || defined(HAVE_I2C_DEV)
  return true;
#else
  return false;
#endif
}

bool port_supports_i2c(struct si470x_port_t* port) {
  UNUSED(port);
#if defined(HAVE_WIRING_PI) || defined(HAVE_I2C_DEV)
  return true;
#else
  return false;
#endif
}

void port_delay(struct si470x_port_t* port, uint16_t msec) {
  UNUSED(port);
  // usleep isn't always available on RPi, but delay is. Checking for
  // __arm__ is a poor way of detecting RPi.
#if defined(HAVE_WIRING_PI)
  delay(msec);
#elif defined(HAVE_NANOSLEEP)
  UNUSED(msec);
#if 0
  const struct timespec spec = {.tv_sec = msec / 1000,
                                .tv_nsec = 1000000 * (msec % 1000)};
  nanosleep(&spec, NULL);
#endif
#elif defined(HAVE_USLEEP)
  usleep(msec * 1000);
#else
#error "Don't have a delay function."
#endif
}

bool port_enable_gpio(struct si470x_port_t* port) {
  UNUSED(port);
#if defined(HAVE_WIRING_PI)
  wiringPiSetupGpio();
#endif
  return true;
}

void port_set_pin_mode(struct si470x_port_t* port,
                       gpio_pin_t pin,
                       enum gpio_pin_mode_t mode) {
  UNUSED(port);
#if defined(HAVE_WIRING_PI)
  pinMode(pin, xlate_pin_mode(mode));
#else
  UNUSED(pin);
  UNUSED(mode);
#endif
}

void port_digital_write(struct si470x_port_t* port,
                        gpio_pin_t pin,
                        enum gpio_ttl_level_t level) {
  UNUSED(port);
#if defined(HAVE_WIRING_PI)
  digitalWrite(pin, xlate_ttl_level(level));
#else
  UNUSED(pin);
  UNUSED(level);
#endif
}

bool port_enable_i2c(struct si470x_port_t* port,
                     const struct si470x_i2c_params_t* i2c_params) {
  port->i2c.conf = *i2c_params;
#if defined(HAVE_I2C_DEV)
  const char filename[] = "/dev/i2c-1";
  // Open I2C slave device.
  if ((port->i2c.fd = open(filename, O_RDWR)) < 0) {
    perror(filename);
    return false;
  }

  // Set slave address.
  if (ioctl(port->i2c.fd, I2C_SLAVE, port->i2c.conf.slave_addr) < 0) {
    perror("Failed to acquire bus access and/or talk to slave");
    close(port->i2c.fd);
    port->i2c.fd = -1;
    return false;
  }

  // Enable packet error checking.
  if (ioctl(port->i2c.fd, I2C_PEC, 1) < 0) {
    perror("Failed to enable PEC");
    close(port->i2c.fd);
    port->i2c.fd = -1;
    return false;
  }
  return true;
#else
  return true;
#endif
}

bool port_i2c_enabled(struct si470x_port_t* port) {
  return port->i2c.fd >= 0;
}

#if defined(HAVE_WIRING_PI)
static void local_interrupt_handler(void) {
  g_interrupt_handler(g_isr_user_data);
}
#endif

bool port_set_interrupt_handler(struct si470x_port_t* port,
                                gpio_pin_t pin,
                                enum gpio_edge_type_t edge_type,
                                InterruptHandler handler,
                                void* user_data) {
  UNUSED(port);
  UNUSED(user_data);
#if defined(HAVE_WIRING_PI)
  if (g_interrupt_handler) {
    fprintf(stderr, "Interrupt handler already installed\n");
    return false;
  }
  g_interrupt_handler = handler;
  g_isr_user_data = user_data;
  return wiringPiISR(pin, xlate_edge_type(edge_type),
                     &local_interrupt_handler) >= 0;
#else
  UNUSED(pin);
  UNUSED(edge_type);
  UNUSED(handler);
  return true;
#endif
}

bool port_i2c_write(struct si470x_port_t* port, const void* data, size_t len) {
  if (port->i2c.fd < 0)
    return false;
  int bytes_written = write(port->i2c.fd, data, len);
  if (bytes_written < 0)
    return false;
  return bytes_written == (int)len;
}

bool port_i2c_read(struct si470x_port_t* port, void* data, size_t len) {
  if (port->i2c.fd < 0)
    return false;
  int bytes_read = read(port->i2c.fd, data, len);
  if (bytes_read < 0)
    return false;
  return bytes_read == (int)len;
}
