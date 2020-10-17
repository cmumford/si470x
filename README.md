# A library for controlling a Si470X radio chip.

## Overview

This library is for controlling the Silicon Labs Si470X family of radio
receiver chips.  These are the Si4700, Si4701, Si4702, and
[Si4703](https://www.silabs.com/audio-and-radio/fm-radios/si4702-03-radio-receivers/device.si4703).

There are two supported platforms: [Mongoose OS](https://mongoose-os.com/) and
Linux/UNIX (but really Raspberry Pi).

These tuner chips communicate on the I<sup>2</sup>C bus with their host
controller. They have a hard-coded bus address of 0x10. This library
supports multiple tuners, but requires that they be on different I<sup>2</sup>C
buses to allow for proper communication.

Contributions are welcome, please see [CONTRIBUTING.md](CONTRIBUTING.md)
for more information.

## Example use:

### Mongoose OS

First add the following to the `mos.yml` file:

```yaml
libs:
  - origin: https://github.com/mongoose-os-libs/i2c
  - origin: https://github.com/cmumford/rds
  - origin: https://github.com/cmumford/si470x

config_schema:
  - ["i2c.enable", true]
  - ["si470x.rst_gpio", 2]
  - ["si470x.advanced_ps", true]
  - ["si470x.gpio2_int_gpio", 14]
```

And to control the tuner:

```c
struct si470x* tuner = mgos_si470x_get_global();
mgos_si470x_power_on(tuner);

const int frequency = 98500000; // 98.5 MHz FM
mgos_si470x_set_frequency(tuner, frequency);

const int volume = 7;
mgos_si470x_set_volume(tuner, volume);

mgos_si470x_set_mute(tuner, false);
mgos_si470x_set_soft_mute(tuner, false);
```

Full example programs can be found at https://github.com/cmumford/si470x_examples.

### Linux/UNIX

The Linux/UNIX uses a porting layer. This exists because there
is not a consistent way to control GPIO pins, and also makes it
easier to support other platforms. This library ships with a
port that uses [wiringPi](http://wiringpi.com) for Raspberry Pi.

```c
#include <port_unix.h>
#include <si470x.h>

// These are all wiringPi pin numbers. See http://wiringpi.com/pins/
const struct si470x_config_t device_config = {
    .region = REGION_US,
    .advanced_ps_decoding = true,
    .gpio2_int_pin = 5,  // GPIO5
    .reset_pin = 6,      // GPIO6
    .sdio_pin = 8,       // GPIO2
    .sclk_pin = 9,       // GPIO3
};

const struct si470x_port port_config {
  .delay = port_delay, .enable_gpio = port_enable_gpio,
  .set_pin_mode = port_set_pin_mode, .digital_write = port_digital_write,
  .set_interrupt_handler = port_set_interrupt_handler,
  .set_i2c_addr = port_set_i2c_addr,
  .enable_i2c_packet_error_checking = port_enable_i2c_packet_error_checking
};

struct si470x* tuner = si470x_create(&device_config, &port_config);
si470x_power_on(tuner);

const int frequency = 98500000; // 98.5 MHz FM
si470x_set_frequency(tuner, frequency);

const int volume = 7;
si470x_set_volume(tuner, volume);

si470x_set_mute(tuner, false);
si470x_set_soft_mute(tuner, false);
```

## Items of note:

* Only the 2-wire interface is supported - The 3-wire interfaces is not
  yet implemented.
* Interrupts are supported to enhance communication with the tuner chip.
  These can be disabled, but RDS data will not be processed.
* The Linux/UNIX and Mongoose OS API's are *nearly* identical:
  * They have different names so that the Mongoose OS implementation
    can adhere to the common function naming convention.
  * On RPi the RDS data changed callback is called on a thread, but on
    Mongoose OS it is called on the main (and only) thread.
  * Mongoose OS can create the tuners via the mos.yml file, so extra
    functions exist (see `mgos_si470x_get_global`) to retrieve these.
