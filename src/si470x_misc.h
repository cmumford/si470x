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

/*
 * Internal methods not part of this library's exposed API.
 */

#pragma once

#include <si470x.h>
#include <si470x_port.h>
#if !defined(MGOS)
#include <pthread.h>
#endif

#if defined(ESP_PLATFORM)
#define USE_FREERTOS_SEMAPHORE
#elif defined(MGOS)
// MGOS is single threaded - no need to synchronize.
#else
#define USE_PTHREADS
#endif

#if defined(USE_FREERTOS_SEMAPHORE)
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#endif

#if !defined(ARRAY_SIZE)
#define ARRAY_SIZE(ARRAY) (sizeof(ARRAY) / sizeof((ARRAY)[0]))
#endif

#define SET_BITS(value, bits) (value |= (bits))
#define CLEAR_BITS(value, bits) (value &= ~(bits))

#define UNUSED(expr) \
  do {               \
    (void)(expr);    \
  } while (0)

// clang-format off

// Register indexes (names match Si4702-03-C19.pdf pg. 22).
#define DEVICEID   0x00
#define CHIPID     0x01
#define POWERCFG   0x02
#define CHANNEL    0x03
#define SYSCONFIG1 0x04
#define SYSCONFIG2 0x05
#define SYSCONFIG3 0x06
#define TEST1      0x07
#define TEST2      0x08
#define BOOTCONFIG 0x09
#define STATUSRSSI 0x0A
#define READCHAN   0x0B
#define RDSA       0x0C
#define RDSB       0x0D
#define RDSC       0x0E
#define RDSD       0x0F

// Register 0x00 - DEVICEID
#define MANUFACTURER_MASK 0x0FFF // Manufacturer ID.
#define PART_MASK         0xF000 // Part number (high nibble).

// Register 0x01 - CHIPID
#define FIRMWARE_MASK     0b0000000000111111
#define DEVICE_MASK       0b0000001111000000
#define REVISION_MASK     0b1111110000000000

// Register 0x02 - POWERCFG
#define DSMUTE  0x8000  // (1 << 15), AN230 sect. 3.3.2
#define DMUTE   0x4000  // (1 << 14), AN230 sect. 3.5.1
#define MONO    0x2000
#define RDSM    0x0800  // RDSM Mode.
#define SKMODE  0x0400  // (1 << 10), AN230 sect. 3.6.2
#define SEEKUP  0x0200  // (1 << 9), AN230 sect. 3.6.1
#define SEEK    0x0100  // (1 << 8), AN230 sect. 3.6.3
#define DISABLE 0x0080  // (1 << 6)
#define ENABLE  0x0001

// Register 0x03 - CHANNEL
#define TUNE         0x8000  // 1 << 15, AN230 sect. 3.7.1
#define CHANNEL_MASK 0b000001111111111  // AN230 sect. 3.7.1

// Register 0x04 - SYSCONFIG1
#define RDSIEN       0b1000000000000000  // RDS interrupt enable, sect. 3.2.5
#define STCIEN       0b0100000000000000  // STC int. enable, sect. 3.2.5
#define RDS          0b0001000000000000  // RDS Enable sect. 3.4.4
#define DE           0b0000100000000000  // FM De-Emphasis, AN230 sect. 3.4.3
#define AGCD         0b0000010000000000
#define BLNDADJ_MASK 0b0000000011000000
#define GPIO3_MASK   0b0000000000110000
#define GPIO2_MASK   0b0000000000001100
#define GPIO1_MASK   0b0000000000000011

// GPIO2 values.
#define RDS_HIGH_IMPEDENCE 0b0000
#define RDS_STC_RDS_INT    0b0100
#define RDS_LOW_OUTPUT     0b1000
#define RDS_HIGH_OUTPUT    0b1100

// Register 0x05 - SYSCONFIG2
#define CHAN_SPACE_200  0x0000  // Channels are 200 kHz apart. (US/Australia)
#define CHAN_SPACE_100  0x0010  // Channels are 100 kHz apart. (Eur/Japan wide)
#define CHAN_SPACE_50   0x0020  // Channels are 50 kHz apart. (Japan /narrow)
#define CHAN_SPACE_MASK 0x0030  // bits 5:4
#define BAND_US_EUROPE  0x0000  // USA/Europe
#define BAND_JAPANW     0x0040  // Japan wide
#define BAND_JAPANN     0x0080  // Japan narrow
#define BAND_MASK       0x00C0  // bits 7:6
#define VOLUME_MASK     0x000F  // bottom nibble.
#define SEEKTH_MASK     0xFF00  // Seek threshold (bits 15:8). sect. 3.3.4

// Register 0x06 - SYSCONFIG3
#define SMUTER_MASK 0xC000  // Softmute Attack/Recover Rate, (0b11 << 14)
#define SMUTEA_MASK 0x3000  // Softmute Attenuation Level, (0b11 << 12)
#define VOLEXT      0x0100  // Extended Volume Range, (1 << 8)
#define SKSNR       0x00F0  // Seek SNR Threshold, (0b1111 << 4)
#define SKCNT       0x000F  // Seek Impulse Detection Threshold, (0b1111)

// Register 0x07 - TEST1
#define XOSCEN    0x8000  // Crystal Oscillator Enable, AN230 sect. 3.2.2 (1 << 15)
#define AHIZEN    0x4000  // Audio High-Z Enable, AN230 sect. 3.2.3 (1 << 14)
// This value is not defined in AN230. There is a sample value of 0x8100
// in "Table 3. Powerup Configuration Sequence", and without this bit
// the device (or maybe crystal) does not power up.
#define XO_UNKN   0x0100  // AN230 pg. 12.. Also see 3.2.2.

// Register 0x0A - STATUSRSSI
#define RDSR       0x8000  // (1 << 15)
#define STC        0x4000  // (1 << 14)
#define SFBL       0x2000  // Seek Frequency Band Limit (1 << 13)
#define AFCRL      0x1000  // (1 << 12)
#define RDSS       0x0800  // Unused in standard mode.
#define BLERA_MASK 0x0600  // RDS Block A Errors. (bits 10:9) (verbose only).
#define STEREO     0x0100  // (1 << 8)
#define RSSI_MASK  0x00FF  // RSSI (Received Signal Strength Indicator).

// Register 0x0A - READCHAN
#define BLERB_MASK    0b1100000000000000  // RDS Block B Errors (verbose only).
#define BLERC_MASK    0b0011000000000000  // RDS Block C Errors (verbose only).
#define BLERD_MASK    0b0000110000000000  // RDS Block D Errors (verbose only).
// CHANNEL_MASK works here too.

// clang-format on

#if defined(RDS_DEV) && defined(HAVE_WIRING_PI)
#define SUPPORT_TEST_DATA
#endif

struct si470x_t {
  // Instance values.
  uint16_t shadow_reg[16];         ///< A copy of the device registers.
  struct si470x_i2c_params_t i2c;  ///< I2C parameters.
  uint16_t max_seek_tune_ms;       ///< Maximum Seek/Tune time (milliseconds).
  enum si470x_region_t region;     ///< The broadcast region.
  struct rds_data rds;             ///< Current RDS data.
  struct rds_decoder* decoder;     ///< The RDS decoder.

  /**
   * The GPIO pin (on the Controller/RPi) connected to the Si470Xs GPIO2 pin.
   *
   * On RPi this is a wiringPi pin, otherwise a GPIO pin.
   *
   * A value of -1 disables interrupts and RDS is disabled.
   */
  gpio_pin_t gpio2_int_pin;

  struct {
    /**
     * Function called when the RDS data has changed.
     *
     * This is called when using interrupts.
     *
     * NOTE: On Raspberry Pi this is called on a different thread. On
     * Mongoose OS this is called on the main (and only) thread.
     */
    RDSChangedFunc func;
    void* arg;    ///< argument to pass to rds_changed_cb.
  } rds_changed;  ///< Function to call with RDS changes.

  struct si470x_port_t* port;  ///< The porting layer.

#if defined(SUPPORT_TEST_DATA)
  const struct rds_blocks* test_blocks;  ///< The test blocks to process.
  uint16_t num_test_blocks;              ///< The number of test blocks.
  uint16_t test_block_idx;               ///< The next RDS block to process.
  pthread_t sdr_test_thread;             ///< The thread object.
  bool run_test_thread;                  ///< Keep running the test thread?
  uint16_t block_delay_ms;               ///< Delay between processing blocks.
#endif

#if !defined(MGOS)
  gpio_pin_t reset_pin;  ///< The GPIO pin connected to the Si470X RST pin.
#endif
#if defined(USE_FREERTOS_SEMAPHORE)
  SemaphoreHandle_t xSemaphore;
#elif defined(USE_PTHREADS)
  pthread_mutex_t mutex;  ///< Synchronize access to this data structure.
#endif
};

/**
 * Reset the Si470X device.
 */
bool reset_device(struct si470x_port_t* port,
                  const gpio_pin_t si470x_rst_pin,
                  const gpio_pin_t si470x_gpio2_int_pin,
                  const gpio_pin_t i2c_sda_pin);

/**
 * Return the device type for given device.
 */
enum si470x_device_t get_device(const struct si470x_t* device);

/**
 * Is the STC interrupt enable bit set?
 */
bool get_stc_interrupts_enabled(const struct si470x_t* device);

/**
 * Does the device support RDS.
 */
bool get_supports_rds(const struct si470x_t* device);

/**
 * Does the device support RDS interrupts.
 */
bool get_supports_rds_int(const struct si470x_t* device);

/**
 * Is the RDS interrupt enable bit set?
 */
bool get_rds_interrupts_enabled(const struct si470x_t* device);

/**
 * Retrieve the signal strength (RSSI).
 */
int get_signal_strength(const struct si470x_t* device);

/**
 * Retrieve block A errors.
 */
uint16_t get_block_a_errors(const struct si470x_t* device);

/**
 * Retrieve block B errors.
 */
uint16_t get_block_b_errors(const struct si470x_t* device);

/**
 * Retrieve block C errors.
 */
uint16_t get_block_c_errors(const struct si470x_t* device);

/**
 * Retrieve block D errors.
 */
uint16_t get_block_d_errors(const struct si470x_t* device);

/**
 * Return the current channel.
 */
int get_channel(const struct si470x_t* device);

/**
 * Return the current frequency (Hz).
 */
uint32_t get_frequency(const struct si470x_t* device);

/**
 * Return the manufacturer.
 */
uint16_t get_manufacturer(const struct si470x_t* device);

/**
 * Return the device part.
 */
uint16_t get_part(const struct si470x_t* device);

/**
 * Return the chip firmware.
 */
uint16_t get_firmware(const struct si470x_t* device);

/**
 * Return the chip revision.
 */
uint16_t get_revision(const struct si470x_t* device);

/**
 * Convert frequency (in Hz) to channel.
 */
uint16_t frequency_to_channel(int frequency_hz, const struct si470x_t* device);
