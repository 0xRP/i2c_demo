#include <neorv32.h>
// #include <stdint.h>
#include <string.h>

// Helper I2C (TWI) functions using the NEORV32 driver
static inline void i2c_start(void) { neorv32_twi_generate_start(); }

static inline void i2c_stop(void) { neorv32_twi_generate_stop(); }

void print_hex_byte(uint8_t data) {

  static const char symbols[] = "0123456789abcdef";

  neorv32_uart0_putc(symbols[(data >> 4) & 15]);
  neorv32_uart0_putc(symbols[(data >> 0) & 15]);
}

static inline int i2c_write_byte(uint8_t byte) {
  // Send one byte; device returns ACK (0 = ACK, 1 = NACK)

  // execute transmission (blocking)
  int device_ack = neorv32_twi_trans(&byte, 0);

  neorv32_uart0_printf("DEBUG: RX data=0x");
  print_hex_byte(byte);

  // check device response?
  neorv32_uart0_printf(" Response: ");
  if (device_ack == 0) {
    neorv32_uart0_printf("ACK\n");
  } else {
    neorv32_uart0_printf("NACK\n");
  }

  return device_ack;
}

static inline uint8_t i2c_read_byte(int ack) {
  uint8_t byte = 0xFF;
  // For reads: send ACK for all but the last byte (ack=1) and NACK (ack=0)
  // after final byte.
  neorv32_twi_trans(&byte, ack);
  return byte;
}

// Multi-byte write: send device address and data bytes in one transaction.
static int i2c_write(uint8_t dev_addr, const uint8_t *data, uint8_t len) {
  i2c_start();
  // Write address: 7-bit address shifted left by 1 with write bit (0)
  if (i2c_write_byte((dev_addr << 1) | 0) != 0) {
    i2c_stop();
    return -1;
  }
  for (uint8_t i = 0; i < len; i++) {
    if (i2c_write_byte(data[i]) != 0) {
      i2c_stop();
      return -1;
    }
  }
  i2c_stop();
  return 0;
}

// Multi-byte read: send device read address then read 'len' bytes.
static int i2c_read(uint8_t dev_addr, uint8_t *data, uint8_t len) {
  i2c_start();
  // Read address: write 7-bit address shifted left by 1 with read bit (1)
  if (i2c_write_byte((dev_addr << 1) | 1) != 0) {
    i2c_stop();
    return -1;
  }
  for (uint8_t i = 0; i < len; i++) {
    // For all but the last byte, send ACK (ack=1); for last byte, send NACK
    // (ack=0)
    data[i] = i2c_read_byte((i < (len - 1)) ? 1 : 0);
  }
  i2c_stop();
  return 0;
}

// AHT20 sensor routines
#define AHT20_ADDRESS 0x38

// Begin: send initialization command and check calibration status.
int aht20_begin(void) {
  uint8_t init_cmd[3] = {0xBE, 0x08, 0x00};

  // Send initialization command to AHT20.
  if (i2c_write(AHT20_ADDRESS, init_cmd, 3) != 0) {
    return 0;
  }
  // Allow time for initialization and calibration.
  neorv32_cpu_delay_ms(500);

  // Read one status byte. Calibration bit (bit3) should be set.
  uint8_t status = 0;
  if (i2c_read(AHT20_ADDRESS, &status, 1) != 0) {
    return 0;
  }
  if (!(status & 0x08)) {
    // Calibration bit not set – sensor not ready.
    return 0;
  }
  return 1;
}

// Trigger a measurement and read 6 bytes from the sensor.
int aht20_measure(uint8_t *data, uint8_t len) {
  if (len < 6)
    return -1;
  uint8_t meas_cmd[3] = {0xAC, 0x33, 0x00};

  // Trigger measurement.
  if (i2c_write(AHT20_ADDRESS, meas_cmd, 3) != 0) {
    return -1;
  }
  // Wait for conversion (~100 ms is typical).
  neorv32_cpu_delay_ms(100);

  // Read 6 bytes: status, humidity (20-bit) and temperature (20-bit).
  if (i2c_read(AHT20_ADDRESS, data, 6) != 0) {
    return -1;
  }
  return 0;
}

// Parse humidity from 6-byte measurement data.
float aht20_getHumidity(const uint8_t *data) {
  // Bytes: data[1] (MSB), data[2], and upper 4 bits of data[3]
  uint32_t raw_hum = ((uint32_t)data[1] << 12) | ((uint32_t)data[2] << 4) |
                     ((data[3] >> 4) & 0x0F);
  return (raw_hum * 100.0f) / 1048575.0f;
}

// Parse temperature from 6-byte measurement data.
uint32_t aht20_getTemperature(const uint8_t *data) {
  // Lower 4 bits of data[3], data[4], and data[5]
  uint32_t raw_temp =
      (((uint32_t)(data[3] & 0x0F)) << 16) | ((uint32_t)data[4] << 8) | data[5];

  neorv32_uart0_printf("DEBUG: raw_temp = %u\n", raw_temp);
  neorv32_uart0_printf("DEBUG: (raw_temp*200)/1048576 = %u\n",
                       (raw_temp * 200) / 1048576 - 50);
  uint32_t ret = ((raw_temp * 200) / 1048576 - 50);
  return ret;
}

// Main program: similar to the Arduino example.
int main(void) {
  uint8_t meas_data[6];
  uint32_t temperature, humidity;

  // Setup runtime exception handling (optional debug).
  neorv32_rte_setup();

  // Check and initialize UART0.
  if (neorv32_uart0_available() == 0) {
    return 1;
  }
  neorv32_uart0_setup(19200, 0);
  neorv32_uart0_printf("AHT20 example\n");

  // Check if TWI unit is available.
  if (neorv32_twi_available() == 0) {
    neorv32_uart0_printf("ERROR! TWI controller not available!\n");
    return 1;
  }

  // Setup TWI – using a chosen prescaler, divider, and clock stretching
  // disabled.
  neorv32_twi_setup(CLK_PRSC_2048, 15, 0);

  // Initialize the AHT20 sensor.
  if (!aht20_begin()) {
    neorv32_uart0_printf(
        "AHT20 not detected. Please check wiring. Freezing.\n");
    while (1)
      ;
  }

  // Main loop: read and print temperature and humidity every 2 seconds.
  for (;;) {
    if (aht20_measure(meas_data, 6) != 0) {
      neorv32_uart0_printf("Measurement error\n");
    } else {
      temperature = aht20_getTemperature(meas_data);
      humidity = aht20_getHumidity(meas_data);
      neorv32_uart0_printf("T: %u C\t H: %u RH\n", temperature, humidity);
    }
    neorv32_cpu_delay_ms(2000);
  }

  return 0;
}
