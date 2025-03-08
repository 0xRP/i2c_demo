#include "emulator.h"

static int emulator_mode = MODE_NONE;
static int is_write_transaction = 0;
static int is_read_transaction = 0;
static int write_byte_count = 0;
static int read_byte_count = 0;

#include <stdint.h>
#include <stdbool.h>

#define ACK  0
#define NACK 1

// Define a simple state machine for the I2C transaction:
typedef enum {
  IDLE,
  RECEIVING_COMMAND,
  SENDING_DATA
} twi_state_t;

static twi_state_t twi_state = IDLE;
static uint8_t command_buffer[3];
static int command_index = 0;
static int measurement_index = 0;

// Flags to simulate sensor status:
static bool sensor_calibrated    = false;
static bool measurement_triggered = false;

// Hardcoded measurement data to be sent byte-by-byte on read.
// Make sure the first byte has the calibration bit (bit3, 0x08) set.
static uint8_t emulated_measurement[6] = { 0x08, 0xaa, 0xaa, 0xab, 0xbb, 0xbb };

int neorv32_twi_trans(uint8_t *data, int ack) {
  // Detect a new transaction by checking for the device address.
  // The master sends (AHT20_ADDRESS<<1)|R/W:
  //   - 0x70: write (when R/W = 0)
  //   - 0x71: read  (when R/W = 1)
  if (*data == 0x70 || *data == 0x71) {
    // New transaction: reset counters and select mode.
    command_index   = 0;
    measurement_index = 0;
    if (*data == 0x70) {
      twi_state = RECEIVING_COMMAND;
    }
    else { // 0x71 -> read mode
      twi_state = SENDING_DATA;
    }
    return ACK;
  }

  // In write transactions, accumulate command bytes.
  if (twi_state == RECEIVING_COMMAND) {
    command_buffer[command_index++] = *data;
    // Once three command bytes have been received, process them.
    if (command_index == 3) {
      // 1. Respond to initialization command: {0xBE, 0x08, 0x00}
      if ((command_buffer[0] == 0xBE) &&
          (command_buffer[1] == 0x08) &&
          (command_buffer[2] == 0x00)) {
        sensor_calibrated = true;
      }
      // 2. Respond to measurement trigger: {0xAC, 0x33, 0x00}
      else if ((command_buffer[0] == 0xAC) &&
               (command_buffer[1] == 0x33) &&
               (command_buffer[2] == 0x00)) {
        if (sensor_calibrated) {
          measurement_triggered = true;
          // (Optionally, you could modify emulated_measurement here to simulate new data.)
        }
      }
      // 3. For any other commands, simply ignore the data.
      // End the command phase (in a real system, a STOP condition resets the transaction).
      twi_state = IDLE;
    }
    // Always respond with ACK for write bytes.
    return ACK;
  }

  // In read transactions, send the hardcoded measurement data one byte at a time.
  if (twi_state == SENDING_DATA) {
    if (measurement_index < 6) {
      *data = emulated_measurement[measurement_index++];
    }
    else {
      *data = 0xFF;  // If read beyond 6 bytes, return a default value.
    }
    // Optionally, once all data is sent, return to idle.
    if (measurement_index >= 6) {
      twi_state = IDLE;
    }
    return ACK;
  }

  // Default: if not in a specific state, simply return ACK.
  return ACK;
}


// --- Overridden NEORV32 Functions for Emulation ---
void neorv32_twi_generate_start(void) {
    is_write_transaction = 1;
    write_byte_count = 0;
    read_byte_count = 0;
    is_read_transaction = 0;
    printf("em. I2C Start generated.\n");
}

void neorv32_twi_generate_stop(void) {
    is_write_transaction = 0;
    is_read_transaction = 0;
    emulator_mode = MODE_NONE;
    printf("em. I2C Stop generated.\n");
}


void neorv32_uart0_putc(char c) {
    putchar(c);
}

void neorv32_uart0_printf(const char *format, ...) {
    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);
}

void neorv32_cpu_delay_ms(int ms) {
    printf("em. Delay %d ms\n", ms);
}

int neorv32_uart0_available(void) { return 1; }
void neorv32_uart0_setup(int baud, int mode) {
    printf("em. UART0 setup with baud: %d, mode: %d\n", baud, mode);
}
int neorv32_twi_available(void) { return 1; }
void neorv32_twi_setup(int prescaler, int divider, int stretch) {
    printf("em. TWI setup with prescaler: %d, divider: %d, stretch: %d\n", prescaler, divider, stretch);
}
void neorv32_rte_setup(void) {
    printf("em. Runtime Exception setup done.\n");
}

// --- Emulator Main ---  
int main(void) {
    uint8_t meas_data[6];
    uint32_t temperature, humidity;
  
    // Configurar manejo de excepciones en tiempo de ejecución.
    neorv32_rte_setup();
  
    // Verificar e inicializar la UART0.
    if (neorv32_uart0_available() == 0) {
      return 1;
    }
    neorv32_uart0_setup(19200, 0);
    neorv32_uart0_printf("AHT20 example\n");
  
    // Verificar que el controlador TWI (I2C) esté disponible.
    if (neorv32_twi_available() == 0) {
      neorv32_uart0_printf("ERROR! TWI controller not available!\n");
      return 1;
    }
  
    // Configurar TWI con prescaler, divisor y sin stretching de reloj.
    neorv32_twi_setup(CLK_PRSC_2048, 15, 0);
  
    // Inicializar el sensor AHT20.
    if (!aht20_begin()) {
      neorv32_uart0_printf("AHT20 not detected. Please check wiring. Freezing.\n");
      while (1)
        ;
    }
  
    // Bucle principal: leer e imprimir datos cada 2 segundos.
    //for (;;) {
      if (aht20_measure(meas_data, 6) != 0) {
        neorv32_uart0_printf("Measurement error\n");
      } else {
        temperature = aht20_getTemperature(meas_data);
        humidity = aht20_getHumidity(meas_data);
        neorv32_uart0_printf("T: %u C\t H: %u RH\n", temperature, humidity);
      }
      neorv32_cpu_delay_ms(2000);
    //}
  
    return 0;
}
