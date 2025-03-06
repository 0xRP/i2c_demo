#include <neorv32.h>
#include <string.h>


/*===========================================================
  Ejercicio 1: Funciones de Inicio y Parada del I2C
  -----------------------------------------------------------
  Se definen funciones inline para generar las condiciones de 
  inicio y parada en el bus I2C usando el driver NEORV32.
=============================================================*/
static inline void i2c_start(void) {
  // Genera la condición de inicio en I2C.
  neorv32_twi_generate_start();
}

static inline void i2c_stop(void) {
  // Genera la condición de parada en I2C.
  neorv32_twi_generate_stop();
}

/*===========================================================
  Ejercicio 2: Impresión de un Byte en Formato Hexadecimal
  -----------------------------------------------------------
  Se implementa una función para imprimir un byte en formato 
  hexadecimal usando la UART.
=============================================================*/
void print_hex_byte(uint8_t data) {
  static const char symbols[] = "0123456789abcdef";
  
  // Imprime el nibble superior (4 bits).
  neorv32_uart0_putc(symbols[(data >> 4) & 0x0F]);
  // Imprime el nibble inferior (4 bits).
  neorv32_uart0_putc(symbols[data & 0x0F]);
}

/*===========================================================
  Ejercicio 3: Función para Escribir un Byte en I2C
  -----------------------------------------------------------
  Se envía un byte por I2C y se verifica el acuse de recibo (ACK/NACK)
  mostrando mensajes de depuración.
=============================================================*/
static inline int i2c_write_byte(uint8_t byte) {
  // Transmitir el byte y almacenar el acuse de recibo del dispositivo.
  int device_ack = neorv32_twi_trans(&byte, 0);

  // Depuración: imprimir el byte transmitido en hexadecimal.
  neorv32_uart0_printf("DEBUG: RX data=0x");
  print_hex_byte(byte);

  // Imprimir la respuesta: "ACK" para 0, "NACK" para cualquier otro valor.
  neorv32_uart0_printf(" Response: ");
  if (device_ack == 0) {
    neorv32_uart0_printf("ACK\n");
  } else {
    neorv32_uart0_printf("NACK\n");
  }

  return device_ack;
}

/*===========================================================
  Ejercicio 4: Función para Leer un Byte en I2C
  -----------------------------------------------------------
  Se implementa la función para leer un byte del bus I2C, 
  utilizando un parámetro para configurar el acuse.
=============================================================*/
static inline uint8_t i2c_read_byte(int ack) {
  // Inicializa la variable con un valor por defecto.
  uint8_t byte = 0xFF;
  // Lee el byte utilizando la función de transmisión con el parámetro ack.
  neorv32_twi_trans(&byte, ack);
  return byte;
}

/*===========================================================
  Ejercicio 5: Escritura de Múltiples Bytes en I2C
  -----------------------------------------------------------
  Se escribe una función que envía múltiples bytes, empezando 
  por la dirección del dispositivo, y luego los datos.
=============================================================*/
static int i2c_write(uint8_t dev_addr, const uint8_t *data, uint8_t len) {
  i2c_start();
  
  // Enviar la dirección del dispositivo (7 bits desplazados y bit de escritura 0).
  if (i2c_write_byte((dev_addr << 1) | 0) != 0) {
    i2c_stop();
    return -1;
  }
  
  // Enviar cada uno de los bytes de datos.
  for (uint8_t i = 0; i < len; i++) {
    if (i2c_write_byte(data[i]) != 0) {
      i2c_stop();
      return -1;
    }
  }
  
  i2c_stop();
  return 0;
}

/*===========================================================
  Ejercicio 6: Lectura de Múltiples Bytes en I2C
  -----------------------------------------------------------
  Se implementa la función para leer múltiples bytes desde un 
  dispositivo I2C, enviando ACK para todos excepto el último.
=============================================================*/
static int i2c_read(uint8_t dev_addr, uint8_t *data, uint8_t len) {
  i2c_start();
  
  // Enviar la dirección del dispositivo con el bit de lectura (1).
  if (i2c_write_byte((dev_addr << 1) | 1) != 0) {
    i2c_stop();
    return -1;
  }
  
  // Leer 'len' bytes: enviar ACK para todos excepto el último.
  for (uint8_t i = 0; i < len; i++) {
    data[i] = i2c_read_byte((i < (len - 1)) ? 1 : 0);
  }
  
  i2c_stop();
  return 0;
}

/*===========================================================
  Ejercicio 7: Inicialización del Sensor AHT20
  -----------------------------------------------------------
  Se inicializa el sensor AHT20 enviando el comando de 
  inicialización y comprobando el estado de calibración.
=============================================================*/
#define AHT20_ADDRESS 0x38

int aht20_begin(void) {
  uint8_t init_cmd[3] = {0xBE, 0x08, 0x00};

  // Enviar el comando de inicialización al sensor.
  if (i2c_write(AHT20_ADDRESS, init_cmd, 3) != 0) {
    return 0;
  }
  
  // Esperar para la inicialización y calibración.
  neorv32_cpu_delay_ms(500);

  // Leer el byte de estado para comprobar la calibración.
  uint8_t status = 0;
  if (i2c_read(AHT20_ADDRESS, &status, 1) != 0) {
    return 0;
  }
  
  // Verificar que el bit de calibración (bit 3) esté activado.
  if (!(status & 0x08)) {
    return 0;
  }
  
  return 1;
}

/*===========================================================
  Ejercicio 8: Disparar una Medición
  -----------------------------------------------------------
  Se implementa la función que dispara una medición, 
  espera la conversión y lee 6 bytes de datos.
=============================================================*/
int aht20_measure(uint8_t *data, uint8_t len) {
  if (len < 6)
    return -1;
  
  uint8_t meas_cmd[3] = {0xAC, 0x33, 0x00};

  // Enviar el comando para disparar la medición.
  if (i2c_write(AHT20_ADDRESS, meas_cmd, 3) != 0) {
    return -1;
  }
  
  // Esperar a que la conversión se complete (~100 ms).
  neorv32_cpu_delay_ms(100);

  // Leer 6 bytes de datos de medición.
  if (i2c_read(AHT20_ADDRESS, data, 6) != 0) {
    return -1;
  }
  
  return 0;
}

/*===========================================================
  Ejercicio 9: Interpretación de los Datos de Humedad
  -----------------------------------------------------------
  Se extrae y convierte el valor en bruto de 20 bits de 
  humedad a porcentaje.
=============================================================*/
float aht20_getHumidity(const uint8_t *data) {
  uint32_t raw_hum = ((uint32_t)data[1] << 12) | ((uint32_t)data[2] << 4) |
                     ((data[3] >> 4) & 0x0F);
  return (raw_hum * 100.0f) / 1048575.0f;
}

/*===========================================================
  Ejercicio 10: Interpretación de los Datos de Temperatura
  -----------------------------------------------------------
  Se extrae y convierte el valor en bruto de 20 bits de 
  temperatura a grados Celsius, incluyendo mensajes de 
  depuración para el valor obtenido.
=============================================================*/
uint32_t aht20_getTemperature(const uint8_t *data) {
  uint32_t raw_temp = (((uint32_t)(data[3] & 0x0F)) << 16) | ((uint32_t)data[4] << 8) | data[5];

  // Mensajes de depuración para el valor en bruto.
  neorv32_uart0_printf("DEBUG: raw_temp = %u\n", raw_temp);
  neorv32_uart0_printf("DEBUG: (raw_temp*200)/1048576 = %u\n", (raw_temp * 200) / 1048576 - 50);
  
  uint32_t ret = ((raw_temp * 200) / 1048576 - 50);
  return ret;
}

/*===========================================================
  Ejercicio 11: Programa Principal
  -----------------------------------------------------------
  Se integran todas las funciones: se configura la UART y
  el I2C, se inicializa el sensor AHT20 y se entra en un 
  bucle infinito que lee y muestra los datos de temperatura
  y humedad cada 2 segundos.
=============================================================*/
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
