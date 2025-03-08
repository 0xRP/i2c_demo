#include "i2c_demo.h"

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
  uint32_t temperature;
  uint32_t humidity;

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
    if (aht20_measure(meas_data, 6) != ACK) {
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