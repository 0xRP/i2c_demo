#include <stdio.h>
#include <stdint.h>
#include <stdarg.h>
#include "i2c_demo.h"

// --- Emulator State Variables ---
#define MODE_NONE  0
#define MODE_INIT  1
#define MODE_MEAS  2

#define CLK_PRSC_2048 6 /**< CPU_CLK / 2048 */

// --- Overridden NEORV32 Functions for Emulation ---
void neorv32_twi_generate_start(void);
void neorv32_twi_generate_stop(void);
int neorv32_twi_trans(uint8_t *data, int ack);
void neorv32_uart0_putc(char c); 
void neorv32_uart0_printf(const char *format, ...);
void neorv32_cpu_delay_ms(int ms);
int neorv32_uart0_available(void);
void neorv32_uart0_setup(int baud, int mode);
int neorv32_twi_available(void);
void neorv32_twi_setup(int prescaler, int divider, int stretch) ;
void neorv32_rte_setup(void);