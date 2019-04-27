#include <stdint.h>
#include <stdbool.h>

#define reg_spictrl (*(volatile uint32_t*)0x02000000)
#define reg_uart_clkdiv (*(volatile uint32_t*)0x02000004)
#define reg_uart_data (*(volatile uint32_t*)0x02000008)
#define reg_leds (*(volatile uint32_t*)0x03000000)

#define clk_div_s   12000000  // 1s
#define clk_div_ms  12000     // 1ms
#define clk_div_us  12        // 1us

void print(const char*);
void delay(uint32_t);

void setup(void);
void loop(void);

// --- --- --- make blink working
#define HIGH 0x1
#define LOW  0x0

#define INPUT 0x0
#define OUTPUT 0x1
#define INPUT_PULLUP 0x2

#define LED_BUILTIN 0x0

void pinMode(uint8_t, uint8_t);
void digitalWrite(uint8_t, uint8_t);
