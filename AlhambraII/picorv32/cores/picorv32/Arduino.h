#ifndef Arduino_h
#define Arduino_h

#include <stdint.h>
#include <stdbool.h>

// picorv32: work-a-round; for size_t only - original Arduino.h does NOT have this!
#include <cstddef>

#define reg_spictrl (*(volatile uint32_t*)0x02000000)
#define reg_uart_clkdiv (*(volatile uint32_t*)0x02000004)
#define reg_uart_data (*(volatile uint32_t*)0x02000008)
#define reg_outp (*(volatile uint32_t*)0x03000000)  // contains reg_leds also
//#define reg_inp_zero (*(volatile uint32_t*)0x04000000)
#define reg_inp (*(volatile uint32_t*)0x05000000)

#define F_CPU       12000000    // 12MHz

#define clk_div_s   (F_CPU)          // 1s
#define clk_div_ms  (F_CPU/1000)     // 1ms
#define clk_div_us  (F_CPU/1000000)  // 1us

#define HIGH 0x1
#define LOW  0x0

#define INPUT 0x0
#define OUTPUT 0x1
#define INPUT_PULLUP 0x2

typedef uint8_t byte;

void pinMode(uint8_t, uint8_t);
void digitalWrite(uint8_t, uint8_t);
int digitalRead(uint8_t);
int analogRead(uint8_t);

void print(const char*);

unsigned long millis(void);
unsigned long micros(void);
void delay(unsigned long);

void setup(void);
void loop(void);

#ifdef __cplusplus
//#include "WCharacter.h"
//#include "WString.h"
#include "HardwareSerial.h"
//#include "USBAPI.h"
#if defined(HAVE_HWSERIAL0) && defined(HAVE_CDCSERIAL)
#error "Targets with both UART0 and CDC serial not supported"
#endif

#endif

#include "pins_arduino.h"

#endif
