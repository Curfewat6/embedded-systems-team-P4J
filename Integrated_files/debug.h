#ifndef DEBUGGER_H
#define DEBUGGER_H

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include <stdio.h>

// Pin definitions
#define SWCLK_PIN 2  // GPIO pin for SWCLK
#define SWDIO_PIN 3  // GPIO pin for SWDIO

// Constants for debug commands and addresses
#define PWR_DBG 0x50000000  // Command to enable debug power
#define DHCSR 0xE000EDF0     // DHCSR address
#define DBGKEY 0xA05F0000    // Debug key


// Function declarations
void pulse_swclk(int cycles);
void swdSetWriteMode();
void swdSetReadMode();
bool swdReadBit();
void swdWriteBit(int bit);
void swdWriteBits(uint32_t data, int bits);
void swd_line_reset();
void jtag_to_swd_config();
void swd_send_request(uint8_t request);
void turnaround();
uint32_t swd_read_idcode();
void swd_read_ack();
void swd_init();
void swd_send_wakeup_sequence();
void powerDebug();
void checkClear();
void resumeCpu();
void initialise_debugger();

#endif // DEBUGGER_H
