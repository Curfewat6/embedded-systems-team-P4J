/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include <stdio.h>
#include "pico/stdlib.h"

int main() {
stdio_init_all();    
#define PICO_DEFAULT_LED_PIN 2
#ifdef PICO_DEFAULT_LED_PIN
    
    uint a = 1;
    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    while (true) {
        gpio_put(LED_PIN, 1);
        a = a<<1;
        sleep_ms(a);
        printf("%i\n",a);
        gpio_put(LED_PIN, 0);
        a = a<<1;
        sleep_ms(a);
        printf("%i\n",a);
        sleep_ms(1000);
    
  if(a>=2048) a=1;
    }
#else
#warning blink example requires a board with a regular LED
#endif
}
