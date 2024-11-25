#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"

#include "b2_functions.h"

void setup_pwm(uint gpio, float freq, float duty_cycle)
{
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(gpio);
    float clock_freq = 125000000.0f;
    uint16_t divider = clock_freq / (freq * 65536);
    pwm_set_clkdiv(slice_num, divider);
    pwm_set_wrap(slice_num, 65535);
    pwm_set_gpio_level(gpio, (uint16_t)(duty_cycle * 65536));
    pwm_set_enabled(slice_num, true);
}

int main()
{
    stdio_init_all();
    sleep_ms(3000);
    printTitle();
    setup_pins();
    setup_adc();

    setup_pwm(0, 600.0f, 0.5f);

    struct repeating_timer timer1, timer2;
    add_repeating_timer_ms(1000, measure_digi, NULL, &timer1);
    add_repeating_timer_ms(4000, get_all_values, NULL, &timer2);

    bool signal_above_threshold = false;

    while (1)
        read_adc(signal_above_threshold);

    return 0;
}