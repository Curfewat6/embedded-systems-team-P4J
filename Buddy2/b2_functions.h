#ifndef B2_FUNCTIONS_H
#define B2_FUNCTIONS_H

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

#define PULSE_PIN 2
#define ADC_PIN 26
#define DIGI_PIN 7

#define PULSE_NUM 10
#define SAMPLE_SIZE 1000
#define SAMPLE_THRESHOLD 1000
#define DEBOUNCE_DELAY 300

typedef struct
{
    float total_pulse_width;
    float pulse_width_high_time;
    float pulse_width_low_time;
} pulse_t;

typedef struct
{
    float adc_frequency;
    float adc_rms_value;
    float adc_peak_to_peak_value;
    float adc_snr_value;
} adc_t;

void read_pulse(uint gpio, uint32_t events);
void print_pulses();

void read_adc(bool signal_above_threshold);
void print_adc();
float calculate_rms();
float calculate_peak_to_peak();
float calculate_snr();

void read_digi(uint gpio, uint32_t events);
bool measure_digi(struct repeating_timer *t);
float digi_calculate_frequency();
float calculate_duty_cycle();

void gpio_callback(uint gpio, uint32_t events);
bool get_all_values(struct repeating_timer *t);
void setup_pins();
void setup_adc();
void printTitle();

#endif
