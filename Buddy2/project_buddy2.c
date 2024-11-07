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
#define PULSE_NUM 10

#define ADC_BTN_PIN 22
#define ADC_PIN 26
#define ADC_MAX 4095.0f
#define REF_VOLTAGE 2.0f
#define ADC_SAMPLE_RATE 1.0f
#define SAMPLE_THRESHOLD 1000

#define DIGI_PIN 7

#define SAMPLE_SIZE 1000
#define BUFFER_SIZE 24408

typedef struct
{
    float high_duration;
    float low_duration;
} pulse_t;

volatile pulse_t pulses[PULSE_NUM + 2];
volatile bool pulse_read = false;
volatile uint8_t pulse_index = 0;
volatile uint64_t last_pulse_time = 0;
volatile uint64_t last_adc_peak_time = 0;
volatile uint64_t pulse_last_edge_time = 0;

volatile bool adc_timer = false;
volatile float adc_frequency;
volatile float adc_rms_value;
volatile float adc_peak_to_peak_value;
volatile float adc_snr_value;
volatile uint32_t buffer_index = 0;
volatile uint32_t sample_index = 0;
volatile uint32_t adc_last_edge_time = 0;
volatile uint32_t cycles_counted = 0;
volatile float adc_signal[SAMPLE_SIZE];
char adc_scan_buffer[BUFFER_SIZE];
#define DEBOUNCE_DELAY 300

volatile uint32_t last_rise_time = 0;
volatile uint32_t last_fall_time = 0;
volatile uint32_t high_time = 0;
volatile uint32_t low_time = 0;
volatile uint32_t period = 0;
volatile uint32_t total_width = 0;
volatile uint32_t measurement_count = 0;
volatile float pwm_frequency = 0.0f;
volatile float pwm_duty_cycle = 0.0f;
volatile bool new_cycle_complete = false;

void read_pulse(uint gpio, uint32_t events);

bool read_adc(struct repeating_timer *t);
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

    setup_pwm(0, 1000.0f, 0.5f);

    gpio_set_irq_enabled_with_callback(PULSE_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
    gpio_set_irq_enabled_with_callback(DIGI_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

    struct repeating_timer timer1, timer2;
    add_repeating_timer_ms(1000, measure_digi, NULL, &timer1);
    add_repeating_timer_ms(4000, get_all_values, NULL, &timer2);

    bool signal_above_threshold = false;

    while (1)
    {
        uint16_t adc_value = adc_read();

        if (adc_value > SAMPLE_THRESHOLD && !signal_above_threshold)
        {
            signal_above_threshold = true;
            uint32_t current_time = time_us_32();

            if (adc_last_edge_time != 0)
            {
                uint32_t interval = current_time - adc_last_edge_time;
                adc_signal[sample_index++] = interval;
                adc_last_edge_time = current_time;

                if (sample_index >= SAMPLE_SIZE)
                {
                    uint64_t total_interval = 0;
                    for (int i = 0; i < SAMPLE_SIZE; i++)
                    {
                        total_interval += adc_signal[i];
                    }

                    float average_interval = total_interval / (float)SAMPLE_SIZE;
                    adc_frequency = 1.0 / (average_interval / 1e6);
                    adc_rms_value = calculate_rms();
                    adc_peak_to_peak_value = calculate_peak_to_peak();
                    adc_snr_value = calculate_snr();

                    printf("\n--- Analog Signal Analyzer ---\n");
                    printf("Average Frequency: %.2f Hz\n", adc_frequency);
                    printf("RMS Voltage: %.2fV\tPeak-to-Peak Voltage: %.2fV\nSignal-to-Noise Ratio (SNR): %.2fdB\n", adc_rms_value, adc_peak_to_peak_value, adc_snr_value);

                    sample_index = 0;
                }
            }
            else
            {
                adc_last_edge_time = current_time;
            }

            sleep_us(DEBOUNCE_DELAY);
        }
        else if (adc_value < SAMPLE_THRESHOLD && signal_above_threshold)
        {
            signal_above_threshold = false;
        }
    }
    return 0;
}

bool get_all_values(struct repeating_timer *t)
{
    printf("\nGetting all values...\n");
    printf("Printing stored pulses (%i):\n", pulse_index - 1);
    for (int i = 1; i < pulse_index; i++)
    {
        printf("Pulse %d - High Duration: %0.3f us, Low Duration: %0.3f us\n",
               i, pulses[i].high_duration, pulses[i].low_duration);
    }
    printf("ADC: Average Frequency: %.2f Hz\n", adc_frequency);
    printf("PWM: Frequency: %.2f Hz\tDuty Cycle: %.2f\n", pwm_frequency, pwm_duty_cycle);

    return true;
}

void gpio_callback(uint gpio, uint32_t events)
{
    if (gpio == PULSE_PIN)
        read_pulse(gpio, events);
    else if (gpio == DIGI_PIN)
        read_digi(gpio, events);
    else
        printf("Invalid Input!!!\n");
}

void read_pulse(uint gpio, uint32_t events)
{
    static float high_duration_us = 0.0;
    static float low_duration_us = 0.0;
    uint64_t current_time = to_us_since_boot(get_absolute_time());

    if (current_time - pulse_last_edge_time < DEBOUNCE_DELAY)
    {
        return;
    }

    float pulse_duration_us = (float)(current_time - pulse_last_edge_time);
    pulse_last_edge_time = current_time;

    if (events & GPIO_IRQ_EDGE_RISE)
    {
        low_duration_us = pulse_duration_us;
        pulses[pulse_index].low_duration = low_duration_us;
    }

    if (events & GPIO_IRQ_EDGE_FALL)
    {
        high_duration_us = pulse_duration_us;
        pulses[pulse_index].high_duration = high_duration_us;

        if (low_duration_us > 0 && high_duration_us > 0)
        {
            pulse_index++;
            low_duration_us = 0;
            high_duration_us = 0;
        }

        if (pulse_index >= PULSE_NUM + 1)
        {
            gpio_set_irq_enabled_with_callback(PULSE_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, false, &gpio_callback);
            pulse_read = true;
        }
    }
}

void print_pulses()
{
    if (pulse_read == true)
    {
        printf("Printing stored pulses (%i):\n", pulse_index - 1);
        for (int i = 1; i < pulse_index; i++)
        {
            printf("Pulse %d - High Duration: %0.3f us, Low Duration: %0.3f us\n",
                   i + 1, pulses[i].high_duration, pulses[i].low_duration);
        }
    }
}

float calculate_rms()
{
    float sum_squares = 0.0f;
    for (int i = 0; i < SAMPLE_SIZE; i++)
    {
        sum_squares += adc_signal[i] * adc_signal[i];
    }

    return sqrt(sum_squares / SAMPLE_SIZE);
}

float calculate_peak_to_peak()
{
    float min_value = adc_signal[0];
    float max_value = adc_signal[0];
    for (int i = 1; i < SAMPLE_SIZE; i++)
    {
        if (adc_signal[i] < min_value)
            min_value = adc_signal[i];
        if (adc_signal[i] > max_value)
            max_value = adc_signal[i];
    }

    return max_value - min_value;
}

float calculate_snr()
{
    float signal_power = 0.0f;
    float noise_power = 0.0f;
    float mean_signal = 0.0f;

    for (int i = 0; i < SAMPLE_SIZE; i++)
        mean_signal += adc_signal[i];

    mean_signal /= SAMPLE_SIZE;

    for (int i = 0; i < SAMPLE_SIZE; i++)
    {
        float deviation = adc_signal[i] - mean_signal;
        signal_power += adc_signal[i] * adc_signal[i];
        noise_power += deviation * deviation;
    }

    signal_power /= SAMPLE_SIZE;
    noise_power /= SAMPLE_SIZE;

    if (noise_power == 0)
        return INFINITY;

    return (float)10 * log10(signal_power / noise_power);
}

void read_digi(uint gpio, uint32_t events)
{
    uint32_t current_time = time_us_32();

    if (events & GPIO_IRQ_EDGE_RISE)
    {
        if (last_fall_time != 0)
            low_time = current_time - last_fall_time;

        if (last_rise_time != 0)
        {
            period = current_time - last_rise_time;
            total_width += period;
            measurement_count++;

            new_cycle_complete = true;
        }

        last_rise_time = current_time;
    }
    else if (events & GPIO_IRQ_EDGE_FALL)
    {
        if (last_rise_time != 0)
            high_time = current_time - last_rise_time;

        last_fall_time = current_time;
    }
}

bool measure_digi(struct repeating_timer *t)
{
    if (new_cycle_complete)
    {
        pwm_frequency = digi_calculate_frequency();
        pwm_duty_cycle = calculate_duty_cycle();

        printf("\n--- Digital Signal Analyzer ---\n");
        printf("Frequency: %.2f Hz\tDuty Cycle: %.2f%%\n", pwm_frequency, pwm_duty_cycle);
        printf("Total Pulse Width: %u us\n", period);
        printf("PW High Time: %u us\tPW Low Time: %u us\n", high_time, low_time);

        new_cycle_complete = false;
        total_width = 0;
        measurement_count = 0;
    }
    return true;
}

float digi_calculate_frequency()
{
    uint32_t average_period = 0;
    average_period = total_width / measurement_count;

    if (average_period > 0)
        return 1000000.0f / average_period; // Calculate frequency in Hz

    return 0; // Avoid division by zero
}

float calculate_duty_cycle()
{
    if (period > 0)
    {
        float duty_cycle = ((float)high_time / (float)period) * 100.0f; // Calculate duty cycle percentage
        return duty_cycle;
    }
    return 0.0f; // Avoid division by zero
}

void setup_pins()
{
    gpio_init(PULSE_PIN);
    gpio_set_dir(PULSE_PIN, GPIO_IN);
    gpio_pull_down(PULSE_PIN);

    gpio_init(ADC_PIN);
    gpio_set_dir(ADC_PIN, GPIO_IN);
    gpio_pull_up(ADC_PIN);

    gpio_init(ADC_BTN_PIN);
    gpio_set_dir(ADC_BTN_PIN, GPIO_IN);
    gpio_pull_up(ADC_BTN_PIN);

    gpio_init(DIGI_PIN);
    gpio_set_dir(DIGI_PIN, GPIO_IN);
    gpio_pull_down(DIGI_PIN);
}

void setup_adc()
{
    adc_init();
    adc_gpio_init(ADC_PIN);
    adc_select_input(0);
}

void printTitle()
{
    printf("===========================\n");
    printf("= INF2004 Project Buddy 2 =\n");
    printf("===========================\n");
}
