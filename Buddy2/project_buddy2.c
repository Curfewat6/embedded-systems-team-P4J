#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <string.h>

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"

#define PULSE_PIN 2
#define ADC_BTN_PIN 21
#define ADC_PIN 26
#define DIGI_PIN 7
#define PULSE_NUM 10
#define BUFFER_MS 1000
#define ADC_MAX 4095.0f
#define REF_VOLTAGE 3.3f
#define SAMPLE_SIZE 10

volatile uint32_t prev_rise_time = 0;
volatile uint32_t prev_fall_time = 0;
volatile uint32_t pulse_count = 0;
volatile bool new_pulse_detected = false;

volatile bool adc_timer = false;
volatile float ad_signal[SAMPLE_SIZE];
volatile int sample_index = 0;
char adc_scan_buffer[10240];

volatile uint32_t last_rise_time = 0;
volatile uint32_t last_fall_time = 0;
volatile uint32_t high_time = 0;
volatile uint32_t low_time = 0;
volatile uint32_t period = 0;
volatile bool new_cycle_complete = false;

void gpio_callback(uint gpio, uint32_t events);
void read_pulse(uint gpio, uint32_t events);
bool read_adc(struct repeating_timer *t);
float calculate_rms();
float calculate_peak_to_peak();
float calculate_snr();
void read_digi(uint gpio, uint32_t events);
float calculate_frequency(uint32_t period);
float calculate_duty_cycle(uint32_t high_time, uint32_t period);
bool measure_digi(struct repeating_timer *t);
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
    setup_pwm(0, 400.0f, 0.5f);

    gpio_set_irq_enabled_with_callback(PULSE_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
    gpio_set_irq_enabled_with_callback(ADC_BTN_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
    gpio_set_irq_enabled_with_callback(DIGI_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

    struct repeating_timer timer1, timer2;
    add_repeating_timer_ms(500, read_adc, NULL, &timer1);
    add_repeating_timer_ms(1000, measure_digi, NULL, &timer2);

    while (1)
    {
        tight_loop_contents();
    }
    return 0;
}

void gpio_callback(uint gpio, uint32_t events)
{
    if (gpio == PULSE_PIN)
    {
        read_pulse(gpio, events);
    }
    else if (gpio == ADC_BTN_PIN)
    {
        if (events & GPIO_IRQ_EDGE_FALL)
        {
            printf("Reading ADC ...\n");
            adc_timer = true;
        }
        else
        {
            printf("Reading ADC Stopped.\n");
            adc_timer = false;
        }
    }
    else if (gpio == DIGI_PIN)
    {
        read_digi(gpio, events);
    }
    else
    {
        printf("Invalid Input!!!\n");
    }
}

void read_pulse(uint gpio, uint32_t events)
{
    uint32_t current_time = time_us_32();

    if (events & GPIO_IRQ_EDGE_RISE)
    {
        if (prev_fall_time != 0)
        {
            uint32_t pulse_width = current_time - prev_fall_time; // Calculate pulse width
            printf("Pulse %d: Time: %u us, Width: %u us\n", pulse_count + 1, current_time, pulse_width);
            new_pulse_detected = true; // Set the flag for the main loop to process
        }
        prev_rise_time = current_time; // Update previous rising edge time
        pulse_count++;
    }
    else if (events & GPIO_IRQ_EDGE_FALL)
        prev_fall_time = current_time;

    if (pulse_count >= PULSE_NUM)
        gpio_set_irq_enabled_with_callback(PULSE_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, false, &gpio_callback);
}

bool read_adc(struct repeating_timer *t)
{
    if (adc_timer)
    {
        uint16_t adc_value = adc_read();
        float voltage = (adc_value / ADC_MAX) * REF_VOLTAGE;
        ad_signal[sample_index++] = voltage;

        if (sample_index >= SAMPLE_SIZE)
        {
            char adc_scan_buffer[10240] = {0}; // Reset buffer
            printf("--- Analog Signal Analysis ---\n");
            float rms_value = calculate_rms();
            float peak_to_peak_value = calculate_peak_to_peak();
            float snr_value = calculate_snr();

            printf("RMS Voltage: %.3fV\tPeak-to-Peak Voltage: %.3fV\tSignal-to-Noise Ratio (SNR): %.2fdB\n",
                   rms_value, peak_to_peak_value, snr_value);
            sample_index = 0; // Reset for next sample
        }
    }
    return true;
}

float calculate_rms()
{
    float sum_squares = 0.0f;
    for (int i = 0; i < SAMPLE_SIZE; i++)
    {
        sum_squares += ad_signal[i] * ad_signal[i];
    }
    return sqrt(sum_squares / SAMPLE_SIZE);
}

float calculate_peak_to_peak()
{
    float min_value = ad_signal[0];
    float max_value = ad_signal[0];
    for (int i = 1; i < SAMPLE_SIZE; i++)
    {
        if (ad_signal[i] < min_value)
            min_value = ad_signal[i];
        if (ad_signal[i] > max_value)
            max_value = ad_signal[i];
    }
    return max_value - min_value;
}

float calculate_snr()
{
    float signal_power = 0.0f;
    float noise_power = 0.0f;
    float mean_signal = 0.0f;

    for (int i = 0; i < SAMPLE_SIZE; i++)
    {
        mean_signal += ad_signal[i];
    }
    mean_signal /= SAMPLE_SIZE;

    for (int i = 0; i < SAMPLE_SIZE; i++)
    {
        float deviation = ad_signal[i] - mean_signal;
        signal_power += ad_signal[i] * ad_signal[i];
        noise_power += deviation * deviation;
    }

    signal_power /= SAMPLE_SIZE;
    noise_power /= SAMPLE_SIZE;

    return (noise_power == 0) ? INFINITY : (float)10 * log10(signal_power / noise_power);
}

void read_digi(uint gpio, uint32_t events)
{
    uint32_t current_time = time_us_32();

    if (events & GPIO_IRQ_EDGE_RISE)
    {
        if (last_fall_time != 0)
        {
            high_time = current_time - last_fall_time;
            period = current_time - last_fall_time;
            new_cycle_complete = true;
        }
        last_rise_time = current_time;
    }
    else if (events & GPIO_IRQ_EDGE_FALL)
    {
        last_fall_time = current_time;
        if (last_rise_time != 0)
        {
            low_time = current_time - last_rise_time;
            period += low_time;
        }
    }
}

bool measure_digi(struct repeating_timer *t)
{
    if (new_cycle_complete)
    {
        float frequency = calculate_frequency(period);
        float duty_cycle = calculate_duty_cycle(high_time, period);
        float total_pulse_width = high_time + low_time;

        printf("\n--- Digital Signal Analyzer ---\n");
        printf("Frequency: %.2f Hz\n", frequency);
        printf("Duty Cycle: %.2f%%\n", duty_cycle);
        printf("Total Pulse Width: %.2f us\n", total_pulse_width);

        new_cycle_complete = false;
    }
    return true;
}

float calculate_frequency(uint32_t period)
{
    return (period == 0) ? 0.0f : 1e6 / (float)period;
}

float calculate_duty_cycle(uint32_t high_time, uint32_t period)
{
    return (period == 0) ? 0.0f : ((float)high_time / (float)period) * 100.0f;
}

void setup_pins()
{
    gpio_init(ADC_PIN);
    gpio_set_dir(ADC_PIN, GPIO_IN);
    gpio_pull_up(ADC_PIN);

    gpio_init(ADC_BTN_PIN);
    gpio_set_dir(ADC_BTN_PIN, GPIO_IN);
    gpio_pull_up(ADC_BTN_PIN);

    gpio_init(DIGI_PIN);
    gpio_set_dir(DIGI_PIN, GPIO_IN);
    gpio_pull_down(DIGI_PIN);

    gpio_init(PULSE_PIN);
    gpio_set_dir(PULSE_PIN, GPIO_IN);
    gpio_pull_down(PULSE_PIN);
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
