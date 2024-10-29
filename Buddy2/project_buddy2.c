#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <string.h>

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/timer.h"

#define PULSE_PIN 2
#define ADC_BTN_PIN 21
#define ADC_PIN 26
#define DIGI_PIN 7
#define DEBOUNCE_DELAY_MS 50

#define PULSE_NUM 10

#define ADC_MAX 4095.0f
#define REF_VOLTAGE 3.3f
#define M_PI 3.14159265358979323846

#define BUFFER_SIZE 10240
#define INTERRUPT_INTERVAL 600
#define SAMPLE_INTERVAL 25
#define SAMPLE_SIZE 10

volatile uint32_t pulse_times[PULSE_NUM];
volatile uint32_t pulse_widths[PULSE_NUM];
volatile uint32_t last_rising_edge = 0;
volatile uint32_t pulse_counter = 0;

volatile bool adc_timer = false;
volatile bool adc_cycle = true;
volatile float ad_signal[SAMPLE_SIZE];
volatile int sample_index = 0;
char adc_scan_buffer[BUFFER_SIZE];
size_t buffer_index = 0;

volatile uint32_t rising_time;
volatile uint32_t falling_time;
volatile uint32_t pulse_width_high;
volatile uint32_t pulse_width_low;
volatile uint32_t current_time;

void gpio_callback(uint gpio, uint32_t events);

void read_pulse(uint gpio, uint32_t events);

bool read_adc(struct repeating_timer *t);
float calculate_rms();
float calculate_peak_to_peak();
float calculate_snr();

void read_digi(uint gpio, uint32_t events);

void setup_pins();
void setup_adc();
void printTitle();

int main()
{
    stdio_init_all();

    sleep_ms(4000);

    printTitle();
    setup_pins();
    setup_adc();

    gpio_set_irq_enabled_with_callback(PULSE_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
    gpio_set_irq_enabled_with_callback(ADC_BTN_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
    gpio_set_irq_enabled_with_callback(DIGI_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

    struct repeating_timer timer1;
    add_repeating_timer_ms(600, read_adc, NULL, &timer1);

    while (1)
    {
        tight_loop_contents();
    }
    return 0;
}

void gpio_callback(uint gpio, uint32_t events)
{
    static uint32_t last_time = 0;
    uint32_t current_time = to_ms_since_boot(get_absolute_time());

    if (current_time - last_time < DEBOUNCE_DELAY_MS)
        return;
    last_time = current_time;

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
        else if (events & GPIO_IRQ_EDGE_RISE)
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
    uint32_t current_time = time_us_64();

    if (events & GPIO_IRQ_EDGE_RISE)
    {
        if (pulse_counter == PULSE_NUM)
        {
            pulse_counter = 0;
            last_rising_edge = 0;
        }

        pulse_times[pulse_counter] = current_time;
        printf("Pulse %d: %llu us", pulse_counter + 1, pulse_times[pulse_counter]);

        if (last_rising_edge != 0)
        {
            pulse_widths[pulse_counter - 1] = current_time - last_rising_edge;
            printf(", Width: %llu us", pulse_widths[pulse_counter - 1]);
        }
        printf("\n");

        last_rising_edge = current_time;
        pulse_counter++;
    }
}

bool read_adc(struct repeating_timer *t)
{
    if (adc_cycle)
    {
        sample_index = 0;
        buffer_index = 0;
        memset(adc_scan_buffer, 0, BUFFER_SIZE);
        adc_cycle = false;
    }

    if (adc_timer)
    {
        uint16_t adc_value = adc_read();
        float voltage = (adc_value / ADC_MAX) * REF_VOLTAGE;

        ad_signal[sample_index++] = voltage;
        buffer_index += snprintf(adc_scan_buffer + buffer_index, BUFFER_SIZE - buffer_index,
                                 "- %d ", adc_value);

        if (sample_index >= SAMPLE_SIZE)
        {
            printf("--- Analog Signal Analysis ---\nADC values captured: %s\n", adc_scan_buffer);

            float rms_value = calculate_rms();
            float peak_to_peak_value = calculate_peak_to_peak();
            float snr_value = calculate_snr();

            printf("RMS Voltage: %.3fV\tPeak-to-Peak Voltage: %.3fV\tSignal-to-Noise Ratio (SNR): %.2fdB\n",
                   rms_value, peak_to_peak_value, snr_value);
            adc_cycle = true;
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

    if (noise_power == 0)
        return INFINITY;

    return (float)10 * log10(signal_power / noise_power);
}

void read_digi(uint gpio, uint32_t events)
{
    if (events & GPIO_IRQ_EDGE_FALL)
    {
        falling_time = to_ms_since_boot(get_absolute_time());
        pulse_width_high = falling_time - rising_time;
        printf("--- Digital Signal Analyzer ---\nReading Digital Signal...\n");
    }

    if (events & GPIO_IRQ_EDGE_RISE)
    {
        rising_time = to_ms_since_boot(get_absolute_time());
        pulse_width_low = rising_time - falling_time;

        float T_high = pulse_width_high / 1000.0f;
        float T_low = pulse_width_low / 1000.0f;

        float T_total = T_high + T_low;
        float frequency = 1.0f / T_total;
        float duty_cycle = (T_high / T_total) * 100.0f;

        printf("PWM Frequency: %.3fHz, Duty Cycle: %.2f%%\n", frequency, duty_cycle);
        printf("Measured Pulse Width: %.2fus\n", T_total);
    }
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
    gpio_pull_up(DIGI_PIN);

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
