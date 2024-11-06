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

#define ADC_PIN 26
#define ADC_SAMPLE_SIZE 500

#define PWM_PIN 7
#define PWM_SAMPLE_SIZE 500

typedef struct
{
    uint32_t rise_times[PULSE_NUM];   // Array to store rise times
    uint32_t fall_times[PULSE_NUM];   // Array to store fall times
    uint32_t pulse_widths[PULSE_NUM]; // Array to store pulse widths
    uint32_t count;                   // Pulse count
    bool sampling_complete;           // Flag for sampling completion
} PulseData;

typedef struct
{
    float signal_intervals[ADC_SAMPLE_SIZE];
    size_t sample_count;
    float rms;
    float peak_to_peak;
    float snr;
    float frequency;
    bool sampling_complete;
} ADCData;

typedef struct
{
    uint32_t rise_time;
    uint32_t fall_time;
    uint32_t high_time;
    uint32_t low_time;
    uint32_t period;
    uint32_t total_width;
    uint32_t measurement_count;
    float frequency;
    float duty_cycle;
    bool sampling_complete;
} PWMData;

static PulseData pulse_data = {
    .count = 0,
    .sampling_complete = false};

static ADCData adc_data = {
    .sample_count = 0,
    .rms = 0,
    .peak_to_peak = 0,
    .snr = 0,
    .frequency = 0,
    .sampling_complete = false};

PWMData pwm_data = {
    .sampling_complete = false};

volatile uint32_t last_edge_time_adc = 0;
volatile uint32_t sample_index_adc = 0;
volatile float adc_signal[ADC_SAMPLE_SIZE];
#define DEBOUNCE_DELAY 300
#define ADC_SAMPLE_THRESHOLD 1000

void read_pulse(uint gpio, uint32_t events);
void process_pulse_data();

void read_adc();
bool process_adc_data(struct repeating_timer *t);
float calculate_rms();
float calculate_peak_to_peak();
float calculate_snr();

void read_pwm(uint gpio, uint32_t events);
bool process_pwm_data(struct repeating_timer *t);
float digi_calculate_frequency();
float calculate_duty_cycle();

bool get_all_values(struct repeating_timer *t);

void gpio_callback(uint gpio, uint32_t events);
void setup_adc();
void setup_pins();
void setup_pwm(uint gpio, float freq, float duty_cycle);

int main()
{
    stdio_init_all();
    sleep_ms(3000);

    setup_pins();
    setup_adc();

    // Used for testing, not needed in final version
    setup_pwm(0, 900.0f, 0.6f);

    struct repeating_timer timer1, timer2, timer3;
    add_repeating_timer_ms(1000, process_pwm_data, NULL, &timer1);
    add_repeating_timer_ms(1000, process_adc_data, NULL, &timer2);

    add_repeating_timer_ms(3000, get_all_values, NULL, &timer3);

    while (1)
    {
        if (pulse_data.sampling_complete)
            process_pulse_data(&pulse_data);

        read_adc();

        sleep_ms(DEBOUNCE_DELAY);
    }

    return 0;
}

void read_pulse(uint gpio, uint32_t events)
{
    uint32_t current_time = time_us_32();

    if (events & GPIO_IRQ_EDGE_RISE)
        pulse_data.rise_times[pulse_data.count] = current_time;
    else if (events & GPIO_IRQ_EDGE_FALL)
    {
        pulse_data.fall_times[pulse_data.count] = current_time;

        if (pulse_data.rise_times[pulse_data.count] != 0)
            pulse_data.count++;
    }

    // Check if we've captured enough pulses
    if (pulse_data.count >= PULSE_NUM)
    {
        gpio_set_irq_enabled(PULSE_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, false);
        pulse_data.sampling_complete = true;
    }
}

void process_pulse_data()
{
    printf("Pulse measurements:\n");
    for (uint32_t i = 0; i < PULSE_NUM; i++)
    {
        pulse_data.pulse_widths[i] = pulse_data.fall_times[i] - pulse_data.rise_times[i];
        printf("Pulse %d duration: %u us\n", i + 1, pulse_data.pulse_widths[i]);
    }

    // Reset for the next set of measurements
    pulse_data.count = 0;
    pulse_data.sampling_complete = false;
}

void read_adc()
{
    uint16_t adc_value = adc_read(); // Read ADC value

    if (adc_value > ADC_SAMPLE_THRESHOLD)
    {
        uint32_t current_time = time_us_32();

        if (last_edge_time_adc != 0)
        {
            uint32_t interval = current_time - last_edge_time_adc;
            adc_signal[sample_index_adc++] = interval;
            last_edge_time_adc = current_time;

            if (sample_index_adc >= ADC_SAMPLE_SIZE)
                sample_index_adc = 0; // Reset the sample index for the next measurement

            adc_data.sampling_complete = true;
        }
        else
            last_edge_time_adc = current_time;

        sleep_us(DEBOUNCE_DELAY); // Small debounce delay to prevent false triggers
    }
}

bool process_adc_data(struct repeating_timer *t)
{
    if (adc_data.sampling_complete)
    {
        uint64_t total_interval = 0;
        for (int i = 0; i < ADC_SAMPLE_SIZE; i++)
        {
            total_interval += adc_signal[i];
        }

        // Calculate average frequency
        float average_interval = total_interval / (float)ADC_SAMPLE_SIZE;
        adc_data.frequency = 1.0f / (average_interval / 1e6); // Convert to frequency in Hz

        // Calculate RMS, Peak-to-Peak, and SNR
        adc_data.rms = calculate_rms();
        adc_data.peak_to_peak = calculate_peak_to_peak();
        adc_data.snr = calculate_snr();

        // Print the calculated ADC data
        printf("--- ADC Signal Analysis ---\n");
        printf("- Frequency: %.2f Hz\t- RMS: %.3fV\n", adc_data.frequency, adc_data.rms);
        printf("- Peak-to-Peak: %.3fV\t- SNR: %.2fdB\n", adc_data.peak_to_peak, adc_data.snr);

        adc_data.sampling_complete = false;
    }
}

float calculate_rms()
{
    float sum_squares = 0.0f;
    for (int i = 0; i < ADC_SAMPLE_SIZE; i++)
    {
        sum_squares += adc_signal[i] * adc_signal[i];
    }

    return sqrt(sum_squares / ADC_SAMPLE_SIZE);
}

float calculate_peak_to_peak()
{
    float min_value = adc_signal[0];
    float max_value = adc_signal[0];
    for (int i = 1; i < ADC_SAMPLE_SIZE; i++)
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
    // Using a simple signal-to-noise ratio estimation
    return 20 * log10(adc_data.peak_to_peak / adc_data.rms);
}

void read_pwm(uint gpio, uint32_t events)
{
    uint32_t current_time = time_us_32();

    if (events & GPIO_IRQ_EDGE_RISE)
    {
        if (pwm_data.fall_time != 0)
            pwm_data.low_time = current_time - pwm_data.fall_time;

        if (pwm_data.rise_time != 0)
        {
            pwm_data.period = current_time - pwm_data.rise_time;
            pwm_data.total_width += pwm_data.period;
            pwm_data.measurement_count++;

            pwm_data.sampling_complete = true;
        }

        pwm_data.rise_time = current_time; // Update the rise time
    }
    else if (events & GPIO_IRQ_EDGE_FALL)
    {
        if (pwm_data.rise_time != 0)
            pwm_data.high_time = current_time - pwm_data.rise_time;

        pwm_data.fall_time = current_time; // Update the fall time
    }
}

bool process_pwm_data(struct repeating_timer *t)
{
    if (pwm_data.sampling_complete)
    {
        pwm_data.frequency = digi_calculate_frequency(); // Calculate frequency
        pwm_data.duty_cycle = calculate_duty_cycle();    // Calculate duty cycle

        // Print the calculated data
        printf("\n--- Digital Signal Analyzer ---\n");
        printf("Frequency: %.2f Hz\tDuty Cycle: %.2f%%\n", pwm_data.frequency, pwm_data.duty_cycle);
        printf("Total Pulse Width: %u us\n", pwm_data.period); // Total pulse width
        printf("PW High Time: %u us\tPW Low Time: %u us\n", pwm_data.high_time, pwm_data.low_time);

        // Reset for the next cycle
        pwm_data.sampling_complete = false;
        pwm_data.total_width = 0;
        pwm_data.measurement_count = 0;
    }
    return true; // Keep the timer running
}

float digi_calculate_frequency()
{
    uint32_t average_period = 0;

    if (pwm_data.measurement_count > 0)
    {
        average_period = pwm_data.total_width / pwm_data.measurement_count;

        if (average_period > 0)
            return 1000000.0f / average_period; // Calculate frequency in Hz
    }

    return 0; // Avoid division by zero
}

float calculate_duty_cycle()
{
    if (pwm_data.period > 0)
    {
        float duty_cycle = ((float)pwm_data.high_time / (float)pwm_data.period) * 100.0f; // Calculate duty cycle percentage
        return duty_cycle;
    }
    return 0.0f; // Avoid division by zero
}

bool get_all_values(struct repeating_timer *t)
{
    printf("\nGetting all values\n");

    // printf("Pulse Data:\n");
    // for (uint32_t i = 0; i < PULSE_NUM; i++)
    // {
    //     printf("Pulse %d duration: %u us\n", i + 1, pulse_data.pulse_widths[i]);
    // }

    printf("ADC Data:\nFrequency: %.2f Hz\tRMS: %.3fV\nPeak-to-Peak: %.3fV\tSNR: %.2fdB\n", adc_data.frequency, adc_data.rms, adc_data.peak_to_peak, adc_data.snr);
    printf("PWM Data:\n\tFrequency: %.2f Hz\tDuty Cycle: %.2f%%\n\n", pwm_data.frequency, pwm_data.duty_cycle);

    return true;
}

void gpio_callback(uint gpio, uint32_t events)
{
    if (gpio == PULSE_PIN)
        read_pulse(gpio, events);
    else if (gpio == PWM_PIN)
        read_pwm(gpio, events);
    else
        printf("Invalid Input!!!\n");
}

void setup_adc()
{
    adc_init();
    adc_gpio_init(ADC_PIN);
    adc_select_input(0);
}

void setup_pins()
{
    gpio_init(PULSE_PIN);
    gpio_set_dir(PULSE_PIN, GPIO_IN);
    gpio_pull_down(PULSE_PIN);
    gpio_set_irq_enabled_with_callback(PULSE_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

    gpio_init(ADC_PIN);
    gpio_set_dir(ADC_PIN, GPIO_IN);
    gpio_pull_down(ADC_PIN);

    gpio_init(PWM_PIN);
    gpio_set_dir(PWM_PIN, GPIO_IN);
    gpio_pull_down(PWM_PIN);
    gpio_set_irq_enabled_with_callback(PWM_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
}

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
