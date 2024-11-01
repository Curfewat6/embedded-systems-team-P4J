// Declare Dependencies:
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

// Declare Constants:
// Used for pulse measurement
#define PULSE_PIN 2
#define PULSE_NUM 10

// Used for analog signal measurement
#define ADC_BTN_PIN 21
#define ADC_PIN 26
#define ADC_MAX 4095.0f
#define REF_VOLTAGE 3.3f

// Used for digital signal measurement
#define DIGI_PIN 7

// General constants
#define SAMPLE_SIZE 10
#define BUFFER_SIZE 24408

// Declare Global Variables:
// Used for pulse measurement
volatile uint32_t prev_rise_time = 0;
volatile uint32_t prev_fall_time = 0;
volatile uint32_t pulse_count = 0;

// Used for analog signal measurement
volatile bool adc_timer = false;
volatile float ad_signal[SAMPLE_SIZE];
volatile int sample_index = 0;
char adc_scan_buffer[BUFFER_SIZE];
size_t buffer_index = 0;

// Used for digital signal measurement
volatile uint32_t last_rise_time = 0;
volatile uint32_t last_fall_time = 0;
volatile uint32_t high_time = 0;
volatile uint32_t low_time = 0;
volatile uint32_t period = 0;
volatile bool new_cycle_complete = false;
volatile uint32_t total_width = 0;
volatile uint32_t measurement_count = 0;

// Declare Prototype Functions:
// Pulse measurement functions
void read_pulse(uint gpio, uint32_t events);

// Analog signal measurement functions
bool read_adc(struct repeating_timer *t);
float calculate_rms();
float calculate_peak_to_peak();
float calculate_snr();

// Digital signal measurement functions
void read_digi(uint gpio, uint32_t events);
bool measure_digi(struct repeating_timer *t);
float calculate_frequency();
float calculate_duty_cycle();

// General setup functions
void gpio_callback(uint gpio, uint32_t events);
void setup_pins();
void setup_adc();
void printTitle();

// Used for testing, not needed in final version
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
    // Initialize standard I/O
    stdio_init_all();
    sleep_ms(3000);
    printTitle();
    setup_pins();
    setup_adc();

    // Used for testing, not needed in final version
    setup_pwm(0, 500.0f, 0.5f);

    // Initialize GPIO interrupts
    gpio_set_irq_enabled_with_callback(PULSE_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
    gpio_set_irq_enabled_with_callback(ADC_BTN_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
    gpio_set_irq_enabled_with_callback(DIGI_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

    // Initialize timers
    struct repeating_timer timer1, timer2;
    add_repeating_timer_ms(300, read_adc, NULL, &timer1);      // Timer for reading ADC, 3s interval
    add_repeating_timer_ms(1000, measure_digi, NULL, &timer2); // Timer for reading digital signal, 1s interval

    while (1)
    {
        tight_loop_contents(); // Loop forever
    }
    return 0;
}

// Interrupt callback function, determines which GPIO triggered the interrupt
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
        printf("Invalid Input!!!\n");
}

// Only measures up to 10 pulses, then disables the interrupt. Prints pulse width of each pulse
void read_pulse(uint gpio, uint32_t events)
{
    uint32_t current_time = time_us_32();

    if (events & GPIO_IRQ_EDGE_RISE)
    {
        if (prev_fall_time != 0)
        {
            uint32_t pulse_width = current_time - prev_fall_time; // Calculate pulse width
            printf("Pulse %d: Time: %u us, Width: %u us\n", pulse_count + 1, current_time, pulse_width);
            pulse_count++;
        }
        prev_rise_time = current_time; // Update previous rising edge time
    }
    else if (events & GPIO_IRQ_EDGE_FALL)
        prev_fall_time = current_time;

    if (pulse_count >= PULSE_NUM)
        gpio_set_irq_enabled_with_callback(PULSE_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, false, &gpio_callback);
}

// Reads the ADC value, calculates RMS, peak-to-peak, and SNR values, and prints the results.
// Also prints the ADC values captured
bool read_adc(struct repeating_timer *t)
{
    if (adc_timer)
    {
        uint16_t adc_value = adc_read();
        float voltage = (adc_value / ADC_MAX) * REF_VOLTAGE;

        ad_signal[sample_index++] = voltage;
        buffer_index += snprintf(adc_scan_buffer + buffer_index, BUFFER_SIZE - buffer_index,
                                 "- %d ", adc_value);

        if (sample_index >= SAMPLE_SIZE)
        {

            float rms_value = calculate_rms();
            float peak_to_peak_value = calculate_peak_to_peak();
            float snr_value = calculate_snr();

            printf("--- Analog Signal Analysis ---\nADC values captured: %s\n", adc_scan_buffer);
            printf("RMS Voltage: %.3fV\tPeak-to-Peak Voltage: %.3fV\tSignal-to-Noise Ratio (SNR): %.2fdB\n",
                   rms_value, peak_to_peak_value, snr_value);

            sample_index = 0;
            buffer_index = 0;
            memset(adc_scan_buffer, 0, BUFFER_SIZE);
        }
    }
    return true;
}

// Calculates the RMS value of the analog signal
float calculate_rms()
{
    float sum_squares = 0.0f;
    for (int i = 0; i < SAMPLE_SIZE; i++)
    {
        sum_squares += ad_signal[i] * ad_signal[i];
    }
    return sqrt(sum_squares / SAMPLE_SIZE);
}

// Calculates the peak-to-peak value of the analog signal
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

// Calculates the signal-to-noise ratio of the analog signal
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

// Reads the digital signal, calculates frequency, duty cycle, and pulse width, and prints the results
void read_digi(uint gpio, uint32_t events)
{
    uint32_t current_time = time_us_32();

    if (events & GPIO_IRQ_EDGE_RISE)
    {
        if (last_fall_time != 0)
        {
            low_time = current_time - last_fall_time;
        }

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
        {
            high_time = current_time - last_rise_time;
        }
        last_fall_time = current_time;
    }
}

// Prints the results of the digital signal analysis if a new cycle is complete
bool measure_digi(struct repeating_timer *t)
{
    if (new_cycle_complete)
    {
        float frequency = calculate_frequency();   // Calculate frequency
        float duty_cycle = calculate_duty_cycle(); // Calculate duty cycle

        // Print results
        printf("\n--- Digital Signal Analyzer ---\n");
        printf("Frequency: %.2f Hz\tDuty Cycle: %.2f%%\n", frequency, duty_cycle);
        printf("Total Pulse Width: %u us\n", period); // Total pulse width
        printf("PW High Time: %u us\tPW Low Time: %u us\n", high_time, low_time);

        // Reset for the next cycle
        new_cycle_complete = false;
        total_width = 0;
        measurement_count = 0;
    }
    return true; // Keep the timer running
}

// Calculate frequency using averaged pulse width
float calculate_frequency()
{
    uint32_t average_period = 0;
    average_period = total_width / measurement_count;

    if (average_period > 0)
    {
        return 1000000.0f / average_period; // Calculate frequency in Hz
    }
    return 0; // Avoid division by zero
}

// Calculate duty cycle
float calculate_duty_cycle()
{
    if (period > 0)
    {
        float duty_cycle = ((float)high_time / (float)period) * 100.0f; // Calculate duty cycle percentage
        return duty_cycle;
    }
    return 0.0f; // Avoid division by zero
}

// Initializes the GPIO pins
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

// Initializes the ADC
void setup_adc()
{
    adc_init();
    adc_gpio_init(ADC_PIN);
    adc_select_input(0);
}

// Prints the project title
void printTitle()
{
    printf("===========================\n");
    printf("= INF2004 Project Buddy 2 =\n");
    printf("===========================\n");
}
