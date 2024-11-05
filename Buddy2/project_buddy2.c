// LINES 126-176 FOR ANALOG READING
// LINES 340-355 FOR PWM READING

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
#define ADC_BTN_PIN 22
#define ADC_PIN 26
#define ADC_MAX 4095.0f
#define REF_VOLTAGE 2.0f
#define ADC_SAMPLE_RATE 1.0f
#define SAMPLE_THRESHOLD 1000 // Adjust based on signal characteristics

// Used for digital signal measurement
#define DIGI_PIN 7

// General constants
#define SAMPLE_SIZE 1000
#define BUFFER_SIZE 24408

// Declare Global Variables:
// Used for pulse measurement
volatile uint32_t prev_rise_time = 0;
volatile uint32_t prev_fall_time = 0;
volatile uint32_t pulse_count = 0;

// Used for analog signal measurement
volatile bool adc_timer = false;
volatile uint32_t buffer_index = 0;
volatile uint32_t sample_index = 0;
volatile uint32_t last_edge_time = 0;
volatile uint32_t cycles_counted = 0;
volatile float adc_signal[SAMPLE_SIZE];
char adc_scan_buffer[BUFFER_SIZE];
#define DEBOUNCE_DELAY 300

// Used for digital signal measurement
volatile uint32_t last_rise_time = 0;
volatile uint32_t last_fall_time = 0;
volatile uint32_t high_time = 0;
volatile uint32_t low_time = 0;
volatile uint32_t period = 0;
volatile uint32_t total_width = 0;
volatile uint32_t measurement_count = 0;
volatile bool new_cycle_complete = false;

// Declare Prototype Functions:
// Pulse measurement functions
void read_pulse(uint gpio, uint32_t events);

// Analog signal measurement functions
bool read_adc(struct repeating_timer *t);
float calculate_rms();
float calculate_peak_to_peak();
float calculate_snr();
float adc_calculate_frequency();

// Digital signal measurement functions
void read_digi(uint gpio, uint32_t events);
bool measure_digi(struct repeating_timer *t);
float digi_calculate_frequency();
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
    setup_pwm(0, 1000.0f, 0.5f);

    // Initialize GPIO interrupts
    gpio_set_irq_enabled_with_callback(PULSE_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
    gpio_set_irq_enabled_with_callback(DIGI_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

    // Initialize timers
    struct repeating_timer timer;
    add_repeating_timer_ms(1000, measure_digi, NULL, &timer); // Timer for reading digital signal, 1s interval

    bool signal_above_threshold = false; // Tracks the last signal state

    while (1)
    {
        // Read ADC VALUE HERE
        uint16_t adc_value = adc_read();

        // Detect a rising edge (crossing from below threshold to above)
        if (adc_value > SAMPLE_THRESHOLD && !signal_above_threshold)
        {
            signal_above_threshold = true;
            uint32_t current_time = time_us_32();

            if (last_edge_time != 0)
            {
                // Calculate interval since the last rising edge
                uint32_t interval = current_time - last_edge_time;
                adc_signal[sample_index++] = interval;

                last_edge_time = current_time;

                // Check if we've collected enough cycles
                if (sample_index >= SAMPLE_SIZE)
                {
                    // Calculate the average interval for the set of cycles
                    uint64_t total_interval = 0;
                    for (int i = 0; i < SAMPLE_SIZE; i++)
                    {
                        total_interval += adc_signal[i];
                    }

                    // Calculate average frequency
                    float average_interval = total_interval / (float)SAMPLE_SIZE;
                    float frequency = 1.0 / (average_interval / 1e6); // Frequency in Hz

                    printf("Measuring Analog Signal\n");
                    printf("Average Frequency: %.2f Hz\n", frequency); // REPLACE PRINT FUNCTION TO SENDING TO DASHBOARD

                    // Reset for the next measurement
                    sample_index = 0;
                }
            }
            else
            {
                // Initialize last_edge_time on first rising edge detection
                last_edge_time = current_time;
            }

            // Debounce delay to prevent immediate re-triggering
            sleep_us(DEBOUNCE_DELAY);
        }
        else if (adc_value < SAMPLE_THRESHOLD && signal_above_threshold)
        {
            // Falling edge detected - signal went below threshold
            signal_above_threshold = false;
        }

        // tight_loop_contents(); // Loop forever
    }
    return 0;
}

// Interrupt callback function, determines which GPIO triggered the interrupt
void gpio_callback(uint gpio, uint32_t events)
{
    if (gpio == PULSE_PIN)
        read_pulse(gpio, events);
    else if (gpio == ADC_BTN_PIN)
    {
        if (events & GPIO_IRQ_EDGE_FALL)
        {
            printf("Reading ADC ...\n");
            adc_timer = true;
            sample_index = 0;   // Reset index to start a new measurement series
            cycles_counted = 0; // Reset cycles counted
        }
        else
        {
            printf("Reading ADC Stopped.\n");
            adc_timer = false;
        }
    }
    else if (gpio == DIGI_PIN)
        read_digi(gpio, events);
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
            uint32_t pulse_width = current_time - prev_rise_time; // Calculate pulse width
            printf("Pulse %d: Time - %u us, Width: %u us\n", pulse_count + 1, current_time, pulse_width);
            pulse_count++;
        }
        prev_rise_time = current_time; // Update previous rising edge time
    }
    else if (events & GPIO_IRQ_EDGE_FALL)
        prev_fall_time = current_time;

    if (pulse_count >= PULSE_NUM)
        gpio_set_irq_enabled_with_callback(PULSE_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, false, &gpio_callback);
}

// float voltage = adc_value * (REF_VOLTAGE / ADC_MAX);
// adc_signal[sample_index++] = voltage;

// if (sample_index >= SAMPLE_SIZE)
// {
//     float rms_value = calculate_rms();
//     float peak_to_peak_value = calculate_peak_to_peak();
//     float snr_value = calculate_snr();
//     float frequency = adc_calculate_frequency();

//     printf("--- Analog Signal Analysis ---\nVoltage captured: %.2f, %.2f, %.2f, %.2f...\n", adc_signal[0], adc_signal[1], adc_signal[2], adc_signal[3]);
//     printf("RMS Voltage: %.3fV\tPeak-to-Peak Voltage: %.3fV\tSignal-to-Noise Ratio (SNR): %.2fdB\tFrequency: %.2f Hz\n",
//            rms_value, peak_to_peak_value, snr_value, frequency);

//     sample_index = 0;
//     buffer_index = 0;
//     memset(adc_scan_buffer, 0, BUFFER_SIZE);
// }

// Calculates the RMS value of the analog signal
float calculate_rms()
{
    float sum_squares = 0.0f;
    for (int i = 0; i < SAMPLE_SIZE; i++)
    {
        sum_squares += adc_signal[i] * adc_signal[i];
    }

    return sqrt(sum_squares / SAMPLE_SIZE);
}

// Calculates the peak-to-peak value of the analog signal
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

// Calculates the signal-to-noise ratio of the analog signal
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

// Reads the digital signal, calculates frequency, duty cycle, and pulse width, and prints the results
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

// Prints the results of the digital signal analysis if a new cycle is complete
// MEASURES FREQ AND DUTY CYCLE FOR PWM
bool measure_digi(struct repeating_timer *t)
{
    if (new_cycle_complete)
    {
        float frequency = digi_calculate_frequency(); // Calculate frequency
        float duty_cycle = calculate_duty_cycle();    // Calculate duty cycle

        // REPLACE PRINT FUNCTION TO SENDING TO DASHBOARD
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
float digi_calculate_frequency()
{
    uint32_t average_period = 0;
    average_period = total_width / measurement_count;

    if (average_period > 0)
        return 1000000.0f / average_period; // Calculate frequency in Hz

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
