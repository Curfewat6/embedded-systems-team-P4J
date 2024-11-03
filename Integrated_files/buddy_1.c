#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/rtc.h"
#include <string.h> // Include string.h for strlen
#include <stdlib.h> // Include stdlib.h for free
#include "hardware/spi.h"
#include "ff.h" // FATFS library for file operations

#include "hardware/adc.h"
#include "hardware/pwm.h" // PWM library for pulse generation

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "hardware/gpio.h"
#include "hardware/timer.h"

#include <time.h> // For random number generation

#include "pico/mutex.h"
//for my array to prevent simutaneous access
mutex_t pulse_data_mutex;

// Create instances for FATFS and FIL
FATFS fs;
FIL fil; // File object

DIR dir;
FILINFO fno;
FRESULT fr;

#define MAX_DATA_POINTS 10 // Limit the number of data points

float pwm_frequency_values[MAX_DATA_POINTS] = {0}; // Array to hold the signal values
int pwm_frequency_index = 0; // Current index for adding new values

float pwm_duty_cycle_values[MAX_DATA_POINTS] = {0};
int pwm_duty_cycle_index = 0;

float adc_frequency_values[MAX_DATA_POINTS] = {0}; // Array to hold the signal values
int adc_frequency_index = 0; // Current index for adding new values

// define button for GP20 (acting as creating file to sd)
const uint WRITE_BTNPIN_20 = 20;
// define button for GP21 (acting as reading file from sd)
const uint READ_BTNPIN_21 = 21;
// define button for GP22 (acting as exiting program)
const uint EXIT_BTNPIN_22 = 22;

// --------------------------------------------
// Declare Constants:
// Used for pulse measurement
#define PULSE_PIN 2
#define PULSE_NUM 10

// Used for analog signal measurement
#define ADC_BTN_PIN 21
#define ADC_PIN 26
#define ADC_MAX 4095.0f
#define REF_VOLTAGE 3.3f
#define ADC_SAMPLE_RATE 300.0f

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
volatile uint32_t buffer_index = 0;
volatile uint32_t sample_index = 0;
volatile uint32_t cycles_counted = 0;
volatile float adc_signal[SAMPLE_SIZE];
char adc_scan_buffer[BUFFER_SIZE];

// Used for digital signal measurement
volatile uint32_t last_rise_time = 0;
volatile uint32_t last_fall_time = 0;
volatile uint32_t high_time = 0;
volatile uint32_t low_time = 0;
volatile uint32_t period = 0;
volatile uint32_t total_width = 0;
volatile uint32_t measurement_count = 0;
volatile bool new_cycle_complete = false;

void init_mutexes() {
    mutex_init(&pulse_data_mutex);
}

// Declare Prototype Functions:
// Pulse measurement functions
void read_pulse(uint gpio, uint32_t events);

// Analog signal measurement functions
void read_adc();
float calculate_rms();
float calculate_peak_to_peak();
float calculate_snr();
float adc_calculate_frequency();

// Digital signal measurement functions
void read_digi(uint gpio, uint32_t events);
void measure_digi();
float digi_calculate_frequency();
float calculate_duty_cycle();
// --------------------------------------------

// simple counter tht increments when creating file
static int filename_counter = 1;

// for debounce configuration (300ms)
#define DEBOUNCE_DELAY 300
static absolute_time_t last_button_press;

// Global variable to track if filesystem is mounted
static bool fs_mounted = false;

// experimental ------------------------------------------------
typedef struct
{
    uint32_t timestamp; // need the buddies to retrieve from web console via NTP
    uint32_t frequency;
    float duty_cycle;
    uint32_t total_pulse_width; // Duration of the pulse in microseconds (from falling to rising edge)
    uint32_t pulse_width_high_time;
    uint32_t pulse_width_low_time;
} PulseData;

PulseData *pulsedata_array = NULL; // Pointer for dynamic memory allocation for PulseData
// uint32_t pulse_count = 0; //Counter to keep track of number of pulses
uint32_t pulsedata_array_size = 0; // Size of the array (to make dynamic)


void initialize_pulsedata_array(uint32_t size) {
    pulsedata_array_size = size;
    pulsedata_array = (PulseData *)malloc(pulsedata_array_size * sizeof(PulseData));
    if (pulsedata_array == NULL) {
        printf("Failed to allocate memory for PulseData array\n");
        exit(1);
    }
    printf("Initialized PulseData array with size: %u\n", pulsedata_array_size);
}

// freeing dynamic memory allocation for PulseData array
void free_pulsedata_array()
{
    if (pulsedata_array != NULL)
    {
        free(pulsedata_array);
        pulsedata_array = NULL;
    }
}

void resize_pulsedata_array(uint32_t new_size) {
    // pulsedata_array = (PulseData *)realloc(pulsedata_array, new_size * sizeof(PulseData));
    PulseData *temp = (PulseData *)realloc(pulsedata_array, new_size * sizeof(PulseData));
    if (pulsedata_array == NULL) {
        printf("Failed to reallocate memory for PulseData array\n");
        free_pulsedata_array();
        exit(1);
    }
    // pulsedata_array_size = new_size;
    pulsedata_array = temp;
    pulsedata_array_size = new_size;
    printf("Resized PulseData array to size: %u\n", pulsedata_array_size);
}

// --------------------------------------------------------------


volatile bool write_flag = false; // Flag for write operation
volatile bool read_flag = false;  // Flag for read operation

void button_callback(uint gpio, uint32_t events)
{
    absolute_time_t current_time = get_absolute_time();
    // experimental Debouncing logic (if issues in reading from oscilloscope, comment out this)
    if (absolute_time_diff_us(last_button_press, current_time) < DEBOUNCE_DELAY * 1000) {
        return;
    }
    last_button_press = current_time;

    if (gpio == PULSE_PIN) // migrated from gpio callback - jl
        read_pulse(gpio, events);
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
    if (gpio == DIGI_PIN){
        read_digi(gpio, events);
    }
}

// to initialize the buttons
void init_buttons(void)
{
    // Initialize the button GPIO - 20, 21, 22
    gpio_init(WRITE_BTNPIN_20);
    gpio_set_dir(WRITE_BTNPIN_20, GPIO_IN);
    gpio_pull_up(WRITE_BTNPIN_20);
    gpio_set_irq_enabled_with_callback(WRITE_BTNPIN_20, GPIO_IRQ_EDGE_FALL, true, &button_callback);

    gpio_init(READ_BTNPIN_21);
    gpio_set_dir(READ_BTNPIN_21, GPIO_IN);
    gpio_pull_up(READ_BTNPIN_21);
    gpio_set_irq_enabled_with_callback(READ_BTNPIN_21, GPIO_IRQ_EDGE_FALL, true, &button_callback);

    gpio_init(EXIT_BTNPIN_22);
    gpio_set_dir(EXIT_BTNPIN_22, GPIO_IN);
    gpio_pull_up(EXIT_BTNPIN_22);

    // ---------------------------------------------------------------
    gpio_init(PULSE_PIN); //GPIO Pin 2 for detecting pulse
    gpio_set_dir(PULSE_PIN, GPIO_IN);
    gpio_pull_down(PULSE_PIN);

    gpio_init(ADC_PIN); //GPIO Pin 26 for ADC
    gpio_set_dir(ADC_PIN, GPIO_IN);
    gpio_pull_up(ADC_PIN);

    gpio_init(ADC_BTN_PIN); //GPIO Pin 21 for ADC button
    gpio_set_dir(ADC_BTN_PIN, GPIO_IN);
    gpio_pull_up(ADC_BTN_PIN);

    gpio_init(DIGI_PIN); //GPIO Pin 7 for digital signal
    gpio_set_dir(DIGI_PIN, GPIO_IN);
    gpio_pull_down(DIGI_PIN);

    gpio_set_irq_enabled_with_callback(PULSE_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &button_callback);
    gpio_set_irq_enabled_with_callback(ADC_BTN_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &button_callback);
    gpio_set_irq_enabled_with_callback(DIGI_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &button_callback);
    // ---------------------------------------------------------------

    last_button_press = get_absolute_time();
}

// Initializes the ADC
void setup_adc()
{
    adc_init();
    adc_gpio_init(ADC_PIN);
    adc_select_input(0);
}


// seems to work fine writing high time and low time to txt correctly based on output.
void read_pulse(uint gpio, uint32_t events) {
    uint32_t current_time = time_us_32();

    if (events & GPIO_IRQ_EDGE_RISE) {
        if (prev_fall_time != 0) {
            low_time = current_time - prev_fall_time; // Calculate low time
        }
        prev_rise_time = current_time;
    } else if (events & GPIO_IRQ_EDGE_FALL) {
        high_time = current_time - prev_rise_time; // Calculate high time

        if (prev_fall_time != 0) {
            printf("Pulse %d: Time - %u us, High time: %u us, Low time: %u us\n", 
                   pulse_count + 1, current_time, high_time, low_time);

            if (pulse_count < pulsedata_array_size) {
                pulsedata_array[pulse_count].pulse_width_high_time = high_time;
                pulsedata_array[pulse_count].pulse_width_low_time = low_time;
                pulse_count++;
            } else {
                printf("Error: Pulse count exceeds array size.\n");
            }
        }

        prev_fall_time = current_time;
    }

    if (pulse_count >= PULSE_NUM) {
        gpio_set_irq_enabled_with_callback(PULSE_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, false, &button_callback);
    }
}



// ======================================= TESTING =============================================================== //
// Reads the ADC value, calculates RMS, peak-to-peak, and SNR values, and prints the results.
// Also prints the ADC values captured
void read_adc()
{
    if (adc_timer)
    {
        uint16_t adc_value = adc_read();
        float voltage = (adc_value / ADC_MAX) * REF_VOLTAGE;

        adc_signal[sample_index++] = voltage;
        buffer_index += snprintf(adc_scan_buffer + buffer_index, BUFFER_SIZE - buffer_index,
                                 "- %d ", adc_value);

        if (sample_index >= SAMPLE_SIZE)
        {
            float rms_value = calculate_rms();
            float peak_to_peak_value = calculate_peak_to_peak();
            float snr_value = calculate_snr();
            float frequency = adc_calculate_frequency();

            printf("\n--- Analog Signal Analysis ---\nADC values captured: %s\n", adc_scan_buffer);
            printf("RMS Voltage: %.3fV\tPeak-to-Peak Voltage: %.3fV\tSignal-to-Noise Ratio (SNR): %.2fdB\n",
                   rms_value, peak_to_peak_value, snr_value);
            printf("Frequency: %.2f Hz\n", frequency);

            adc_frequency_values[adc_frequency_index] = frequency;
            adc_frequency_index = (adc_frequency_index + 1) % MAX_DATA_POINTS; // Update index in a circular manner

            sample_index = 0;
            buffer_index = 0;
            memset(adc_scan_buffer, 0, BUFFER_SIZE);
        }
    }
}

// ======================================================================================================================= //

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

// Function to calculate frequency based on zero crossings or peaks
float adc_calculate_frequency()
{
    // Ensure we have enough samples to work with
    if (sample_index < 2)
        return 0; // Not enough samples for frequency calculation

    // Track time intervals for rising and falling edges
    uint32_t last_time = 0;
    uint32_t current_time = 0;
    uint32_t pulse_count = 0;

    // Iterate through samples to find pulse edges
    for (int i = 1; i < sample_index; i++)
    {
        current_time = i * (1000000 / ADC_SAMPLE_RATE); // Convert index to time in microseconds

        // Check if we've detected a rising edge
        if (adc_signal[i - 1] < (REF_VOLTAGE / 2) && adc_signal[i] >= (REF_VOLTAGE / 2))
        {
            // Count a pulse
            pulse_count++;
            last_time = current_time; // Update last time on rising edge
        }
    }

    // Calculate the frequency
    if (pulse_count > 0)
    {
        float period = (last_time / pulse_count); // Average period in microseconds

        if (period > 0)
            return 1000000.0f / period; // Return frequency in Hz
    }

    return 0; // No pulses detected
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

// ================================================ TESTING =======================================================//

void measure_digi() {
    // mutex_enter_blocking(&pulse_data_mutex); // Use mutex if accessing shared resources

    
    if (new_cycle_complete) {
        float frequency = digi_calculate_frequency(); // Calculate frequency
        float duty_cycle = calculate_duty_cycle();    // Calculate duty cycle

        pwm_frequency_values[pwm_frequency_index] = frequency;
        pwm_frequency_index = (pwm_frequency_index + 1) % MAX_DATA_POINTS; // Update index in a circular manner

        pwm_duty_cycle_values[pwm_duty_cycle_index] = duty_cycle;
        pwm_duty_cycle_index = (pwm_duty_cycle_index + 1) % MAX_DATA_POINTS; // Update index in a circular manner

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

    // mutex_exit(&pulse_data_mutex); // Use mutex if accessing shared resources
}

// ================================================================================================================= //

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

