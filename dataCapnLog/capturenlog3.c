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

// ---------------------------------------------------------------
// result to verify the file status
void print_fresult(FRESULT fr)
{
    switch (fr)
    {
    // different cases to check status of file operation
    case FR_OK:
        printf("Success\n");
        break;
    case FR_NOT_READY:
        printf("Not Ready\n");
        break;
    case FR_NO_FILE:
        printf("No File\n");
        break;
    case FR_NO_PATH:
        printf("No Path\n");
        break;
    case FR_INVALID_DRIVE:
        printf("Invalid Drive\n");
        break;
    // Add other cases as necessary
    default:
        printf("Unknown Error: %d\n", fr);
        break;
    }
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

// performing freeing memory allocation for PulseData array
// terminating of processes
void cleanup_exit()
{
    // Free dynamically allocated memory
    free_pulsedata_array();
    // Disable interrupts for buttons
    gpio_set_irq_enabled(WRITE_BTNPIN_20, GPIO_IRQ_EDGE_FALL, false);
    gpio_set_irq_enabled(READ_BTNPIN_21, GPIO_IRQ_EDGE_FALL, false);
    gpio_set_irq_enabled(EXIT_BTNPIN_22, GPIO_IRQ_EDGE_FALL, false);
    // Unmount filesystem if mounted
    if (fs_mounted)
    {
        f_unmount("");
        printf("Filesystem unmounted successfully\n");
    }
    // Exit the program
    printf("Exiting program...\n");
    exit(0); // Clean exit
}

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

    // Set flags instead of directly calling writeToSD() or readSD()
    if (gpio == WRITE_BTNPIN_20)
    {
        write_flag = true;
        // if button press is GP20, set write_flag to true -> writePulseHighLowToSD() will be called
    }
    else if (gpio == READ_BTNPIN_21)
    {
        read_flag = true;
        // if button press is GP21, set read_flag to true -> readSD() will be called
    }
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

void init_system_with_filesystem()
{
    printf("Initializing USB Serial...\n");
    stdio_init_all();
    sleep_ms(5000);
    printf("USB Serial Initialized\n");

    // Initialize filesystem
    printf("Attempting to mount filesystem...\n");
    for (int i = 0; i < 3 && !fs_mounted; i++)
    {
        fr = f_mount(&fs, "", 1);
        if (fr == FR_OK)
        {
            fs_mounted = true;
            printf("Filesystem mounted successfully\n");
            break;
        }
        else
        {
            printf("Failed to mount filesystem: ");
            print_fresult(fr);
            printf("Retrying mount attempt %d...\n", i + 1);
            sleep_ms(1000); // Wait 1 second before retrying
        }
    }

    if (!fs_mounted)
    {
        printf("Failed to mount filesystem after 3 attempts. Exiting...\n");
        return;
    }

    init_buttons();
    printf("System Initialization Completed\n");
    printf("Press GP20 to write a file, GP21 to read files\n");
}

// Only measures up to 10 pulses, then disables the interrupt. Prints pulse width of each pulse
// void read_pulse(uint gpio, uint32_t events) {
//     // Enter the mutex to protect shared resources
//     // mutex_enter_blocking(&pulse_data_mutex);

//     uint32_t current_time = time_us_32();

//     if (events & GPIO_IRQ_EDGE_RISE) {
//         if (prev_fall_time != 0) {
//             uint32_t pulse_width = current_time - prev_rise_time; // Calculate pulse width
//             low_time = current_time - last_fall_time;
//             printf("Pulse %d: Time - %u us, Width: %u us, High time: %u us, Low time: %u us\n", pulse_count + 1, current_time, pulse_width, high_time, low_time);
//             pulse_count++;

//             // low_time = current_time - prev_fall_time;
//             printf("Debug: Calculated Low Time = %u us\n", low_time);
//             // pulse_count++;
//         }
//         prev_rise_time = current_time;
//     } else if (events & GPIO_IRQ_EDGE_FALL) {
//         high_time = current_time - prev_rise_time;
//         printf("Debug: Calculated High Time = %u us\n", high_time);
//         prev_fall_time = current_time;
//     }

//     if (pulse_count >= PULSE_NUM) {
//         gpio_set_irq_enabled_with_callback(PULSE_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, false, &button_callback);
//     }

//     // Update pulsedata_array
//     // Exit the mutex once done
//     // mutex_exit(&pulse_data_mutex);
// }

// void read_pulse(uint gpio, uint32_t events) {
//     uint32_t current_time = time_us_32();

//     if (events & GPIO_IRQ_EDGE_RISE) {
//         if (prev_fall_time != 0) {
//             uint32_t pulse_width = current_time - prev_rise_time; // Calculate pulse width
//             low_time = current_time - last_fall_time;
//             printf("Pulse %d: Time - %u us, Width: %u us, High time: %u us, Low time: %u us\n", pulse_count + 1, current_time, pulse_width, high_time, low_time);
//             // low_time = current_time - prev_fall_time;
//             // printf("Debug: Pulse %d: Calculated Low Time = %u us\n", pulse_count + 1, low_time);
//             pulse_count++;

            
//             // pulse_count++;
//         }
//         prev_rise_time = current_time;
//     } else if (events & GPIO_IRQ_EDGE_FALL) {
//         high_time = current_time - prev_rise_time;
//         printf("Debug: Pulse %d: Calculated High Time = %u us\n", pulse_count+1, high_time);
//         prev_fall_time = current_time;
//     }

//     if (pulse_count >= PULSE_NUM) {
//         gpio_set_irq_enabled_with_callback(PULSE_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, false, &button_callback);
//     }

//     // Update pulsedata_array
//     if (pulse_count < pulsedata_array_size) {
//         pulsedata_array[pulse_count].pulse_width_high_time = high_time;
//         pulsedata_array[pulse_count].pulse_width_low_time = low_time;
//     } else {
//         printf("Error: Pulse count exceeds array size.\n");
//     }
// }

void read_pulse(uint gpio, uint32_t events) {
    uint32_t current_time = time_us_32();

    if (events & GPIO_IRQ_EDGE_RISE) {
        if (prev_fall_time != 0) {
            uint32_t pulse_width = current_time - prev_rise_time; // Calculate pulse width
            low_time = current_time - prev_fall_time; // Calculate low time
            printf("Pulse %d: Time - %u us, Width: %u us, High time: %u us, Low time: %u us\n", pulse_count + 1, current_time, pulse_width, high_time, low_time);
            pulse_count++;
        }
        prev_rise_time = current_time;
    } else if (events & GPIO_IRQ_EDGE_FALL) {
        high_time = current_time - prev_rise_time; // Calculate high time
        printf("Debug: Pulse %d: Calculated High Time = %u us\n", pulse_count + 1, high_time);
        prev_fall_time = current_time;
    }

    if (pulse_count >= PULSE_NUM) {
        gpio_set_irq_enabled_with_callback(PULSE_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, false, &button_callback);
    }

    // Update pulsedata_array
    if (pulse_count < pulsedata_array_size) {
        pulsedata_array[pulse_count].pulse_width_high_time = high_time;
        pulsedata_array[pulse_count].pulse_width_low_time = low_time;
    } else {
        printf("Error: Pulse count exceeds array size.\n");
    }
}

// void read_pulse(uint gpio, uint32_t events) {
//     uint32_t current_time = time_us_32();

//     if (events & GPIO_IRQ_EDGE_RISE) {
//         if (prev_fall_time != 0) {
//             uint32_t pulse_width = current_time - prev_rise_time; // Calculate pulse width
//             low_time = current_time - prev_fall_time; // Calculate low time
//             printf("Pulse %d: Time - %u us, Width: %u us, High time: %u us, Low time: %u us\n", pulse_count + 1, current_time, pulse_width, high_time, low_time);
//             pulse_count++;
//         }
//         prev_rise_time = current_time;
//     } else if (events & GPIO_IRQ_EDGE_FALL) {
//         high_time = current_time - prev_rise_time; // Calculate high time
//         printf("Debug: Pulse %d: Calculated High Time = %u us\n", pulse_count + 1, high_time);
//         prev_fall_time = current_time;
//     }

//     if (pulse_count >= PULSE_NUM) {
//         gpio_set_irq_enabled_with_callback(PULSE_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, false, &button_callback);
//     }

//     // Update pulsedata_array
//     if (pulse_count < pulsedata_array_size) {
//         pulsedata_array[pulse_count].pulse_width_high_time = high_time;
//         pulsedata_array[pulse_count].pulse_width_low_time = low_time;
//     } else {
//         printf("Error: Pulse count exceeds array size.\n");
//     }
// }

// void read_pulse(uint gpio, uint32_t events) {
//     uint32_t current_time = time_us_32();

//     if (events & GPIO_IRQ_EDGE_RISE) {
//         if (prev_fall_time != 0) {
//             uint32_t pulse_width = current_time - prev_rise_time; // Calculate pulse width
//             low_time = current_time - prev_fall_time; // Calculate low time
//             printf("Pulse %d: Time - %u us, Width: %u us, High time: %u us, Low time: %u us\n", pulse_count + 1, current_time, pulse_width, high_time, low_time);
//             pulse_count++;
//         }
//         prev_rise_time = current_time;
//     } else if (events & GPIO_IRQ_EDGE_FALL) {
//         high_time = current_time - prev_rise_time; // Calculate high time
//         printf("Debug: Pulse %d: Calculated High Time = %u us\n", pulse_count + 1, high_time);
//         prev_fall_time = current_time;
//     }

//     if (pulse_count >= PULSE_NUM) {
//         gpio_set_irq_enabled_with_callback(PULSE_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, false, &button_callback);
//     }

//     // Update pulsedata_array
//     if (pulse_count < pulsedata_array_size) {
//         pulsedata_array[pulse_count].pulse_width_high_time = high_time;
//         pulsedata_array[pulse_count].pulse_width_low_time = low_time;
//     } else {
//         printf("Error: Pulse count exceeds array size.\n");
//     }
// }

// Reads the ADC value, calculates RMS, peak-to-peak, and SNR values, and prints the results.
// Also prints the ADC values captured
bool read_adc(struct repeating_timer *t)
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

            printf("--- Analog Signal Analysis ---\nADC values captured: %s\n", adc_scan_buffer);
            printf("RMS Voltage: %.3fV\tPeak-to-Peak Voltage: %.3fV\tSignal-to-Noise Ratio (SNR): %.2fdB\n",
                   rms_value, peak_to_peak_value, snr_value);
            printf("Frequency: %.2f Hz\n", frequency);

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

// Prints the results of the digital signal analysis if a new cycle is complete
bool measure_digi(struct repeating_timer *t)
{
    // mutex_enter_blocking(&pulse_data_mutex);

    if (new_cycle_complete) {
        float frequency = digi_calculate_frequency(); // Calculate frequency
        float duty_cycle = calculate_duty_cycle();    // Calculate duty cycle

        // Print results
        printf("\n--- Digital Signal Analyzer ---\n");
        printf("Frequency: %.2f Hz\tDuty Cycle: %.2f%%\n", frequency, duty_cycle);
        printf("Total Pulse Width: %u us\n", period); // Total pulse width
        printf("PW High Time: %u us\tPW Low Time: %u us\n", high_time, low_time);

        // Save high and low time to pulsedata_array
        if (pulse_count >= pulsedata_array_size) {
            // Resize the array if it's full
            resize_pulsedata_array(pulsedata_array_size * 2); // Double the size as needed
        }
        
        if (pulse_count < pulsedata_array_size) {
            // Add pulse data to the array
            pulsedata_array[pulse_count].timestamp = time_us_32();
            pulsedata_array[pulse_count].frequency = frequency;
            pulsedata_array[pulse_count].duty_cycle = duty_cycle;
            pulsedata_array[pulse_count].total_pulse_width = period;
            pulsedata_array[pulse_count].pulse_width_high_time = high_time;
            pulsedata_array[pulse_count].pulse_width_low_time = low_time;

            pulse_count++;  // Increment pulse count
        }else{
            printf("Error: Pulse count exceeds array size.\n");
        }
        
        
        // Reset for the next cycle
        new_cycle_complete = false;
        total_width = 0;
        measurement_count = 0;
    }
    // mutex_exit(&pulse_data_mutex);
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

// only pulse_width_high_time and pulse_width_low_time are saved to the file (binary)
void savepulse_sequenceToBin(FIL *file)
{
    UINT bw;
    // Pointer to the data buffer containing the pulse times to be written to the file.
    uint32_t pulse_times[2 * pulse_count];

    for (int i = 0; i < pulse_count; i++) {
        pulse_times[2 * i] = pulsedata_array[i].pulse_width_high_time;
        pulse_times[2 * i + 1] = pulsedata_array[i].pulse_width_low_time;
    }
    // 2 * pulse_count * sizeof(uint32_t) -> Specifies the number of bytes to write.
    FRESULT res = f_write(file, pulse_times, 2 * pulse_count * sizeof(uint32_t), &bw);
    // checking result of file operation
    if (res != FR_OK || bw < 2 * pulse_count * sizeof(uint32_t)) {
        printf("Failed to write to file\n");
    } else {
        printf("Successfully wrote %u pulses to file\n", pulse_count);
    }
}

// only pulse_width_high_time and pulse_width_low_time are saved to the file (txt)
void savepulse_sequence_toTxt(FIL *file) {
    UINT bw;
    FRESULT fr;
    char buffer[50];
    
    // Iterate over each valid entry in pulsedata_array and write high/low times
    for (uint32_t i = 0; i < pulse_count; i++) {
        printf("Debug: Pulse %d - High Time: %u, Low Time: %u\n", 
               i + 1, 
               pulsedata_array[i].pulse_width_high_time,
               pulsedata_array[i].pulse_width_low_time);
               
        snprintf(buffer, sizeof(buffer), "%u %u ", 
                 pulsedata_array[i].pulse_width_high_time,
                 pulsedata_array[i].pulse_width_low_time);

        fr = f_write(file, buffer, strlen(buffer), &bw);
        if (fr != FR_OK || bw < strlen(buffer)) {
            printf("Failed to write to text file\n");
            break;
        }
        printf("Saving pulse %d: High = %u, Low = %u\n", i + 1, 
               pulsedata_array[i].pulse_width_high_time, 
               pulsedata_array[i].pulse_width_low_time);
    }
    printf("Successfully wrote %u pulses to text file\n", pulse_count);
}

int writePulseHighLowToSD()
{
    // mutex_enter_blocking(&pulse_data_mutex);

    printf("Starting write operation...\n");
    // check if fs is working properly
    if (!fs_mounted) {
        printf("Filesystem not mounted!\n");
        return -1;
    }

    // Create a unique binary filename
    char bin_filename[20];
    snprintf(bin_filename, sizeof(bin_filename), "file%d.bin", filename_counter++);

    // Create a unique text filename
    char txt_filename[20];
    snprintf(txt_filename, sizeof(txt_filename), "file%d.txt", filename_counter++);

    // writing binary -----------------------------------------------
    // Attempt to create and open the binary file for writing
    printf("Attempting to create binary file: %s\n", bin_filename);
    FRESULT fr = f_open(&fil, bin_filename, FA_WRITE | FA_CREATE_ALWAYS);
    if (fr != FR_OK) {
        printf("Failed to open binary file for writing. Error code: ");
        print_fresult(fr);
        return -1;
    }
    printf("Binary file opened successfully\n");
    // Save the pulse sequence (only high and low times) to the binary file
    savepulse_sequenceToBin(&fil);
    // Close the binary file after writing the pulses
    f_close(&fil);
    printf("Binary file closed successfully\n");
    // -------------------------------------------------------------

    // writing text ------------------------------------------------
    // Attempt to create and open the text file for writing
    printf("Attempting to create text file: %s\n", txt_filename);
    fr = f_open(&fil, txt_filename, FA_WRITE | FA_CREATE_ALWAYS);
    if (fr != FR_OK) {
        printf("Failed to open text file for writing. Error code: ");
        print_fresult(fr);
        return -1;
    }
    printf("Text file opened successfully\n");
    // Write the pulse data (only high and low times) to the text file in a human-readable format
    savepulse_sequence_toTxt(&fil);
    // Close the text file after writing
    f_close(&fil);
    printf("Text file closed successfully\n");
    // -----------------------------------------------------------
    
    // Clear the array and reset pulse count
    // currently with this code, reading via GP21 will not work
    memset(pulsedata_array, 0, pulsedata_array_size * sizeof(PulseData));
    pulse_count = 0;

    // mutex_exit(&pulse_data_mutex);
    return 0;
}

// reading from sd card - list all files (from pico)
int readSD(void)
{
    printf("Starting read operation...\n");
    // performing check if fs is working properly
    if (!fs_mounted)
    {
        printf("Filesystem not mounted!\n");
        return -1;
    }
    printf("Attempting to open directory...\n");
    fr = f_opendir(&dir, "/");
    if (fr != FR_OK)
    {
        printf("Failed to open directory: ");
        print_fresult(fr);
        return -1;
    }
    // simple design for easier viewing
    printf("SD card mounted successfully!\n");
    printf("====================================================================\n");
    printf("| %-20s | %-30s \n", "Name", "Pulse Data");
    printf("====================================================================\n");

    while (1)
    {
        fr = f_readdir(&dir, &fno);
        if (fr != FR_OK || fno.fname[0] == 0)
        {
            break;
        }

        // If it's a file (not a directory) and has a .bin extension
        if (!(fno.fattrib & AM_DIR) && strstr(fno.fname, ".bin"))
        {
            // display file name
            printf("| %-20s | \n", fno.fname);
            // Calculate number of pulse pairs (each pulse has high and low time)
            uint32_t file_size = fno.fsize;
            uint32_t num_pulses = file_size / (2 * sizeof(uint32_t)); // Each pulse has 2 uint32_t values

            if (num_pulses > 0)
            {
                // Allocate memory for high and low times
                uint32_t *pulse_times = (uint32_t *)malloc(2 * num_pulses * sizeof(uint32_t));
                if (pulse_times == NULL)
                {
                    printf("Failed to allocate memory for reading pulse data.\n");
                    continue;
                }

                // Open the file for reading
                fr = f_open(&fil, fno.fname, FA_READ);
                if (fr == FR_OK)
                {
                    UINT br;
                    fr = f_read(&fil, pulse_times, 2 * num_pulses * sizeof(uint32_t), &br);
                    if (fr != FR_OK || br < 2 * num_pulses * sizeof(uint32_t))
                    {
                        printf("Failed to read pulse data from file (read %u bytes).\n", br);
                    }
                    else
                    {
                        // Print all the pulses read from the file
                        for (int i = 0; i < num_pulses; i++)
                        {
                            printf("--- Digital Signal Analyzer33 ---\n");
                            printf("Timestamp: %u\n", 123456789 + i * 100000); // Example timestamp

                            uint32_t high_time = pulse_times[2 * i];
                            uint32_t low_time = pulse_times[2 * i + 1];
                            uint32_t total_time = high_time + low_time;
                            float duty_cycle = (float)high_time / total_time * 100.0f;
                            float frequency = 1000000.0f / total_time; // Convert to Hz

                            printf("Frequency33: %.2f Hz   Duty Cycle: %.2f%%\n", frequency, duty_cycle);
                            printf("Total Pulse Width: %u us\n", total_time);
                            printf("PW High Time: %u us   PW Low Time: %u us\n", high_time, low_time);
                            printf("\n");
                        }
                    }
                    f_close(&fil);
                }
                else
                {
                    printf("Failed to open file for reading: ");
                    print_fresult(fr);
                }
                free(pulse_times);
            }
            else
            {
                printf("File %s is empty or contains no valid pulse data.\n", fno.fname);
            }
        }
    }
    printf("====================================================================\n");
    f_closedir(&dir);
    return 0;
}

int main()
{
    free_pulsedata_array();
    init_system_with_filesystem();
    setup_adc();
    // simulate_pulse_sequence(pulse_count);

    // initialize pulsedata array to 10 -> for 10 pulses atm
    initialize_pulsedata_array(10);

    // Initialize timers
    struct repeating_timer timer1, timer2;
    add_repeating_timer_ms(ADC_SAMPLE_RATE, read_adc, NULL, &timer1); // Timer for reading ADC, 3s interval
    add_repeating_timer_ms(1000, measure_digi, NULL, &timer2);        // Timer for reading digital signal, 1s interval

    while (1)
    {
        // Check if the write button was pressed
        if (write_flag)
        {
            write_flag = false;                   // Reset the flag
            int result = writePulseHighLowToSD(); // Perform the write operation
            printf("WriteToSD result: %d\n", result);
        }
        // Check if the read button was pressed
        if (read_flag)
        {
            read_flag = false;     // Reset the flag
            int result = readSD(); // Perform the read operation
            printf("ReadSD result: %d\n", result);
        }

        // Check if the exit button (GP22) is pressed
        if (gpio_get(EXIT_BTNPIN_22) == 0)
        { // Button pressed (active low)
            printf("Exit button pressed. Exiting the loop...\n");
            cleanup_exit();
            break; // Exit the loop
        }

        tight_loop_contents(); // Prevent the system from sleeping and keep checking
        sleep_ms(100);         // Add a delay to reduce CPU usage
    }
    // free_pulsedata_array();
    cleanup_exit();
    printf("Program exited\n");
    return 0; // This will never be reached
}



