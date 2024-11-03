#ifndef PULSE_DATA_H
#define PULSE_DATA_H

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
extern mutex_t pulse_data_mutex;

// Create instances for FATFS and FIL
extern FATFS fs;
extern FIL fil; // File object

extern DIR dir;
extern FILINFO fno;
extern FRESULT fr;

#define MAX_DATA_POINTS 10 // Limit the number of data points

// Global array to hold the frequency values
extern float pwm_frequency_values[MAX_DATA_POINTS]; 
extern int pwm_frequency_index; 

extern float pwm_duty_cycle_values[MAX_DATA_POINTS];
extern int pwm_duty_cycle_index;

extern float adc_frequency_values[MAX_DATA_POINTS]; 
extern int adc_frequency_index; 


// define button for GP20 (acting as creating file to sd)
extern const uint WRITE_BTNPIN_20;
// define button for GP21 (acting as reading file from sd)
extern const uint WRITE_BTNPIN_21;
// define button for GP22 (acting as exiting program)
extern const uint WRITE_BTNPIN_22;

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
extern volatile uint32_t prev_rise_time;
extern volatile uint32_t prev_fall_time;
extern volatile uint32_t pulse_count;

// Used for analog signal measurement
extern volatile bool adc_timer;
extern volatile uint32_t buffer_index;
extern volatile uint32_t sample_index; 
extern volatile uint32_t cycles_counted;
extern volatile float adc_signal[SAMPLE_SIZE];
extern char adc_scan_buffer[BUFFER_SIZE];

// Used for digital signal measurement
extern volatile uint32_t last_rise_time;
extern volatile uint32_t last_fall_time;
extern volatile uint32_t high_time;
extern volatile uint32_t low_time;
extern volatile uint32_t period;
extern volatile uint32_t total_width;
extern volatile uint32_t measurement_count;
extern volatile bool new_cycle_complete;

void init_mutexes();
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
// -------------------------------------------

// simple counter tht increments when creating file
static int filename_counter;

// for debounce configuration (300ms)
#define DEBOUNCE_DELAY 300
static absolute_time_t last_button_press;

// Global variable to track if filesystem is mounted
static bool fs_mounted;

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

extern PulseData *pulsedata_array; // Pointer for dynamic memory allocation for PulseData
// uint32_t pulse_count = 0; //Counter to keep track of number of pulses
extern uint32_t pulsedata_array_size;// Size of the array (to make dynamic)
void initialize_pulsedata_array(uint32_t size);
void resize_pulsedata_array(uint32_t new_size);

// ---------------------------------------------------------------
// result to verify the file status
void print_fresult(FRESULT fr);
// freeing dynamic memory allocation for PulseData array
void free_pulsedata_array();
// performing freeing memory allocation for PulseData array
// terminating of processes
void cleanup_exit();
extern volatile bool write_flag;// Flag for write operation
extern volatile bool read_flag;// Flag for read operation
void button_callback(uint gpio, uint32_t events);
// to initialize the buttons
void init_buttons(void);
// Initializes the ADC
void setup_adc();
void init_system_with_filesystem();
// seems to work fine writing high time and low time to txt correctly based on output.
void read_pulse(uint gpio, uint32_t events);
// Reads the ADC value, calculates RMS, peak-to-peak, and SNR values, and prints the results.
// Also prints the ADC values captured
void read_adc();
// Calculates the RMS value of the analog signal
float calculate_rms();
// Calculates the peak-to-peak value of the analog signal
float calculate_peak_to_peak();
// Calculates the signal-to-noise ratio of the analog signal
float calculate_snr();
// Function to calculate frequency based on zero crossings or peaks
float adc_calculate_frequency();
// Reads the digital signal, calculates frequency, duty cycle, and pulse width, and prints the results
void read_digi(uint gpio, uint32_t events);
// Prints the results of the digital signal analysis if a new cycle is complete
//void measure_digi(struct repeating_timer *t);
void measure_digi();
// Calculate frequency using averaged pulse width
float digi_calculate_frequency();
// Calculate duty cycle
float calculate_duty_cycle();
// only pulse_width_high_time and pulse_width_low_time are saved to the file (binary)
void savepulse_sequenceToBin(FIL *file);
// only pulse_width_high_time and pulse_width_low_time are saved to the file (txt)
void savepulse_sequence_toTxt(FIL *file);
int writePulseHighLowToSD();
// reading from sd card - list all files (from pico)
int readSD(void);
//testing
int retrievePulseDataFromTxt(const char *filename);


// int main()
// {
//     free_pulsedata_array();
//     init_system_with_filesystem();
//     setup_adc();
//     // simulate_pulse_sequence(pulse_count);

//     // initialize pulsedata array to 10 -> for 10 pulses atm
//     initialize_pulsedata_array(10);

//     // Initialize timers
//     struct repeating_timer timer1, timer2;
//     add_repeating_timer_ms(ADC_SAMPLE_RATE, read_adc, NULL, &timer1); // Timer for reading ADC, 3s interval
//     add_repeating_timer_ms(1000, measure_digi, NULL, &timer2);        // Timer for reading digital signal, 1s interval

//     while (1)
//     {
//         // Check if the write button was pressed
//         if (write_flag)
//         {
//             write_flag = false;                   // Reset the flag
//             int result = writePulseHighLowToSD(); // Perform the write operation
//             printf("WriteToSD result: %d\n", result);
//         }
//         // Check if the read button was pressed
//         if (read_flag)
//         {
//             read_flag = false;     // Reset the flag
//             // int result = readSD(); // Perform the read operation 
            
//             // experimental 
//             int result = retrievePulseDataFromTxt("file2.txt");
//             printf("ReadSD result: %d\n", result);
//         }

//         // Check if the exit button (GP22) is pressed
//         if (gpio_get(EXIT_BTNPIN_22) == 0)
//         { // Button pressed (active low)
//             printf("Exit button pressed. Exiting the loop...\n");
//             cleanup_exit();
//             break; // Exit the loop
//         }

//         tight_loop_contents(); // Prevent the system from sleeping and keep checking
//         sleep_ms(100);         // Add a delay to reduce CPU usage
//     }
//     // free_pulsedata_array();
//     cleanup_exit();
//     printf("Program exited\n");
//     return 0; // This will never be reached
// }

#endif // PULSE_DATA_H
