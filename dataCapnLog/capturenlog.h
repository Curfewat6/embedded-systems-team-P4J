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

// Create instances for FATFS and FIL
extern FATFS fs;
extern FIL fil; // File object

extern DIR dir;
extern FILINFO fno;
extern FRESULT fr;

enum ProtocolType {
    PROTOCOL_UART,
    PROTOCOL_I2C,
    PROTOCOL_SPI,
    PROTOCOL_PWM,
    PROTOCOL_GPIO
};

// define button for GP20 (acting as creating file to sd)
extern const uint WRITE_BTNPIN_20;
// define button for GP21 (acting as reading file from sd)
extern const uint READ_BTNPIN_21;
// define button for GP22 (acting as exiting program)
extern const uint EXIT_BTNPIN_22;

// Used for pulse measurement
#define PULSE_PIN 2
#define PULSE_NUM 10

#define PULSE_PIN1 3

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

#define DEBOUNCE_DELAY 300

typedef struct
{
    float high_duration;
    float low_duration;
} pulse_t;


extern volatile pulse_t pulses[PULSE_NUM + 2];
extern volatile bool pulse_read;
extern volatile uint8_t pulse_index;
extern volatile uint64_t last_pulse_time;
extern volatile uint64_t pulse_last_edge_time;


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

//testing
extern volatile uint32_t pwm_high_time;
extern volatile uint32_t pwm_low_time;

extern volatile uint32_t period;
extern volatile uint32_t pwm_period;

extern volatile uint32_t total_width;
extern volatile uint32_t measurement_count;
extern volatile bool new_cycle_complete;

// testing
extern char detected_protocol_name_buffer[10];
const char* getProtocol(enum ProtocolType protocol);

void read_pulse(uint gpio, uint32_t events);
bool read_adc(struct repeating_timer *t);
float calculate_rms();
float calculate_peak_to_peak();
float calculate_snr();
float adc_calculate_frequency();

void read_digi(uint gpio, uint32_t events);
bool measure_digi(struct repeating_timer *t);
float digi_calculate_frequency();
float calculate_duty_cycle();

static int filename_counter;
static absolute_time_t last_button_press;

static bool fs_mounted;

typedef struct
{
    uint32_t timestamp; // need the buddies to retrieve from web console via NTP
    uint32_t frequency;
    float duty_cycle;
    uint32_t total_pulse_width; // Duration of the pulse in microseconds (from falling to rising edge)
    uint32_t pulse_width_high_time;
    uint32_t pulse_width_low_time;
} PWMSignalData;

typedef struct{
    // float pulse_width_high_time;
    // float pulse_width_low_time;
    uint32_t timestamp; // need the buddies to retrieve from web console via NTP
    uint32_t frequency;
    float duty_cycle;
    uint32_t total_pulse_width; // Duration of the pulse in microseconds (from falling to rising edge)
    uint32_t pulse_width_high_time;
    uint32_t pulse_width_low_time;
} PulseData;

typedef struct{
    uint32_t timestamp;
    float rms_value;
    float peak_to_peak_value;
    float snr_value;
    float frequency;
} AnalogData;

extern PulseData *pulsedata_array; // Pointer for dynamic memory allocation for PulseData
// uint32_t pulse_count = 0; //Counter to keep track of number of pulses
extern uint32_t pulsedata_array_size;// Size of the array (to make dynamic)

extern PWMSignalData *pwmsignaldata_array; // Pointer for dynamic memory allocation for PulseData
extern uint32_t pwmsignaldata_array_size; // Size of the array (to make dynamic)

extern AnalogData *analogdata_array;
extern uint32_t analogdata_array_size;

void initialize_pulsedata_array(uint32_t size);
void resize_pulsedata_array(uint32_t new_size);

void print_fresult(FRESULT fr);
void free_pulsedata_array();

// freeing dynamic memory allocation for PulseData array
void cleanup_exit();

// Flags for different operations coordination
extern volatile bool write_flag; // Flag for write operation
extern volatile bool read_flag;  // Flag for read operation
extern volatile bool pulse_flag; // Flag for pulse detection
extern volatile bool adc_flag;   // Flag for ADC reading
extern volatile bool digi_flag;  // Flag for digital signal reading
extern volatile bool pwm_flag;   // Flag for PWM signal reading

// Flag to detect the communication protocol
extern volatile bool uart_flag;
extern volatile bool i2c_flag;
extern volatile bool spi_flag;
extern volatile bool pwm_flag;
extern volatile bool gpio_flag;

//testing
void setup_pwm(uint gpio, float freq, float duty_cycle);

void initialize_arrays(uint32_t size);
void resize_arrays(uint32_t new_size);
void print_fresult(FRESULT fr);

void free_pulsedata_array();
void button_callback(uint gpio, uint32_t events);

void init_buttons(void);
void setup_adc();

void init_system_with_filesystem();

void savesinglepulse_sequenceToBin(FIL *file);
void savesinglepulse_sequenceToTxt(FIL *file);
int writeToSDCard();

//testing
int writeToSDCard2(uint32_t timestamp, const char* protocol_name);

void savepwmpulse_sequenceToBin(FIL *file);
void savepwmpulse_sequenceToTxt(FIL *file);

//testing
void saveSignalsToBin(FIL *file);
void saveSignalsToTxt(FIL *file);

int writePWMSignalToSD(enum ProtocolType protocol);

void saveanalog_signalToBin(FIL *file);
void saveanalog_signalToTxt(FIL *file);
int writeAnalogSignalToSD();

void appendPulse_pulsedataArray(float high_time, float low_time);
void appendPWMPulse_pulsedataArray(float frequency, float duty_cycle, uint32_t period, uint32_t high_time, uint32_t low_time);
void appendAnalogSignal_pulsedataArray(float rms, float peak_to_peak, float snr, float frequency);

void read_pulse(uint gpio, uint32_t events);
bool read_adc(struct repeating_timer *t);

float calculate_rms();
float calculate_peak_to_peak();
float calculate_snr();

float adc_calculate_frequency();
void read_digi(uint gpio, uint32_t events);

bool measure_digi(struct repeating_timer *t);
float digi_calculate_frequency();

float calculate_duty_cycle();

int readSD(void);

int retrieveDataFromTxt(const char *filename);

//testing
int retrieveDataFromTxt2(const char *filename);

//testing
int retrieveDataFromTxt3(const char *filename, char *content, size_t max_len);