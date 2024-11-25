#include "b2_functions.h"

volatile pulse_t pulses[PULSE_NUM + 1];
volatile uint32_t pulse_index = 0;
volatile uint32_t last_pulse_time = 0;
volatile uint32_t pulse_last_edge_time = 0;

volatile bool adc_timer = false;
volatile adc_t adc_results[SAMPLE_SIZE];
volatile float adc_signal[SAMPLE_SIZE];
volatile uint32_t adc_index = 0;
volatile uint32_t sample_index = 0;
volatile uint32_t adc_last_edge_time = 0;

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

// Start Pulse section =========================================
void read_pulse(uint gpio, uint32_t events)
{
    uint64_t current_time = to_us_since_boot(get_absolute_time());

    float pulse_duration_us = (float)(current_time - pulse_last_edge_time);
    pulse_last_edge_time = current_time;

    if (events & GPIO_IRQ_EDGE_RISE)
        pulses[pulse_index].pulse_width_low_time = pulse_duration_us;

    if (events & GPIO_IRQ_EDGE_FALL)
    {
        pulses[pulse_index].pulse_width_high_time = pulse_duration_us;

        if (pulses[pulse_index].pulse_width_low_time > 0 &&
            pulses[pulse_index].pulse_width_high_time > 0)
        {
            pulses[pulse_index].total_pulse_width =
                pulses[pulse_index].pulse_width_low_time +
                pulses[pulse_index].pulse_width_high_time;

            printf("Pulse %d - High Duration: %.2f us, Low Duration: %.2f us, Total Duration: %.2f\n",
                   pulse_index + 1,
                   pulses[pulse_index].pulse_width_high_time,
                   pulses[pulse_index].pulse_width_low_time,
                   pulses[pulse_index].total_pulse_width);

            pulse_index++;
        }

        if (pulse_index >= PULSE_NUM + 1)
        {
            // print_pulses();
            gpio_set_irq_enabled_with_callback(PULSE_PIN,
                                               GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
                                               false, &gpio_callback);
        }
    }
}

void print_pulses()
{
    printf("\n--- Pulse Analyzer ---\n");
    for (int i = 0; i < pulse_index; i++)
    {
        printf("Pulse %d - High Duration: %.2f us, Low Duration: %.2f us, Total Duration: %.2f\n",
               i + 1,
               pulses[i].pulse_width_high_time,
               pulses[i].pulse_width_low_time,
               pulses[i].total_pulse_width);
    }
}
// End Pulse section =========================================

// Start ADC section =========================================
void read_adc(bool signal_above_threshold)
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
                print_adc();
                sample_index = 0;
            }
        }
        else
            adc_last_edge_time = current_time;

        sleep_us(DEBOUNCE_DELAY);
    }
    else if (adc_value < SAMPLE_THRESHOLD && signal_above_threshold)
        signal_above_threshold = false;
}

void print_adc()
{
    uint64_t total_interval = 0;
    for (int i = 0; i < SAMPLE_SIZE; i++)
        total_interval += adc_signal[i];

    float average_interval = total_interval / (float)SAMPLE_SIZE;

    adc_results[adc_index].adc_frequency = 1.0 / (average_interval / 1e6);
    adc_results[adc_index].adc_rms_value = calculate_rms();
    adc_results[adc_index].adc_snr_value = calculate_snr();
    adc_results[adc_index].adc_peak_to_peak_value = calculate_peak_to_peak();

    printf("\n--- Analog Signal Analyzer ---\n");
    printf("Average Frequency: %.2f Hz\n",
           adc_results[adc_index].adc_frequency);
    printf("RMS Voltage: %.2fV\tPeak-to-Peak Voltage: %.2fV\nSignal-to-Noise Ratio (SNR): %.2fdB\n",
           adc_results[adc_index].adc_rms_value,
           adc_results[adc_index].adc_peak_to_peak_value,
           adc_results[adc_index].adc_snr_value);

    adc_index++;
}

float calculate_rms()
{
    float sum_squares = 0.0f;
    for (int i = 0; i < SAMPLE_SIZE; i++)
        sum_squares += adc_signal[i] * adc_signal[i];

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
// End ADC section =========================================

// Start PWM section =========================================
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
        printf("Frequency: %.2f Hz\tDuty Cycle: %.2f%%\n",

               pwm_frequency, pwm_duty_cycle);
        printf("Total Pulse Width: %u us\n", period);
        printf("PW High Time: %u us\tPW Low Time: %u us\n",
               high_time, low_time);

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
        float duty_cycle =
            ((float)high_time / (float)period) * 100.0f; // Calculate duty cycle percentage
        return duty_cycle;
    }
    return 0.0f; // Avoid division by zero
}
// End PWM section =========================================

// Start MISC section =========================================
bool get_all_values(struct repeating_timer *t)
{
    printf("\nGetting all values...\n");
    printf("Printing stored pulses (%i):\n", pulse_index - 1);
    for (int i = 1; i < pulse_index; i++)
    {
        printf("Pulse %d - High Duration: %.2f us, Low Duration: %.2f us, Total Duration: %.2f\n",
               i + 1,
               pulses[i].pulse_width_high_time,
               pulses[i].pulse_width_low_time,
               pulses[i].total_pulse_width);
    }
    printf("ADC: Average Frequency: %.2f Hz\n",
           adc_results[adc_index - 1].adc_frequency);
    printf("PWM: Frequency: %.2f Hz\tDuty Cycle: %.2f\n",
           pwm_frequency, pwm_duty_cycle);
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

void setup_pins()
{
    gpio_init(PULSE_PIN);
    gpio_set_dir(PULSE_PIN, GPIO_IN);
    gpio_pull_down(PULSE_PIN);
    gpio_set_irq_enabled_with_callback(PULSE_PIN,
                                       GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
                                       true, &gpio_callback);

    gpio_init(ADC_PIN);
    gpio_set_dir(ADC_PIN, GPIO_IN);
    gpio_pull_up(ADC_PIN);

    gpio_init(DIGI_PIN);
    gpio_set_dir(DIGI_PIN, GPIO_IN);
    gpio_pull_down(DIGI_PIN);
    gpio_set_irq_enabled_with_callback(DIGI_PIN,
                                       GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
                                       true, &gpio_callback);
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
// End MISC section =========================================