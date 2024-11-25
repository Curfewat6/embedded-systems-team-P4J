#include "capturenlog.h"

FATFS fs;
FIL fil; // File object

DIR dir;
FILINFO fno;
FRESULT fr;

volatile pulse_t pulses[PULSE_NUM + 2];
volatile bool pulse_read = false;
volatile uint8_t pulse_index = 0;
volatile uint64_t last_pulse_time = 0;
volatile uint64_t pulse_last_edge_time = 0;
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

volatile uint32_t pwm_high_time = 0;
volatile uint32_t pwm_low_time = 0;

volatile uint32_t period = 0;

volatile uint32_t pwm_period = 0;

volatile uint32_t total_width = 0;
volatile uint32_t measurement_count = 0;
volatile bool new_cycle_complete = false;

const uint WRITE_BTNPIN_20 = 20;
const uint READ_BTNPIN_21 = 21;
const uint EXIT_BTNPIN_22 = 22;

PulseData *pulsedata_array = NULL; // Pointer for dynamic memory allocation for PulseData
uint32_t pulsedata_array_size = 0; // Size of the array (to make dynamic)

PWMSignalData *pwmsignaldata_array = NULL; // Pointer for dynamic memory allocation for PulseData
uint32_t pwmsignaldata_array_size = 0; // Size of the array (to make dynamic)

AnalogData *analogdata_array = NULL; // Pointer for dynamic memory allocation for AnalogData
uint32_t analogdata_array_size = 0; // Size of the array (to make dynamic)

volatile bool write_flag = false; // Flag for write operation
volatile bool read_flag = false;  // Flag for read operation

char detected_protocol_name_buffer[10];

const char* getProtocol(enum ProtocolType protocol) {
    switch (protocol) {
        case PROTOCOL_UART:
            return "UART";
        case PROTOCOL_I2C:
            return "I2C";
        case PROTOCOL_SPI:
            return "SPI";
        case PROTOCOL_PWM:
            return "PWM";
        case PROTOCOL_GPIO:
            return "GPIO";
        default:
            return "UNKNOWN";
    }
}

// volatile bool pulse_flag = false; // Flag for pulse detection
// volatile bool adc_flag = false;   // Flag for ADC reading
// volatile bool digi_flag = false;  // Flag for digital signal reading
// volatile bool pwm_flag = false;   // Flag for PWM signal reading

// // Flag to detect the communication protocol
volatile bool uart_flag = false;
volatile bool i2c_flag = false;
volatile bool spi_flag = false;
volatile bool pwm_flag = false;
volatile bool gpio_flag = false;

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

//I want to see if its possible to just use 1 array for all the signals
//a Signal struct
void initialize_arrays(uint32_t size) {
    pulsedata_array_size = size;
    pulsedata_array = (PulseData *)malloc(pulsedata_array_size * sizeof(PulseData));
    pwmsignaldata_array_size = size;
    pwmsignaldata_array = (PWMSignalData *)malloc(pwmsignaldata_array_size * sizeof(PWMSignalData));
    analogdata_array_size = size;
    analogdata_array = (AnalogData *)malloc(analogdata_array_size * sizeof(AnalogData));
    if (pulsedata_array == NULL) {
        printf("Failed to allocate memory for PulseData array\n");
        exit(1);
    }
    if (pwmsignaldata_array == NULL) {
        printf("Failed to allocate memory for pwm array\n");
        exit(1);
    }
    if (analogdata_array == NULL) {
        printf("Failed to allocate memory for AnalogData array\n");
        exit(1);
    }
    printf("Initialized PulseData array with size: %u\n", pulsedata_array_size);
}

// adjust here accordingly if become 1 array only
void resize_arrays(uint32_t new_size) {
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

void print_fresult(FRESULT fr){
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
void free_pulsedata_array(){
    if (pulsedata_array != NULL)
    {
        free(pulsedata_array);
        pulsedata_array = NULL;
    }
}

void cleanup_exit(){
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

void button_callback(uint gpio, uint32_t events){
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
    if (gpio == DIGI_PIN)
        read_digi(gpio, events);
    else if (gpio == EXIT_BTNPIN_22)
    {
        cleanup_exit();
    }
}

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
    
    gpio_init(DIGI_PIN);
    gpio_set_dir(DIGI_PIN, GPIO_IN);
    gpio_pull_down(DIGI_PIN);
    

    gpio_set_irq_enabled_with_callback(PULSE_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &button_callback);
    gpio_set_irq_enabled_with_callback(DIGI_PIN,
                                       GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
                                       true, &button_callback);
    
    // Kane's btn
    gpio_init(PULSE_PIN1);
    gpio_set_dir(PULSE_PIN1, GPIO_OUT);

    last_button_press = get_absolute_time();
}

// Initializes the ADC (not my part)
// void setup_adc()
// {
//     adc_init();
//     adc_gpio_init(ADC_PIN);
//     adc_select_input(0);
// }

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

// only pulse_width_high_time and pulse_width_low_time are saved to the file (binary)
// currently file is catering for GPIO signals only
void saveSignalsToBin(FIL *file){
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
// currently file is catering for GPIO signals only
void saveSignalsToTxt(FIL *file) {
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

//Protocol - GPIO
void read_pulse(uint gpio, uint32_t events){ 
    enum ProtocolType protocol = PROTOCOL_GPIO;
    const char* protocol_detected = getProtocol(protocol); //this returns "GPIO"
    //putting the protocol detected into detected_protocol_name_buffer
    strncpy(detected_protocol_name_buffer, protocol_detected, sizeof(detected_protocol_name_buffer) - 1);
    detected_protocol_name_buffer[sizeof(detected_protocol_name_buffer) - 1] = '\0'; // Null-terminate the string

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

//Protocol - PWM
void read_digi(uint gpio, uint32_t events)
{   
    enum ProtocolType protocol = PROTOCOL_PWM;
    const char* protocol_detected = getProtocol(protocol); //this returns "PWM"
    //putting the protocol detected into detected_protocol_name_buffer
    strncpy(detected_protocol_name_buffer, protocol_detected, sizeof(detected_protocol_name_buffer) - 1);
    detected_protocol_name_buffer[sizeof(detected_protocol_name_buffer) - 1] = '\0'; // Null-terminate the string


    uint32_t current_time = time_us_32();

    if (events & GPIO_IRQ_EDGE_RISE)
    {
        if (last_fall_time != 0)
            pwm_low_time = current_time - last_fall_time;

        if (last_rise_time != 0)
        {
            pwm_period = current_time - last_rise_time;
            total_width += pwm_period;
            measurement_count++;

            new_cycle_complete = true;
        }

        last_rise_time = current_time;
    }
    else if (events & GPIO_IRQ_EDGE_FALL)
    {
        if (last_rise_time != 0)
            pwm_high_time = current_time - last_rise_time;

        last_fall_time = current_time;
    }
}

bool measure_digi(struct repeating_timer *t)
{
    if (new_cycle_complete) {
        float frequency = digi_calculate_frequency(); // Calculate frequency
        float duty_cycle = calculate_duty_cycle();    // Calculate duty cycle

        // Print results
        printf("\n--- Digital Signal Analyzer ---\n");
        printf("Frequency: %.2f Hz\tDuty Cycle: %.2f%%\n", frequency, duty_cycle);
        printf("Total Pulse Width: %u us\n", pwm_period); // Total pulse width
        printf("PW High Time: %u us\tPW Low Time: %u us\n", pwm_high_time, pwm_low_time);

        // Save high and low time to pulsedata_array
        if (pulse_count >= pulsedata_array_size) {
            // Resize the array if it's full
            resize_arrays(pulsedata_array_size * 2); // Double the size as needed
        }
        
        if (pulse_count < pulsedata_array_size) {
            pulsedata_array[pulse_count].pulse_width_high_time = pwm_high_time;
            pulsedata_array[pulse_count].pulse_width_low_time = pwm_low_time;
            pulse_count++;  // Increment pulse count
            
        }else{
            printf("Error: Pulse count exceeds array size.\n");
        }

        

        // Reset for the next cycle
        new_cycle_complete = false;
        total_width = 0;
        measurement_count = 0;
        // pwm_high_time = 0;
        // pwm_low_time = 0;
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
    if (pwm_period > 0)
    {
        float duty_cycle = ((float)pwm_high_time / (float)pwm_period) * 100.0f; // Calculate duty cycle percentage
        return duty_cycle;
    }
    return 0.0f; // Avoid division by zero
}

int writeToSDCard2(uint32_t timestamp, const char* protocol_name){
    // mutex_enter_blocking(&pulse_data_mutex);
    printf("Starting write operation...\n");
    // check if fs is working properly
    if (!fs_mounted) {
        printf("Filesystem not mounted!\n");
        return -1;
    }
    // Create a unique binary filename
    char bin_filename[30];
    // const char* protocol_folder = getProtocolFolder(protocol);

    // uint32_t timestamp;
    snprintf(bin_filename, sizeof(bin_filename), "%d_%sfile%d.bin", timestamp, protocol_name,filename_counter);
    // Create a unique text filename
    char txt_filename[30];
    snprintf(txt_filename, sizeof(txt_filename), "%d_%sfile%d.txt", timestamp, protocol_name,filename_counter);
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
    saveSignalsToBin(&fil);
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
    saveSignalsToTxt(&fil);
    // Close the text file after writing
    f_close(&fil);
    printf("Text file closed successfully\n");
    // -----------------------------------------------------------
    // Clear the array and reset pulse count
    // currently with this code, reading via GP21 will not work
    // memset(pulses, 0, sizeof(pulse_t));
    memset(pulsedata_array, 0, pulsedata_array_size * sizeof(PulseData));
    filename_counter++;
    pulse_count = 0;
    
    //testing
    pulse_index = 0;
    // Re-enable the GPIO interrupt for the pulse pin
    gpio_set_irq_enabled_with_callback(PULSE_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &button_callback);

    return 0;
}

//testing
int findFilesWithProtocol(const char* protocol_name, char found_filenames[][50], size_t max_files) {
    DIR dir;
    FILINFO fno;
    FRESULT fr;
    size_t file_count = 0;

    fr = f_opendir(&dir, "/"); // Open the root directory
    if (fr != FR_OK) {
        printf("Failed to open directory: ");
        print_fresult(fr);
        return -1;
    }

    while (1) {
        fr = f_readdir(&dir, &fno); // Read a directory item
        if (fr != FR_OK || fno.fname[0] == 0) break; // Break on error or end of dir

        if (strstr(fno.fname, protocol_name) != NULL) { // Check if the filename contains the protocol name
            strncpy(found_filenames[file_count], fno.fname, 49);
            found_filenames[file_count][49] = '\0'; // Null-terminate the string
            file_count++;
            if (file_count >= max_files) break; // Stop if max_files is reached
        }
    }

    f_closedir(&dir);
    return file_count; // Return the number of files found
}

int retrieveDataFromTxt2(const char *filename)
{
    printf("Starting read operation for file: %s\n", filename);
    // Check if the filesystem is mounted
    if (!fs_mounted) {
        printf("Filesystem not mounted!\n");
        return -1;
    }

    // Open the specified file for reading
    FRESULT fr = f_open(&fil, filename, FA_READ);
    if (fr != FR_OK) {
        printf("Failed to open file for reading: ");
        print_fresult(fr);
        return -1;
    }

    printf("File opened successfully: %s\n", filename);
    printf("====================================================================\n");
    printf("| %-20s | %-30s \n", "Name", "File Content");
    printf("====================================================================\n");

    // Display file name
    printf("| %-20s | ", filename);

    // Read the file contents
    char buffer[128];
    UINT br;
    while ((fr = f_read(&fil, buffer, sizeof(buffer) - 1, &br)) == FR_OK && br > 0) {
        buffer[br] = '\0';  // Null-terminate the string
        printf("%s", buffer);  // Print file contents
    }
    if (fr != FR_OK) {
        printf("Failed to read from file: ");
        print_fresult(fr);
    }

    printf("\n====================================================================\n");

    // Close the file after reading
    f_close(&fil);
    return 0;
}

//testing
int retrieveDataFromTxt3(const char *filename, char *content, size_t max_len) {
    // printf("Starting read operation for file: %s\n", filename);
    // Check if the filesystem is mounted
    if (!fs_mounted) {
        printf("Filesystem not mounted!\n");
        return -1;
    }

    // Open the specified file for reading
    FRESULT fr = f_open(&fil, filename, FA_READ);
    if (fr != FR_OK) {
        printf("Failed to open file for reading: ");
        print_fresult(fr);
        return -1;
    }

    // printf("File opened successfully: %s\n", filename);

    // Read the file contents
    char buffer[128];
    UINT br;
    size_t content_len = 0;
    while ((fr = f_read(&fil, buffer, sizeof(buffer) - 1, &br)) == FR_OK && br > 0) {
        buffer[br] = '\0';  // Null-terminate the string
        content_len += snprintf(content + content_len, max_len - content_len, "%s", buffer);
        if (content_len >= max_len) break; // Stop if max_len is reached
    }
    if (fr != FR_OK) {
        printf("Failed to read from file: ");
        print_fresult(fr);
    }

    // Close the file after reading
    f_close(&fil);
    return 0;
}

int main(){
    
    // free_pulsedata_array();
    init_system_with_filesystem();
    // setup_adc();
    // simulate_pulse_sequence(pulse_count);

    // initialize pulsedata array to 10 -> for 10 pulses atm
    initialize_arrays(10);

    setup_pwm(0, 600.0f, 0.5f);
    struct repeating_timer timer1, timer2;
    add_repeating_timer_ms(1000, measure_digi, NULL, &timer1);
    

    //get timestamp from buddy 2
    uint32_t timestamp = 123456789; // Example timestamp
    while (1)
    {
        
        // Check if the write button was pressed
        if (write_flag)
        {
            write_flag = false;                   // Reset the flag

            // i think here need to determine what protocol is buddy 2 saving
            // if (xxx flag == true) {}
            int result = writeToSDCard2(timestamp, detected_protocol_name_buffer); // Perform the write operation
            printf("WriteToSD result: %d\n", result);
        }
        // Check if the read button was pressed
        if (read_flag)
        {
            read_flag = false;     // Reset the flag
            
            // Find files with the specific protocol name
            char found_filenames[10][50]; // Array to store up to 10 filenames
            
            int file_count = findFilesWithProtocol(detected_protocol_name_buffer, found_filenames, 10);
            

            //testing
            if (file_count > 0) {
                printf("Found %d files with protocol name '%s':\n", file_count, detected_protocol_name_buffer);
                printf("====================================================================\n");
                printf("| %-20s | %-30s \n", "Name", "File Content");
                printf("====================================================================\n");
                for (int i = 0; i < file_count; i++) {
                    char content[1024] = {0}; // Buffer to store file content
                    int result = retrieveDataFromTxt3(found_filenames[i], content, sizeof(content));
                    // printf("ReadSD result: %d\n", result);
                    printf("| %-20s | %s\n", found_filenames[i], content);
                    printf("====================================================================\n");
                }
            } else {
                printf("No files with protocol name '%s' found\n", detected_protocol_name_buffer);
            }
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


