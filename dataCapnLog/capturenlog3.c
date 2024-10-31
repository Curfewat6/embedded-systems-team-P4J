#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/rtc.h"
#include <string.h>  // Include string.h for strlen
#include <stdlib.h>  // Include stdlib.h for free
#include "hardware/spi.h"
#include "ff.h"  // FATFS library for file operations

#include "hardware/pwm.h"  // PWM library for pulse generation

#include <time.h>  // For random number generation

// Create instances for FATFS and FIL
FATFS fs;
FIL fil;  // File object

DIR dir;
FILINFO fno;
FRESULT fr;

// define button for GP20 (acting as creating file to sd)
const uint WRITE_BTNPIN_20 = 20;
// define button for GP21 (acting as reading file from sd)
const uint READ_BTNPIN_21 = 21;
// define button for GP22 (acting as exiting program)
const uint EXIT_BTNPIN_22 = 22;

//simple counter tht increments when creating file
static int filename_counter = 1;

// for debounce configuration (300ms)
#define DEBOUNCE_DELAY 300
static absolute_time_t last_button_press;

// Global variable to track if filesystem is mounted
static bool fs_mounted = false;

// experimental ------------------------------------------------
typedef struct{
    uint32_t timestamp; // need the buddies to retrieve from web console via NTP
    bool state; //TRUE for high, FALSE for low
    uint32_t duration;  // Time duration the pulse stayed in the current state (optional for now)
    //optional atm
    // float frequency;
    //optional atm
    // float duty_cycle;
    uint32_t pulse_width; //Duration of the pulse in microseconds (from falling to rising edge)
} PulseData;

PulseData *pulsedata_array = NULL; //Pointer for dynamic memory allocation for PulseData
uint32_t pulse_count = 0; //Counter to keep track of number of pulses
uint32_t pulsedata_array_size = 0; //Size of the array (to make dynamic)

// initialize the PulseData array
void initialize_pulsedata_array() {
    pulsedata_array_size = 10; //setting size for 10 pulses for now
    pulsedata_array = (PulseData *)malloc(pulsedata_array_size * sizeof(PulseData));
    if(pulsedata_array == NULL) {
        printf("Failed to allocate memory for PulseData array\n");
        exit(1);
    }
}

// ---------------------------------------------------------------

// result to verify the file status
void print_fresult(FRESULT fr) {
    switch (fr) {
        // different cases to check status of file operation
        case FR_OK: printf("Success\n"); break;
        case FR_NOT_READY: printf("Not Ready\n"); break;
        case FR_NO_FILE: printf("No File\n"); break;
        case FR_NO_PATH: printf("No Path\n"); break;
        case FR_INVALID_DRIVE: printf("Invalid Drive\n"); break;
        // Add other cases as necessary
        default: printf("Unknown Error: %d\n", fr); break;
    }
}


// Simulate the pulse sequence and populate the global pulsedata_array
void simulate_pulse_sequence(uint32_t pulse_count) {
    // Ensure pulsedata_array is large enough
    // if (pulse_count > pulsedata_array_size) {
    //     pulsedata_array = (PulseData *)realloc(pulsedata_array, pulse_count * sizeof(PulseData));
    //     if (pulsedata_array == NULL) {
    //         printf("Failed to reallocate memory for PulseData array\n");
    //         exit(1);  // Exit if memory allocation fails
    //     }
    //     pulsedata_array_size = pulse_count;  // Update the array size
    // }

    // // Simulate the pulse sequence
    // for (int i = 0; i < pulse_count; i++) {
    //     pulsedata_array[i].timestamp = 123456789 + i * 100000;  // Incremented timestamps for each pulse
    //     pulsedata_array[i].state = (i % 2 == 0);  // Alternating high/low states
    //     pulsedata_array[i].duration = 5000 + i * 1000;  // Incrementing durations
    //     pulsedata_array[i].pulse_width = 30 + i * 5;  // Incrementing pulse widths
    // }

    // printf("Simulated %u pulses\n", pulse_count);

    // Seed random number generator
    srand(time(NULL));

    if (pulse_count > pulsedata_array_size) {
        pulsedata_array = (PulseData *)realloc(pulsedata_array, pulse_count * sizeof(PulseData));
        if (pulsedata_array == NULL) {
            printf("Failed to reallocate memory for PulseData array\n");
            exit(1);
        }
        pulsedata_array_size = pulse_count;
    }

    for (int i = 0; i < pulse_count; i++) {
        pulsedata_array[i].timestamp = 123456789 + i * 100000;  // Incremented timestamps
        pulsedata_array[i].state = (i % 2 == 0);  // Alternating high/low states
        pulsedata_array[i].duration = 5000 + (rand() % 1000);  // Random duration with some variation
        pulsedata_array[i].pulse_width = 30 + (rand() % 10);  // Random pulse widths
    }

    printf("Simulated %u pulses\n", pulse_count);
}

// to verify the data being saved is correct
void print_pulse_sequence(PulseData pulse_data_array[], int size) {
    for (int i = 0; i < size; i++) {
        printf("Pulse %d: Timestamp: %u, State: %s, Duration: %u us, Pulse Width: %u ms\n",
               i + 1,
               pulse_data_array[i].timestamp,
               pulse_data_array[i].state ? "HIGH" : "LOW",
               pulse_data_array[i].duration,
               pulse_data_array[i].pulse_width);
    }
}

// attempting to save sequence of pulse data to SD card
void save_pulse_sequence(FIL *file){
    UINT bw;
    FRESULT res = f_write(file, pulsedata_array, pulse_count * sizeof(PulseData), &bw);
    if (res != FR_OK || bw < pulse_count * sizeof(PulseData)) {
        //compare bw (the number of bytes written) with pulse_count * sizeof(PulseData) 
        //because we're writing multiple PulseData entries, not just one.
        printf("Failed to write to file\n");
    } else {
        printf("Successfully wrote %u pulses to file\n", pulse_count);
    }
}


// freeing dynamic memory allocation for PulseData array
void free_pulsedata_array(){
    // free(pulsedata_array);
    // pulsedata_array = NULL;
    if (pulsedata_array != NULL) {
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
    if (fs_mounted) {
        f_unmount("");
        printf("Filesystem unmounted successfully\n");
    }

    // Optionally, perform other cleanup tasks (e.g., stopping peripherals)

    // Exit the program
    printf("Exiting program...\n");
    exit(0);  // Clean exit
}

// improvise writing to SD card via button press
int writeToSD(){
    printf("Starting write operation...\n");
    if (!fs_mounted) {
        printf("Filesystem not mounted!\n");
        return -1;
    }

    // Create a unique binary filename
    char bin_filename[20];
    snprintf(bin_filename, sizeof(bin_filename), "file%d.bin", filename_counter++);  // Use .bin extension for binary file

    // Create a unique text filename
    char txt_filename[20];
    snprintf(txt_filename, sizeof(txt_filename), "file%d.txt", filename_counter++);  // Use .txt extension for text file

    // Attempt to create and open the file for writing
    // printf("Attempting to create file: %s\n", filename);
    // FRESULT fr = f_open(&fil, filename, FA_WRITE | FA_CREATE_ALWAYS);
    // if (fr != FR_OK) {
    //     printf("Failed to open file for writing. Error code: ");
    //     print_fresult(fr);
    //     return -1;
    // }
    // printf("File opened successfully\n");

    // // Save the pulse sequence to the file in binary format
    // save_pulse_sequence(&fil);

    // // Close the file after writing the pulses
    // f_close(&fil);
    // printf("File closed successfully\n");

    // return 0;
    // Attempt to create and open the binary file for writing
    printf("Attempting to create binary file: %s\n", bin_filename);
    FRESULT fr = f_open(&fil, bin_filename, FA_WRITE | FA_CREATE_ALWAYS);
    if (fr != FR_OK) {
        printf("Failed to open binary file for writing. Error code: ");
        print_fresult(fr);
        return -1;
    }
    printf("Binary file opened successfully\n");

    // Save the pulse sequence to the binary file
    save_pulse_sequence(&fil);

    // Close the binary file after writing the pulses
    f_close(&fil);
    printf("Binary file closed successfully\n");

    // Attempt to create and open the text file for writing
    printf("Attempting to create text file: %s\n", txt_filename);
    fr = f_open(&fil, txt_filename, FA_WRITE | FA_CREATE_ALWAYS);
    if (fr != FR_OK) {
        printf("Failed to open text file for writing. Error code: ");
        print_fresult(fr);
        return -1;
    }
    printf("Text file opened successfully\n");

    // Write the pulse data to the text file in a human-readable format
    printf("Writing pulse data to text file...\n");
    for (int i = 0; i < pulse_count; i++) {
        char buffer[100];  // Buffer to store formatted pulse data
        snprintf(buffer, sizeof(buffer), 
                 "Pulse %d: Timestamp: %u, State: %s, Duration: %u us, Pulse Width: %u ms\n",
                 i + 1,
                 pulsedata_array[i].timestamp,
                 pulsedata_array[i].state ? "HIGH" : "LOW",
                 pulsedata_array[i].duration,
                 pulsedata_array[i].pulse_width);

        UINT bw;
        fr = f_write(&fil, buffer, strlen(buffer), &bw);  // Write the formatted string to the file
        if (fr != FR_OK || bw < strlen(buffer)) {
            printf("Failed to write to text file\n");
            f_close(&fil);
            return -1;
        }
    }

    // Close the text file after writing
    f_close(&fil);
    printf("Text file closed successfully\n");

    return 0;
}

int readSD(void) {
    printf("Starting read operation...\n");
    if (!fs_mounted) {
        printf("Filesystem not mounted!\n");
        return -1;
    }

    printf("Attempting to open directory...\n");
    fr = f_opendir(&dir, "/");
    if (fr != FR_OK) {
        printf("Failed to open directory: ");
        print_fresult(fr);
        return -1;
    }

    printf("SD card mounted successfully!\n");
    printf("====================================================================\n");
    printf("| %-20s | %-30s \n", "Name", "Pulse Data");
    printf("====================================================================\n");

    // PulseData pulse_data[10];  // Buffer to store pulse data (assuming max 10 pulses per file)
    // int pulse_index;

    // Reading through the directory
    while (1) {
        fr = f_readdir(&dir, &fno);
        if (fr != FR_OK || fno.fname[0] == 0) {
            break;  // Break on error or end of directory
        }

        // If it's a file (not a directory) and has a .bin extension
        if (!(fno.fattrib & AM_DIR) && strstr(fno.fname, ".bin")) {
            printf("| %-20s | \n", fno.fname);  // Print the file name
            
            // Get file size directly from fno.fsize
            uint32_t file_size = fno.fsize;
            uint32_t num_pulses = file_size / sizeof(PulseData);  // Calculate number of PulseData entries

            if (num_pulses > 0) {
                PulseData *pulse_data = (PulseData *)malloc(num_pulses * sizeof(PulseData));
                if (pulse_data == NULL) {
                    printf("Failed to allocate memory for reading pulse data.\n");
                    continue;  // Skip to next file
                }

                // Open the file for reading
                fr = f_open(&fil, fno.fname, FA_READ);
                if (fr == FR_OK) {
                    UINT br;
                    fr = f_read(&fil, pulse_data, num_pulses * sizeof(PulseData), &br);
                    if (fr != FR_OK || br < num_pulses * sizeof(PulseData)) {
                        printf("Failed to read pulse data from file (read %u bytes).\n", br);
                    } else {
                        // Print all the pulses read from the file
                        for (int i = 0; i < num_pulses; i++) {
                            printf("  Pulse %d: Timestamp: %u, State: %s\n",
                                   i + 1,
                                   pulse_data[i].timestamp,
                                   pulse_data[i].state ? "HIGH" : "LOW");
                            printf("          Duration: %u us, Pulse Width: %u ms\n",
                                   pulse_data[i].duration,
                                   pulse_data[i].pulse_width);
                        }
                    }
                    f_close(&fil);
                } else {
                    printf("Failed to open file for reading: ");
                    print_fresult(fr);
                }
                free(pulse_data);
            } else {
                printf("File %s is empty or contains no valid pulse data.\n", fno.fname);
            }
        }
    }

    printf("====================================================================\n");
    f_closedir(&dir);
    return 0;
}

volatile bool write_flag = false;  // Flag for write operation
volatile bool read_flag = false;   // Flag for read operation

void button_callback(uint gpio, uint32_t events) {
    absolute_time_t current_time = get_absolute_time();
    // Debouncing logic
    if (absolute_time_diff_us(last_button_press, current_time) < DEBOUNCE_DELAY * 1000) {
        return;
    }
    last_button_press = current_time;
    // Set flags instead of directly calling writeToSD() or readSD()
    if (gpio == WRITE_BTNPIN_20) {
        write_flag = true;
    } else if (gpio == READ_BTNPIN_21) {
        read_flag = true;
    }
}

// to initialize the buttons
void init_buttons(void) {
    // Initialize the button GPIO - 20, 21
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

    last_button_press = get_absolute_time();
}

void init_system_with_filesystem(){
    printf("Initializing USB Serial...\n");
    stdio_init_all();
    sleep_ms(5000);
    printf("USB Serial Initialized\n");

    // Initialize filesystem
    printf("Attempting to mount filesystem...\n");
    for (int i = 0; i < 3 && !fs_mounted; i++) {
        fr = f_mount(&fs, "", 1);
        if (fr == FR_OK) {
            fs_mounted = true;
            printf("Filesystem mounted successfully\n");
            break;
        } else {
            printf("Failed to mount filesystem: ");
            print_fresult(fr);
            printf("Retrying mount attempt %d...\n", i + 1);
            sleep_ms(1000);  // Wait 1 second before retrying
        }
    }

    if (!fs_mounted) {
        printf("Failed to mount filesystem after 3 attempts. Exiting...\n");
        return;
    }

    init_buttons();
    printf("System Initialization Completed\n");
    printf("Press GP20 to write a file, GP21 to read files\n");
}

int main() {
    init_system_with_filesystem();

    pulse_count = 10;
    initialize_pulsedata_array();
    simulate_pulse_sequence(pulse_count);

    while (1) {
        // Check if the write button was pressed
        if (write_flag) {
            write_flag = false;  // Reset the flag
            int result = writeToSD();  // Perform the write operation
            printf("WriteToSD result: %d\n", result);
        }
        // Check if the read button was pressed
        if (read_flag) {
            read_flag = false;  // Reset the flag
            int result = readSD();  // Perform the read operation
            printf("ReadSD result: %d\n", result);
        }

        // Check if the exit button (GP22) is pressed
        if (gpio_get(EXIT_BTNPIN_22) == 0) {  // Button pressed (active low)
            printf("Exit button pressed. Exiting the loop...\n");
            break;  // Exit the loop
        }

        tight_loop_contents();  // Prevent the system from sleeping and keep checking
        sleep_ms(100);  // Add a delay to reduce CPU usage
    }
    // free_pulsedata_array();
    cleanup_exit();
    printf("Program exited\n");
    return 0;  // This will never be reached
}


