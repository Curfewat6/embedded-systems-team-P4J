#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include <stdlib.h>
#include <stdio.h>
#include "lwip/apps/sntp.h"
#include "lwip/api.h"
#include "lwip/tcp.h"
#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "lwip/inet.h"
#include <sys/time.h>
#include "lwip/ip4_addr.h"

#include "FreeRTOS.h"
#include "task.h"
#include "lwipopts_examples_common.h"
#include "ssi.h"
#include "cgi.h"
#include "message_buffer.h"
#include <lwip/apps/sntp_opts.h>
#include "ntp.h"
#include "debug.h"
#include "buddy_1.h"
#include "semphr.h"

#define LED_PIN 0

// Define your WiFi credentials
#define WIFI_SSID01 "TP-Link_E4A4"
#define WIFI_PASSWD01 "84330369"

TaskHandle_t sntp_handle = NULL;
TaskHandle_t debug_handle = NULL;
TaskHandle_t pulse_handle = NULL;
TaskHandle_t adc_pulse_handle = NULL;

SemaphoreHandle_t xPWM_Mutex;
SemaphoreHandle_t xADC_Mutex;


struct repeating_timer timer1, timer2;

volatile bool debug_in_progress = true;

// Interrupt handler form PULSE_PIN
// void gpio_callback(uint gpio, uint32_t events){

//     BaseType_t xHigherPriorityTaskWoken = pdTRUE;

//     if (gpio == DIGI_PIN){
//         printf("Interrrupting now...\n");
//         vTaskNotifyGiveFromISR(pulse_handle, &xHigherPriorityTaskWoken);
//     }
// }

void led_task(__unused void *params) {
    printf("Initializing LED Pin...\n");
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    while (true) {
        gpio_put(LED_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(1000));
        gpio_put(LED_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Task to set up the web server
void web_server(__unused void *params) {
    printf("Initializing Web Server...\n");

    cyw43_arch_init();
    cyw43_arch_enable_sta_mode();

    while (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID01, WIFI_PASSWD01, CYW43_AUTH_WPA2_AES_PSK, 30000) != 0) {
        printf("Attempting to connect...\n");
    }

    printf("Connected!\n");

    // Initialize web server
    httpd_init();
    printf("HTTP server initialized\n");

    // Configure SSI and CGI handlers
    ssi_init(); 
    printf("SSI Handler initialized\n");
    cgi_init();
    printf("CGI Handler initialized\n");
    printf("------------------------------\n");

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


// Currently not in used
void sync_ntp(__unused void *params) {

    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    ntp_init_data();
    printf("NTP Handler initialised\n");
    if (resolve_ntp_server()) {
        get_ntp_time();
        print_rtc_time();
    } else {
        printf("Error getting NTP server\n");
    }

    fflush(stdout);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Used to start the debugging task. This task is different from the rest, only when the button is pressed from the webpage,
// then the task will run and get the IDCODE.
void debugging(__unused void *params){

    while(1){
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        printf("Starting Debugging initialization...\n");
        initialise_debugger();
        printf("Debugging successfully initialized!\n");

        debug_in_progress = false; // Reset the flag after completion
    }
}


// Task that is constantly reading for pulse coming in GPIO 7. I test this by connecting GPIO 0 to 7 as 0 is 
// currently a blinking task which technically producing a pulse.
void read_PWM_pulse(__unused void *params) {
    
    printf("\nStarting PWM pulse reading...\n");
    while(1) {
        measure_digi();
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
    
}

// This task is to read ADC but only when ADC_BTN_PIN 21 is pressed.
void read_ADC_pulse(__unused void *params) {
    
    printf("\nStarting ADC reading...\n");

    while(1) {
        read_adc();
        vTaskDelay(pdMS_TO_TICKS(300));
    }
}

void vLaunch(void) {

    TaskHandle_t led;
    xTaskCreate(led_task, "DiscoLED", configMINIMAL_STACK_SIZE, NULL, 5, &led);

    TaskHandle_t web;
    xTaskCreate(web_server, "Webserver", configMINIMAL_STACK_SIZE, NULL, 3, &web);
    
    // if (xTaskCreate(debugging, "DebugTask", configMINIMAL_STACK_SIZE, NULL, 1, &debug_handle) != pdPASS) {
    //     printf("Failed to create Debug task\n");
    //     debug_handle = NULL;  // Set to NULL to avoid invalid handle issues
    // }
    xTaskCreate(debugging, "DebugTask", configMINIMAL_STACK_SIZE, NULL, 1, &debug_handle);

    gpio_set_irq_enabled_with_callback(DIGI_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &button_callback);
    gpio_set_irq_enabled_with_callback(ADC_BTN_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &button_callback);


    xTaskCreate(read_PWM_pulse, "ReadPulse", configMINIMAL_STACK_SIZE * 2, NULL, 2, &pulse_handle);
    
    xTaskCreate(read_ADC_pulse, "ReadADCPulse", configMINIMAL_STACK_SIZE * 2, NULL, 3, &adc_pulse_handle);


    vTaskStartScheduler();
}

int main(void) {
    stdio_init_all();
    setup_adc();
    init_buttons();
    vLaunch();
}
