#include "lwip/apps/httpd.h"
#include "pico/cyw43_arch.h"
#include "FreeRTOS.h"
#include "task.h"

extern TaskHandle_t debug_handle;
extern TaskHandle_t pulse_handle;
extern volatile bool debug_in_progress;

// CGI handler which is run when a request for /led.cgi is detected
const char * cgi_led_handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[])
{
    // Check if an request for LED has been made (/led.cgi?led=x)
    if (strcmp(pcParam[0] , "led") == 0){
        // Look at the argument to check if LED is to be turned on (x=1) or off (x=0)
        if(strcmp(pcValue[0], "0") == 0){
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
        }
        else if(strcmp(pcValue[0], "1") == 0)
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    }
    
    // Send the index page back to the user
    return "/index.shtml";
}

const char * send_letter_handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[])
{
    // Check if an request for letter to be send (/letter.cgi?letter=x)
    if (strcmp(pcParam[0], "letter") == 0) {
        // Print the received letter
        printf("Letter pressed: %s\n", pcValue[0]); // Use %s to print the entire string
    }

    // Send the index page back to the user
    return "/index.shtml";
}

const char * send_debug_idcode(int iIndex, int iNumParams, char *pcParam[], char *pcValue[])
{
    if (strcmp(pcParam[0], "debug") == 0){
        if (strcmp(pcValue[0], "1") == 0){
            if (debug_handle != NULL) {  // Check if debug_handle is initialized
                debug_in_progress = true; // Set the flag
                xTaskNotifyGive(debug_handle);
            } else {
                printf("Debugging task handle is NULL\n");
            }
        }
    }
    return "/index.shtml";
}

// ====================== Pulse Reading ========================================================== //
const char * read_pulse_handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[])
{
    if (strcmp(pcParam[0], "pulse") == 0){
        if (strcmp(pcValue[0], "1") == 0){
            if (pulse_handle != NULL) {  // Check if debug_handle is initialized
                xTaskNotifyGive(pulse_handle);
            } else {
                printf("Pulse task handle is NULL\n");
            }
        }
    }
    return "/index.shtml";
}



// tCGI Struct
// Fill this with all of the CGI requests and their respective handlers
// E.g. /led.cgi is like a URL endpoint to trigger the function.
static const tCGI cgi_handlers[] = {
    {
        // Html request for "/led.cgi" triggers cgi_handler
        "/led.cgi", cgi_led_handler
    },
    {
        // Html request for "/letter.cgi" triggers send_letter_handler
        "/letter.cgi", send_letter_handler
    },
    {
        "/debug.cgi", send_debug_idcode
    },
    {
        "/pulse.cgi", read_pulse_handler
    }
};

// Initialise all the handlers that will be needed for the web server.
// Now is set to 1 as there is only 1 handler.
void cgi_init(void)
{
    http_set_cgi_handlers(cgi_handlers, 4);
}