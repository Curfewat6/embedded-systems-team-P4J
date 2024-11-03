#include "lwip/apps/httpd.h"
#include "pico/cyw43_arch.h"
#include "hardware/adc.h"
#include <stdlib.h>
#include <stdio.h>
#include "buddy_1.h"

extern uint32_t id_code;
extern volatile bool debug_in_progress;

// SSI tags - tag length limited to 8 bytes by default
const char * ssi_tags[] = {"volt","temp","led", "idcode", "pwm_freq", "pwm_dc", "adc_freq"};

u16_t ssi_handler(int iIndex, char *pcInsert, int iInsertLen) {
  size_t printed;
  switch (iIndex) {
  case 0: // volt
    {
      const float voltage = adc_read() * 3.3f / (1 << 12);
      printed = snprintf(pcInsert, iInsertLen, "%f", voltage);
    }
    break;
  case 1: // temp
    {
    const float voltage = adc_read() * 3.3f / (1 << 12);
    const float tempC = 27.0f - (voltage - 0.706f) / 0.001721f;
    printed = snprintf(pcInsert, iInsertLen, "%f", tempC);
    }
    break;
  case 2: // led
    {
      bool led_status = cyw43_arch_gpio_get(CYW43_WL_GPIO_LED_PIN);
      if(led_status == true){
        printed = snprintf(pcInsert, iInsertLen, "ON");
      }
      else{
        printed = snprintf(pcInsert, iInsertLen, "OFF");
      }
    }
    break;
  case 3:
    {
      sleep_ms(800);
      printf("[***] IDCODE of the target is: 0x%08X\n", id_code);
      printed = snprintf(pcInsert, iInsertLen, "0x%08X", id_code);

      // if (debug_in_progress) {
      //     printed = snprintf(pcInsert, iInsertLen, "Updating...");
      // } else {
      //     printed = snprintf(pcInsert, iInsertLen, "0x%08X", id_code);
      // }
    }
    break;
  case 4: // pwm - New case for PWM values
  {
      printed = 0; // Reset printed count
      for (int i = 0; i < MAX_DATA_POINTS; i++) {
          // Use %f for float, adjust precision as needed (e.g., "%.2f")
          int result = snprintf(pcInsert + printed, iInsertLen - printed, "%.2f%s", pwm_frequency_values[i], (i < MAX_DATA_POINTS - 1) ? ", " : "");

          // Check for snprintf success and prevent overflow
          if (result < 0) {
              // An error occurred during printing
              printed = iInsertLen; // Prevent further writes
              break;
          }

          printed += result; // Increment printed count

          // Check if we are about to overflow
          if (printed >= iInsertLen) {
              printed = iInsertLen - 1; // Prevent overflow
              break; // Exit the loop
          }
      }
      // Ensure null termination
      pcInsert[printed] = '\0'; // Null-terminate the string
  }
  break;
  case 5: // pwm - New case for PWM values
  {
      printed = 0; // Reset printed count
      for (int i = 0; i < MAX_DATA_POINTS; i++) {
          // Use %f for float, adjust precision as needed (e.g., "%.2f")
          int result = snprintf(pcInsert + printed, iInsertLen - printed, "%.2f%s", pwm_duty_cycle_values[i], (i < MAX_DATA_POINTS - 1) ? ", " : "");

          // Check for snprintf success and prevent overflow
          if (result < 0) {
              // An error occurred during printing
              printed = iInsertLen; // Prevent further writes
              break;
          }

          printed += result; // Increment printed count

          // Check if we are about to overflow
          if (printed >= iInsertLen) {
              printed = iInsertLen - 1; // Prevent overflow
              break; // Exit the loop
          }
      }
      // Ensure null termination
      pcInsert[printed] = '\0'; // Null-terminate the string
  }
  break;
  case 6: // pwm - New case for PWM values
  {
      printed = 0; // Reset printed count
      for (int i = 0; i < MAX_DATA_POINTS; i++) {
          // Use %f for float, adjust precision as needed (e.g., "%.2f")
          int result = snprintf(pcInsert + printed, iInsertLen - printed, "%.2f%s", adc_frequency_values[i], (i < MAX_DATA_POINTS - 1) ? ", " : "");

          // Check for snprintf success and prevent overflow
          if (result < 0) {
              // An error occurred during printing
              printed = iInsertLen; // Prevent further writes
              break;
          }

          printed += result; // Increment printed count

          // Check if we are about to overflow
          if (printed >= iInsertLen) {
              printed = iInsertLen - 1; // Prevent overflow
              break; // Exit the loop
          }
      }
      // Ensure null termination
      pcInsert[printed] = '\0'; // Null-terminate the string
  }
  break;
  default:
    printed = 0;
    break;
  }

  return (u16_t)printed;
}

// Initialise the SSI handler
void ssi_init() {
  // Initialise ADC (internal pin)
  adc_init();
  adc_set_temp_sensor_enabled(true);
  adc_select_input(4);

  http_set_ssi_handler(ssi_handler, ssi_tags, LWIP_ARRAYSIZE(ssi_tags));
}
