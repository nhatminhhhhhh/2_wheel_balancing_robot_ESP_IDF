#ifndef GPIO_H 
#define GPIO_H

#include "driver/gpio.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define LED_PIN     2   // Example GPIO pin for an LED
#define BUTTON_PIN  15  // Example GPIO pin for a button

void gpio_init(void);
void set_led_state(bool state);
#endif // GPIO_H