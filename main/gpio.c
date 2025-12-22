#include "gpio.h"
#include "esp_log.h"
static const char *TAG = "GPIO";
void gpio_init(void)
{
    // Configure LED pin as output
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << LED_PIN),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE
    };
    gpio_config(&io_conf);
    ESP_LOGI(TAG, "LED GPIO initialized");

    // Configure Button pin as input with pull-up
    io_conf.intr_type = GPIO_INTR_POSEDGE; // Interrupt on rising edge
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << BUTTON_PIN);
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);
    ESP_LOGI(TAG, "Button GPIO initialized");
}

void set_led_state(bool state)
{
    gpio_set_level(LED_PIN, state ? 1 : 0);
}
