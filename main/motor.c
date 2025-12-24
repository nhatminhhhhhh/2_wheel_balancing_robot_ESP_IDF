#include "motor.h"
#include "esp_log.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include <math.h>

static const char *TAG = "MOTOR";

// Encoder state variables
int32_t encoder1_count = 0;
int32_t encoder2_count = 0;
int32_t encoder1_last_count = 0;
int32_t encoder2_last_count = 0;
int64_t encoder_last_time = 0;
float encoder1_velocity = 0.0f;  // RPM
float encoder2_velocity = 0.0f;  // RPM

// Encoder ISR for Encoder 1 Pin A
static void IRAM_ATTR encoder1_isr_handler(void* arg)
{
    int b_state = gpio_get_level(ENCODER1_PIN_B);
    if (b_state == 0) {
        encoder1_count++;
    } else {
        encoder1_count--;
    }
}

// Encoder ISR for Encoder 2 Pin A
static void IRAM_ATTR encoder2_isr_handler(void* arg)
{
    int b_state = gpio_get_level(ENCODER2_PIN_A);
    if (b_state == 0) {
        encoder2_count++;
    } else {
        encoder2_count--;
    }
}

// LEDC channels for two motors
#define LEDC_TIMER       LEDC_TIMER_0
#define LEDC_MODE        LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL_A   LEDC_CHANNEL_0
#define LEDC_CHANNEL_B   LEDC_CHANNEL_1
#define LEDC_DUTY_MAX    ((1 << LEDC_TIMER_8_BIT) - 1)  // 255

void servo_init(gpio_num_t gpio, ledc_channel_t channel)
{
    ledc_timer_config_t timer = {
        .duty_resolution = LEDC_TIMER_16_BIT,
        .freq_hz = SERVO_FREQ_HZ,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_1,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer);

    ledc_channel_config_t ch = {
        .channel = channel,
        .duty = 0,
        .gpio_num = gpio,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .hpoint = 0,
        .timer_sel = LEDC_TIMER_1
    };
    ledc_channel_config(&ch);

    ESP_LOGI(TAG, "Servo channel %d initialized on GPIO %d", channel, gpio);
}

void encoder_init(void)
{
    // Configure Encoder 1 pins
    gpio_config_t enc1_cfg = {
        .pin_bit_mask = (1ULL << ENCODER1_PIN_A) | (1ULL << ENCODER1_PIN_B),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&enc1_cfg);

    // Configure Encoder 2 pins
    gpio_config_t enc2_cfg = {
        .pin_bit_mask = (1ULL << ENCODER2_PIN_A) | (1ULL << ENCODER2_PIN_B),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&enc2_cfg);

    // Install GPIO ISR service
    gpio_install_isr_service(0);

    // Set interrupt type and add ISR handler for Encoder 1
    gpio_set_intr_type(ENCODER1_PIN_A, GPIO_INTR_POSEDGE);
    gpio_isr_handler_add(ENCODER1_PIN_A, encoder1_isr_handler, NULL);

    // Set interrupt type and add ISR handler for Encoder 2
    gpio_set_intr_type(ENCODER2_PIN_B, GPIO_INTR_POSEDGE);
    gpio_isr_handler_add(ENCODER2_PIN_B, encoder2_isr_handler, NULL);

    // Initialize timing
    encoder_last_time = esp_timer_get_time();

    ESP_LOGI(TAG, "Encoders initialized on GPIOs %d/%d and %d/%d", 
             ENCODER1_PIN_A, ENCODER1_PIN_B, ENCODER2_PIN_A, ENCODER2_PIN_B);
}

float encoder_get_velocity_1(void)
{
    int64_t current_time = esp_timer_get_time();
    float delta_t = (current_time - encoder_last_time) / 1000000.0f; // Convert to seconds
    
    if (delta_t > 0.0f) {
        int32_t delta_count = encoder1_count - encoder1_last_count;
        // Angular velocity in RPM = (delta_count / PPR) / delta_t * 60
        encoder1_velocity = (delta_count / (float)ENCODER_PPR) / delta_t * 60.0f;
        encoder1_last_count = encoder1_count;
        encoder_last_time = current_time;
    }
    
    return encoder1_velocity;
}

float encoder_get_velocity_2(void)
{
    int64_t current_time = esp_timer_get_time();
    float delta_t = (current_time - encoder_last_time) / 1000000.0f; // Convert to seconds
    
    if (delta_t > 0.0f) {
        int32_t delta_count = encoder2_count - encoder2_last_count;
        // Angular velocity in RPM = (delta_count / PPR) / delta_t * 60
        encoder2_velocity = (delta_count / (float)ENCODER_PPR) / delta_t * 60.0f;
        encoder2_last_count = encoder2_count;
    }
    
    return encoder2_velocity;
}

float encoder_get_angle_1(void)
{
    // Calculate angle in degrees
    float angle = (encoder1_count / (float)ENCODER_PPR) * 360.0f;
    return angle;
}

float encoder_get_angle_2(void)
{
    // Calculate angle in degrees
    float angle = (encoder2_count / (float)ENCODER_PPR) * 360.0f;
    return angle;
}
void encoder_reset(void)
{
    encoder1_count = 0;
    encoder2_count = 0;
    encoder1_last_count = 0;
    encoder2_last_count = 0;
    encoder1_velocity = 0.0f;
    encoder2_velocity = 0.0f;
    encoder_last_time = esp_timer_get_time();
}

int32_t encoder_get_count_1(void)
{
    return encoder1_count;
}

int32_t encoder_get_count_2(void)
{
    return encoder2_count;
}

void servo_set_angle(ledc_channel_t channel,float angle)
{
    if (angle < 0) angle = 0;
    if (angle > 40) angle = 40;

    uint32_t pulse_us = SERVO_MIN_US + (angle / 40.0f) * (SERVO_MAX_US - SERVO_MIN_US);
    //uint32_t pulse_us2 = SERVO_MIN_US + (40 - angle / 40.0f) * (SERVO_MAX_US - SERVO_MIN_US);
    // convert microseconds → duty based on 16-bit resolution
    uint32_t max_duty = (1 << 16) - 1;
    uint32_t duty = (pulse_us * max_duty) / SERVO_PERIOD_US;
    //uint32_t duty2 = (pulse_us2 * max_duty) / SERVO_PERIOD_US;

    // ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, duty1);
    // ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);
}
void motor_init(void)
{
    // Initialize GPIO pins for motor enable
    gpio_set_direction(EN_PINA1, GPIO_MODE_OUTPUT);
    gpio_set_direction(EN_PINA2, GPIO_MODE_OUTPUT);
    gpio_set_direction(EN_PINB1, GPIO_MODE_OUTPUT);
    gpio_set_direction(EN_PINB2, GPIO_MODE_OUTPUT);

    ESP_LOGI(TAG, "Initializing LEDC for motor control");

    // Configure LEDC timer
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = MOTOR_PWM_RESOLUTION,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = MOTOR_PWM_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Configure LEDC channel for Motor A
    ledc_channel_config_t ledc_channel_a = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_A,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = PWM_A_PIN,
        .duty           = 0,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_a));

    // Configure LEDC channel for Motor B
    ledc_channel_config_t ledc_channel_b = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_B,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = PWM_B_PIN,
        .duty           = 0,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_b));

        ESP_LOGI(TAG, "LEDC PWM initialized successfully (20kHz, 8-bit resolution)");
}

void motor_control(float direction, int16_t speed, int frontoffset, int backoffset)
{
    // // int16_t offset_speed = speed;
    // // if (speed < 140) offset_speed = 0;
    // else offset_speed = speed;
    if (direction > 0) {
        // Forward direction
        gpio_set_level(EN_PINA1, 0);
        gpio_set_level(EN_PINA2, 1);
        gpio_set_level(EN_PINB1, 1);
        gpio_set_level(EN_PINB2, 0);

        // Set PWM duty cycle (0-1023)
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_A, speed + frontoffset);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_A);
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_B, speed + frontoffset);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_B);
    } else if (direction < 0) {
        // Backward direction
        gpio_set_level(EN_PINA1, 1);
        gpio_set_level(EN_PINA2, 0);
        gpio_set_level(EN_PINB1, 0);
        gpio_set_level(EN_PINB2, 1);

        // Set PWM duty cycle
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_A, speed + backoffset);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_A);
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_B, speed + backoffset);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_B);
    } else {
        // Stop motors
        gpio_set_level(EN_PINA1, 0);
        gpio_set_level(EN_PINA2, 0);
        gpio_set_level(EN_PINB1, 0);
        gpio_set_level(EN_PINB2, 0);

        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_A, 0);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_A);
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_B, 0);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_B);
    }
}

void stop_motor(void)
{
    // Disable motor drivers
    gpio_set_level(EN_PINA1, 0);
    gpio_set_level(EN_PINA2, 0);
    gpio_set_level(EN_PINB1, 0);
    gpio_set_level(EN_PINB2, 0);

    // Set PWM duty cycle to 0
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_A, 0);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_A);
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_B, 0);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_B);

    //ESP_LOGI(TAG, "Motors stopped");
}