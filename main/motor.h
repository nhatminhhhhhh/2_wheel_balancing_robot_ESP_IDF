#ifndef MOTOR_H
#define MOTOR_H

#include "driver/ledc.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#define PWM_A_PIN 5
#define PWM_B_PIN 33
#define EN_PINA1 25
#define EN_PINA2 26
#define EN_PINB1 18
#define EN_PINB2 19
#define MOTOR_PWM_FREQUENCY 20000 // 20 kHz
#define MOTOR_PWM_RESOLUTION LEDC_TIMER_8_BIT // 0-255 range
#define SERVO_MIN_US 500
#define SERVO_MAX_US 2500
#define SERVO_FREQ_HZ 50
#define SERVO_PERIOD_US 20000
#define SERVO1_PIN -1
#define SERVO2_PIN -1

// Encoder pins
#define ENCODER1_PIN_A 16
#define ENCODER1_PIN_B 17
#define ENCODER2_PIN_A 27
#define ENCODER2_PIN_B 14

extern int32_t encoder1_count;
extern int32_t encoder2_count;
extern int32_t encoder1_last_count;
extern int32_t encoder2_last_count;
extern int64_t encoder_last_time;
extern float encoder1_velocity;  // RPM
extern float encoder2_velocity;  // RPM

// Encoder parameters
#define ENCODER_PPR 616  // Pulses per revolution - adjust based on your encoder specs

void motor_init(void);
void motor_control(float output, int16_t speed, int frontoffset, int backoffset);
void stop_motor(void);
void servo_init(gpio_num_t gpio, ledc_channel_t channel);
void servo_set_angle(ledc_channel_t channel,float angle);

// Encoder functions
void encoder_init(void);
float encoder_get_velocity_1(void);  // Returns angular velocity in RPM for encoder 1
float encoder_get_velocity_2(void);  // Returns angular velocity in RPM for encoder 2
float encoder_get_angle_1(void);     // Returns angle in degrees for encoder 1
float encoder_get_angle_2(void);     // Returns angle in degrees for encoder 2
void encoder_reset(void);   // Reset velocity calculations
int32_t encoder_get_count_1(void);   // Get raw pulse count for encoder 1
int32_t encoder_get_count_2(void);   // Get raw pulse count for encoder 2

#endif // MOTOR_H