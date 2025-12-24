#include <stdio.h>
#include "mpu6050.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "motor.h"
#include <math.h>
#include "gpio.h"
#include "webserver.h"
static const char *TAG = "MAIN";

// PID parameters - now controlled via web server
float Kp ;
float Ki ;
float Kd ;
float Kx;

float Max_Output = 240.0f; // previous 230.0f
float Min_Output = 80.0f;
int Offset = 0;


float SETPOINT_ANGLE = 0.0f;
float Integral = 0.0f;
float Iterm = 0.0f;
float Pterm = 0.0f;
float Prev_error = 0.0f;
float Prev_Measured = 0.0f;
float Output = 0.0f;
float Derivative_filter = 0.0f;
float Derivative_Alpha = 0.40f; // lower value means less filtering
float Delta_t = 0.02f; // Default to 20ms

float Deadzone = 2.0f;
bool Fail_detect = false;
float ReverseAngle = 9.0f;
float pos = 5.0f; // Initial servo position
int64_t StartBalanc = 0;
int64_t StopBalanc = 0;
int64_t Now = 0;
int64_t LastAngleTime = 0;
float LastTime = 0.0f;
float PrevErrorAngle = 0.0f;
float contrains(float value, float min, float max){
    if(value > max) return max;
    if(value < min) return min;
    return value;
}

void angle_pid(float target_angle, float angle){
    int64_t now = esp_timer_get_time(); 
    float timeChange = (now - LastAngleTime) / 1000000.0f; // in milliseconds
    LastAngleTime = now;
    if(timeChange >= 0.02f){
        float ErrorAngle = target_angle - angle;
        float Kp_angle = 10.0f; // Proportional gain for angle
        float Ki_angle = 1.0f; // Integral gain for angle
        float Kd_angle = 0.1f; // Derivative gain for angle
        float IntegralAngle = 0.0f;
        IntegralAngle += ErrorAngle * timeChange;
        float DerivativeAngle = (ErrorAngle - PrevErrorAngle) / timeChange;
        
        float OutputPWM = Kp_angle * ErrorAngle + Ki_angle * IntegralAngle + Kd_angle * DerivativeAngle;
        OutputPWM = contrains(OutputPWM, -255, 255);
        int16_t speed = (int16_t) fabsf(OutputPWM);
        //speed = contrains(speed, 0, 176); // Limit speed to safe range
        motor_control(-OutputPWM, speed, 0, 0);
        printf("Target Angle: %.2f, Measured Angle: %.2f, Error: %.2f, Output PWM: %.2f, Speed: %d\n", target_angle, angle, ErrorAngle, OutputPWM, speed);
        PrevErrorAngle = ErrorAngle;
    }
}

void pid_control(float setpoint, float measured){
    if( measured > 60.0f || measured < -60.0f ){
            Fail_detect = true;
            stop_motor();
            Prev_Measured = measured; // Reset previous measured
            Integral = 0.0f; // Reset integral term
            Derivative_filter = 0.0f; // Reset derivative filter
            //printf("Fail-safe activated! Angle exceeded safe limit.\n");
            //encoder_reset();
            gpio_set_level(LED_PIN, 0); // Turn off LED
            return;
    }
    gpio_set_level(LED_PIN, 1);
    //webserver_get_pid(&Kp, &Ki, &Kd, &Kx); //tunning real time 

    float error = (setpoint - measured) ; //
    Integral = Integral + error * Delta_t;
    //Iterm += Ki * error * Delta_t; // avoid overshoot cause of tuning Ki real time
    Iterm = Ki * Integral;
    if (Iterm > 160) {
        Iterm = 160;
        Integral = Iterm / Ki; // Adjust integral to prevent windup
    } else if (Iterm < -160) {
        Iterm = -160;
        Integral = Iterm / Ki; // Adjust integral to prevent windup
    }


    float Derivative = (measured - Prev_Measured) / Delta_t; // Using measured change for derivative
    //float Derivative = (error - Prev_error) / Delta_t; // Using error change for derivative
    Derivative_filter = (Derivative_Alpha * Derivative_filter) + ((1.0f - Derivative_Alpha) * Derivative); // Apply low-pass filter to derivative to reduce noise
    Derivative_filter = contrains(Derivative_filter, -150, 150);
    //Output = (Kp * error) + Iterm - (Kd * Derivative_filter);
    Pterm = Kp * (error + Kx * Integral);
    if(fabsf(error) < Deadzone){
        Integral = 0.0f;
    }
    // float SpeedFix =3.0f;
    // if(fabsf(error) * SpeedFix > 10.0f){
    //     Max_Output += SpeedFix;
    // } else {
    //     Max_Output -= SpeedFix;
    // }
    // Max_Output = contrains(Max_Output, 220, 255);
    
    Output = Pterm + Iterm - (Kd * Derivative_filter);
    // if(Output < Min_Output && Output > 0){
    //     Output = Min_Output;
    // } else if(Output > -Min_Output && Output < 0){
    //     Output = -Min_Output;
    // }
    if (Output > 0) Output += Min_Output;
    else if (Output < 0) Output -= Min_Output;
    Output = contrains(Output, -Max_Output, Max_Output);

    int16_t speed = (int16_t) fabsf(Output);
    //speed = contrains(speed, Min_Output, Max_Output);
    motor_control(Output, speed, -15, 0);  
        printf("Setpoint: %.2f, Pitch: %.2f, Error: %.2f, Output: %.2f, Pterm: %.2f, I: %.2f, D: %.2f\n", setpoint, measured, error, Output, Pterm, Iterm, Kd * Derivative_filter);
        //printf(">Setpoint:%.2f, Measured:%.2f, Output:%.2f\r\n", setpoint, measured, Output);
    //Prev_error = error;
    Prev_Measured = measured;  
}


void app_main(void)
{
    // Initialize WiFi and Web Server
    ESP_LOGI(TAG, "Connecting to WiFi...");
    if (wifi_init() == ESP_OK) {
        ESP_LOGI(TAG, "WiFi connected! Starting web server...");
        vTaskDelay(pdMS_TO_TICKS(1000)); // Wait for WiFi to stabilize
        
        // Load PID parameters from NVS (EEPROM)
        pid_load_from_nvs();
        
        webserver_start();
        ESP_LOGI(TAG, "Web server started. Access PID tuning at http://[ESP32_IP]");
        vTaskDelay(pdMS_TO_TICKS(500));
    } else {
        ESP_LOGE(TAG, "WiFi connection failed. Web server not started.");
    }

    motor_init();
    stop_motor();
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_init();
    vTaskDelay(pdMS_TO_TICKS(100));

    //servo_init(SERVO1_PIN, LEDC_CHANNEL_2);
    vTaskDelay(pdMS_TO_TICKS(100));
        //servo_init(SERVO2_PIN, LEDC_CHANNEL_3);
        vTaskDelay(pdMS_TO_TICKS(100));
        gpio_set_level(LED_PIN, 1); // Turn on LED to indicate ready
        vTaskDelay(pdMS_TO_TICKS(300));
        gpio_set_level(LED_PIN, 0); // Turn off LED to indicate ready
        vTaskDelay(pdMS_TO_TICKS(300));
        gpio_set_level(LED_PIN, 1); // Turn on LED to indicate ready
        vTaskDelay(pdMS_TO_TICKS(300));
        gpio_set_level(LED_PIN, 0); // Turn off LED to indicate ready
        
        // servo_set_angle(LEDC_CHANNEL_2, pos);
        // servo_set_angle(LEDC_CHANNEL_3, 9 - pos);
        vTaskDelay(pdMS_TO_TICKS(100));
        gpio_set_level(LED_PIN, 1); // Turn on LED to indicate ready
        vTaskDelay(pdMS_TO_TICKS(300));
        gpio_set_level(LED_PIN, 0); // Turn off LED to indicate ready
        vTaskDelay(pdMS_TO_TICKS(300));
        gpio_set_level(LED_PIN, 1); // Turn on LED to indicate ready
        vTaskDelay(pdMS_TO_TICKS(300));
    gpio_set_level(LED_PIN, 0); // Turn off LED to indicate ready

    //encoder_init();
    vTaskDelay(pdMS_TO_TICKS(100));
    // Initialize I2C for MPU6050
    ESP_ERROR_CHECK(mpu6050_i2c_init());
    ESP_LOGI(TAG, "I2C initialized successfully");
    // Detect MPU6050
    // if (!mpu6050_detect()) {
    //     ESP_LOGE(TAG, "MPU6050 detection failed! Check your wiring.");
    //     return;
    // }
    // Initialize and calibrate MPU6050
    mpu6050_init_and_calibrate();
    vTaskDelay(pdMS_TO_TICKS(100));
    printf("Target: %.2f, Max Speed: %.1f, Min Speed: %.1f\n", SETPOINT_ANGLE, Max_Output, Min_Output);
    printf("Robot ready. Press the button to start balancing...\n");

    while (1) {
        //printf("Check button press to start balancing...\n");
        gpio_set_level(LED_PIN, 1); // Turn on LED to indicate ready
        webserver_get_pid(&Kp, &Ki, &Kd, &Kx);
        stop_motor();
        Integral = 0.0f;
        Prev_error = 0.0f;
        // encoder_get_velocity_1();
        // encoder_get_velocity_2();

        //printf("Encoder 1 vel: %.2f, Encoder 2 vel: %.2f\n", encoder1_velocity, encoder2_velocity);
        // printf("Angle Encoder 1: %.2f, Angle Encoder 2: %.2f\n", encoder_get_angle_1(), encoder_get_angle_2());
        read_mpu();
        //printf("Initial Pitch Angle: %.2f\n", filteredAnglePitch);
        vTaskDelay(pdMS_TO_TICKS(20));
        
        while(gpio_get_level(BUTTON_PIN) == 0){
            /* //Debug angle
            read_mpu();
            printf(">Pitch:%.2f\r\n", filteredAnglePitch);
            printf(",");
            printf("Gyro:%.2f\r\n", gyroAnglePitch);
            vTaskDelay(pdMS_TO_TICKS(40));
            */
           read_mpu();
            float AngleFix = 7.0f;
                if(filteredAnglePitch < SETPOINT_ANGLE){
                    SETPOINT_ANGLE += AngleFix * delta_t;
                } else if(filteredAnglePitch > SETPOINT_ANGLE){
                    SETPOINT_ANGLE -= AngleFix * delta_t;
                }
                if (SETPOINT_ANGLE > 2.0f) SETPOINT_ANGLE = 2.0f;
                if (SETPOINT_ANGLE < -2.0f) SETPOINT_ANGLE = -2.0f;
            
            pid_control(SETPOINT_ANGLE, filteredAnglePitch);
            vTaskDelay(pdMS_TO_TICKS(20)); 
        }
        
    }
}

