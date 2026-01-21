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
#include "autotune.h"
#include "neural_network.h"
static const char *TAG = "MAIN";

// PID parameters - now controlled via web server
float Kp ;
float Ki ;
float Kd ;
float Kx;

float Kpmax = 35.0f;
float Kpmin = 10.0f;
float Kimax = 60.0f;
float Kimin = 0.0f;
float Kdmax = 1.7f;
float Kdmin = 0.0f;

// Base PID parameters (initial values)
float Kp_base = 24.1f;
float Ki_base = 40.0f;
float Kd_base = 1.05f;

// Flag to enable/disable fuzzy autotuning
bool use_fuzzy_autotune = true;

// Flag to enable/disable neural network
bool use_neural_network = false;

float Max_Output = 255.0f; // previous 230.0f
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
float Derivative_Alpha = 0.20f; // lower value means less filtering
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
float checkDirection = 0.0f;

void pid_control(float setpoint, float measured){
    if( measured > 60.0f || measured < -60.0f ){
            Fail_detect = true;
            stop_motor();
            Prev_Measured = measured; // Reset previous measured
            Integral = 0.0f; // Reset integral term
            Derivative_filter = 0.0f; // Reset derivative filter
            //printf("Fail-safe activated! Angle exceeded safe limit.\n");
            gpio_set_level(LED_PIN, 0); // Turn off LED
            return;
    }
    gpio_set_level(LED_PIN, 1);
    
    bool prev_fuzzy_state = use_fuzzy_autotune;
    use_fuzzy_autotune = webserver_get_fuzzy_autotune(); // Update fuzzy flag in real-time

    // Initialize PID gains when switching from fuzzy to manual mode
    if (prev_fuzzy_state && !use_fuzzy_autotune) {
        Kp = Kp_base;
        Ki = Ki_base;
        Kd = Kd_base;
    }
    
    // Get real-time PID tuning from webserver when fuzzy is disabled
    if (!use_fuzzy_autotune) {
        webserver_get_pid(&Kp, &Ki, &Kd, &Kx);
    }

    float error = (setpoint - measured) ; 
    
    Integral = Integral + error * Delta_t;

    float Derivative = (measured - Prev_Measured) / Delta_t; // Using measured change for derivative
    Derivative_filter = (Derivative_Alpha * Derivative_filter) + ((1.0f - Derivative_Alpha) * Derivative); // Apply low-pass filter to derivative to reduce noise
    Derivative_filter = contrains(Derivative_filter, -150, 150);

    // Neural Network control
    if (use_neural_network) {
        // Use neural network to compute Kp and Kd based on error and derivative
        nn_pid(error, Derivative_filter, &Kp, &Kd);
        
        // Clamp Kp and Kd to safe ranges
        Kp = contrains(Kp, Kpmin, Kpmax+10);
        Kd = contrains(Kd, Kdmin, Kdmax);
        
        Pterm = Kp * error;
        Output = Pterm - (Kd * Derivative_filter) + Iterm;
        //printf("NN - Kp: %.2f, Kd: %.2f\n", Kp, Kd);
    }
    // Fuzzy autotuning logic
    else if (use_fuzzy_autotune) {
        // Fuzzify error and derivative
        ErrorMF error_mf = fuzzifyError(error);
        DerivativeMF de_mf = fuzzifyDE(Derivative_filter);
        
        // Inference
        OutputMF output_mf = inference(error_mf, de_mf);
        
        // Defuzzify to get gain adjustment
        float kp_dot = defuzzify(output_mf, -8.0f, 0.0f, 8.0f);
        Kp = (Kpmax - Kpmin) * (kp_dot + 8) / 16 + Kpmin; // Map from [-8, 8] to [Kpmin, Kpmax]
        float kd_dot = defuzzify(output_mf, -0.2f, 0.0f, 0.2f);
        Kd = (Kdmax - Kdmin) * (kd_dot + 0.2f) / 0.4f + Kdmin; // Map from [-0.2, 0.2] to [Kdmin, Kdmax]
        Pterm = Kp * error;
        Output = Pterm - (Kd * Derivative_filter);
        //printf("Kp fuzzy: %.2f, Kd fuzzy: %.2f\n", Kp, Kd);
    } else{
        Iterm = Ki * Integral;
        if (Iterm > 160) {
            Iterm = 160;
            Integral = Iterm / Ki; // Adjust integral to prevent windup
        } else if (Iterm < -160) {
            Iterm = -160;
            Integral = Iterm / Ki; // Adjust integral to prevent windup
        }
        Pterm = Kp * (error + Kx * Integral);
        //Pterm = Kp * error;
        if(fabsf(error) < Deadzone){
            Integral = 0.0f;
        }
        Output = Pterm + Iterm - (Kd * Derivative_filter);
    }
    if (Output > 0) Output += Min_Output;
    else if (Output < 0) Output -= Min_Output;
    Output = contrains(Output, -Max_Output, Max_Output);

    int16_t speed = (int16_t) fabsf(Output);
    motor_control(Output, speed, -15, 0);  
    //printf("Setpoint: %.2f, Pitch: %.2f, Error: %.2f, Output: %.2f, Kp: %.2f, I: %.2f, D: %.2f\n", setpoint, measured, error, Output, Kp, Iterm, Kd );    
    //printf(">Error: %.2f\n", error);
    printf("%.5f,%.5f,%.3f,%.3f\n", error, Derivative_filter, Kp, Kd);

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
        use_fuzzy_autotune = webserver_get_fuzzy_autotune();
        use_neural_network = webserver_get_neural_network();
        stop_motor();
        Integral = 0.0f;
        Prev_error = 0.0f;
  
        read_mpu();
        //printf("Initial Pitch Angle: %.2f\n", filteredAnglePitch);
        vTaskDelay(pdMS_TO_TICKS(20));
        
        while(gpio_get_level(BUTTON_PIN) == 0){
            read_mpu();
            // float AngleFix = 7.0f;
            // if(filteredAnglePitch < SETPOINT_ANGLE){
            //     SETPOINT_ANGLE += AngleFix * delta_t;
            // } else if(filteredAnglePitch > SETPOINT_ANGLE){
            //     SETPOINT_ANGLE -= AngleFix * delta_t;
            // }
            // if (SETPOINT_ANGLE > 2.2f) SETPOINT_ANGLE = 2.2f;
            // if (SETPOINT_ANGLE < -1.6f) SETPOINT_ANGLE = -1.6f;
            float AngleFix = 6.8f;
            SETPOINT_ANGLE += AngleFix * Output/100 * delta_t;
            if (SETPOINT_ANGLE > 3.5f) SETPOINT_ANGLE = 3.5f;
            if (SETPOINT_ANGLE < -2.2f) SETPOINT_ANGLE = -2.2f;
            pid_control(SETPOINT_ANGLE, filteredAnglePitch);
            vTaskDelay(pdMS_TO_TICKS(20)); 
        }
        
    }
}

