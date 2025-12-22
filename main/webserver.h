#ifndef WEBSERVER_H
#define WEBSERVER_H

#include "esp_err.h"

// WiFi credentials - update these with your router details
#define WIFI_SSID      "Tran Van Tuan"
#define WIFI_PASSWORD  "0353070860"

// #define WIFI_SSID      "P2.3->2.6"
// #define WIFI_PASSWORD  "Chinhanh2"

// NVS namespace for storing PID parameters
#define NVS_NAMESPACE  "pid_storage"

// PID parameters structure
typedef struct {
    float kp;
    float ki;
    float kd;
    float kx;
} pid_params_t;

// Global PID parameters (accessible from main)
extern pid_params_t pid_params;

/**
 * @brief Initialize WiFi and connect to router
 * @return ESP_OK on success
 */
esp_err_t wifi_init(void);

/**
 * @brief Start the web server for PID tuning
 * @return ESP_OK on success
 */
esp_err_t webserver_start(void);

/**
 * @brief Get current PID parameters
 * @param kp Pointer to store Kp value
 * @param ki Pointer to store Ki value
 * @param kd Pointer to store Kd value
 * @param kx Pointer to store Kx value
 */
void webserver_get_pid(float *kp, float *ki, float *kd, float *kx);
esp_err_t pid_load_from_nvs(void);
esp_err_t pid_save_to_nvs(void);


#endif // WEBSERVER_H
