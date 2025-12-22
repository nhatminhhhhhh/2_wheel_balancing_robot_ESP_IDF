#include "webserver.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_netif.h"
#include "esp_http_server.h"

static const char *TAG = "WEBSERVER";

// WiFi event group
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static int s_retry_num = 0;
#define MAX_RETRY 10

// Global PID parameters with default values
pid_params_t pid_params = {
    .kp = 2.0f,
    .ki = 3.0f,
    .kd = 1.0f,
    .kx = 0.0f
};

// Load PID parameters from NVS
esp_err_t pid_load_from_nvs(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t err;

    err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "NVS not found, using default PID values");
        return err;
    }

    size_t size = sizeof(float);
    err = nvs_get_blob(nvs_handle, "kp", &pid_params.kp, &size);
    if (err == ESP_OK) {
        nvs_get_blob(nvs_handle, "ki", &pid_params.ki, &size);
        nvs_get_blob(nvs_handle, "kd", &pid_params.kd, &size);
        nvs_get_blob(nvs_handle, "kx", &pid_params.kx, &size);
        ESP_LOGI(TAG, "\033[34mPID loaded from NVS - Kp: %.2f, Ki: %.2f, Kd: %.3f, Kx: %.2f\033[0m",
         pid_params.kp, pid_params.ki, pid_params.kd, pid_params.kx);

    } else {
        ESP_LOGW(TAG, "No saved PID values, using defaults");
    }

    nvs_close(nvs_handle);
    return err;
}

// Save PID parameters to NVS
esp_err_t pid_save_to_nvs(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t err;

    err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS handle");
        return err;
    }

    err = nvs_set_blob(nvs_handle, "kp", &pid_params.kp, sizeof(float));
    err |= nvs_set_blob(nvs_handle, "ki", &pid_params.ki, sizeof(float));
    err |= nvs_set_blob(nvs_handle, "kd", &pid_params.kd, sizeof(float));
    err |= nvs_set_blob(nvs_handle, "kx", &pid_params.kx, sizeof(float));

    if (err == ESP_OK) {
        err = nvs_commit(nvs_handle);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "PID saved to NVS - Kp: %.2f, Ki: %.2f, Kd: %.3f, Kx: %.2f", 
                     pid_params.kp, pid_params.ki, pid_params.kd, pid_params.kx);
        }
    }

    nvs_close(nvs_handle);
    return err;
}

// HTML page for PID tuning
static const char *html_page = 
"<!DOCTYPE html>"
"<html>"
"<head>"
"<meta name='viewport' content='width=device-width, initial-scale=1'>"
"<title>PID Tuning</title>"
"<style>"
"body { font-family: Arial; margin: 20px; background: #f0f0f0; }"
"h1 { color: #333; text-align: center; }"
".container { max-width: 600px; margin: 0 auto; background: white; padding: 30px; border-radius: 10px; box-shadow: 0 2px 10px rgba(0,0,0,0.1); }"
".param { margin: 25px 0; padding: 15px; background: #f9f9f9; border-radius: 5px; }"
"label { display: block; font-weight: bold; margin-bottom: 8px; color: #555; }"
"input[type='number'] { width: 100%; padding: 10px; font-size: 16px; border: 2px solid #ddd; border-radius: 5px; box-sizing: border-box; }"
"input[type='range'] { width: 100%; height: 30px; }"
".value { display: inline-block; min-width: 80px; font-size: 18px; font-weight: bold; color: #007bff; }"
"button { width: 100%; padding: 15px; font-size: 18px; background: #007bff; color: white; border: none; border-radius: 5px; cursor: pointer; margin-top: 20px; }"
"button:hover { background: #0056b3; }"
".info { text-align: center; margin-top: 20px; color: #666; font-size: 14px; }"
"</style>"
"</head>"
"<body>"
"<div class='container'>"
"<h1>PID Tuning Panel</h1>"

"<div class='param'>"
"<label>Kp (Proportional): <span class='value' id='kp_val'>%.1f</span></label>"
"<input type='range' id='kp' min='0' max='150' step='0.1' value='%.1f' oninput='updateValue(\"kp\", this.value)'>"
"<input type='number' id='kp_num' min='0' max='150' step='0.1' value='%.1f' oninput='updateSlider(\"kp\", this.value)'>"
"</div>"

"<div class='param'>"
"<label>Ki (Integral): <span class='value' id='ki_val'>%.1f</span></label>"
"<input type='range' id='ki' min='0' max='200' step='0.1' value='%.1f' oninput='updateValue(\"ki\", this.value)'>"
"<input type='number' id='ki_num' min='0' max='200' step='0.1' value='%.1f' oninput='updateSlider(\"ki\", this.value)'>"
"</div>"

"<div class='param'>"
"<label>Kd (Derivative): <span class='value' id='kd_val'>%.3f</span></label>"
"<input type='range' id='kd' min='0' max='20' step='0.001' value='%.3f' oninput='updateValue(\"kd\", this.value)'>"
"<input type='number' id='kd_num' min='0' max='20' step='0.001' value='%.3f' oninput='updateSlider(\"kd\", this.value)'>"
"</div>"
"<div class='param'>" 
"<label>Kx (Custom): <span class='value' id='kx_val'>%.2f</span></label>" 
"<input type='range' id='kx' min='0' max='50' step='0.01' value='%.2f' oninput='updateValue(\"kx\", this.value)'>" 
"<input type='number' id='kx_num' min='0' max='50' step='0.01' value='%.2f' oninput='updateSlider(\"kx\", this.value)'>" 
"</div>" 
"<button onclick='sendPID()'>Apply PID Parameters</button>"
"<div class='info' id='status'>Ready to tune PID</div>"
"</div>"

"<script>"
"function updateValue(param, val) {"
"  document.getElementById(param + '_val').innerText = parseFloat(val).toFixed(param === 'kd' ? 3 : 1);"
"  document.getElementById(param + '_num').value = val;"
"}"
"function updateSlider(param, val) {"
"  document.getElementById(param).value = val;"
"  document.getElementById(param + '_val').innerText = parseFloat(val).toFixed(param === 'kd' ? 3 : 1);"
"}"
"function sendPID() {"
"  var kp = document.getElementById('kp').value;"
"  var ki = document.getElementById('ki').value;"
"  var kd = document.getElementById('kd').value;"
"  var kx = document.getElementById('kx').value;"
"  var xhr = new XMLHttpRequest();"
"  xhr.open('POST', '/set_pid', true);"
"  xhr.setRequestHeader('Content-Type', 'application/x-www-form-urlencoded');"
"  xhr.onload = function() {"
"    if (xhr.status === 200) {"
"      document.getElementById('status').innerText = 'PID updated: Kp=' + kp + ', Ki=' + ki + ', Kd=' + kd + ', Kx=' + kx;"
"      document.getElementById('status').style.color = 'green';"
"    } else {"
"      document.getElementById('status').innerText = 'Failed to update PID';"
"      document.getElementById('status').style.color = 'red';"
"    }"
"  };"
"  xhr.send('kp=' + kp + '&ki=' + ki + '&kd=' + kd + '&kx=' + kx);"
"}"
"</script>"
"</body>"
"</html>";

// WiFi event handler
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < MAX_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "Retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG, "Connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        //ESP_LOGI(TAG, "Got IP:" IPSTR, IP2STR(&event->ip_info.ip));
        printf("Got IP: " IPSTR "\n", IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

// Initialize WiFi
esp_err_t wifi_init(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi initialization finished.");

    // Wait for connection
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Connected to AP SSID:%s", WIFI_SSID);
        return ESP_OK;
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s", WIFI_SSID);
        return ESP_FAIL;
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
        return ESP_FAIL;
    }
}

// HTTP GET handler for main page
static esp_err_t root_get_handler(httpd_req_t *req)
{
    // Allocate response buffer on heap to avoid stack overflow
    char *response = (char *)malloc(4096);
    if (response == NULL) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Out of memory");
        return ESP_FAIL;
    }
    
    snprintf(response, 4096, html_page, 
             pid_params.kp, pid_params.kp, pid_params.kp,
             pid_params.ki, pid_params.ki, pid_params.ki,
             pid_params.kd, pid_params.kd, pid_params.kd,
             pid_params.kx, pid_params.kx, pid_params.kx);
    
    httpd_resp_set_type(req, "text/html");
    esp_err_t ret = httpd_resp_send(req, response, strlen(response));
    
    free(response);
    return ret;
}

// HTTP POST handler for setting PID
static esp_err_t set_pid_post_handler(httpd_req_t *req)
{
    char buf[100];
    int ret, remaining = req->content_len;

    if (remaining >= sizeof(buf)) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Content too long");
        return ESP_FAIL;
    }

    ret = httpd_req_recv(req, buf, remaining);
    if (ret <= 0) {
        if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
            httpd_resp_send_408(req);
        }
        return ESP_FAIL;
    }
    buf[ret] = '\0';

    // Parse parameters
    float kp = 0, ki = 0, kd = 0, kx = 0;
    char *token = strtok(buf, "&");
    while (token != NULL) {
        if (strncmp(token, "kp=", 3) == 0) {
            kp = atof(token + 3);
        } else if (strncmp(token, "ki=", 3) == 0) {
            ki = atof(token + 3);
        } else if (strncmp(token, "kd=", 3) == 0) {
            kd = atof(token + 3);
        } else if (strncmp(token, "kx=", 3) == 0) {
            kx = atof(token + 3);
        }
        token = strtok(NULL, "&");
    }

    // Update PID parameters
    pid_params.kp = kp;
    pid_params.ki = ki;
    pid_params.kd = kd;
    pid_params.kx = kx;

    // Save to NVS (EEPROM)
    if (pid_save_to_nvs() == ESP_OK) {
        ESP_LOGI(TAG, "PID Updated & Saved - Kp: %.2f, Ki: %.2f, Kd: %.3f, Kx: %.2f", kp, ki, kd, kx);
    } else {
        ESP_LOGW(TAG, "PID Updated (not saved) - Kp: %.2f, Ki: %.2f, Kd: %.3f, Kx: %.2f", kp, ki, kd, kx);
    }

    httpd_resp_set_type(req, "text/plain");
    httpd_resp_send(req, "OK", 2);
    return ESP_OK;
}

// Favicon handler to suppress 404 warnings
static esp_err_t favicon_get_handler(httpd_req_t *req)
{
    httpd_resp_set_status(req, "204 No Content");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

// Start web server
esp_err_t webserver_start(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;
    config.stack_size = 8192;  // Increase stack size to handle large HTML

    ESP_LOGI(TAG, "Starting HTTP server on port %d", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        // Register URI handlers
        httpd_uri_t root = {
            .uri       = "/",
            .method    = HTTP_GET,
            .handler   = root_get_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &root);

        httpd_uri_t set_pid = {
            .uri       = "/set_pid",
            .method    = HTTP_POST,
            .handler   = set_pid_post_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &set_pid);

        httpd_uri_t favicon = {
            .uri       = "/favicon.ico",
            .method    = HTTP_GET,
            .handler   = favicon_get_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &favicon);

        ESP_LOGI(TAG, "Web server started successfully!");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Error starting server!");
    return ESP_FAIL;
}

// Get current PID parameters
void webserver_get_pid(float *kp, float *ki, float *kd, float *kx)
{
    *kp = pid_params.kp;
    *ki = pid_params.ki;
    *kd = pid_params.kd;
    *kx = pid_params.kx;
}
