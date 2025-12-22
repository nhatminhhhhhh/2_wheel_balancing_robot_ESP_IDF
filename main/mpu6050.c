#include "mpu6050.h"
#include <math.h>
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "MPU6050";

// Global offset variables
static float GyroX_offset = 0;
static float GyroY_offset = 0;
static float GyroZ_offset = 0;

// Accelerometer readings (in g)
float AccX = 0, AccY = 0, AccZ = 0;

// Gyro rate readings (in °/s)
float RateRoll = 0, RatePitch = 0, RateYaw = 0;

// Gyro-only angle integration
float gyroAngleRoll = 0, gyroAnglePitch = 0;
// Accelerometer-only angles
float accelAnglePitch = 0;

// Previous filtered angles for low-pass filter
static float prevFilteredAngleRoll = 0, prevFilteredAnglePitch = 0;

// Final filtered angles
float filteredAngleRoll = 0, filteredAnglePitch = 0;

// Timing variables
float delta_t = 0.0f;                   
int64_t prevTime = 0;

// Filter constants
static const float alpha = ALPHA;
const float angleFilterAlpha = ANGLE_FILTER_ALPHA;

/**
 * @brief Initialize I2C master
 */
esp_err_t mpu6050_i2c_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) {
        return err;
    }
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

/**
 * @brief Write a byte to MPU6050 register
 */
static esp_err_t mpu6050_write_byte(uint8_t reg_addr, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief Read multiple bytes from MPU6050 register
 */
static esp_err_t mpu6050_read_bytes(uint8_t reg_addr, uint8_t *data, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);  // Repeated start
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_READ, true);
    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief Detect MPU6050 by reading WHO_AM_I register
 */
bool mpu6050_detect(void)
{
    uint8_t who_am_i = 0;
    
    esp_err_t ret = mpu6050_read_bytes(MPU6050_WHO_AM_I, &who_am_i, 1);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read WHO_AM_I register");
        return false;
    }
    
    ESP_LOGI(TAG, "WHO_AM_I register: 0x%02X (expected 0x68)", who_am_i);
    
    if (who_am_i == 0x68) {
        ESP_LOGI(TAG, "MPU6050 detected successfully!");
        return true;
    } else {
        ESP_LOGE(TAG, "MPU6050 not detected. Wrong chip ID!");
        return false;
    }
}

/**
 * @brief Initialize and calibrate MPU6050
 */
void mpu6050_init_and_calibrate(void)
{
    // Wake up MPU6050 (write 0 to PWR_MGMT_1 register 0x6B)
    mpu6050_write_byte(0x6B, 0x00);
    vTaskDelay(pdMS_TO_TICKS(100));

    // DLPF = 3 → ~44Hz (CONFIG register 0x1A)
    mpu6050_write_byte(0x1A, 0x03);

    // Accel ±8g (ACCEL_CONFIG register 0x1C)
    mpu6050_write_byte(0x1C, 0x10);

    // Gyro ±500°/s (GYRO_CONFIG register 0x1B)
    mpu6050_write_byte(0x1B, 0x08);

    vTaskDelay(pdMS_TO_TICKS(200));

    // ---- Calibrate gyro offset ----
    const int samples = 10000;
    long gx_sum = 0, gy_sum = 0, gz_sum = 0;
    
    ESP_LOGI(TAG, "Starting gyro calibration...");
    
    for (int i = 0; i < samples; i++) {
        uint8_t data[6];
        
        // Read gyro data from register 0x43 (6 bytes)
        if (mpu6050_read_bytes(0x43, data, 6) == ESP_OK) {
            int16_t gx = (int16_t)((data[0] << 8) | data[1]);
            int16_t gy = (int16_t)((data[2] << 8) | data[3]);
            int16_t gz = (int16_t)((data[4] << 8) | data[5]);

            gx_sum += gx;
            gy_sum += gy;
            gz_sum += gz;
        }
        
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    
    GyroX_offset = gx_sum / (float)samples;
    GyroY_offset = gy_sum / (float)samples;
    GyroZ_offset = gz_sum / (float)samples;

    ESP_LOGI(TAG, "MPU6050 initialized and calibrated done!");
    ESP_LOGI(TAG, "Gyro offsets - X: %.2f, Y: %.2f, Z: %.2f", 
             GyroX_offset, GyroY_offset, GyroZ_offset);
    
    // Initialize timing
    prevTime = esp_timer_get_time();
}

/**
 * @brief Read gyro and accelerometer data, calculate angles using complementary filter + low-pass filter
 */
void read_mpu(void)
{
    // ----- Loop timing (dt calculation) -----
    int64_t now = esp_timer_get_time(); // Get time in microseconds
    delta_t = (now - prevTime) / 1000000.0f; // Convert to seconds
    prevTime = now;

    uint8_t data[14];
    
    // Read 14 bytes starting from ACCEL_XOUT_H (0x3B)
    if (mpu6050_read_bytes(0x3B, data, 14) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read MPU6050 data");
        return;
    }

    // Parse accelerometer data (bytes 0-5)
    int16_t AccX_LSB = (int16_t)((data[0] << 8) | data[1]);
    int16_t AccY_LSB = (int16_t)((data[2] << 8) | data[3]);
    int16_t AccZ_LSB = (int16_t)((data[4] << 8) | data[5]);
    
    // Skip temperature (bytes 6-7)
    
    // Parse gyroscope data (bytes 8-13)
    int16_t GyroX_LSB = (int16_t)((data[8] << 8) | data[9]);
    int16_t GyroY_LSB = (int16_t)((data[10] << 8) | data[11]);
    int16_t GyroZ_LSB = (int16_t)((data[12] << 8) | data[13]);

    // Convert accelerometer raw values to g
    AccX = (float)AccX_LSB / ACCEL_SCALE;
    AccY = (float)AccY_LSB / ACCEL_SCALE;
    AccZ = (float)AccZ_LSB / ACCEL_SCALE;

    // Convert gyroscope raw values to °/s and apply offset calibration
    //RateRoll  = ((float)GyroX_LSB - GyroX_offset) / GYRO_SCALE;
    RatePitch = ((float)GyroY_LSB - GyroY_offset) / GYRO_SCALE;
    //RateYaw   = ((float)GyroZ_LSB - GyroZ_offset) / GYRO_SCALE;

    // ----- Angle from Accelerometer -----
    //float accelAngleRoll  = atan2(AccY, sqrt(AccX * AccX + AccZ * AccZ)) * RAD_TO_DEG;
    accelAnglePitch = -atan2(AccX, sqrt(AccY * AccY + AccZ * AccZ)) * RAD_TO_DEG;

    // ----- Calculate angle from gyroscope (integrate angular velocity) -----
    //gyroAngleRoll  += RateRoll * delta_t;
    gyroAnglePitch += RatePitch * delta_t;

    // ----- Apply complementary filter -----
    //float rawAngleRoll  = alpha * (prevFilteredAngleRoll + RateRoll * delta_t) + (1.0f - alpha) * accelAngleRoll;
    float rawAnglePitch = alpha * (prevFilteredAnglePitch + RatePitch * delta_t) + (1.0f - alpha) * accelAnglePitch;
    // ----- Additional low-pass filter to reduce noise -----
    //filteredAngleRoll  = angleFilterAlpha * prevFilteredAngleRoll + (1.0f - angleFilterAlpha) * rawAngleRoll;
    filteredAnglePitch = angleFilterAlpha * prevFilteredAnglePitch + (1.0f - angleFilterAlpha) * rawAnglePitch;
    //printf("Time Change: %.2f\r\n", delta_t);

    // Update previous filtered angles
    //prevFilteredAngleRoll  = filteredAngleRoll;
    prevFilteredAnglePitch = filteredAnglePitch;
}
