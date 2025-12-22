#ifndef MPU6050_H
#define MPU6050_H

#include <stdbool.h>
#include "esp_err.h"

// MPU6050 I2C Configuration
#define MPU6050_ADDR        0x68
#define MPU6050_WHO_AM_I    0x75
#define I2C_MASTER_SCL_IO   22      // GPIO number for I2C clock
#define I2C_MASTER_SDA_IO   21      // GPIO number for I2C data
#define I2C_MASTER_NUM      I2C_NUM_0
#define I2C_MASTER_FREQ_HZ  400000

// MPU6050 scale factors
#define ACCEL_SCALE         4096.0f  // ±8g → 4096 LSB/g
#define GYRO_SCALE          65.5f    // ±500°/s → 65.5 LSB/(°/s)
#define RAD_TO_DEG          57.2958f // 180/π

// Filter constants
#define ALPHA               0.66f    // Complementary filter // lower value means more trust on accel
#define ANGLE_FILTER_ALPHA  0.15f     // Low-pass filter  // higher value means more filtering
// Global sensor data variables (extern - defined in mpu6050.c)
extern float AccX, AccY, AccZ;                  // Accelerometer (g)
extern float RateRoll, RatePitch, RateYaw;      // Gyro rate (°/s)
extern float AngleRoll, AnglePitch;             // Final angles (degrees)
extern float gyroAngleRoll, gyroAnglePitch;     // Gyro-only angles
extern float accelAnglePitch;                    // Accelerometer-only angle
extern float filteredAngleRoll, filteredAnglePitch; // Filtered angles
extern float delta_t;                         // Loop time in seconds
extern int64_t prevTime;

// Function prototypes
/**
 * @brief Initialize I2C master for MPU6050
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t mpu6050_i2c_init(void);

/**
 * @brief Detect MPU6050 by reading WHO_AM_I register
 * @return true if MPU6050 is detected, false otherwise
 */
bool mpu6050_detect(void);

/**
 * @brief Initialize and calibrate MPU6050 sensor
 *        - Wakes up the sensor
 *        - Configures DLPF, accel range, gyro range
 *        - Calibrates gyro offsets (500 samples)
 */
void mpu6050_init_and_calibrate(void);

/**
 * @brief Read sensor data and update angles
 *        - Reads accelerometer and gyroscope data
 *        - Calculates angles using complementary filter + low-pass filter
 *        - Updates global variables (AngleRoll, AnglePitch, etc.)
 * 
 * Call this function in your main loop at regular intervals (e.g., 100Hz)
 */
void read_mpu(void);

#endif // MPU6050_H