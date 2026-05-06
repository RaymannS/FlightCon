#pragma once

/**
 * imu.h — BNO055 IMU interface
 *
 * Sensor: Teyleten Robot BNO055 9-axis attitude sensor module
 * Bus:    I2C
 *
 * Wiring:
 *   BNO055 VIN/VCC -> ESP32 3.3V
 *   BNO055 GND     -> ESP32 GND
 *   BNO055 SDA     -> ESP32 GPIO21
 *   BNO055 SCL     -> ESP32 GPIO22
 *
 * Notes:
 *   - BNO055 uses I2C, not UART.
 *   - Most BNO055 modules use I2C address 0x28.
 *   - Some modules use 0x29.
 *   - imuRead() returns linear acceleration, meaning gravity is already removed.
 */

// ─── Data Structure ───────────────────────────────────────────────────────────

struct ImuSample {
    // Euler angles in degrees
    float yaw;    // heading
    float pitch;  // nose up/down
    float roll;   // rotation around body axis

    // Linear acceleration in m/s², gravity removed
    //
    // Your main flight code uses accel_y for:
    //   - liftoff detection
    //   - burnout detection
    //   - Kalman filter input
    //
    // So imu.cpp maps the rocket's vertical/body axis into accel_y.
    float accel_x;
    float accel_y;
    float accel_z;

    // Tilt from vertical in degrees
    // 0° = vertical, 90° = horizontal
    float tilt_deg;
};

// ─── Public API ───────────────────────────────────────────────────────────────

/**
 * @brief  Initialize the BNO055 over I2C.
 * @return true if the BNO055 was detected and initialized.
 */
bool imuInit();

/**
 * @brief  Read latest BNO055 orientation and linear acceleration.
 * @param[out] sample Populated with latest IMU data.
 * @return true if data was read successfully.
 */
bool imuRead(ImuSample &sample);

/**
 * @brief  Returns true if imuInit() successfully initialized the IMU.
 */
bool imuReady();