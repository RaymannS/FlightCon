#pragma once

/**
 * imu.h — BNO055 IMU interface
 *
 * Sensor: Teyleten Robot BNO055 9-axis attitude sensor module
 * Bus:    I2C
 * Mode:   IMUPLUS — accel + gyro only, no magnetometer
 *
 * Wiring:
 *   BNO055 VCC -> ESP32 3.3V
 *   BNO055 GND -> ESP32 GND
 *   BNO055 SDA -> ESP32 GPIO 25
 *   BNO055 SCL -> ESP32 GPIO 26
 *
 * Calibration:
 *   - Loads from flash automatically on boot if previously saved.
 *   - Auto-saves once SYS, GYRO, and ACCEL all reach 3.
 *   - MAG is not used or checked.
 *   - Only needs recalibration if the board is physically remounted.
 *
 * Axis mapping:
 *   - accel_y is the rocket vertical/body axis (toward nose = positive).
 *   - If accel_y is negative during upward flight, flip kRocketAxisYSign
 *     in imu.cpp to -1.0f.
 */

// ─── Data Structure ───────────────────────────────────────────────────────────

struct ImuSample {
    // Euler angles (degrees)
    float yaw;    // heading   0-360 deg
    float pitch;  // nose up/down  +-90 deg
    float roll;   // rotation around body axis  +-180 deg

    // Linear acceleration (m/s^2) — gravity removed by BNO055 on-chip
    // accel_y = rocket vertical axis (used for liftoff, burnout, Kalman)
    // At rest all three should read ~0 m/s^2
    float accel_x;
    float accel_y;
    float accel_z;

    // Tilt from vertical (degrees)
    // 0 deg = perfectly vertical, 90 deg = horizontal
    // Used for IREC 7.3.1 safety check (retract if > 30 deg)
    float tilt_deg;
};

// ─── Public API ───────────────────────────────────────────────────────────────

/**
 * @brief  Initialize BNO055 over I2C in IMUPLUS mode.
 *         Attempts to load saved calibration from flash on boot.
 *         Call once from setup() after Serial.begin().
 * @return true if BNO055 was detected and initialized.
 */
bool imuInit();

/**
 * @brief  Read latest orientation and linear acceleration.
 *         Also checks calibration status and auto-saves when complete.
 *         Call every loop iteration — not blocking.
 * @param[out] sample  Populated with latest IMU data.
 * @return true on success.
 */
bool imuRead(ImuSample &sample);

/**
 * @brief  Returns true if imuInit() succeeded.
 */
bool imuReady();