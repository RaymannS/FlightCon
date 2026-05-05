#pragma once

// ─── I2C Pins ────────────────────────────────────────────────────────────────
// IMU uses Wire (I2C0) on GPIO 25/26, separate from Wire1 (BMP280 + PCA9685)
#define IMU_SDA_PIN   25
#define IMU_SCL_PIN   26

// BNO055 I2C address
// ADD pin → 3.3V : 0x29
// ADD pin → GND  : 0x28
#define BNO055_ADDR   0x29

// ─── Data Structures ─────────────────────────────────────────────────────────

struct ImuSample {
    // Euler angles (degrees)
    float roll;   // rotation around Z axis
    float pitch;  // rotation around X axis
    float yaw;    // rotation around Y axis (vertical axis)

    // Linear acceleration (m/s^2) — gravity removed by BNO055 on-chip fusion
    // Y axis is vertical (along rocket axis)
    float accel_x;
    float accel_y;  // vertical acceleration — used for liftoff + burnout detect
    float accel_z;

    // Tilt angle from vertical (degrees) — derived from pitch + roll
    // Used for the IREC 30-degree safety check
    float tilt_deg;
};

// ─── Public API ──────────────────────────────────────────────────────────────

/**
 * @brief  Initialise the BNO055 over I2C0 (Wire) on GPIO 25/26.
 *         Call once from setup() after Serial.begin().
 * @return true on success, false if sensor not found.
 */
bool imuInit();

/**
 * @brief  Read latest fused data from BNO055.
 *         Returns false if no new data is available or sensor not ready.
 * @param[out] sample  Filled with current orientation + linear acceleration.
 * @return true on success.
 */
bool imuRead(ImuSample &sample);

/**
 * @brief  Returns true if the BNO055 has been successfully initialised.
 */
bool imuReady();