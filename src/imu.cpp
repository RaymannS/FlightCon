/**
 * imu.cpp — BNO055 I2C driver for ESP32
 *
 * Sensor: Teyleten Robot BNO055 9-axis attitude sensor module
 * Bus:    I2C on ESP32 GPIO21 SDA, GPIO22 SCL
 *
 * This replaces the old BNO08x UART-RVC parser.
 */

#include "imu.h"

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>

// ─── I2C config ───────────────────────────────────────────────────────────────

static constexpr int kImuSdaPin = 21;
static constexpr int kImuSclPin = 22;

// Most BNO055 modules use 0x28.
// If the IMU is not detected, try changing this to 0x29.
static constexpr uint8_t kBno055Address = 0x28;

// If your module acts weird or does not have an external 32.768 kHz crystal,
// change this to false.
static constexpr bool kUseExternalCrystal = true;

// ─── Axis mapping ─────────────────────────────────────────────────────────────
//
// Your flight code expects:
//   sample.accel_y = rocket vertical/body-axis acceleration, gravity removed.
//
// The BNO055 gives linear acceleration as sensor X/Y/Z.
// Change these mappings if your physical mounting is different.
//
// Start with the board mounted so the BNO055 +Y axis points toward the rocket nose.
// If lifting the rocket upward does not make AccY go positive, change the mapping.

static constexpr int kRocketAxisX = 0;   // sensor X -> sample.accel_x
static constexpr int kRocketAxisY = 1;   // sensor Y -> sample.accel_y
static constexpr int kRocketAxisZ = 2;   // sensor Z -> sample.accel_z

static constexpr float kRocketAxisXSign =  1.0f;
static constexpr float kRocketAxisYSign =  1.0f;
static constexpr float kRocketAxisZSign =  1.0f;

// ─── Internal state ───────────────────────────────────────────────────────────

static Adafruit_BNO055 bno = Adafruit_BNO055(55, kBno055Address, &Wire);
static bool _ready = false;

// ─── Helpers ─────────────────────────────────────────────────────────────────

static float _axisValue(const imu::Vector<3>& v, int axis)
{
    switch (axis) {
        case 0: return v.x();
        case 1: return v.y();
        case 2: return v.z();
        default: return 0.0f;
    }
}

/**
 * Compute tilt angle from vertical using pitch and roll.
 *
 * When the rocket is vertical:
 *   pitch ≈ 0
 *   roll  ≈ 0
 *   tilt  ≈ 0
 */
static float _compute_tilt(float pitch_deg, float roll_deg)
{
    float pitch_rad = pitch_deg * DEG_TO_RAD;
    float roll_rad  = roll_deg  * DEG_TO_RAD;

    float cos_tilt = cosf(pitch_rad) * cosf(roll_rad);
    cos_tilt = constrain(cos_tilt, -1.0f, 1.0f);

    return acosf(cos_tilt) * RAD_TO_DEG;
}

static void _printCalibration()
{
    uint8_t sys = 0;
    uint8_t gyro = 0;
    uint8_t accel = 0;
    uint8_t mag = 0;

    bno.getCalibration(&sys, &gyro, &accel, &mag);

    Serial.printf("[IMU] Calibration SYS=%u GYRO=%u ACCEL=%u MAG=%u\n",
                  sys, gyro, accel, mag);
}

// ─── Public implementation ───────────────────────────────────────────────────

bool imuInit()
{
    Wire.begin(kImuSdaPin, kImuSclPin);
    Wire.setClock(400000);

    delay(100);

    if (!bno.begin()) {
        Serial.println("[IMU] BNO055 not detected.");
        Serial.println("[IMU] Check wiring:");
        Serial.println("[IMU]   VIN/VCC -> 3.3V");
        Serial.println("[IMU]   GND     -> GND");
        Serial.println("[IMU]   SDA     -> GPIO21");
        Serial.println("[IMU]   SCL     -> GPIO22");
        Serial.println("[IMU] If wiring is correct, try changing kBno055Address from 0x28 to 0x29.");
        _ready = false;
        return false;
    }

    delay(1000);

    bno.setExtCrystalUse(kUseExternalCrystal);

    _ready = true;

    Serial.printf("[IMU] BNO055 ready on I2C address 0x%02X\n", kBno055Address);
    Serial.printf("[IMU] SDA=GPIO%d, SCL=GPIO%d\n", kImuSdaPin, kImuSclPin);
    Serial.println("[IMU] Linear acceleration is gravity-removed.");
    Serial.println("[IMU] Make sure accel_y is your rocket vertical/body axis.");

    _printCalibration();

    return true;
}

// ─────────────────────────────────────────────────────────────────────────────

bool imuRead(ImuSample &sample)
{
    if (!_ready) return false;

    // BNO055 Euler vector:
    //   x = heading/yaw
    //   y = roll
    //   z = pitch
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

    // BNO055 linear acceleration:
    //   acceleration with gravity removed, units = m/s²
    imu::Vector<3> linAccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

    // Orientation
    sample.yaw   = euler.x();
    sample.roll  = euler.y();
    sample.pitch = euler.z();

    // Acceleration axis mapping
    sample.accel_x = kRocketAxisXSign * _axisValue(linAccel, kRocketAxisX);
    sample.accel_y = kRocketAxisYSign * _axisValue(linAccel, kRocketAxisY);
    sample.accel_z = kRocketAxisZSign * _axisValue(linAccel, kRocketAxisZ);

    // Tilt safety value used by your state machine
    sample.tilt_deg = _compute_tilt(sample.pitch, sample.roll);

    return true;
}

// ─────────────────────────────────────────────────────────────────────────────

bool imuReady()
{
    return _ready;
}