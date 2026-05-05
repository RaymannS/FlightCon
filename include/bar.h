#pragma once

#include <stdint.h>
#include <stdbool.h>

// ─── I2C address ────────────────────────────────────────────────────────────
// SDO pin → GND  : 0x76  (default, recommended)
// SDO pin → 3.3V : 0x77  (use if another device occupies 0x76)
#define BMP280_I2C_ADDR   0x76

// Dedicated I2C1 bus pins (GPIO 18/19)
// GPIO 21/22 are reserved for LoRa UART; GPIO 25/26 are reserved for IMU I2C
#define BMP280_SDA_PIN    18
#define BMP280_SCL_PIN    19

// SEA_LEVEL_HPA is kept for reference / manual overrides but is no longer
// used as the default — bar_calibrate() derives ground pressure at boot.
#define SEA_LEVEL_HPA 1019.022f

// ─── Public API ──────────────────────────────────────────────────────────────

/**
 * @brief  Initialise the BMP280 over I2C1 (Wire1).
 *         Call once from setup(). Initializes Wire1 on GPIO 18/19 (separate
 *         from the IMU's I2C0 on GPIO 25/26). Safe to call after imuInit().
 * @return true  on success
 *         false if the sensor is not detected or chip-ID check fails
 */
bool bar_init(void);

/**
 * @brief  Sample ground pressure over a set window and store it as the AGL
 *         reference. Call once from setup() after bar_init().
 *         Prints progress and the final calibrated value to Serial.
 * @param  samples     Number of pressure readings to average (default 100)
 * @param  duration_ms Total sampling window in ms (default 10 000)
 * @return Calibrated ground pressure in hPa, or -999.0f on failure.
 */
float bar_calibrate(uint16_t samples = 100, uint32_t duration_ms = 10000);

/**
 * @brief  Return the ground pressure established by bar_calibrate().
 * @return Ground pressure in hPa, or 0.0f if not yet calibrated.
 */
float bar_get_ground_pressure(void);

/**
 * @brief  Read compensated temperature from the BMP280.
 * @return Temperature in degrees Celsius, or -999.0f on error.
 */
float bar_get_temperature(void);

/**
 * @brief  Read compensated pressure from the BMP280.
 * @return Pressure in hPa (= mbar), or -999.0f on error.
 */
float bar_get_pressure(void);

/**
 * @brief  Estimate altitude above sea level using the ISA formula.
 * @param  sea_level_hpa  Local QNH in hPa (default 1013.25 for ISA).
 * @return Altitude in metres, or -999.0f on error.
 */
float bar_get_altitude(float sea_level_hpa = 1013.25f);

/**
 * @brief  Altitude above ground level using the calibrated ground pressure.
 *         Returns -999.0f if bar_calibrate() has not been called yet.
 * @return Altitude AGL in metres.
 */
float bar_get_altitude_agl(void);

/**
 * @brief  Convenience: fill all three values in one I2C transaction burst.
 * @param[out] temp_c        Temperature in °C
 * @param[out] pressure_hpa  Pressure in hPa
 * @param[out] altitude_m    Altitude AGL in metres (uses calibrated ground pressure)
 * @return true on success
 */
bool bar_read_all(float &temp_c,
                  float &pressure_hpa,
                  float &altitude_m);