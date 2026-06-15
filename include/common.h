/*
All variables in this file were declared in main.cpp and moved here
to centralize the global variables for easy editing.

*/
#include <Arduino.h>
// ─── Transmit interval ────────────────────────────────────────────────────────
static constexpr uint32_t kTransmitIntervalMs = 1000; // 1 Hz downlink
static constexpr uint32_t kCommandRxWindowMs  = 12;   // brief RX dwell for uplink command checks
static uint32_t lastTxMs   = 0;


// ─── Airbrake config ──────────────────────────────────────────────────────────
static constexpr int   kServoChannel  = 0;
static constexpr float kAirbrakeAngle = 45.0f;   // degrees — deployed
static constexpr float kNeutralAngle  = 8.0f;    // degrees — retracted / safe

// ─── IREC compliance constants ────────────────────────────────────────────────

// 7.4.1.3.2 — 10K flight: airbrakes locked until 2,000 m AGL (1,200 m AGL for Test launch)
//TODO change back after done testing - 1200.0f
static constexpr float kAltitudeLockoutM = 1200.0f;

// 7.3.1 — retract immediately if tilt exceeds 30° from vertical
static constexpr float kMaxTiltDeg = 30.0f;

// Airbrake deployment altitude (AGL, metres).
// Set from OpenRocket simulation. Must be above kAltitudeLockoutM.
static constexpr float kDeployAltM        = 1500.0f; // 2500 m for real -- (1,500 m AGL for Test launch)
static constexpr float kDeployHysteresisM = 5.0f;

// ─── Liftoff / burnout detection ─────────────────────────────────────────────
// 550 lb thrust / 42 lb rocket = ~13G off pad.
//
// BNO055 in fusion mode clips at ~4G (39.2 m/s²) so we cannot use a 5G
// threshold directly. Instead we use a dual confirm:
//   1. IMU accel_y exceeds 2G (19.6 m/s²) — below clip limit, reliable
//   2. Baro altitude has risen at least 20m above launch altitude
// Both must be true simultaneously to trigger PAD -> BOOST.
// This prevents false triggers from pad bumps (IMU alone) or baro noise (baro alone).
static constexpr float    kLiftoffAccelMs2   = 20.0f;   // 2G — below BNO055 4G clip limit
static constexpr float    kLiftoffAltM       = 20.0f;   // metres AGL — baro confirm
static constexpr float    kBurnoutAccelMs2   = 2.0f;    // near-zero = motor out
static constexpr uint32_t kBurnoutBackstopMs = 5000;    // REAL 4s burn + 2s margin (test is 3.5s burn + 1.5s margin)

// ─── Landing detection ────────────────────────────────────────────────────────
// DESCENDING → DESCENDED when velocity stays below threshold for 3 seconds.
// Prevents false landing detection from momentary velocity dips during descent.
static constexpr float    kLandedVelThreshMs = 2.0f;    // m/s
static constexpr uint32_t kLandedConfirmMs   = 3000;    // ms
static uint32_t           landedTimerStart   = 0;
static bool               landedTimerRunning = false;

// ─── Barometer timing ─────────────────────────────────────────────────────────
// BMP280 at a slow 1 Hz cadence to match telemetry and reduce read failures.
// Kalman only runs on fresh baro reads to prevent velocity drift.
static constexpr uint32_t kBaroPeriodMs = 1000;
static uint32_t lastBaroMs   = 0;
static uint32_t lastKalmanMs = 0;

#ifdef ENABLE_OTA
static constexpr uint32_t kWifiActiveWindowMs = 1UL * 60UL * 1000UL;
#endif

// ─── Flight state machine ─────────────────────────────────────────────────────

enum class FlightState {
    PAD,         // Sitting on launch pad. Waiting for liftoff.
    BOOST,       // Motor burning. CAS locked neutral per IREC 7.4.1.
    COAST,       // Motor out, ascending. Airbrakes active if conditions met.
    DESCENDING,  // Past apogee, falling. Airbrakes retracted.
    DESCENDED    // On the ground. Flight over. Velocity forced to zero (ZUPT).
};
