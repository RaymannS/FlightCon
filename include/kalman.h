#pragma once

/**
 * kalman.h — 1D Kinematic Kalman Filter for Rocket Altitude + Velocity
 *
 * ═══════════════════════════════════════════════════════════════════════════
 * WHAT THIS FILTER DOES
 * ═══════════════════════════════════════════════════════════════════════════
 * Fuses two noisy sensors into one clean estimate:
 *
 *   INPUT 1 — Barometer altitude  (slow, absolute, noisy at high speed)
 *   INPUT 2 — IMU vertical accel  (fast, relative, drifts over time)
 *
 *   OUTPUT 1 — Filtered altitude AGL (metres)
 *   OUTPUT 2 — Vertical velocity     (m/s, positive = upward)
 *
 * The filter tracks two states:
 *   x[0] = altitude   (metres)
 *   x[1] = velocity   (m/s)
 *
 * ═══════════════════════════════════════════════════════════════════════════
 * HOW TO TUNE (read this before changing Q or R values)
 * ═══════════════════════════════════════════════════════════════════════════
 *
 * There are two noise matrices you can tune:
 *
 * --- Q (Process Noise) ---
 * Represents how much you DISTRUST the physics model between steps.
 * Higher Q = filter reacts faster to real changes, but noisier output.
 * Lower Q  = smoother output, but slower to respond to sudden changes.
 *
 *   KF_Q_ALTITUDE  — process noise for altitude state
 *   KF_Q_VELOCITY  — process noise for velocity state
 *
 * Without IMU: keep Q_VELOCITY LOW (0.1) — no accel input means velocity
 * is derived purely from baro changes, so we want it to be conservative.
 * With IMU: raise Q_VELOCITY to 1.0–5.0 to let accel drive velocity fast.
 *
 * --- R (Measurement Noise) ---
 * Represents how much you DISTRUST the barometer reading.
 * Higher R = filter trusts the physics model more, smoother but slower.
 * Lower R  = filter follows baro closely, noisier output.
 *
 *   KF_R_ALTITUDE  — barometer measurement noise variance (metres^2)
 *
 * Rule of thumb: take 100 baro readings at rest, compute variance.
 * For BMP280 at ~80Hz with X4 oversampling, measure ~2.0–5.0 m² variance.
 * Set KF_R_ALTITUDE to your measured variance.
 *
 * --- Baro-only vs IMU+Baro tuning ---
 *
 *   BARO ONLY (IMU unavailable):
 *     KF_Q_ALTITUDE = 0.05
 *     KF_Q_VELOCITY = 0.1    ← low: velocity estimated from baro only
 *     KF_R_ALTITUDE = 4.0    ← high: baro is noisy at X4 oversampling
 *
 *   IMU + BARO (normal flight):
 *     KF_Q_ALTITUDE = 0.1
 *     KF_Q_VELOCITY = 1.0    ← higher: IMU drives velocity estimate
 *     KF_R_ALTITUDE = 1.0    ← lower: IMU anchors altitude well
 *
 * --- Tuning procedure ---
 * 1. Log raw baro altitude to your database while stationary for 30 seconds.
 * 2. Compute variance of those readings → set KF_R_ALTITUDE to that value.
 * 3. If filtered altitude is still jumpy → increase KF_R_ALTITUDE.
 * 4. If filtered altitude lags real movement → decrease KF_R_ALTITUDE.
 * 5. If velocity drifts while stationary → decrease KF_Q_VELOCITY.
 * 6. When IMU arrives: decrease KF_R_ALTITUDE, increase KF_Q_VELOCITY.
 *
 * ═══════════════════════════════════════════════════════════════════════════
 * IMU MOUNTING NOTE
 * ═══════════════════════════════════════════════════════════════════════════
 * The filter uses accel_y from the BNO08x as the vertical acceleration input.
 * This assumes:
 *   - The BNO08x Y axis points ALONG the rocket body axis (nose-to-tail)
 *   - Positive Y = toward nose = upward when rocket is vertical
 *   - accel_y already has gravity removed (done in imu.cpp)
 *   - At rest on pad: accel_y ≈ 0 m/s²
 *
 * If your board is mounted differently, update imu.cpp to use the correct axis.
 * ═══════════════════════════════════════════════════════════════════════════
 */

// ─── Tuning constants ─────────────────────────────────────────────────────────
// Currently set for BARO-ONLY operation (no IMU).
// When IMU arrives: set KF_Q_VELOCITY = 1.0, KF_R_ALTITUDE = 1.0

#define KF_Q_ALTITUDE   0.05f   // metres^2    — altitude process noise
#define KF_Q_VELOCITY   0.1f    // (m/s)^2     — velocity process noise (low = stable without IMU)
#define KF_R_ALTITUDE   4.0f    // metres^2    — baro noise variance (measure and tune this)

// ─── Output structure ─────────────────────────────────────────────────────────

struct KfState {
    float altitude_m;   // Kalman-filtered altitude AGL (metres)
    float velocity_ms;  // Kalman-filtered vertical velocity (m/s, + = up)
};

// ─── Public API ───────────────────────────────────────────────────────────────

/**
 * @brief  Initialise the Kalman filter.
 *         Call once after bar_calibrate() so the initial altitude is valid.
 * @param  initial_alt_m   Starting altitude AGL in metres
 */
void kf_init(float initial_alt_m);

/**
 * @brief  Run one filter cycle. Only call when barometer has fresh data.
 *         When IMU is unavailable pass accel_y_ms2 = 0.0f and
 *         set baro_fresh = true only when a new baro reading exists.
 *
 * @param  baro_alt_m   Barometer altitude AGL in metres
 * @param  accel_y_ms2  Vertical linear acceleration (m/s^2, gravity removed)
 * @param  baro_fresh   true if barometer has a new reading this cycle
 * @param  dt_s         Time since last call in seconds
 * @return KfState      Current best estimate of altitude and velocity
 */
KfState kf_update(float baro_alt_m,
                  float accel_y_ms2,
                  bool  baro_fresh,
                  float dt_s);

/**
 * @brief  Return the last computed KfState without running a new cycle.
 */
KfState kf_get_state();