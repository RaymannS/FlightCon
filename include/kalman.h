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
 * Rule of thumb: set Q_VELOCITY ≈ (max expected acceleration uncertainty)^2 * dt
 * For a rocket with ~130 m/s² peak accel, start around 0.5–2.0.
 *
 * --- R (Measurement Noise) ---
 * Represents how much you DISTRUST the barometer reading.
 * Higher R = filter trusts IMU more, ignores baro more.
 * Lower R  = filter trusts baro more, follows it closely.
 *
 *   KF_R_ALTITUDE  — barometer measurement noise variance (metres^2)
 *
 * Rule of thumb: take 100 baro readings at rest, compute variance. 
 * For BMP280 at ~80Hz with X4 oversampling, expect ~0.5–2.0 m² variance.
 * Set KF_R_ALTITUDE to that measured variance.
 *
 * --- Tuning procedure ---
 * 1. Log raw baro altitude and raw accel_y to your database during a static test.
 * 2. Compute variance of baro altitude while stationary → set KF_R_ALTITUDE.
 * 3. Start with KF_Q_VELOCITY = 1.0, KF_Q_ALTITUDE = 0.1.
 * 4. If filtered altitude lags behind real events → increase Q values.
 * 5. If filtered altitude is noisy → decrease Q values or increase R.
 * 6. Compare filtered velocity to finite-difference of baro altitude post-flight.
 *    They should agree in trend but filtered should be much smoother.
 *
 * ═══════════════════════════════════════════════════════════════════════════
 * IMU MOUNTING NOTE
 * ═══════════════════════════════════════════════════════════════════════════
 * The filter uses accel_y from the BNO055 as the vertical acceleration input.
 * This assumes:
 *   - The BNO055 Y axis points ALONG the rocket body axis (nose-to-tail)
 *   - Positive Y = toward nose = upward when rocket is vertical
 *   - The BNO055 is mounted FLAT on your PCB with the Y silkscreen label
 *     aligned with the rocket's long axis
 *   - The BNO055 gravity compensation is active (LINEAR_ACCEL vector used,
 *     NOT raw accelerometer) so accel_y = 0 when rocket is stationary
 *
 * If your board is mounted differently (Y axis horizontal, Z vertical, etc.),
 * change kf_update() to use the correct axis from ImuSample.
 * ═══════════════════════════════════════════════════════════════════════════
 */

// ─── Tuning constants ─────────────────────────────────────────────────────────

// Process noise — how much we distrust the physics model per step
#define KF_Q_ALTITUDE   0.1f    // metres^2   — altitude process noise
#define KF_Q_VELOCITY   1.0f    // (m/s)^2    — velocity process noise

// Measurement noise — how much we distrust the barometer
#define KF_R_ALTITUDE   1.0f    // metres^2   — baro noise variance

// ─── Output structure ─────────────────────────────────────────────────────────

struct KfState {
    float altitude_m;   // Kalman-filtered altitude AGL (metres)
    float velocity_ms;  // Kalman-filtered vertical velocity (m/s, + = up)
};

// ─── Public API ───────────────────────────────────────────────────────────────

/**
 * @brief  Initialise the Kalman filter.
 *         Call once after bar_calibrate() so the initial altitude is valid.
 * @param  initial_alt_m   Starting altitude AGL in metres (use bar_get_altitude_agl())
 */
void kf_init(float initial_alt_m);

/**
 * @brief  Run one filter cycle. Call every loop iteration.
 *
 *         The filter runs in two steps:
 *           1. PREDICT — uses IMU accel to propagate state forward by dt
 *           2. UPDATE  — corrects with barometer reading (when available)
 *
 * @param  baro_alt_m   Barometer altitude AGL in metres
 * @param  accel_y_ms2  Vertical linear acceleration from BNO055 (m/s^2, gravity removed)
 * @param  baro_fresh   true if the barometer has a new reading this cycle
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