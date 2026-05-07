/**
 * kalman.cpp — 1D Kinematic Kalman Filter Implementation
 *
 * State vector (2x1):
 *   x = [ altitude  ]   (metres)
 *       [ velocity  ]   (m/s, positive = upward)
 *
 * ─────────────────────────────────────────────────────────────────────────────
 * MATH OVERVIEW (read before modifying)
 * ─────────────────────────────────────────────────────────────────────────────
 *
 * The Kalman filter has two phases each cycle:
 *
 * ── PREDICT (using IMU acceleration) ─────────────────────────────────────────
 *
 * Physics model (kinematic equations):
 *   altitude_new = altitude + velocity*dt + 0.5*accel*dt^2
 *   velocity_new = velocity + accel*dt
 *
 * In matrix form:  x_pred = F*x + B*u
 *   F = [ 1  dt ]   (state transition matrix)
 *       [ 0   1 ]
 *
 *   B = [ 0.5*dt^2 ]   (control input matrix)
 *       [    dt    ]
 *
 *   u = accel_y   (IMU vertical acceleration, scalar)
 *
 * Covariance predict:  P_pred = F*P*F' + Q
 *   Q = [ KF_Q_ALTITUDE    0          ]   (process noise matrix)
 *       [     0         KF_Q_VELOCITY ]
 *
 * ── UPDATE (using barometer) ──────────────────────────────────────────────────
 *
 * We only measure altitude, not velocity, so:
 *   H = [ 1  0 ]   (measurement matrix — picks altitude from state)
 *
 * Innovation (how wrong was our prediction):
 *   y = baro_alt - H*x_pred
 *
 * Innovation covariance:
 *   S = H*P_pred*H' + R
 *   R = KF_R_ALTITUDE   (scalar — baro noise variance)
 *
 * Kalman gain (how much to trust the measurement vs prediction):
 *   K = P_pred*H' / S
 *   K = [ P[0][0] / S ]   (2x1 vector, expanded below)
 *       [ P[1][0] / S ]
 *
 * State update:
 *   x = x_pred + K*y
 *
 * Covariance update (Joseph form for numerical stability):
 *   P = (I - K*H) * P_pred
 *
 * ─────────────────────────────────────────────────────────────────────────────
 * All 2x2 matrix operations are written out explicitly (no matrix library).
 * This makes the math easy to follow and modify.
 * ─────────────────────────────────────────────────────────────────────────────
 */

#include "kalman.h"
#include <Arduino.h>
#include <math.h>

// ─── Internal state ───────────────────────────────────────────────────────────

// State estimate vector: x[0] = altitude (m), x[1] = velocity (m/s)
static float x[2] = {0.0f, 0.0f};

// Error covariance matrix P (2x2), stored row-major: P[row][col]
// Represents our uncertainty in the state estimate.
// Initialised large (high uncertainty) so the filter converges quickly.
static float P[2][2] = {
    {1000.0f,    0.0f},
    {   0.0f, 1000.0f}
};

// Last returned state
static KfState _last = {0.0f, 0.0f};

static bool _initialised = false;

// ─── Public implementation ────────────────────────────────────────────────────

void kf_init(float initial_alt_m)
{
    // Seed state with known starting altitude, zero velocity
    x[0] = initial_alt_m;
    x[1] = 0.0f;

    // Reset covariance — high uncertainty in velocity, low in altitude
    // since we just calibrated the barometer
    P[0][0] = 1.0f;      // altitude variance — we trust the baro cal
    P[0][1] = 0.0f;
    P[1][0] = 0.0f;
    P[1][1] = 100.0f;    // velocity variance — we don't know velocity yet

    _last = {initial_alt_m, 0.0f};
    _initialised = true;

    Serial.printf("[KF] Initialised — alt0=%.2f m\n", initial_alt_m);
}

// ─────────────────────────────────────────────────────────────────────────────

KfState kf_update(float baro_alt_m,
                  float accel_y_ms2,
                  bool  baro_fresh,
                  float dt_s)
{
    if (!_initialised) {
        Serial.println("[KF] kf_update() called before kf_init()");
        return _last;
    }

    // Clamp dt to reasonable range — prevents explosion if loop stalls
    if (dt_s <= 0.0f || dt_s > 0.5f) dt_s = 0.01f;

    // ── PREDICT STEP ─────────────────────────────────────────────────────────
    //
    // Propagate state forward using kinematics:
    //   altitude_pred = altitude + velocity*dt + 0.5*accel*dt^2
    //   velocity_pred = velocity + accel*dt
    //
    // This is F*x + B*u written out for our 2-state system.

    float dt2 = dt_s * dt_s;   // dt squared — used in altitude update

    float x0_pred = x[0] + x[1]*dt_s + 0.5f*accel_y_ms2*dt2;  // altitude

    // Velocity predict with damping.
    // kVelDecay < 1.0 means velocity bleeds toward zero each step
    // unless real acceleration is actively driving it.
    //
    // This prevents baro noise from freely integrating into velocity
    // when no IMU accel input is available (accel_y_ms2 = 0.0f).
    //
    // Tuning:
    //   0.99 = very light damping, velocity takes ~4s to halve at 40Hz
    //   0.97 = moderate damping, velocity halves in ~1.5s at 40Hz  <- current
    //   0.95 = aggressive damping, velocity halves in ~0.8s at 40Hz
    //
    // When IMU is re-enabled for flight, keep at 0.97 — during boost the
    // accel input (130+ m/s²) overwhelms the tiny decay term completely.
    static constexpr float kVelDecay = 0.97f;
    float x1_pred = x[1] * kVelDecay + accel_y_ms2*dt_s;       // velocity

    // ── Covariance predict: P_pred = F*P*F' + Q ──────────────────────────────
    //
    // F = [ 1  dt ]
    //     [ 0   1 ]
    //
    // F*P (2x2 result):
    //   row0: [ P[0][0] + dt*P[1][0],  P[0][1] + dt*P[1][1] ]
    //   row1: [ P[1][0],               P[1][1]               ]
    //
    // (F*P)*F' then adds the transpose:
    //   P_pred[0][0] = FP[0][0] + dt*FP[0][1]
    //   P_pred[0][1] = FP[0][1] + dt*FP[1][1]   (= P_pred[1][0] by symmetry)
    //   P_pred[1][1] = FP[1][1]
    //
    // Then add Q (diagonal process noise matrix):
    //   P_pred[0][0] += KF_Q_ALTITUDE
    //   P_pred[1][1] += KF_Q_VELOCITY

    // Intermediate: FP = F * P
    float FP00 = P[0][0] + dt_s * P[1][0];
    float FP01 = P[0][1] + dt_s * P[1][1];
    float FP10 = P[1][0];
    float FP11 = P[1][1];

    // P_pred = FP * F' + Q
    float Pp00 = FP00 + dt_s * FP01  + KF_Q_ALTITUDE;   // [0][0]
    float Pp01 = FP01 + dt_s * FP11;                     // [0][1]
    float Pp10 = Pp01;                                    // [1][0] symmetric
    float Pp11 = FP11                 + KF_Q_VELOCITY;   // [1][1]

    // ── UPDATE STEP (only when barometer has fresh data) ──────────────────────
    if (baro_fresh) {
        //
        // H = [ 1  0 ]  — we observe altitude only
        //
        // Innovation: y = baro_alt - H*x_pred = baro_alt - x0_pred
        float y = baro_alt_m - x0_pred;

        // Innovation covariance: S = H*P_pred*H' + R
        // Since H picks only the first row/column: S = Pp00 + R  (scalar)
        float S = Pp00 + KF_R_ALTITUDE;

        // Guard against divide-by-zero
        if (fabsf(S) < 1e-6f) S = 1e-6f;

        // Kalman gain: K = P_pred * H' / S
        // H' = [ 1 ]  so P_pred*H' = first column of P_pred
        //      [ 0 ]
        float K0 = Pp00 / S;   // gain for altitude state
        float K1 = Pp10 / S;   // gain for velocity state

        // State update: x = x_pred + K * y
        x[0] = x0_pred + K0 * y;
        x[1] = x1_pred + K1 * y;

        // Covariance update: P = (I - K*H) * P_pred
        // (I - K*H) = [ 1-K0   0 ]
        //             [ -K1    1 ]
        //
        // Multiplied out with P_pred:
        P[0][0] = (1.0f - K0) * Pp00;
        P[0][1] = (1.0f - K0) * Pp01;
        P[1][0] = -K1 * Pp00 + Pp10;
        P[1][1] = -K1 * Pp01 + Pp11;

    } else {
        // No new baro data — accept predict step as-is
        x[0] = x0_pred;
        x[1] = x1_pred;
        P[0][0] = Pp00;
        P[0][1] = Pp01;
        P[1][0] = Pp10;
        P[1][1] = Pp11;
    }

    _last = {x[0], x[1]};
    return _last;
}

// ─────────────────────────────────────────────────────────────────────────────

KfState kf_get_state()
{
    return _last;
}