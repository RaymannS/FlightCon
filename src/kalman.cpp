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
 * ── PREDICT ──────────────────────────────────────────────────────────────────
 *
 *   x0_pred = altitude + velocity*dt + 0.5*accel*dt^2
 *   x1_pred = velocity * kVelDecay  + accel*dt
 *
 * kVelDecay < 1.0 damps velocity toward zero each step when no real accel
 * is driving it. Prevents baro noise integrating into runaway velocity.
 * This modifies the state transition matrix F:
 *
 *   F = [ 1          dt ]
 *       [ 0   kVelDecay ]
 *
 * Covariance predict:
 *   P_pred = F * P * F' + Q
 *
 *   Q = [ KF_Q_ALTITUDE        0 ]
 *       [         0  KF_Q_VELOCITY ]
 *
 * ── UPDATE ───────────────────────────────────────────────────────────────────
 *
 * Measurement matrix (altitude only):
 *   H = [ 1  0 ]
 *
 * Innovation:         y = baro_alt - x0_pred
 * Innovation cov:     S = Pp00 + R
 * Kalman gain:        K = [ Pp00/S ]
 *                         [ Pp10/S ]
 * State update:       x = x_pred + K * y
 *
 * Covariance — Joseph form (numerically stable, guarantees positive-definite):
 *   IKH = I - K*H = [ 1-K0   0 ]
 *                   [  -K1   1 ]
 *   P = IKH * P_pred * IKH' + K * R * K'
 *
 * The Joseph form adds the K*R*K' term which the simple (I-KH)*P form omits.
 * That omission was causing P to go negative and produce NaN.
 *
 * ─────────────────────────────────────────────────────────────────────────────
 */

#include "kalman.h"
#include <Arduino.h>
#include <math.h>

// ─── Runtime tunables (defined here so they can be changed at runtime) ──────
// Defaults chosen to match the previous "Bench" settings in the header.
float KF_Q_ALTITUDE = KF_TUNE_PAD_Q_ALT;
float KF_Q_VELOCITY = KF_TUNE_PAD_Q_VEL;
float KF_R_ALTITUDE = KF_TUNE_PAD_R_ALT;

// ─── Velocity damping ─────────────────────────────────────────────────────────
// kVelDecay is applied every predict step.
//
// Tuning guide:
//   0.99 = light  — velocity halves in ~3s  at 40Hz
//   0.97 = medium — velocity halves in ~1s  at 40Hz  <- current
//   0.95 = heavy  — velocity halves in ~0.5s at 40Hz
//
// During flight with IMU, motor accel (130+ m/s²) completely overwhelms
// the tiny decay so flight behaviour is unaffected.
static constexpr float kVelDecay = 0.995f;

// Default dt used when measured dt is out of valid range
static constexpr float kBaroPeriodDefault = 0.025f;  // 40Hz = 25ms

// Maximum allowed P matrix element — prevents covariance runaway
static constexpr float kPMax = 1.0e5f;

// ─── Internal state ───────────────────────────────────────────────────────────

static float x[2]    = {0.0f, 0.0f};  // x[0]=altitude(m), x[1]=velocity(m/s)
static float P[2][2] = {{500.0f, 0.0f}, {0.0f, 500.0f}};
static KfState _last = {0.0f, 0.0f};
static bool _initialised = false;

// ─── Helpers ─────────────────────────────────────────────────────────────────

/** Reset filter to safe state if a NaN or Inf is detected. */
static bool _nanCheck(float v, const char* label)
{
    if (!isfinite(v)) {
        Serial.printf("[KF] WARNING: %s is NaN/Inf — resetting\n", label);
        x[0] = _last.altitude_m;
        x[1] = 0.0f;
        P[0][0] = 500.0f; P[0][1] = 0.0f;
        P[1][0] = 0.0f;   P[1][1] = 500.0f;
        return true;
    }
    return false;
}

/** Enforce P symmetry, positive diagonal, and max clamp. */
static void _sanitiseP()
{
    // Symmetry — average off-diagonal to remove floating point drift
    float avg = 0.5f * (P[0][1] + P[1][0]);
    P[0][1] = avg;
    P[1][0] = avg;

    // Positive diagonal — variances must be >= 0
    if (P[0][0] < KF_Q_ALTITUDE) P[0][0] = KF_Q_ALTITUDE;
    if (P[1][1] < KF_Q_VELOCITY) P[1][1] = KF_Q_VELOCITY;

    // Clamp — prevents runaway during long predict-only runs
    for (int i = 0; i < 2; i++)
        for (int j = 0; j < 2; j++)
            P[i][j] = constrain(P[i][j], -kPMax, kPMax);
}

// ─── Public implementation ────────────────────────────────────────────────────

void kf_init(float initial_alt_m)
{
    x[0] = initial_alt_m;
    x[1] = 0.0f;

    // Low altitude uncertainty — we averaged 20 baro readings for this.
    // High velocity uncertainty — no velocity measurement available yet.
    P[0][0] = 1.0f;   P[0][1] = 0.0f;
    P[1][0] = 0.0f;   P[1][1] = 100.0f;

    _last        = {initial_alt_m, 0.0f};
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

    // Clamp dt to valid range
    if (dt_s <= 0.0f || dt_s > 0.5f) dt_s = kBaroPeriodDefault;

    float dt2 = dt_s * dt_s;

    // ── PREDICT ───────────────────────────────────────────────────────────────

    float x0_pred = x[0] + x[1]*dt_s + 0.5f*accel_y_ms2*dt2;
    float x1_pred = x[1]*kVelDecay   + accel_y_ms2*dt_s;

    // ── Covariance predict: P_pred = F*P*F' + Q ──────────────────────────────
    //
    // F = [ 1          dt ]
    //     [ 0   kVelDecay ]   (d = kVelDecay for brevity)
    //
    // FP = F*P:
    //   FP[0][0] = P00 + dt*P10
    //   FP[0][1] = P01 + dt*P11
    //   FP[1][0] = d*P10
    //   FP[1][1] = d*P11
    //
    // Pp = FP*F' + Q:
    //   Pp[0][0] = FP00 + dt*FP01        + Q_alt
    //   Pp[0][1] = FP01 + dt*FP11
    //   Pp[1][0] = d*FP10 + d*dt*FP11    (= FP10 + dt*FP11 scaled by d... see below)
    //   Pp[1][1] = d*FP11                + Q_vel
    //
    // Note: Pp[1][0] = d*(P10 + dt*P11) which equals d*FP01 (not Pp[0][1])
    // This is why Pp is NOT symmetric when d != 1.
    // We force symmetry after by averaging.

    float d    = kVelDecay;
    float FP00 = P[0][0] + dt_s*P[1][0];
    float FP01 = P[0][1] + dt_s*P[1][1];
    float FP10 = d * P[1][0];
    float FP11 = d * P[1][1];

    float Pp00 = FP00 + dt_s*FP01 + KF_Q_ALTITUDE;
    float Pp01 = FP01 + dt_s*FP11;
    float Pp10 = FP10 + d*dt_s*FP11;   // Note: NOT equal to Pp01 when d<1
    float Pp11 = d*FP11               + KF_Q_VELOCITY;

    // Force symmetry and positive diagonal before update
    float Pp_offdiag = 0.5f*(Pp01 + Pp10);
    Pp01 = Pp_offdiag;
    Pp10 = Pp_offdiag;
    if (Pp00 < KF_Q_ALTITUDE) Pp00 = KF_Q_ALTITUDE;
    if (Pp11 < KF_Q_VELOCITY) Pp11 = KF_Q_VELOCITY;

    // ── UPDATE ────────────────────────────────────────────────────────────────
    if (baro_fresh) {

        // Innovation
        float y = baro_alt_m - x0_pred;

        // Innovation covariance: S = Pp00 + R
        float S = Pp00 + KF_R_ALTITUDE;
        if (S < 1e-4f) S = 1e-4f;   // hard floor against divide-by-zero

        // Kalman gain — clamped to physically valid range
        float K0 = constrain(Pp00 / S, 0.0f,  1.0f);
        float K1 = constrain(Pp10 / S, -1.0f, 1.0f);

        // State update
        x[0] = x0_pred + K0 * y;
        x[1] = x1_pred + K1 * y;

        // ── Covariance update — Joseph form ───────────────────────────────────
        //
        // IKH = (I - K*H):
        //   [ 1-K0    0 ]
        //   [ -K1     1 ]
        //
        // Step 1: IKH * Pp
        float A00 = (1.0f-K0)*Pp00;
        float A01 = (1.0f-K0)*Pp01;
        float A10 = -K1*Pp00 + Pp10;
        float A11 = -K1*Pp01 + Pp11;

        // Step 2: (IKH * Pp) * IKH'  +  K * R * K'
        // IKH' = [ 1-K0   -K1 ]
        //        [   0      1 ]
        //
        // Combined in one step — avoids unused variable warnings:
        // P[i][j] = (IKH*Pp)*IKH'[i][j] + R*K[i]*K[j]
        P[0][0] = A00*(1.0f-K0) + KF_R_ALTITUDE*K0*K0;
        P[0][1] = A00*(-K1) + A01 + KF_R_ALTITUDE*K0*K1;
        P[1][0] = A10*(1.0f-K0) + KF_R_ALTITUDE*K1*K0;
        P[1][1] = A10*(-K1) + A11 + KF_R_ALTITUDE*K1*K1;

    } else {
        // No baro update — carry forward predict
        x[0] = x0_pred;
        x[1] = x1_pred;
        P[0][0] = Pp00; P[0][1] = Pp01;
        P[1][0] = Pp10; P[1][1] = Pp11;
    }

    // ── Post-update sanity ────────────────────────────────────────────────────
    _sanitiseP();

    if (_nanCheck(x[0], "altitude")) return _last;
    if (_nanCheck(x[1], "velocity")) return _last;

    // Hard clamp velocity to physical limits (10K ft rocket max ~400 m/s)
    x[1] = constrain(x[1], -500.0f, 500.0f);

    _last = {x[0], x[1]};
    return _last;
}

void kf_zero_velocity()
{
    if (!_initialised) return;

    x[1] = 0.0f;          // actually reset internal velocity state
    P[1][0] = 0.0f;
    P[0][1] = 0.0f;
    P[1][1] = 1.0f;      // low velocity uncertainty while stationary

    _last.velocity_ms = 0.0f;
}

// ─────────────────────────────────────────────────────────────────────────────

KfState kf_get_state()
{
    return _last;
}

// ---------------------------------------------------------------------------
// Runtime tuning API
// ---------------------------------------------------------------------------

void kf_set_tuning(float q_alt, float q_vel, float r_alt)
{
    if (q_alt >= 0.0f) KF_Q_ALTITUDE = q_alt;
    if (q_vel >= 0.0f) KF_Q_VELOCITY = q_vel;
    if (r_alt >= 0.0f) KF_R_ALTITUDE = r_alt;
    Serial.printf("[KF] Tuning set -> Q_alt=%.6f Q_vel=%.6f R_alt=%.6f\n",
                  KF_Q_ALTITUDE, KF_Q_VELOCITY, KF_R_ALTITUDE);
}

void kf_set_phase(KfFlightPhase phase)
{
    switch (phase) {
        case KF_PHASE_PAD:
            kf_set_tuning(KF_TUNE_PAD_Q_ALT, KF_TUNE_PAD_Q_VEL, KF_TUNE_PAD_R_ALT);
            break;
        case KF_PHASE_LIFTOFF:
            kf_set_tuning(KF_TUNE_LIFTOFF_Q_ALT, KF_TUNE_LIFTOFF_Q_VEL, KF_TUNE_LIFTOFF_R_ALT);
            break;
        case KF_PHASE_BOOST:
            kf_set_tuning(KF_TUNE_BOOST_Q_ALT, KF_TUNE_BOOST_Q_VEL, KF_TUNE_BOOST_R_ALT);
            break;
        case KF_PHASE_COAST:
            kf_set_tuning(KF_TUNE_COAST_Q_ALT, KF_TUNE_COAST_Q_VEL, KF_TUNE_COAST_R_ALT);
            break;
        case KF_PHASE_APOGEE:
            kf_set_tuning(KF_TUNE_APOGEE_Q_ALT, KF_TUNE_APOGEE_Q_VEL, KF_TUNE_APOGEE_R_ALT);
            break;
        case KF_PHASE_DESCENT:
            kf_set_tuning(KF_TUNE_DESCENT_Q_ALT, KF_TUNE_DESCENT_Q_VEL, KF_TUNE_DESCENT_R_ALT);
            break;
        case KF_PHASE_LANDING:
            kf_set_tuning(KF_TUNE_LANDING_Q_ALT, KF_TUNE_LANDING_Q_VEL, KF_TUNE_LANDING_R_ALT);
            break;
        default:
            Serial.println("[KF] Unknown phase in kf_set_phase()");
            break;
    }
}