#pragma once

#include <stdint.h>

namespace fc {

struct AltitudePitchControllerConfig {
    // Runtime A/B toggle persisted through Params (ALT_CTRL_EN):
    // 1 = this controller drives nav_pitch_deg in AUTO/LOITER,
    // 0 = legacy behaviour (nav_pitch_deg from Tecs::pitchDemandDeg()).
    float enabled = 1.0f;

    // ---- Outer loop: altitude error -> climb-rate demand ----
    float altitude_kp = 0.3f;              // ALT_KP, (m/s) per m of altitude error
    float max_climb_rate_mps = 3.0f;       // ALT_CLMB_MAX
    float max_sink_rate_mps = 2.5f;        // ALT_SINK_MAX

    // ---- Inner loop: climb-rate error -> pitch demand ----
    float rate_kp_deg_per_mps = 3.0f;      // ALT_RATE_KP, deg per (m/s) of climb-rate error
    float rate_ki_deg_per_m = 0.5f;        // ALT_RATE_KI, deg per m of integrated climb-rate error
    float integrator_limit_deg = 5.0f;

    // Feedforward: flight-path angle needed for the demanded climb rate,
    // asin(climb_rate_dem / airspeed), scaled by this gain (0 = off).
    float climb_feedforward_gain = 1.0f;   // ALT_PTCH_FF

    // Extra nose-up to make up the lift lost in a bank:
    // roll_compensation_deg * (1/cos(roll) - 1). Matters in LOITER (35 deg bank).
    float roll_compensation_deg = 10.0f;   // ALT_ROLL_CMP

    // Level-flight pitch needed at cruise (airframe trim), added to every demand.
    float pitch_trim_deg = 0.0f;           // ALT_PTCH_TRIM

    // ---- Output shaping ----
    float pitch_min_deg = -5.0f;           // copied from NavigationConfig PITCH_MIN
    float pitch_max_deg = 14.0f;           // copied from NavigationConfig PITCH_MAX
    float pitch_slew_rate_dps = 10.0f;     // pitch-demand rate limit, deg/s
    float climb_rate_lpf_hz = 2.0f;        // low-pass on measured climb rate (0 = off)
    float max_roll_for_compensation_deg = 60.0f;

    // ---- Underspeed protection (throttle is manual in AUTO/LOITER) ----
    // With a valid airspeed sensor, the nose-up limit is blended down from
    // pitch_max_deg (at min_airspeed + margin) to 0 deg (at min_airspeed),
    // so the altitude loop cannot stall the aircraft trying to climb.
    float min_airspeed_mps = 14.0f;        // copied from TecsConfig MIN_AIRSPEED
    float cruise_airspeed_mps = 18.0f;     // copied from TecsConfig AIRSPEED_CRUISE
    float underspeed_margin_mps = 2.0f;
};

/**
 * Altitude-hold pitch controller for AUTO/LOITER, independent of TECS.
 *
 * TECS is kept only for auto-throttle; this class decides nav_pitch_deg
 * on its own, so altitude is held even when throttle stays on the RC stick.
 *
 *   climb_rate_dem = clamp(Kp_alt * (h_target - h), -sink_max, +climb_max)
 *   e_rate         = climb_rate_dem - climb_rate_lpf
 *   I             += Ki_rate * e_rate * dt              (anti-windup, clamped)
 *   pitch_dem      = trim + FF*asin(climb_rate_dem / V) + Kp_rate*e_rate + I
 *                    + roll_cmp * (1/cos(roll) - 1)
 *   pitch_dem      = slew(clamp(pitch_dem, pitch_min, pitch_max_eff))
 *
 * Units: altitude m (positive up, same frame as BarometerData::altitude_m
 * and MissionState::target_altitude), climb rate m/s positive up, pitch in
 * the nav_pitch_deg convention AttitudeController uses (positive = nose up,
 * i.e. -ImuData::pitch_deg).
 *
 * Pure computation (no I/O, no globals), same pattern as Tecs/L1Controller.
 */
class AltitudePitchController final {
public:
    struct Input {
        float target_altitude_m = 0.0f;
        float altitude_m = 0.0f;
        float climb_rate_mps = 0.0f;
        float airspeed_mps = 0.0f;
        bool airspeed_valid = false;
        float roll_deg = 0.0f;
        float dt_s = 0.0f;
    };

    explicit AltitudePitchController(const AltitudePitchControllerConfig& config = AltitudePitchControllerConfig{});

    /** Bumpless start: call on mode entry with the current nav-convention pitch. */
    void reset(float current_pitch_deg);

    /** Returns the pitch demand in degrees (nav_pitch_deg convention). */
    float update(const Input& input);

    bool enabled() const { return config_.enabled >= 0.5f; }

    float pitchDemandDeg() const { return pitch_dem_deg_; }
    float altitudeErrorM() const { return altitude_error_m_; }
    float climbRateDemandMps() const { return climb_rate_dem_mps_; }
    float climbRateFilteredMps() const { return climb_rate_lpf_mps_; }
    float integratorDeg() const { return integrator_deg_; }
    float pitchMaxEffectiveDeg() const { return pitch_max_eff_deg_; }
    bool underspeed() const { return underspeed_; }

private:
    AltitudePitchControllerConfig config_{};

    bool initialized_ = false;
    float pitch_dem_deg_ = 0.0f;
    float altitude_error_m_ = 0.0f;
    float climb_rate_dem_mps_ = 0.0f;
    float climb_rate_lpf_mps_ = 0.0f;
    float integrator_deg_ = 0.0f;
    float pitch_max_eff_deg_ = 0.0f;
    bool underspeed_ = false;
};

}  // namespace fc
