#include "control/AltitudePitchController.h"

#include <AP_Math.h>
#include <math.h>

namespace fc {

namespace {
constexpr float kRadToDeg = 57.29578f;
constexpr float kDegToRadLocal = 0.01745329f;
constexpr float kMinAirspeedForFfMps = 5.0f;
}  // namespace

AltitudePitchController::AltitudePitchController(const AltitudePitchControllerConfig& config)
    : config_(config)
{
    pitch_max_eff_deg_ = config_.pitch_max_deg;
}

void AltitudePitchController::reset(float current_pitch_deg)
{
    // Bumpless transfer: start the demand from the attitude the aircraft is
    // actually flying, so mode entry doesn't step the elevator.
    pitch_dem_deg_ = constrain_float(current_pitch_deg, config_.pitch_min_deg, config_.pitch_max_deg);
    integrator_deg_ = 0.0f;
    altitude_error_m_ = 0.0f;
    climb_rate_dem_mps_ = 0.0f;
    underspeed_ = false;
    pitch_max_eff_deg_ = config_.pitch_max_deg;
    initialized_ = false;  // first update() seeds the climb-rate filter
}

float AltitudePitchController::update(const Input& in)
{
    // Ignore bad dt (first loop, scheduler hiccup) instead of integrating garbage.
    if (!(in.dt_s > 0.0f) || in.dt_s > 0.1f || isnan(in.altitude_m) || isnan(in.target_altitude_m)) {
        return pitch_dem_deg_;
    }
    const float dt = in.dt_s;

    // ---- Measured climb-rate filter ----
    const float climb_meas = (isnan(in.climb_rate_mps) || isinf(in.climb_rate_mps)) ? 0.0f : in.climb_rate_mps;
    if (!initialized_) {
        climb_rate_lpf_mps_ = climb_meas;
        initialized_ = true;
    } else if (config_.climb_rate_lpf_hz > 0.0f) {
        const float rc = 1.0f / (2.0f * M_PI * config_.climb_rate_lpf_hz);
        const float alpha = dt / (dt + rc);
        climb_rate_lpf_mps_ += alpha * (climb_meas - climb_rate_lpf_mps_);
    } else {
        climb_rate_lpf_mps_ = climb_meas;
    }

    // ---- Outer loop: altitude -> climb-rate demand ----
    altitude_error_m_ = in.target_altitude_m - in.altitude_m;
    climb_rate_dem_mps_ = constrain_float(config_.altitude_kp * altitude_error_m_,
                                          -config_.max_sink_rate_mps, config_.max_climb_rate_mps);

    // ---- Effective pitch-up limit (underspeed protection) ----
    pitch_max_eff_deg_ = config_.pitch_max_deg;
    underspeed_ = false;
    if (in.airspeed_valid && config_.underspeed_margin_mps > 0.0f) {
        const float low = config_.min_airspeed_mps;
        const float high = config_.min_airspeed_mps + config_.underspeed_margin_mps;
        if (in.airspeed_mps < high) {
            const float frac = constrain_float((in.airspeed_mps - low) / (high - low), 0.0f, 1.0f);
            pitch_max_eff_deg_ = MAX(0.0f, config_.pitch_max_deg) * frac;
            underspeed_ = in.airspeed_mps <= low;
        }
    }
    pitch_max_eff_deg_ = MAX(pitch_max_eff_deg_, config_.pitch_min_deg);

    // ---- Inner loop: climb-rate error -> pitch ----
    const float rate_error = climb_rate_dem_mps_ - climb_rate_lpf_mps_;

    // Feedforward flight-path angle for the demanded climb rate.
    const float v = in.airspeed_valid ? MAX(in.airspeed_mps, kMinAirspeedForFfMps)
                                      : MAX(config_.cruise_airspeed_mps, kMinAirspeedForFfMps);
    const float gamma_deg =
        asinf(constrain_float(climb_rate_dem_mps_ / v, -0.5f, 0.5f)) * kRadToDeg;
    const float ff_deg = config_.climb_feedforward_gain * gamma_deg;

    // Bank compensation: extra nose-up to recover lift lost in a turn.
    const float roll_abs_rad =
        MIN(fabsf(in.roll_deg), config_.max_roll_for_compensation_deg) * kDegToRadLocal;
    const float cos_roll = MAX(cosf(roll_abs_rad), 0.1f);
    const float roll_cmp_deg = config_.roll_compensation_deg * (1.0f / cos_roll - 1.0f);

    const float p_deg = config_.rate_kp_deg_per_mps * rate_error;
    const float base_deg = config_.pitch_trim_deg + ff_deg + p_deg + roll_cmp_deg;

    // Integrator with conditional anti-windup: don't integrate further into
    // a limit the output is already pinned against.
    const float unclamped_before = base_deg + integrator_deg_;
    const bool sat_high = unclamped_before >= pitch_max_eff_deg_;
    const bool sat_low = unclamped_before <= config_.pitch_min_deg;
    const float i_step = config_.rate_ki_deg_per_m * rate_error * dt;
    if (!((sat_high && i_step > 0.0f) || (sat_low && i_step < 0.0f))) {
        integrator_deg_ += i_step;
    }
    integrator_deg_ = constrain_float(integrator_deg_, -config_.integrator_limit_deg,
                                      config_.integrator_limit_deg);

    float pitch_cmd = constrain_float(base_deg + integrator_deg_, config_.pitch_min_deg, pitch_max_eff_deg_);

    // ---- Slew limit (smooth elevator, no step on altitude-target change) ----
    if (config_.pitch_slew_rate_dps > 0.0f) {
        const float max_step = config_.pitch_slew_rate_dps * dt;
        pitch_cmd = constrain_float(pitch_cmd, pitch_dem_deg_ - max_step, pitch_dem_deg_ + max_step);
    }

    if (isnan(pitch_cmd) || isinf(pitch_cmd)) {
        pitch_cmd = config_.pitch_trim_deg;
        integrator_deg_ = 0.0f;
    }

    pitch_dem_deg_ = pitch_cmd;
    return pitch_dem_deg_;
}

}  // namespace fc
