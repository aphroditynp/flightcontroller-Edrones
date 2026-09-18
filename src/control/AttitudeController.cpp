#include "control/AttitudeController.h"

#include <AP_Math.h>
#include <math.h>

namespace fc {

namespace {
// Same conversion used by the refactored Actuator and the TD legacy servo
// offsets: 400 raw units == 35.156 deg surface deflection.
constexpr float kRawUnitsPerSurfaceDeg = 11.378f;
constexpr float kTdRawServoLimit = 400.0f;
}

AttitudeController::AttitudeController(const AttitudeControllerConfig& config)
    : config_(config)
{
}

float AttitudeController::computeSpeedScaler(float airspeed_mps) const
{
    const float safe_airspeed = MAX(airspeed_mps, config_.min_airspeed_for_scaling_mps);
    const float ratio = config_.trim_airspeed_mps / safe_airspeed;
    return constrain_float(ratio * ratio, config_.scaler_min, config_.scaler_max);
}

float AttitudeController::clampToOutputLimit(float value_deg, float limit_deg)
{
    return constrain_float(value_deg, -limit_deg, limit_deg);
}

AttitudeController::Output AttitudeController::update(float nav_roll_deg, float nav_pitch_deg,
                                                       const ImuData& imu, float airspeed_mps,
                                                       float dt_s)
{
    // Reproduce TD's integral state definition. TD's validated Trainer2 gains
    // have Ki=0, so this is dormant unless intentionally enabled/tuned later.
    if (integral_enabled_ && dt_s > 0.0f && dt_s < 0.1f) {
        roll_integral_ += config_.roll.ki * imu.roll_deg * dt_s;
        pitch_integral_ += config_.pitch.ki * imu.pitch_deg * dt_s;
        roll_integral_ = constrain_float(roll_integral_,
                                         -config_.roll.integral_limit_deg,
                                         config_.roll.integral_limit_deg);
        pitch_integral_ = constrain_float(pitch_integral_,
                                          -config_.pitch.integral_limit_deg,
                                          config_.pitch.integral_limit_deg);
    }

    // Current repo's ImuData axes have already been bench-corrected so x is
    // roll-rate and y is pitch-rate. TD passed legacy gyro_y/gyro_x because
    // its old BNO055 wrapper used a different raw-axis convention. We port the
    // physical controller law, not that obsolete sensor-variable swap.
    float roll_raw =
        config_.roll.kp * (imu.roll_deg - nav_roll_deg - roll_integral_) -
        config_.roll.kd * imu.angular_rate_dps.x;

    float pitch_raw =
        config_.pitch.kp * (nav_pitch_deg - imu.pitch_deg + pitch_integral_) -
        config_.pitch.kd * imu.angular_rate_dps.y;

    // TD clamps raw servo offsets before writing 1500 +/- command.
    roll_raw = constrain_float(roll_raw, -kTdRawServoLimit, kTdRawServoLimit);
    pitch_raw = constrain_float(pitch_raw, -kTdRawServoLimit, kTdRawServoLimit);

    const float computed_scaler = computeSpeedScaler(airspeed_mps);
    last_speed_scaler_ = computed_scaler;
    const float scaler = (config_.speed_scaler_enabled != 0.0f) ? computed_scaler : 1.0f;

    Output output{};
    output.aileron_deg = clampToOutputLimit(
        (roll_raw / kRawUnitsPerSurfaceDeg) * scaler,
        config_.roll.output_limit_deg);
    output.elevator_deg = clampToOutputLimit(
        (pitch_raw / kRawUnitsPerSurfaceDeg) * scaler,
        config_.pitch.output_limit_deg);
    output.rudder_deg = 0.0f;  // TD active FW path: u_yaw = 0
    return output;
}

void AttitudeController::resetIntegrators()
{
    roll_integral_ = 0.0f;
    pitch_integral_ = 0.0f;
}

void AttitudeController::setIntegralEnabled(bool enabled)
{
    integral_enabled_ = enabled;
    if (!enabled) {
        resetIntegrators();
    }
}

float AttitudeController::rollIntegratorState() const
{
    return roll_integral_;
}

float AttitudeController::pitchIntegratorState() const
{
    return pitch_integral_;
}

float AttitudeController::lastSpeedScaler() const
{
    return last_speed_scaler_;
}

float AttitudeController::lastYawRateSetpointDps() const
{
    return 0.0f;
}

}  // namespace fc
