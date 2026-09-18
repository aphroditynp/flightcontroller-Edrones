#pragma once

#include "drivers/Imu.h"

namespace fc {

/**
 * TD / V10-trainer2 axis gains.
 *
 * These gains operate in the same legacy raw servo-offset units used by
 * KHAGESWARA's FW_control.h. The final raw command is converted to surface
 * degrees before it reaches Actuator::writeAttitude().
 */
struct TdAxisConfig {
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
    float integral_limit_deg = 15.0f;
    float output_limit_deg = 35.156f;  // +/-400 raw units / 11.378 raw-units-per-degree
};

struct AttitudeControllerConfig {
    // Flight-proven TD/V10-trainer2 baseline.
    TdAxisConfig roll{10.0f, 0.0f, 0.0f, 15.0f, 35.156f};
    TdAxisConfig pitch{10.0f, 0.0f, 0.20f, 15.0f, 35.156f};

    // Compatibility/experiment hook. TD itself does not use this V^-2 output
    // multiplier, so the default remains OFF. Keep it OFF when reproducing TD.
    float trim_airspeed_mps = 18.0f;
    float scaler_min = 0.6f;
    float scaler_max = 1.8f;
    float min_airspeed_for_scaling_mps = 3.0f;
    float speed_scaler_enabled = 0.0f;
};

/**
 * Fixed-wing inner-loop attitude controller ported from TD/V10-trainer2.
 *
 * Unified desired-attitude convention used by this refactor:
 *   roll desired  = nav_roll_deg
 *   pitch desired = nav_pitch_deg
 *
 * The equations reproduce the active TD FW_control.h path:
 *   u_roll_raw  = Kp_r * (roll - roll_des - I_r) - Kd_r * roll_rate
 *   u_pitch_raw = Kp_p * (pitch_des - pitch + I_p) - Kd_p * pitch_rate
 *
 * TD's final aileron PWM direction is reversed (1500 - u_roll). That reversal
 * is kept in Actuator::writeAttitude(), not hidden inside this controller.
 * Rudder is neutral, matching the active TD fixed-wing path (u_yaw = 0).
 *
 * The legacy TD integrator is also reproduced for completeness:
 *   I_r += Ki_r * roll * dt
 *   I_p += Ki_p * pitch * dt
 * but the flight-proven TD defaults use Ki = 0 for both axes.
 */
class AttitudeController final {
public:
    struct Output {
        float aileron_deg = 0.0f;
        float elevator_deg = 0.0f;
        float rudder_deg = 0.0f;
    };

    explicit AttitudeController(const AttitudeControllerConfig& config = AttitudeControllerConfig{});

    Output update(float nav_roll_deg, float nav_pitch_deg, const ImuData& imu,
                  float airspeed_mps, float dt_s);

    void resetIntegrators();
    void setIntegralEnabled(bool enabled);

    float rollIntegratorState() const;
    float pitchIntegratorState() const;
    float lastSpeedScaler() const;
    float lastYawRateSetpointDps() const;

private:
    float computeSpeedScaler(float airspeed_mps) const;
    static float clampToOutputLimit(float value_deg, float limit_deg);

    AttitudeControllerConfig config_{};
    float roll_integral_ = 0.0f;
    float pitch_integral_ = 0.0f;
    bool integral_enabled_ = true;

    float last_speed_scaler_ = 1.0f;
};

}  // namespace fc
