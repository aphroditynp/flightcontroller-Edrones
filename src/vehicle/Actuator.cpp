#include "vehicle/Actuator.h"

namespace fc {

Actuator::Actuator(const ActuatorConfig& config)
    : config_(config)
{
}

void Actuator::begin()
{
    pinMode(config_.pin_aileron_left, OUTPUT);
    pinMode(config_.pin_elevator, OUTPUT);
    pinMode(config_.pin_rudder_left, OUTPUT);
    pinMode(config_.pin_aileron_right, OUTPUT);
    pinMode(config_.pin_rudder_right, OUTPUT);
    pinMode(config_.pin_throttle, OUTPUT);
    pinMode(config_.pin_payload, OUTPUT);

    aileron_left_.attach(config_.pin_aileron_left);
    elevator_.attach(config_.pin_elevator);
    rudder_left_.attach(config_.pin_rudder_left);
    aileron_right_.attach(config_.pin_aileron_right);
    rudder_right_.attach(config_.pin_rudder_right);
    payload_.attach(config_.pin_payload);
    throttle_.attach(config_.pin_throttle);

    throttle_.writeMicroseconds(config_.pwm_safe_throttle);
    payload_.writeMicroseconds(config_.pwm_payload_hold);
}

uint16_t Actuator::angleToPwm(float angle_deg, float gain, uint16_t center)
{
    // Sum in float FIRST, cast once at the end. The previous
    // "static_cast<uint16_t>(gain * angle_deg) + center" cast the signed
    // (often negative) intermediate to uint16_t before adding center --
    // casting a negative float directly to an unsigned type is undefined
    // behavior in C++, and on this ARM/Teensy toolchain it saturates to 0,
    // silently dropping every negative-direction correction to "no
    // correction" instead of "center - offset". Bench-diagnosed 2026-08-18
    // as the cause of FBWA feeling choppy/unresponsive in one direction.
    const float pwm = gain * angle_deg + static_cast<float>(center);
    return static_cast<uint16_t>(pwm);
}

float Actuator::scaleToPercent(uint16_t pwm)
{
    return 0.0976f * static_cast<float>(pwm) - 96.067f;
}

uint16_t Actuator::percentToPwm(float percent)
{
    return static_cast<uint16_t>(map(static_cast<long>(percent), 0, 100, 988, 2012));
}

void Actuator::writeManual(uint16_t ch_roll, uint16_t ch_pitch, uint16_t ch_yaw)
{
    rudder_left_.writeMicroseconds(ch_yaw);
    rudder_right_.writeMicroseconds(ch_yaw);
    // TD/V10-trainer2 manual output: aileron PWM is reversed around 1500.
    const uint16_t reversed_roll = config_.pwm_min + config_.pwm_max - ch_roll;
    aileron_left_.writeMicroseconds(reversed_roll);
    aileron_right_.writeMicroseconds(reversed_roll);
    elevator_.writeMicroseconds(ch_pitch);
}

void Actuator::writeAttitude(const AttitudeController::Output& attitude)
{
    // TD fw_servos_out_fbwa(): pwm_ail = 1500 - u_roll.
    const uint16_t pwm_ail =
        angleToPwm(-attitude.aileron_deg, config_.angle_to_pwm_gain, config_.angle_to_pwm_center);
    const uint16_t pwm_ele =
        angleToPwm(attitude.elevator_deg, config_.angle_to_pwm_gain, config_.angle_to_pwm_center);
    const uint16_t pwm_rud =
        angleToPwm(attitude.rudder_deg, config_.angle_to_pwm_gain, config_.angle_to_pwm_center);

    aileron_left_.writeMicroseconds(pwm_ail);
    aileron_right_.writeMicroseconds(pwm_ail);
    elevator_.writeMicroseconds(pwm_ele);
    rudder_left_.writeMicroseconds(pwm_rud);
    rudder_right_.writeMicroseconds(pwm_rud);
}

void Actuator::writeThrottleManual(uint16_t ch_throttle, bool is_fbwa_plane, bool armed)
{
    uint16_t motor_pwm = ch_throttle;
    if (is_fbwa_plane) {
        motor_pwm = constrain(motor_pwm, config_.min_throttle_pwm, config_.max_throttle_pwm);
    }
    throttle_.writeMicroseconds(armed ? motor_pwm : config_.pwm_safe_throttle);
}

void Actuator::writeThrottleAuto(uint16_t throttle_pwm, bool armed)
{
    throttle_.writeMicroseconds(armed ? throttle_pwm : config_.pwm_safe_throttle);
}

void Actuator::updatePayload(bool armed, bool& payload_drop_command, bool manual_drop_switch_active)
{
    const uint32_t now_ms = millis();
    uint16_t payload_pwm = config_.pwm_payload_hold;

    if (!armed) {
        payload_drop_command = false;
        payload_dropping_ = false;
        payload_drop_start_ms_ = 0;
        payload_pwm = config_.pwm_payload_hold;
    } else if (payload_drop_command) {
        if (!payload_dropping_) {
            payload_dropping_ = true;
            payload_drop_start_ms_ = now_ms;
        }
        payload_pwm = config_.pwm_payload_drop;

        if (now_ms - payload_drop_start_ms_ >= config_.payload_drop_duration_ms) {
            payload_drop_command = false;
            payload_dropping_ = false;
        }
    } else if (manual_drop_switch_active) {
        payload_pwm = config_.pwm_payload_drop;
        payload_dropping_ = false;
    } else {
        payload_pwm = config_.pwm_payload_hold;
        payload_dropping_ = false;
    }

    payload_pwm = constrain(payload_pwm, config_.pwm_min, config_.pwm_max);
    payload_.writeMicroseconds(payload_pwm);
}

uint16_t Actuator::aileronLeftPwm() const { return static_cast<uint16_t>(aileron_left_.readMicroseconds()); }
uint16_t Actuator::aileronRightPwm() const { return static_cast<uint16_t>(aileron_right_.readMicroseconds()); }
uint16_t Actuator::elevatorPwm() const { return static_cast<uint16_t>(elevator_.readMicroseconds()); }
uint16_t Actuator::rudderLeftPwm() const { return static_cast<uint16_t>(rudder_left_.readMicroseconds()); }
uint16_t Actuator::rudderRightPwm() const { return static_cast<uint16_t>(rudder_right_.readMicroseconds()); }
uint16_t Actuator::throttlePwm() const { return static_cast<uint16_t>(throttle_.readMicroseconds()); }
uint16_t Actuator::payloadPwm() const { return static_cast<uint16_t>(payload_.readMicroseconds()); }

void Actuator::writeSafe()
{
    throttle_.writeMicroseconds(config_.pwm_safe_throttle);
    payload_.writeMicroseconds(config_.pwm_payload_hold);
}

}  // namespace fc
