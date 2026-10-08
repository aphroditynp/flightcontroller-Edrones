#include "modes/FixedWingModes.h"

#include <AP_Math.h>

#include "FC_Config.h"
#include "vehicle/Actuator.h"

namespace fc {

void ModeManual::_update()
{
    ctx_.ahrs.update(ctx_.imu.data(), ctx_.gnss, ctx_.baro.data(), ctx_.airspeed.data());
    ctx_.navigation.updateHomeAndPosition(ctx_.ahrs, ctx_.gnss, ctx_.baro.data(), ctx_.radio.armed(),
                                          ctx_.now_ms);

    ctx_.actuator.writeManual(ctx_.radio.channelRoll(), ctx_.radio.channelPitch(), ctx_.radio.channelYaw());
    ctx_.actuator.writeThrottleManual(ctx_.radio.channelThrottle(), false, ctx_.radio.armed());
    ctx_.actuator.updatePayload(ctx_.radio.armed(), ctx_.payload_drop_command,
                               ctx_.radio.channelVehicleMode() > 1500);
}

bool ModeFbwa::_enter()
{
    ctx_.attitude.resetIntegrators();
    return true;
}

float ModeFbwa::mapStickToDeg(uint16_t channel_pwm, float max_deg)
{
    const long centered = constrain(static_cast<long>(channel_pwm) - 1500L, -512L, 512L);
    // Use Arduino map() integer quantization exactly like TD/V10-trainer2.
    return static_cast<float>(map(centered, -512L, 512L,
                                  -static_cast<long>(max_deg),
                                  static_cast<long>(max_deg)));
}

float ModeFbwa::mapStickToRange(uint16_t channel_pwm, float min_deg, float max_deg)
{
    const long centered = constrain(static_cast<long>(channel_pwm) - 1500L, -512L, 512L);
    return static_cast<float>(map(centered, -512L, 512L,
                                  static_cast<long>(min_deg),
                                  static_cast<long>(max_deg)));
}

void ModeFbwa::_update()
{
    ctx_.ahrs.update(ctx_.imu.data(), ctx_.gnss, ctx_.baro.data(), ctx_.airspeed.data());
    ctx_.navigation.updateHomeAndPosition(ctx_.ahrs, ctx_.gnss, ctx_.baro.data(), ctx_.radio.armed(), ctx_.now_ms);

    // Exact TD/V10-trainer2 stick mapping. TD's internal roll_cmd is opposite
    // the desired bank convention used by the unified controller, hence the
    // leading minus sign here. The final aileron PWM reversal lives in Actuator.
    const float td_roll_cmd_deg = 1.3f * mapStickToDeg(ctx_.radio.channelRoll(), 35.0f);
    const float roll_cmd_deg = -td_roll_cmd_deg;

    // TD maps pitch stick to [-25,+35] deg then multiplies by 1.1. This means
    // center stick has a +5.5 deg pitch command; preserved intentionally.
    const float pitch_cmd_deg =
        1.1f * mapStickToRange(ctx_.radio.channelPitch(), -25.0f, 35.0f);

    const AttitudeController::Output output =
        ctx_.attitude.update(roll_cmd_deg, pitch_cmd_deg, ctx_.controlImu(),
                             ctx_.airspeed.data().velocity_mps, ctx_.dt_s);

    ctx_.actuator.writeAttitude(output);
    ctx_.actuator.writeThrottleManual(ctx_.radio.channelThrottle(), true, ctx_.radio.armed());
    ctx_.actuator.updatePayload(ctx_.radio.armed(), ctx_.payload_drop_command,
                               ctx_.radio.channelVehicleMode() > 1500);
}

bool ModeAuto::_enter()
{
    MissionState& mission = ctx_.navigation.state();
    mission.auto_navigation_mode = true;
    mission.auto_throttle_mode = false;
    mission.auto_loiter_mode = false;
    ctx_.attitude.resetIntegrators();
    return true;
}

void ModeAuto::_update()
{
    ctx_.ahrs.update(ctx_.imu.data(), ctx_.gnss, ctx_.baro.data(), ctx_.airspeed.data());
    ctx_.navigation.updateHomeAndPosition(ctx_.ahrs, ctx_.gnss, ctx_.baro.data(), ctx_.radio.armed(),
                                          ctx_.now_ms);

    const AhrsData& ahrs_data = ctx_.ahrs.data();
    const ImuData& imu_data = ctx_.imu.data();
    const float eas2tas = ctx_.baro.data().eas2tas;

    ctx_.navigation.navigate(/*is_guided_mode=*/false, ctx_.l1, ahrs_data, imu_data, eas2tas);

    // Optionally hold the final mission waypoint in LOITER. Navigation leaves
    // next_wp_loc pointing at that final waypoint when it marks AUTO complete.
#if FC_AUTO_LOITER_ENABLE
    MissionState& mission = ctx_.navigation.state();
    if (mission.wp_sum > 0 && !mission.auto_navigation_mode && mission.flag_wp < 0) {
        mission.loiter_center_loc = mission.next_wp_loc;
        mission.loiter_target_alt_cm = mission.target_altitude.amsl_cm;
        mission.loiter_custom_target_set = true;
        if (ctx_.mode_manager.setMode(ModeId::Loiter)) {
            return;
        }
    }
#else
    MissionState& mission = ctx_.navigation.state();
#endif

    ctx_.navigation.updateSpeedHeight(ctx_.tecs, ahrs_data, ctx_.baro.data(), imu_data, ctx_.airspeed.data());

    const float throttle_stick_percent = Actuator::scaleToPercent(ctx_.radio.channelThrottle());
    ctx_.navigation.updateAltitude(ctx_.tecs, ahrs_data, imu_data, ctx_.baro.data(), ctx_.airspeed.data(),
                                   static_cast<int16_t>(throttle_stick_percent), ctx_.enableThrottleNudge);

    ctx_.navigation.updateAutoAttitudeTargets(ctx_.l1, ctx_.tecs, imu_data);

    // Thesis contribution: fuzzy adapts L1 period. L1 internally derives its
    // guidance gains from period, so changing period changes Kx/Kv online.
    ctx_.fuzzy_tuner.update(ctx_.l1, ctx_.l1.crosstrackError(), ctx_.dt_s);

    const AttitudeController::Output output =
        ctx_.attitude.update(mission.nav_roll_deg, mission.nav_pitch_deg, ctx_.controlImu(),
                             ctx_.airspeed.data().velocity_mps, ctx_.dt_s);
    ctx_.actuator.writeAttitude(output);
    ctx_.actuator.updatePayload(ctx_.radio.armed(), ctx_.payload_drop_command,
                               ctx_.radio.channelVehicleMode() > 1500);

    // AUTO navigation is autonomous, but throttle remains under direct RC
    // control so the pilot can set propulsion independently of TECS.
    ctx_.actuator.writeThrottleManual(ctx_.radio.channelThrottle(), true, ctx_.radio.armed());
}

void ModeAuto::_exit()
{
    ctx_.navigation.state().auto_navigation_mode = false;
    ctx_.navigation.state().auto_throttle_mode = false;
}

bool ModeGuided::_enter()
{
    ctx_.navigation.state().auto_navigation_mode = true;
    ctx_.navigation.state().auto_throttle_mode = true;
    ctx_.attitude.resetIntegrators();
    return true;
}

void ModeGuided::_update()
{
    ctx_.ahrs.update(ctx_.imu.data(), ctx_.gnss, ctx_.baro.data(), ctx_.airspeed.data());
    ctx_.navigation.updateHomeAndPosition(ctx_.ahrs, ctx_.gnss, ctx_.baro.data(), ctx_.radio.armed(),
                                          ctx_.now_ms);

    const AhrsData& ahrs_data = ctx_.ahrs.data();
    const ImuData& imu_data = ctx_.imu.data();
    const float eas2tas = ctx_.baro.data().eas2tas;

    ctx_.navigation.navigate(/*is_guided_mode=*/true, ctx_.l1, ahrs_data, imu_data, eas2tas);
    ctx_.navigation.updateSpeedHeight(ctx_.tecs, ahrs_data, ctx_.baro.data(), imu_data, ctx_.airspeed.data());

    const float throttle_stick_percent = Actuator::scaleToPercent(ctx_.radio.channelThrottle());
    ctx_.navigation.updateAltitude(ctx_.tecs, ahrs_data, imu_data, ctx_.baro.data(), ctx_.airspeed.data(),
                                   static_cast<int16_t>(throttle_stick_percent), ctx_.enableThrottleNudge);

    ctx_.navigation.updateAutoAttitudeTargets(ctx_.l1, ctx_.tecs, imu_data);
    ctx_.fuzzy_tuner.update(ctx_.l1, ctx_.l1.crosstrackError(), ctx_.dt_s);

    const MissionState& mission = ctx_.navigation.state();
    const AttitudeController::Output output =
        ctx_.attitude.update(mission.nav_roll_deg, mission.nav_pitch_deg, ctx_.controlImu(),
                             ctx_.airspeed.data().velocity_mps, ctx_.dt_s);
    ctx_.actuator.writeAttitude(output);
    ctx_.actuator.updatePayload(ctx_.radio.armed(), ctx_.payload_drop_command,
                               ctx_.radio.channelVehicleMode() > 1500);

    ctx_.actuator.writeThrottleManual(ctx_.radio.channelThrottle(), true, ctx_.radio.armed());
}

void ModeGuided::_exit()
{
    ctx_.navigation.state().auto_navigation_mode = false;
    ctx_.navigation.state().auto_throttle_mode = false;
}

bool ModeLoiter::_enter()
{
    MissionState& mission = ctx_.navigation.state();
    mission.auto_navigation_mode = false;
    mission.auto_throttle_mode = false;
    mission.auto_loiter_mode = true;
    ctx_.attitude.resetIntegrators();

    if (mission.loiter_custom_target_set) {
        mission.loiter_custom_target_set = false;
    } else {
        // Explicit LOITER command: capture the current aircraft location/altitude.
        if (ctx_.ahrs.data().valid) {
            mission.loiter_center_loc = ctx_.ahrs.data().position;
            mission.loiter_target_alt_cm = static_cast<float>(ctx_.ahrs.data().position.alt);
        } else {
            mission.loiter_center_loc = mission.current_loc;
            mission.loiter_target_alt_cm = static_cast<float>(mission.current_loc.alt);
        }
    }

    if (mission.loiter_target_alt_cm == 0.0f) {
        mission.loiter_target_alt_cm = static_cast<float>(mission.loiter_center_loc.alt);
    }
    mission.target_altitude.amsl_cm = mission.loiter_target_alt_cm;
    return true;
}

void ModeLoiter::_update()
{
    ctx_.ahrs.update(ctx_.imu.data(), ctx_.gnss, ctx_.baro.data(), ctx_.airspeed.data());
    ctx_.navigation.updateHomeAndPosition(ctx_.ahrs, ctx_.gnss, ctx_.baro.data(), ctx_.radio.armed(),
                                          ctx_.now_ms);

    const AhrsData& ahrs_data = ctx_.ahrs.data();
    const ImuData& imu_data = ctx_.imu.data();
    MissionState& mission = ctx_.navigation.state();

    if (!ahrs_data.valid) {
        return;
    }

    const float radius_m = MAX(fabsf(mission.loiter_radius_m), 20.0f);
    const int8_t direction = (mission.loiter_direction >= 0) ? 1 : -1;

    // Same ArduPilot-derived circular L1 controller as TD's update_loiter().
    ctx_.l1.updateLoiter(ahrs_data, imu_data, mission.loiter_center_loc,
                         radius_m, direction, ctx_.baro.data().eas2tas,
                         mission.target_airspeed_mps);

    // Thesis-specific difference from TD: fuzzy changes L1 period online.
    // updateLoiter() computes omega=2*pi/T, Kx=omega^2 and Kv=2*zeta*omega;
    // therefore period adaptation is gain adaptation without rewriting L1.
    ctx_.fuzzy_tuner.update(ctx_.l1, ctx_.l1.crosstrackError(), ctx_.dt_s);

    ctx_.navigation.updateSpeedHeight(ctx_.tecs, ahrs_data, ctx_.baro.data(), imu_data,
                                      ctx_.airspeed.data());

    const float throttle_stick_percent = Actuator::scaleToPercent(ctx_.radio.channelThrottle());
    ctx_.navigation.updateAltitude(ctx_.tecs, ahrs_data, imu_data, ctx_.baro.data(),
                                   ctx_.airspeed.data(),
                                   static_cast<int16_t>(throttle_stick_percent),
                                   ctx_.enableThrottleNudge);

    ctx_.navigation.updateAutoAttitudeTargets(ctx_.l1, ctx_.tecs, imu_data);
    mission.nav_roll_deg = constrain_float(mission.nav_roll_deg,
                                           -mission.loiter_bank_limit_deg,
                                           mission.loiter_bank_limit_deg);

    const AttitudeController::Output output =
        ctx_.attitude.update(mission.nav_roll_deg, mission.nav_pitch_deg, ctx_.controlImu(),
                             ctx_.airspeed.data().velocity_mps, ctx_.dt_s);
    ctx_.actuator.writeAttitude(output);
    ctx_.actuator.updatePayload(ctx_.radio.armed(), ctx_.payload_drop_command,
                               ctx_.radio.channelVehicleMode() > 1500);

    // LOITER holds the bank/position autonomously, while propulsion remains
    // directly controlled by the RC throttle stick.
    ctx_.actuator.writeThrottleManual(ctx_.radio.channelThrottle(), true, ctx_.radio.armed());
}

void ModeLoiter::_exit()
{
    MissionState& mission = ctx_.navigation.state();
    mission.auto_loiter_mode = false;
    mission.auto_throttle_mode = false;
}

}  // namespace fc
