#pragma once

#include <AP_Math.h>
#include <Locations.h>

#include "control/AltitudePitchController.h"
#include "estimation/Ahrs.h"
#include "navigation/L1Controller.h"
#include "navigation/MissionState.h"
#include "navigation/Tecs.h"

namespace fc {

struct NavigationConfig {
    float wp_radius_default_m = 40.0f;
    float min_groundspeed_mps = 17.0f;  // 0 disables the undershoot check

    float roll_limit_deg = 36.0f;   // HEAD_MAX
    float pitch_limit_min_deg = -5.0f;   // PITCH_MIN
    float pitch_limit_max_deg = 14.0f;   // PITCH_MAX
    float airspeed_min_mps = 14.0f;
    float airspeed_cruise_mps = 18.0f;

    float adaptive_turn_angle_thresh_deg = 20.0f;
    float adaptive_approach_mult = 2.0f;
    float adaptive_wings_level_deg = 10.0f;

    // Home is (re)latched while disarmed, at least this many ms apart, once
    // barometer altitude settles at/under this many meters (matches legacy
    // `baro.altitude <= 10`).
    uint32_t home_set_interval_ms = 5000;
    float home_altitude_settle_m = 10.0f;

    // RPi external-setpoint bridge: a setpoint older than this is dropped.
    uint32_t rpi_setpoint_timeout_ms = 1000;
};

/**
 * Callback invoked when a waypoint is reached, so Navigation doesn't need to
 * know about MAVLink (ported alongside Mavlink.h in Phase 7, which will
 * register a callback that sends MISSION_ITEM_REACHED).
 */
using WaypointReachedCallback = void (*)(uint16_t seq, void* user_data);

/**
 * FW auto-navigation orchestration (legacy Navigation.h), consolidating
 * Auto_setup.h's loose globals into one owned MissionState and driving
 * L1Controller/Tecs instead of reaching into their globals.
 *
 * Takeoff sequencing (legacy auto_takeoff_mode/auto_takeoff_state) is not
 * ported here yet — see docs/navigation.md; it lands with Mode/Actuator
 * (Phases 5/6), which it's tightly coupled to.
 */
class Navigation final {
public:
    explicit Navigation(const NavigationConfig& config = NavigationConfig{});

    void setWaypointReachedCallback(WaypointReachedCallback callback, void* user_data);

    MissionState& state();
    const MissionState& state() const;

    /** Latch home (once, while disarmed and settled) and refresh position/windspeed. */
    void updateHomeAndPosition(Ahrs& ahrs, const GnssFixData& gnss, const BarometerData& baro,
                               bool armed, uint32_t now_ms);

    void updateWaypointNav(L1Controller& l1, const AhrsData& ahrs, const ImuData& imu);

    void updateAltitude(Tecs& tecs, const AhrsData& ahrs, const ImuData& imu,
                        const BarometerData& baro, const AirspeedData& airspeed,
                        int16_t throttle_stick_percent, bool enable_throttle_nudge);
    void updateSpeedHeight(Tecs& tecs, const AhrsData& ahrs, const BarometerData& baro,
                           const ImuData& imu, const AirspeedData& airspeed);

    float getAdaptiveWaypointAirspeed(float dist_to_wp, float acceptance_dist,
                                      float next_turn_angle_deg, float current_roll_deg) const;

    /** Writes MissionState::nav_roll_deg/nav_pitch_deg from L1/TECS demand — the target the
     * Phase-4 LQR attitude controller tracks. */
    void updateAutoAttitudeTargets(L1Controller& l1, Tecs& tecs, const ImuData& imu);

    /**
     * Overwrites MissionState::nav_pitch_deg with the TECS-independent
     * altitude-hold pitch loop (AUTO/LOITER). Call AFTER
     * updateAutoAttitudeTargets(). Altitude = AhrsData::position.alt (baro,
     * positive up), target = MissionState::target_altitude.amsl_cm.
     */
    void updateAltitudePitchTarget(AltitudePitchController& altitude_pitch, const AhrsData& ahrs,
                                   const BarometerData& baro, const AirspeedData& airspeed,
                                   const ImuData& imu, float dt_s);

    /** Main AUTO/GUIDED navigation state machine. */
    void navigate(bool is_guided_mode, L1Controller& l1, const AhrsData& ahrs, const ImuData& imu,
                 float eas2tas);

    void setNextWaypoint(const Locations& loc);
    float getNextGroundCourse(float default_angle_deg) const;

    float groundspeedUndershoot() const;

private:
    void setupTurnAngle();
    void calcGroundspeedUndershoot(const AhrsData& ahrs, bool have_gps);

    NavigationConfig config_{};
    MissionState mission_{};

    WaypointReachedCallback waypoint_reached_callback_ = nullptr;
    void* waypoint_reached_user_data_ = nullptr;

    uint32_t last_home_set_ms_ = 0;
    float groundspeed_undershoot_mps_ = 0.0f;
};

}  // namespace fc
