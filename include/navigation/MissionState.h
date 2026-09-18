#pragma once

#include <Locations.h>
#include <stdint.h>

namespace fc {

/**
 * Consolidated auto-flight/mission state, replacing the loose file-scope
 * globals in legacy Auto_setup.h (auto_navigation_mode, flag_wp, waypoint[],
 * auto_state, target_altitude, rpi_external_setpoint_*, etc.) with one
 * struct owned by Navigation.
 *
 * Takeoff state (legacy `auto_takeoff_state`) is intentionally NOT included
 * yet — takeoff.h is tightly coupled to Actuator/Mode, neither of which is
 * ported yet (Phases 5/6). It will be added alongside that port.
 */
struct MissionState {
    static constexpr int kMaxWaypoints = 100;

    bool auto_navigation_mode = false;
    bool auto_throttle_mode = false;
    bool auto_loiter_mode = false;

    float target_airspeed_mps = 18.0f;
    float relative_alt_m = 0.0f;

    int flag_wp = -1;
    int wp_sum = 0;
    Locations waypoint[kMaxWaypoints]{};

    Locations prev_wp_loc{};
    Locations current_loc{};
    Locations next_wp_loc{};
    Locations home_wp_loc{};
    bool home_set = false;

    float acceptance_distance_m = 0.0f;

    struct AutoState {
        bool next_wp_crosstrack = false;
        bool crosstrack = false;
        float distance_next_wp = 0.0f;
        float next_wp_altitude = 0.0f;
        float bearing_deg = 0.0f;
        float wp_proportion = 0.0f;
        float next_turn_angle_deg = 0.0f;
    } auto_state{};

    struct TargetAltitude {
        float amsl_cm = 0.0f;
    } target_altitude{};

    // Companion-computer (RPi) guided-setpoint bridge, fed by MAVLink
    // SET_POSITION_TARGET_* (ported alongside Mavlink.h in Phase 7).
    bool rpi_external_setpoint_active = false;
    uint32_t rpi_external_setpoint_last_ms = 0;
    Locations rpi_external_setpoint_target{};

    // Attitude targets for the LQR attitude controller (Phase 4), written by
    // Navigation::updateAutoAttitudeTargets() — replaces legacy
    // FW_ControlModes.h's global nav_roll_deg/nav_pitch_deg.
    float nav_roll_deg = 0.0f;
    float nav_pitch_deg = 0.0f;

    // TD-style fixed-wing LOITER runtime target. Positive radius/direction = CW.
    // AUTO sets loiter_custom_target_set when the final mission waypoint is
    // reached; explicit LOITER entered from GCS captures the current position.
    Locations loiter_center_loc{};
    float loiter_target_alt_cm = 0.0f;
    bool loiter_custom_target_set = false;
    float loiter_radius_m = 50.0f;
    int8_t loiter_direction = 1;
    float loiter_bank_limit_deg = 35.0f;
};

}  // namespace fc
