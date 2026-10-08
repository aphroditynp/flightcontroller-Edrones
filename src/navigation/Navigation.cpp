#include "navigation/Navigation.h"

#include <math.h>

namespace {
constexpr float kGravityMss = 9.80665f;
}

namespace fc {

Navigation::Navigation(const NavigationConfig& config)
    : config_(config)
{
    mission_.target_airspeed_mps = config_.airspeed_cruise_mps;
}

void Navigation::setWaypointReachedCallback(WaypointReachedCallback callback, void* user_data)
{
    waypoint_reached_callback_ = callback;
    waypoint_reached_user_data_ = user_data;
}

MissionState& Navigation::state()
{
    return mission_;
}

const MissionState& Navigation::state() const
{
    return mission_;
}

void Navigation::calcGroundspeedUndershoot(const AhrsData& ahrs, bool have_gps)
{
    if (have_gps) {
        const float gnd_spd_fwd = ahrs.groundspeed_mps;
        groundspeed_undershoot_mps_ =
            (config_.min_groundspeed_mps > 0.0f) ? (config_.min_groundspeed_mps - gnd_spd_fwd) : 0.0f;
    } else {
        groundspeed_undershoot_mps_ = 0.0f;
    }
}

float Navigation::groundspeedUndershoot() const
{
    return groundspeed_undershoot_mps_;
}

void Navigation::updateHomeAndPosition(Ahrs& ahrs, const GnssFixData& gnss,
                                       const BarometerData& baro, bool armed, uint32_t now_ms)
{
    if (!armed && now_ms - last_home_set_ms_ >= config_.home_set_interval_ms &&
        baro.altitude_m <= config_.home_altitude_settle_m) {
        last_home_set_ms_ = now_ms;
        ahrs.setHome(gnss, baro);  // Ahrs::setHome() itself no-ops unless gnss.fix_type is Fix3D.
    }

    const AhrsData& ahrs_data = ahrs.data();
    mission_.current_loc = ahrs_data.position;
    mission_.relative_alt_m = -(static_cast<float>(ahrs_data.position.alt) / 100.0f);
    ahrs.updateWindspeed();

    calcGroundspeedUndershoot(ahrs_data, ahrs_data.gps_lock);
}

void Navigation::updateWaypointNav(L1Controller& l1, const AhrsData& ahrs, const ImuData& imu)
{
    if (mission_.auto_state.crosstrack) {
        l1.updateWaypoint(ahrs, imu, mission_.prev_wp_loc, mission_.next_wp_loc, 0.0f);
    } else {
        l1.updateWaypoint(ahrs, imu, mission_.current_loc, mission_.next_wp_loc, 0.0f);
    }
}

float Navigation::getNextGroundCourse(float default_angle_deg) const
{
    if (mission_.flag_wp < 0 || mission_.flag_wp + 1 >= mission_.wp_sum) {
        return default_angle_deg;
    }
    const Locations& next_wp_after = mission_.waypoint[mission_.flag_wp + 1];
    if (next_wp_after.lat == 0 && next_wp_after.lng == 0) {
        return default_angle_deg;
    }
    return mission_.next_wp_loc.get_bearing_to(next_wp_after) / 100.0f;
}

void Navigation::setupTurnAngle()
{
    const float next_ground_course = getNextGroundCourse(-1.0f);
    if (next_ground_course == -1.0f) {
        mission_.auto_state.next_turn_angle_deg = 90.0f;
    } else {
        const float ground_course = mission_.prev_wp_loc.get_bearing_to(mission_.next_wp_loc) / 100.0f;
        mission_.auto_state.next_turn_angle_deg = wrap_180(next_ground_course - ground_course);
    }
}

void Navigation::setNextWaypoint(const Locations& loc)
{
    if (mission_.auto_state.next_wp_crosstrack) {
        mission_.prev_wp_loc = mission_.next_wp_loc;
        mission_.auto_state.crosstrack = true;
    } else {
        mission_.prev_wp_loc = mission_.current_loc;
        mission_.auto_state.crosstrack = false;
        mission_.auto_state.next_wp_crosstrack = true;
    }
    mission_.next_wp_loc = loc;
    setupTurnAngle();
}

void Navigation::updateAltitude(Tecs& tecs, const AhrsData& ahrs, const ImuData& imu,
                                const BarometerData& baro, const AirspeedData& airspeed,
                                int16_t throttle_stick_percent, bool enable_throttle_nudge)
{
    const int16_t throttle_nudge = enable_throttle_nudge
                                       ? static_cast<int16_t>(0.5f * throttle_stick_percent)
                                       : 0;

    if (mission_.auto_throttle_mode) {
        constexpr float kAerodynamicLoadFactor = 1.0f;  // load-factor compensation is dormant in legacy too
        tecs.updatePitchThrottle(static_cast<int32_t>(mission_.target_altitude.amsl_cm),
                                 mission_.target_airspeed_mps, throttle_nudge,
                                 mission_.relative_alt_m, kAerodynamicLoadFactor, ahrs, baro, imu,
                                 airspeed);
    }
}

void Navigation::updateSpeedHeight(Tecs& tecs, const AhrsData& ahrs, const BarometerData& baro,
                                   const ImuData& imu, const AirspeedData& airspeed)
{
    if (mission_.auto_throttle_mode) {
        tecs.update50Hz(ahrs, baro, imu, airspeed);
    }
}

float Navigation::getAdaptiveWaypointAirspeed(float dist_to_wp, float acceptance_dist,
                                              float next_turn_angle_deg,
                                              float current_roll_deg) const
{
    float target_spd = config_.airspeed_cruise_mps;

    if (fabsf(next_turn_angle_deg) > config_.adaptive_turn_angle_thresh_deg) {
        const float max_bank_rad = radians(config_.roll_limit_deg);
        const float optimal_turn_spd = sqrtf(config_.wp_radius_default_m * kGravityMss * tanf(max_bank_rad));
        const float safe_turn_spd =
            constrain_float(optimal_turn_spd, config_.airspeed_min_mps, config_.airspeed_cruise_mps);

        const float approach_zone = acceptance_dist * config_.adaptive_approach_mult;
        if (approach_zone > 0.1f && dist_to_wp <= approach_zone) {
            const float blend = constrain_float(dist_to_wp / approach_zone, 0.0f, 1.0f);
            target_spd = safe_turn_spd + blend * (config_.airspeed_cruise_mps - safe_turn_spd);
        }
    }

    if (fabsf(current_roll_deg) > config_.adaptive_wings_level_deg) {
        target_spd = constrain_float(target_spd, config_.airspeed_min_mps, target_spd);
    }

    if (isnan(target_spd) || isinf(target_spd) || target_spd < config_.airspeed_min_mps) {
        target_spd = config_.airspeed_cruise_mps;
    }
    return target_spd;
}

void Navigation::updateAutoAttitudeTargets(L1Controller& l1, Tecs& tecs, const ImuData& imu)
{
    // L1's positive lateral-acceleration demand follows the navigation-frame
    // convention, while the fixed-wing attitude loop uses the opposite bank
    // sign (the same convention already compensated in FBWA). Convert once at
    // this navigation-to-attitude boundary so AUTO, GUIDED, and LOITER all
    // command the same physical turn direction.
    mission_.nav_roll_deg =
        constrain_float(-(l1.navRollCd(imu) / 100.0f),
                        -config_.roll_limit_deg, config_.roll_limit_deg);
    mission_.nav_pitch_deg =
        constrain_float(tecs.pitchDemandDeg(), config_.pitch_limit_min_deg, config_.pitch_limit_max_deg);
}

void Navigation::navigate(bool is_guided_mode, L1Controller& l1, const AhrsData& ahrs,
                          const ImuData& imu, float eas2tas)
{
    if (mission_.rpi_external_setpoint_active) {
        if (millis() - mission_.rpi_external_setpoint_last_ms > config_.rpi_setpoint_timeout_ms) {
            mission_.rpi_external_setpoint_active = false;
        } else {
            mission_.prev_wp_loc = mission_.current_loc;
            mission_.next_wp_loc = mission_.rpi_external_setpoint_target;
            mission_.auto_state.next_wp_crosstrack = false;
            mission_.target_altitude.amsl_cm = static_cast<float>(mission_.rpi_external_setpoint_target.alt);
            mission_.auto_navigation_mode = true;
            updateWaypointNav(l1, ahrs, imu);
            return;
        }
    }

    if (is_guided_mode) {
        mission_.prev_wp_loc = mission_.current_loc;
        mission_.next_wp_loc = mission_.current_loc;
        mission_.auto_state.next_wp_crosstrack = false;
        mission_.target_altitude.amsl_cm = static_cast<float>(mission_.current_loc.alt);
        mission_.auto_state.distance_next_wp = 0.0f;
        mission_.auto_state.bearing_deg = 0.0f;
        mission_.auto_state.wp_proportion = 0.0f;
        updateWaypointNav(l1, ahrs, imu);
        return;
    }

    if (mission_.auto_navigation_mode) {
        if (mission_.flag_wp < 0) {
            mission_.flag_wp++;
            mission_.auto_state.next_wp_crosstrack = false;
            mission_.target_altitude.amsl_cm = static_cast<float>(mission_.waypoint[mission_.flag_wp].alt);
            setNextWaypoint(mission_.waypoint[mission_.flag_wp]);
        }
    }
    // NOTE: legacy's auto_takeoff_mode branch here is intentionally not
    // ported yet — see docs/navigation.md.

    mission_.auto_state.distance_next_wp = mission_.current_loc.get_distance(mission_.next_wp_loc);
    mission_.auto_state.bearing_deg = mission_.current_loc.get_bearing_to(mission_.next_wp_loc) / 100.0f;
    mission_.auto_state.wp_proportion =
        mission_.current_loc.line_path_proportion(mission_.prev_wp_loc, mission_.next_wp_loc);

    if (mission_.auto_navigation_mode) {
        mission_.acceptance_distance_m =
            l1.turnDistance(config_.wp_radius_default_m, mission_.auto_state.next_turn_angle_deg, eas2tas);

        mission_.target_airspeed_mps = getAdaptiveWaypointAirspeed(
            mission_.auto_state.distance_next_wp, mission_.acceptance_distance_m,
            mission_.auto_state.next_turn_angle_deg, imu.roll_deg);
    }

    const bool hit_radius = mission_.auto_state.distance_next_wp <= mission_.acceptance_distance_m;
    const bool finish_line =
        mission_.current_loc.past_interval_finish_line(mission_.prev_wp_loc, mission_.next_wp_loc);

    if (hit_radius || finish_line) {
        if (mission_.auto_navigation_mode) {
            if (waypoint_reached_callback_ != nullptr) {
                waypoint_reached_callback_(static_cast<uint16_t>(mission_.flag_wp), waypoint_reached_user_data_);
            }

            mission_.flag_wp++;
            if (mission_.flag_wp >= mission_.wp_sum) {
                mission_.flag_wp = -1;
                mission_.auto_navigation_mode = false;
                // Caller (Mode layer, Phase 6) is responsible for switching
                // out of AUTO when auto_navigation_mode goes false here —
                // legacy set a Radio.h global (`mode_fbwa = true`) directly,
                // which this module doesn't have access to.
            } else {
                mission_.auto_state.next_wp_crosstrack = true;
                mission_.target_altitude.amsl_cm = static_cast<float>(mission_.waypoint[mission_.flag_wp].alt);
                setNextWaypoint(mission_.waypoint[mission_.flag_wp]);
            }
        }
    }

    updateWaypointNav(l1, ahrs, imu);
}

}  // namespace fc
