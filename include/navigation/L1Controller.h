#pragma once

#include <AP_Math.h>
#include <Locations.h>
#include <vector2.h>

#include "drivers/Imu.h"
#include "estimation/Ahrs.h"

namespace fc {

struct L1ControllerConfig {
    float period_s = 20.0f;      // ArduPilot default is 20; see docs/l1-controller.md
    float damping = 0.73f;
    float xtrack_integrator_gain = 0.2f;
    float loiter_bank_limit_deg = 45.0f;
};

/**
 * L1 lateral guidance controller (ArduPilot's AP_L1_Control, ported to a
 * class with injected sensor snapshots instead of file-scope globals and
 * reaching into other modules' singletons).
 *
 * `_L1_period` (period()/setPeriod()) is the parameter FuzzyL1Tuner adjusts
 * at runtime for the thesis's self-tuning comparison — everything else here
 * is the unmodified ArduPilot algorithm.
 */
class L1Controller final {
public:
    explicit L1Controller(const L1ControllerConfig& config = L1ControllerConfig{});

    // Tuning accessors. period() is the fuzzy self-tuner's write target.
    void setPeriod(float period_s);
    float period() const;
    void setDamping(float damping);
    float damping() const;
    void setXtrackIntegratorGain(float gain);
    float xtrackIntegratorGain() const;
    void setLoiterBankLimitDeg(float limit_deg);

    void setReverse(bool reverse);

    /** Line-following guidance update. Returns false if the AHRS position is invalid. */
    bool updateWaypoint(const AhrsData& ahrs, const ImuData& imu, const Locations& prev_wp,
                        const Locations& next_wp, float dist_min);

    /** Circular-loiter guidance update. Returns false if the AHRS position is invalid. */
    bool updateLoiter(const AhrsData& ahrs, const ImuData& imu, const Locations& center_wp,
                      float radius, int8_t loiter_direction, float eas2tas,
                      float target_airspeed_mps);

    void updateHeadingHold(const AhrsData& ahrs, const ImuData& imu, int32_t navigation_heading_cd);
    void updateLevelFlight(const ImuData& imu);

    /** Bank-angle demand in centidegrees, from the last update*() call. */
    int32_t navRollCd(const ImuData& imu) const;
    float lateralAcceleration() const;
    int32_t navBearingCd() const;
    int32_t bearingErrorCd() const;
    int32_t targetBearingCd() const;
    float crosstrackError() const;
    float groundspeedVectorAngle() const;

    float turnDistance(float wp_radius, float eas2tas) const;
    float turnDistance(float wp_radius, float turn_angle, float eas2tas) const;
    float loiterRadius(float radius, float eas2tas, float target_airspeed_mps) const;
    bool reachedLoiterTarget() const;

    bool dataIsStale() const;
    void setDataIsStale();

private:
    float getYaw(const ImuData& imu) const;
    int32_t getYawSensor(const ImuData& imu) const;
    void preventIndecision(float& nu, const ImuData& imu);

    L1ControllerConfig config_{};
    bool reverse_ = false;

    float xtrack_integrator_gain_prev_ = 0.0f;
    uint32_t last_update_waypoint_us_ = 0;
    bool data_is_stale_ = true;

    float lat_accel_demand_ = 0.0f;
    float l1_distance_ = 0.0f;
    bool wp_circle_ = false;
    float nav_bearing_rad_ = 0.0f;
    float bearing_error_rad_ = 0.0f;
    float crosstrack_error_ = 0.0f;
    int32_t target_bearing_cd_ = 0;
    float last_nu_ = 0.0f;
    float xtrack_integrator_ = 0.0f;
    float groundspeed_vector_angle_ = 0.0f;
};

}  // namespace fc
