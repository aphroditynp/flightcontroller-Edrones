#include "navigation/L1Controller.h"

#include <definitions.h>
#include <math.h>

namespace fc {

L1Controller::L1Controller(const L1ControllerConfig& config)
    : config_(config)
{
}

void L1Controller::setPeriod(float period_s)
{
    config_.period_s = period_s;
}

float L1Controller::period() const
{
    return config_.period_s;
}

const L1ControllerConfig& L1Controller::config() const
{
    return config_;
}

void L1Controller::setDamping(float damping)
{
    config_.damping = damping;
}

float L1Controller::damping() const
{
    return config_.damping;
}

void L1Controller::setXtrackIntegratorGain(float gain)
{
    config_.xtrack_integrator_gain = gain;
}

float L1Controller::xtrackIntegratorGain() const
{
    return config_.xtrack_integrator_gain;
}

void L1Controller::setLoiterBankLimitDeg(float limit_deg)
{
    config_.loiter_bank_limit_deg = limit_deg;
}

void L1Controller::setReverse(bool reverse)
{
    reverse_ = reverse;
}

float L1Controller::getYaw(const ImuData& imu) const
{
    if (reverse_) {
        return wrap_PI(M_PI + radians(imu.yaw_deg));
    }
    return imu.yaw_rad;
}

int32_t L1Controller::getYawSensor(const ImuData& imu) const
{
    if (reverse_) {
        return wrap_180_cd(18000 + static_cast<int32_t>(imu.yaw_deg * 100.0f));
    }
    return static_cast<int32_t>(imu.yaw_deg * 100.0f);
}

int32_t L1Controller::navRollCd(const ImuData& imu) const
{
    // liftForce * cos(roll) = gravityForce * cos(pitch);
    // liftForce * sin(roll) = gravityForce * lateralAcceleration / gravityAcceleration;
    // See ArduPilot issue 24319 / PR 24331.
    const float pitch_limit = radians(60.0f);
    const float pitch = constrain_float(imu.pitch_rad, -pitch_limit, pitch_limit);
    float ret = degrees(atanf(lat_accel_demand_ * (1.0f / (GRAVITY_MSS * cosf(pitch))))) * 100.0f;
    return static_cast<int32_t>(constrain_float(ret, -9000.0f, 9000.0f));
}

float L1Controller::lateralAcceleration() const
{
    return lat_accel_demand_;
}

int32_t L1Controller::navBearingCd() const
{
    return wrap_180_cd(static_cast<int32_t>(RadiansToCentiDegrees(nav_bearing_rad_)));
}

int32_t L1Controller::bearingErrorCd() const
{
    return static_cast<int32_t>(RadiansToCentiDegrees(bearing_error_rad_));
}

int32_t L1Controller::targetBearingCd() const
{
    return wrap_180_cd(target_bearing_cd_);
}

float L1Controller::crosstrackError() const
{
    return crosstrack_error_;
}

float L1Controller::groundspeedVectorAngle() const
{
    return groundspeed_vector_angle_;
}

float L1Controller::turnDistance(float wp_radius, float eas2tas) const
{
    wp_radius *= sq(eas2tas);
    return MIN(wp_radius, l1_distance_);
}

float L1Controller::turnDistance(float wp_radius, float turn_angle, float eas2tas) const
{
    const float distance_90 = turnDistance(wp_radius, eas2tas);
    turn_angle = fabsf(turn_angle);
    if (turn_angle >= 90.0f) {
        return distance_90;
    }
    return distance_90 * turn_angle / 90.0f;
}

float L1Controller::loiterRadius(float radius, float eas2tas, float target_airspeed_mps) const
{
    const float sanitized_bank_limit = constrain_float(config_.loiter_bank_limit_deg, 0.0f, 89.0f);
    const float lateral_accel_sea_level = tanf(radians(sanitized_bank_limit)) * GRAVITY_MSS;
    const float nominal_velocity_sea_level = target_airspeed_mps;
    const float eas2tas_sq = sq(eas2tas);

    if (is_zero(sanitized_bank_limit) || is_zero(nominal_velocity_sea_level) ||
        is_zero(lateral_accel_sea_level)) {
        return radius * eas2tas_sq;
    }

    const float sea_level_radius = sq(nominal_velocity_sea_level) / lateral_accel_sea_level;
    if (sea_level_radius > radius) {
        return radius * eas2tas_sq;
    }
    return MAX(sea_level_radius * eas2tas_sq, radius);
}

bool L1Controller::reachedLoiterTarget() const
{
    return wp_circle_;
}

bool L1Controller::dataIsStale() const
{
    return data_is_stale_;
}

void L1Controller::setDataIsStale()
{
    data_is_stale_ = true;
}

void L1Controller::preventIndecision(float& nu, const ImuData& imu)
{
    const float nu_limit = 0.9f * M_PI;
    if (fabsf(nu) > nu_limit && fabsf(last_nu_) > nu_limit &&
        labs(wrap_180_cd(target_bearing_cd_ - getYawSensor(imu))) > 12000 &&
        nu * last_nu_ < 0.0f) {
        nu = last_nu_;
    }
}

bool L1Controller::updateWaypoint(const AhrsData& ahrs, const ImuData& imu,
                                  const Locations& prev_wp, const Locations& next_wp,
                                  float dist_min)
{
    if (!ahrs.valid) {
        data_is_stale_ = true;
        return false;
    }

    const Locations current_loc = ahrs.position;

    const uint32_t now = micros();
    float dt = (now - last_update_waypoint_us_) * 1.0e-6f;
    if (dt > 1.0f) {
        xtrack_integrator_ = 0.0f;
    }
    if (dt > 0.1f) {
        dt = 0.1f;
    }
    last_update_waypoint_us_ = now;

    const float k_l1 = 4.0f * config_.damping * config_.damping;

    Vector2f groundspeed_vector(ahrs.velocity_ned_mps.x, ahrs.velocity_ned_mps.y);

    target_bearing_cd_ = current_loc.get_bearing_to(next_wp);

    float ground_speed = ahrs.groundspeed_mps;
    groundspeed_vector_angle_ = groundspeed_vector.angle();

    // Use a heading-derived ground speed vector (matches legacy — the
    // "moving_forwards"/low-speed branch this fed into was already
    // unconditional in the legacy code, kept as-is here).
    groundspeed_vector = Vector2f(cosf(getYaw(imu)), sinf(getYaw(imu))) * ground_speed;

    l1_distance_ = MAX(0.3183099f * config_.damping * config_.period_s * ground_speed, dist_min);

    Vector2f ab = prev_wp.get_distance_NE(next_wp);
    const float ab_length = ab.length();

    if (ab.length() < 1.0e-6f) {
        ab = current_loc.get_distance_NE(next_wp);
        if (ab.length() < 1.0e-6f) {
            ab = Vector2f(cosf(getYaw(imu)), sinf(getYaw(imu)));
        }
    }
    ab.normalize();

    const Vector2f a_air = prev_wp.get_distance_NE(current_loc);
    crosstrack_error_ = a_air % ab;

    const float wp_a_dist = a_air.length();
    const float along_track_dist = a_air * ab;

    float nu;
    float xtrack_vel;
    float ltrack_vel;

    if (wp_a_dist > l1_distance_ && along_track_dist / MAX(wp_a_dist, 1.0f) < -0.7071f) {
        const Vector2f a_air_unit = a_air.normalized();
        xtrack_vel = groundspeed_vector % (-a_air_unit);
        ltrack_vel = groundspeed_vector * (-a_air_unit);
        nu = atan2f(xtrack_vel, ltrack_vel);
        nav_bearing_rad_ = atan2f(-a_air_unit.y, -a_air_unit.x);
    } else if (along_track_dist > ab_length + ground_speed * 3.0f) {
        const Vector2f b_air = next_wp.get_distance_NE(current_loc);
        const Vector2f b_air_unit = b_air.normalized();
        xtrack_vel = groundspeed_vector % (-b_air_unit);
        ltrack_vel = groundspeed_vector * (-b_air_unit);
        nu = atan2f(xtrack_vel, ltrack_vel);
        nav_bearing_rad_ = atan2f(-b_air_unit.y, -b_air_unit.x);
    } else {
        xtrack_vel = groundspeed_vector % ab;
        ltrack_vel = groundspeed_vector * ab;
        const float nu2 = atan2f(xtrack_vel, ltrack_vel);

        float sine_nu1 = crosstrack_error_ / MAX(l1_distance_, 0.1f);
        sine_nu1 = constrain_float(sine_nu1, -0.7071f, 0.7071f);
        float nu1 = asinf(sine_nu1);

        if (config_.xtrack_integrator_gain <= 0.0f ||
            !is_equal(config_.xtrack_integrator_gain, xtrack_integrator_gain_prev_)) {
            xtrack_integrator_ = 0.0f;
            xtrack_integrator_gain_prev_ = config_.xtrack_integrator_gain;
        } else if (fabsf(nu1) < radians(5.0f)) {
            xtrack_integrator_ += nu1 * config_.xtrack_integrator_gain * dt;
            xtrack_integrator_ = constrain_float(xtrack_integrator_, -0.1f, 0.1f);
        }

        nu1 += xtrack_integrator_;
        nu = nu1 + nu2;
        nav_bearing_rad_ = wrap_PI(atan2f(ab.y, ab.x) + nu1);
    }

    preventIndecision(nu, imu);
    last_nu_ = nu;

    nu = constrain_float(nu, -1.5708f, 1.5708f);
    lat_accel_demand_ = k_l1 * ground_speed * ground_speed / l1_distance_ * sinf(nu);

    wp_circle_ = false;
    bearing_error_rad_ = nu;
    data_is_stale_ = false;
    return true;
}

bool L1Controller::updateLoiter(const AhrsData& ahrs, const ImuData& imu,
                                const Locations& center_wp, float radius,
                                int8_t loiter_direction, float eas2tas,
                                float target_airspeed_mps)
{
    if (!ahrs.valid) {
        data_is_stale_ = true;
        return false;
    }

    float L1Controller::loiterLqrBankDemand(const AhrsData& ahrs, const Locations& center_wp,
                                            float radius, int8_t loiter_direction,
                                            float eas2tas, float target_airspeed_mps)
    {
        if (!ahrs.valid) {
            return 0.0f;
        }
        radius = loiterRadius(fabsf(radius), eas2tas, target_airspeed_mps);
        const Vector2f distance_ne = center_wp.get_distance_NE(ahrs.position);
        const float distance = MAX(distance_ne.length(), 0.1f);
        const Vector2f radial(distance_ne.x / distance, distance_ne.y / distance);
        const Vector2f velocity(ahrs.velocity_ned_mps.x, ahrs.velocity_ned_mps.y);
        const float radial_error = distance - radius;
        loiter_target_radius_m_ = radius;
        crosstrack_error_ = radial_error;
        const float radial_velocity = velocity * radial;
        const float tangent_velocity = radial.x * velocity.y - radial.y * velocity.x;

        // LQR state feedback for [radial error, radial velocity], plus the
        // centripetal acceleration required to remain on the selected circle.
        constexpr float k_error = 0.55f;
        constexpr float k_error_rate = 1.35f;
        const float omega = MAX(target_airspeed_mps, 1.0f) / MAX(radius, 1.0f);
        const float acceleration =
            radial_error * k_error * omega * omega +
            radial_velocity * k_error_rate * omega +
            static_cast<float>(loiter_direction) *
                tangent_velocity * tangent_velocity / MAX(radius, 1.0f);
        return atanf(acceleration / 9.80665f);
    }

    const Locations current_loc = ahrs.position;

    radius = loiterRadius(fabsf(radius), eas2tas, target_airspeed_mps);
    loiter_target_radius_m_ = radius;

    const float omega = 6.2832f / config_.period_s;
    const float kx = omega * omega;
    const float kv = 2.0f * config_.damping * omega;
    const float k_l1 = 4.0f * config_.damping * config_.damping;

    Vector2f groundspeed_vector(ahrs.velocity_ned_mps.x, ahrs.velocity_ned_mps.y);
    const float ground_speed = MAX(groundspeed_vector.length(), 1.0f);

    target_bearing_cd_ = current_loc.get_bearing_to(center_wp);

    l1_distance_ = 0.3183099f * config_.damping * config_.period_s * ground_speed;

    const Vector2f a_air = center_wp.get_distance_NE(current_loc);

    Vector2f a_air_unit;
    if (a_air.length() > 0.1f) {
        a_air_unit = a_air.normalized();
    } else if (groundspeed_vector.length() < 0.1f) {
        a_air_unit = Vector2f(cosf(getYaw(imu)), sinf(getYaw(imu)));
    } else {
        a_air_unit = groundspeed_vector.normalized();
    }

    const float xtrack_vel_cap = a_air_unit % groundspeed_vector;
    const float ltrack_vel_cap = -(groundspeed_vector * a_air_unit);
    float nu = atan2f(xtrack_vel_cap, ltrack_vel_cap);

    preventIndecision(nu, imu);
    last_nu_ = nu;
    nu = constrain_float(nu, -M_PI_2, M_PI_2);

    const float lat_accel_dem_cap = k_l1 * ground_speed * ground_speed / l1_distance_ * sinf(nu);

    const float xtrack_vel_circ = -ltrack_vel_cap;
    const float xtrack_err_circ = a_air.length() - radius;
    crosstrack_error_ = xtrack_err_circ;

    float lat_accel_dem_circ_pd = xtrack_err_circ * kx + xtrack_vel_circ * kv;

    const float vel_tangent = xtrack_vel_cap * static_cast<float>(loiter_direction);
    if (ltrack_vel_cap < 0.0f && vel_tangent < 0.0f) {
        lat_accel_dem_circ_pd = MAX(lat_accel_dem_circ_pd, 0.0f);
    }

    const float lat_accel_dem_circ_ctr =
        vel_tangent * vel_tangent / MAX(0.5f * radius, radius + xtrack_err_circ);
    const float lat_accel_dem_circ =
        static_cast<float>(loiter_direction) * (lat_accel_dem_circ_pd + lat_accel_dem_circ_ctr);

    if (xtrack_err_circ > 0.0f &&
        loiter_direction * lat_accel_dem_cap < loiter_direction * lat_accel_dem_circ) {
        lat_accel_demand_ = lat_accel_dem_cap;
        wp_circle_ = false;
        bearing_error_rad_ = nu;
        nav_bearing_rad_ = atan2f(-a_air_unit.y, -a_air_unit.x);
    } else {
        lat_accel_demand_ = lat_accel_dem_circ;
        wp_circle_ = true;
        bearing_error_rad_ = 0.0f;
        nav_bearing_rad_ = atan2f(-a_air_unit.y, -a_air_unit.x);
    }

    data_is_stale_ = false;
    return true;
}

float L1Controller::loiterTargetRadius() const
{
    return loiter_target_radius_m_;
}

bool L1Controller::loiterActive() const
{
    return wp_circle_;
}

void L1Controller::updateHeadingHold(const AhrsData& ahrs, const ImuData& imu,
                                     int32_t navigation_heading_cd)
{
    const float omega_a = 4.4428f / config_.period_s;  // sqrt(2)*pi/period

    target_bearing_cd_ = wrap_180_cd(navigation_heading_cd);
    nav_bearing_rad_ = radians(navigation_heading_cd * 0.01f);

    int32_t nu_cd = target_bearing_cd_ - wrap_180_cd(getYawSensor(imu));
    nu_cd = wrap_180_cd(nu_cd);
    float nu = radians(nu_cd * 0.01f);

    const Vector2f groundspeed_vector(ahrs.velocity_ned_mps.x, ahrs.velocity_ned_mps.y);
    const float ground_speed = groundspeed_vector.length();

    l1_distance_ = ground_speed / omega_a;
    const float v_omega_a = ground_speed * omega_a;

    wp_circle_ = false;
    crosstrack_error_ = 0.0f;
    bearing_error_rad_ = nu;

    nu = constrain_float(nu, -M_PI_2, M_PI_2);
    lat_accel_demand_ = 2.0f * sinf(nu) * v_omega_a;

    data_is_stale_ = false;
}

void L1Controller::updateLevelFlight(const ImuData& imu)
{
    target_bearing_cd_ = static_cast<int32_t>(imu.yaw_deg * 100.0f);
    nav_bearing_rad_ = imu.yaw_rad;
    bearing_error_rad_ = 0.0f;
    crosstrack_error_ = 0.0f;

    wp_circle_ = false;
    lat_accel_demand_ = 0.0f;

    data_is_stale_ = false;
}

}  // namespace fc
