#include "estimation/Ahrs.h"

#include <math.h>
#include <quaternion.h>

namespace {

constexpr float kGravityMss = 9.80665f;
constexpr uint32_t kGpsFixFreshnessUs = 3000000U;  // 3 s, matches legacy gepees.age<3000ms

Vector3f toVector3f(const fc::ImuVector3f& v)
{
    return Vector3f(v.x, v.y, v.z);
}

}  // namespace

namespace fc {

Ahrs::Ahrs() = default;

bool Ahrs::haveGps(const GnssFixData& gnss) const
{
    return gnss.fix_type != GnssFixType::NoFix;
}

bool Ahrs::homeIsSet() const
{
    return home_set_;
}

bool Ahrs::gpsFixIsFresh(const GnssFixData& gnss)
{
    return gnss.valid &&
           static_cast<uint32_t>(micros() - gnss.timestamp_us) < kGpsFixFreshnessUs;
}

void Ahrs::setHome(const GnssFixData& gnss, const BarometerData& baro)
{
    if (gnss.fix_type != GnssFixType::Fix3D) {
        return;
    }

    home_.lat = static_cast<int32_t>(gnss.latitude_deg * 1e7);
    home_.lng = static_cast<int32_t>(gnss.longitude_deg * 1e7);
    home_.alt = static_cast<int32_t>(baro.altitude_m * 100.0f);
    home_set_ = true;
}

Matrix3f Ahrs::rotationFromEuler(float roll_rad, float pitch_rad, float yaw_rad)
{
    Quaternion q;
    Matrix3f rotation;
    q.from_euler(roll_rad, pitch_rad, yaw_rad);
    q.rotation_matrix(rotation);
    return rotation;
}

void Ahrs::update(const ImuData& imu, const GnssFixData& gnss, const BarometerData& baro,
                   const AirspeedData& airspeed)
{
    data_.rotation_body_to_ned = rotationFromEuler(imu.roll_rad, imu.pitch_rad, imu.yaw_rad);
    data_.acceleration_earth_frame_mss = data_.rotation_body_to_ned * toVector3f(imu.acceleration_mss);
    data_.acceleration_body_frame_mss =
        data_.acceleration_earth_frame_mss - Vector3f(0.0f, 0.0f, -kGravityMss);

    // Auto-detect airspeed sensor health (MS4525DO): valid if not NaN/Inf and
    // within a plausible range; falls back to a no-airspeed-sensor mode
    // otherwise (matches legacy AHRS::update_ahrs()).
    data_.airspeed_sensor_enabled =
        !isnan(airspeed.velocity_mps) && !isinf(airspeed.velocity_mps) &&
        airspeed.velocity_mps < 50.0f;

    updateDriftCorrectedVelocity(imu, gnss, baro, airspeed);

    data_.position.lat = static_cast<int32_t>(gnss.latitude_deg * 1e7);
    data_.position.lng = static_cast<int32_t>(gnss.longitude_deg * 1e7);
    data_.position.alt = static_cast<int32_t>(baro.altitude_m * 100.0f);
    data_.home_set = home_set_;

    data_.timestamp_us = micros();
    data_.sequence += 1U;
    data_.valid = true;
}

void Ahrs::updateDriftCorrectedVelocity(const ImuData& imu, const GnssFixData& gnss,
                                        const BarometerData& baro, const AirspeedData& airspeed)
{
    const float sin_heading = sinf(imu.yaw_rad);
    const float cos_heading = cosf(imu.yaw_rad);

    if (gpsFixIsFresh(gnss) || gnss.fix_type == GnssFixType::Fix3D) {
        data_.groundspeed_mps = gnss.ground_speed_mps; //ini dicekkk sebelum terbang jangan (* 1000)!!!
        data_.velocity_ned_mps = Vector3f(gnss.velocity_ned_mps[0], gnss.velocity_ned_mps[1],
                                          gnss.velocity_ned_mps[2]);
        data_.gps_lock = true;
        return;
    }

    if (!haveGps(gnss)) {
        data_.groundspeed_mps = 0.0f;
        data_.gps_lock = false;
        return;
    }

    // Dead-reckoning fallback: complementary filter blending a low-pass of
    // the last known GPS groundspeed with a high-pass of body acceleration.
    //
    // NOTE: legacy AHRS::estimateGroundspeedVelocity() computed
    // `groundspeed = (hp + lp) * 10000`. That factor is a unit-scale bug,
    // not an intentional design choice — the legacy author's own comment a
    // few lines away in driftCorrection() ("ini dicek lagi, kalau terbang
    // jangan (* 1000)") flags exactly this class of scaling issue as
    // unresolved. Multiplying an m/s groundspeed by 10000 would feed an
    // explosive velocity into L1/TECS. This port omits that factor. This
    // fallback path only runs when a GPS fix exists but is stale/2D-only —
    // validate it in SITL before relying on it in flight, since it was
    // never exercised correctly in the legacy build.
    constexpr float kBeta = 0.1f;
    constexpr float kAlpha = 1.0f - kBeta;
    constexpr float kDt = 0.1f;

    const Vector3f body_accel = data_.acceleration_body_frame_mss;

    low_pass_speed_mps_ = gnss.ground_speed_mps * kBeta + kAlpha * low_pass_speed_mps_;
    high_pass_speed_mps_ =
        (data_.groundspeed_mps - last_groundspeed_mps_) + body_accel.x * cos_heading * kDt +
        kAlpha * high_pass_speed_mps_;
    data_.groundspeed_mps = high_pass_speed_mps_ + low_pass_speed_mps_;
    if (data_.groundspeed_mps < 0.0f) {
        data_.groundspeed_mps = 0.0f;
    }

    last_groundspeed_mps_ = data_.groundspeed_mps;
    last_velocity_ = data_.velocity_ned_mps;

    data_.velocity_ned_mps.x = data_.groundspeed_mps * cos_heading;
    data_.velocity_ned_mps.y = data_.groundspeed_mps * sin_heading;
    data_.velocity_ned_mps.z = -baro.climb_rate_mps;  // NED: climbing UP is negative Z

    float estimated_airspeed = 0.0f;
    if (data_.airspeed_sensor_enabled) {
        estimated_airspeed = airspeed.velocity_mps;
    } else {
        estimated_airspeed = data_.groundspeed_mps - data_.windspeed_horizontal_mps;
    }
    data_.estimated_airspeed_mps = MAX(estimated_airspeed, 0.0f);
    data_.gps_lock = false;
}

void Ahrs::updateWindspeed()
{
    const Vector3f velocity = data_.velocity_ned_mps;

    // Ported from MatrixPilot's wind-speed estimator (Bill Premerlani),
    // adapted for ArduPilot by Jon Challinger.
    const Vector3f fuselage_direction = data_.rotation_body_to_ned.colx();
    const Vector3f fuselage_direction_diff = fuselage_direction - last_fuselage_direction_;
    const uint32_t now_ms = millis();

    if (now_ms - last_wind_update_ms_ > 1000U) {
        last_wind_update_ms_ = now_ms;
        last_fuselage_direction_ = fuselage_direction;
        last_velocity_ = velocity;
        return;
    }

    const float diff_length = fuselage_direction_diff.length();
    if (diff_length <= 0.2f) {
        return;
    }

    const Vector3f velocity_diff = velocity - last_velocity_;
    const float speed_over_ground = velocity_diff.length() / diff_length;

    const Vector3f fuselage_direction_sum = fuselage_direction + last_fuselage_direction_;
    const Vector3f velocity_sum = velocity + last_velocity_;
    last_fuselage_direction_ = fuselage_direction;
    last_velocity_ = velocity;

    const float theta = atan2f(velocity_diff.y, velocity_diff.x) -
                        atan2f(fuselage_direction_diff.y, fuselage_direction_diff.x);
    const float sin_theta = sinf(theta);
    const float cos_theta = cosf(theta);

    Vector3f wind{};
    wind.x = velocity_sum.x -
             speed_over_ground * (cos_theta * fuselage_direction_sum.x -
                                  sin_theta * fuselage_direction_sum.y);
    wind.y = velocity_sum.y -
             speed_over_ground * (sin_theta * fuselage_direction_sum.x +
                                  cos_theta * fuselage_direction_sum.y);
    wind.z = velocity_sum.z - speed_over_ground * fuselage_direction_sum.z;
    wind *= 0.5f;

    if (wind.length() < data_.windspeed_mps.length() + 20.0f) {
        data_.windspeed_mps = data_.windspeed_mps * 0.95f + wind * 0.05f;
    }

    last_wind_update_ms_ = now_ms;
    data_.windspeed_horizontal_mps = sqrtf(data_.windspeed_mps.x * data_.windspeed_mps.x +
                                           data_.windspeed_mps.y * data_.windspeed_mps.y);
}

AhrsData Ahrs::data() const
{
    return data_;
}

}  // namespace fc
