#include "storage/Params.h"

#include <EEPROM.h>
#include <math.h>
#include <string.h>

namespace {

constexpr uint16_t kEepromMagic = 0xFC02;  // New schema (FW-only) -- distinct from legacy 0xCDAC.

// 1000, not the old 650: storage/Waypoints.h's kAddrWpData(8) actually
// extends to 8 + kMaxEepromWaypoints(50)*sizeof(Locations)(16) = 808, which
// swallowed the old 650-782 range whole -- every waypoint save silently
// corrupted these params and vice versa. 1000 leaves a clear 192-byte gap
// past Waypoints' end; kAddrData+kMaxParams*4 = 1132 leaves just as much
// margin before storage/ImuCalibrationStorage.h's block at 2000. See
// docs/params.md and docs/imu-bno055.md for the full EEPROM map.
constexpr uint16_t kAddrMagic = 1000;
constexpr uint16_t kAddrCount = 1002;
constexpr uint16_t kAddrData = 1004;
constexpr uint16_t kTeensy41EepromSize = 4284;

}  // namespace

namespace fc {

void Params::add(const char* name, float* target, float default_val, float min_val, float max_val)
{
    if (count_ >= kMaxParams) {
        return;
    }
    ParamEntry& entry = params_[count_++];
    strncpy(entry.name, name, sizeof(entry.name) - 1);
    entry.name[sizeof(entry.name) - 1] = '\0';
    entry.target = target;
    entry.default_val = default_val;
    entry.min_val = min_val;
    entry.max_val = max_val;
}

void Params::initFixedWing(L1ControllerConfig& l1, FuzzyL1TunerConfig& fuzzy, TecsConfig& tecs,
                           AttitudeControllerConfig& attitude, Imu& imu)
{
    count_ = 0;

    // L1 guidance -- NOT registered as a param at all in the legacy code
    // (bare compile-time global, no EEPROM exposure). This is the parameter
    // the thesis's fuzzy tuner adjusts at runtime; the base value here is
    // what "L1 conventional" uses when the tuner is disabled.
    add("L1_PERIOD", &l1.period_s, 20.0f, 5.0f, 40.0f);
    add("L1_DAMPING", &l1.damping, 0.73f, 0.6f, 1.0f);
    add("L1_XTRACK_I", &l1.xtrack_integrator_gain, 0.2f, 0.0f, 1.0f);

    // Fuzzy L1-period tuner (the thesis's self-tuning mechanism).
    // FUZZY_ENABLE=0 provides the conventional fixed-period baseline using
    // the exact same firmware for a fair A/B comparison.
    add("FUZZY_ENABLE", &fuzzy.enabled, 1.0f, 0.0f, 1.0f);
    add("FUZZY_MIN_PER", &fuzzy.min_period_s, 10.0f, 5.0f, 40.0f);
    add("FUZZY_MAX_PER", &fuzzy.max_period_s, 30.0f, 5.0f, 40.0f);

    // TECS.
    add("TECS_TIME_CNST", &tecs.time_const_s, 5.0f, 0.1f, 15.0f);
    add("TECS_THR_DAMP", &tecs.throttle_damp, 0.5f, 0.0f, 2.0f);
    add("TECS_PTCH_DAMP", &tecs.pitch_damp, 0.0f, 0.0f, 2.0f);
    add("TECS_INTEG_GN", &tecs.integrator_gain, 0.1f, 0.0f, 1.0f);
    add("TECS_CLMB_MAX", &tecs.max_climb_rate_mps, 5.0f, 0.5f, 20.0f);
    add("TECS_SINK_MIN", &tecs.min_sink_rate_mps, 2.0f, 0.5f, 10.0f);
    add("TECS_SINK_MAX", &tecs.max_sink_rate_mps, 5.0f, 0.5f, 20.0f);
    add("TECS_THR_MIN", &tecs.min_throttle_percent, 60.0f, 0.0f, 100.0f);
    add("TECS_THR_MAX", &tecs.max_throttle_percent, 95.0f, 0.0f, 100.0f);
    add("TECS_ROLL_CMP", &tecs.roll_compensation_deg, 10.0f, 0.0f, 20.0f);
    add("TECS_SPD_WGT", &tecs.speed_weight, 1.0f, 0.0f, 2.0f);
    add("TECS_VERT_ACC", &tecs.vertical_accel_limit_mps2, 7.0f, 1.0f, 15.0f);

    // Airspeed envelope (shared by TECS and, at vehicle-init time, copied
    // into Navigation/AttitudeController's own airspeed fields  --  see
    // docs/params.md for why this is the single authoritative source).
    add("AIRSPEED_CRUISE", &tecs.cruise_airspeed_mps, 18.0f, 5.0f, 35.0f);
    add("MIN_AIRSPEED", &tecs.min_airspeed_mps, 14.0f, 5.0f, 30.0f);
    add("MAX_AIRSPEED", &tecs.max_airspeed_mps, 22.0f, 10.0f, 45.0f);

    // TD/V10-trainer2 leaves fixed-wing rudder correction neutral (u_yaw=0),
    // so the previous YAW_CORR_EN/LQR-yaw parameter is intentionally removed.

    // Optional airspeed-based output scaler retained as an experiment hook.
    // TD/V10-trainer2 does not use this multiplier, so keep SPD_SCALE_EN=0
    // when reproducing the TD baseline.
    add("SPD_SCALE_EN", &attitude.speed_scaler_enabled, 0.0f, 0.0f, 1.0f);

    // BNO055 bench-level trim, set interactively via Mission Planner's
    // "Calibrate Level" (MAV_CMD_PREFLIGHT_CALIBRATION param5=2, see
    // Mavlink::handleCommandLong) and persisted here so it survives reboot.
    add("IMU_ROLL_TRIM", &imu.rollTrimDegRef(), 0.0f, -45.0f, 45.0f);
    add("IMU_PITCH_TRIM", &imu.pitchTrimDegRef(), 0.0f, -45.0f, 45.0f);

    // Inner-loop gains. The familiar parameter names are retained so existing
    // Mission Planner workflows do not need to change. Values are copied into
    // AttitudeController at construction, so power-cycle after changing them.
    // TD/V10-trainer2 inner-loop gains. Parameter names are kept compatible
    // with the previous GCS layout even though the active controller is now
    // TD-style P/I/D rather than LQR. KRATE is the derivative/rate gain.
    add("ROLL_KP", &attitude.roll.kp, 10.0f, 0.0f, 20.0f);
    add("ROLL_KRATE", &attitude.roll.kd, 0.0f, 0.0f, 5.0f);
    add("ROLL_KI", &attitude.roll.ki, 0.0f, 0.0f, 5.0f);
    add("PITCH_KP", &attitude.pitch.kp, 10.0f, 0.0f, 20.0f);
    add("PITCH_KRATE", &attitude.pitch.kd, 0.20f, 0.0f, 5.0f);
    add("PITCH_KI", &attitude.pitch.ki, 0.0f, 0.0f, 5.0f);
}

float Params::clampValue(float value, float min_val, float max_val)
{
    if (isnan(value) || isinf(value)) {
        return min_val;
    }
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

void Params::applyDefaults(uint16_t start_index)
{
    for (uint16_t i = start_index; i < count_; ++i) {
        *params_[i].target = params_[i].default_val;
    }
}

int Params::findIndex(const char* name) const
{
    for (uint16_t i = 0; i < count_; ++i) {
        if (strncmp(params_[i].name, name, sizeof(params_[i].name) - 1) == 0) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

ParamLoadResult Params::load()
{
    const uint16_t end_addr = kAddrData + (count_ * sizeof(float));
    if (end_addr > kTeensy41EepromSize) {
        applyDefaults(0);
        return ParamLoadResult::NoValidEepromData;
    }

    uint16_t magic = 0;
    uint16_t saved_count = 0;
    EEPROM.get(kAddrMagic, magic);
    EEPROM.get(kAddrCount, saved_count);

    if (magic != kEepromMagic) {
        applyDefaults(0);
        save();
        return ParamLoadResult::NoValidEepromData;
    }

    const uint16_t load_count = (saved_count < count_) ? saved_count : count_;
    uint16_t addr = kAddrData;
    for (uint16_t i = 0; i < load_count; ++i) {
        float value = 0.0f;
        EEPROM.get(addr, value);
        addr += sizeof(float);
        *params_[i].target = clampValue(value, params_[i].min_val, params_[i].max_val);
    }

    if (saved_count < count_) {
        applyDefaults(saved_count);
        save();
        return ParamLoadResult::SchemaUpgraded;
    }

    return ParamLoadResult::LoadedFromEeprom;
}

void Params::save()
{
    const uint16_t end_addr = kAddrData + (count_ * sizeof(float));
    if (end_addr > kTeensy41EepromSize) {
        return;
    }

    EEPROM.put(kAddrMagic, kEepromMagic);
    EEPROM.put(kAddrCount, count_);

    uint16_t addr = kAddrData;
    for (uint16_t i = 0; i < count_; ++i) {
        const float value = clampValue(*params_[i].target, params_[i].min_val, params_[i].max_val);
        *params_[i].target = value;
        EEPROM.put(addr, value);
        addr += sizeof(float);
    }
}

void Params::resetToDefaults()
{
    applyDefaults(0);
    save();
}

bool Params::setValue(const char* name, float value, bool save_after_set)
{
    const int index = findIndex(name);
    if (index < 0) {
        return false;
    }
    *params_[index].target = clampValue(value, params_[index].min_val, params_[index].max_val);
    if (save_after_set) {
        save();
    }
    return true;
}

bool Params::getValue(const char* name, float& value_out) const
{
    const int index = findIndex(name);
    if (index < 0) {
        return false;
    }
    value_out = *params_[index].target;
    return true;
}

uint16_t Params::count() const
{
    return count_;
}

const ParamEntry& Params::entryAt(uint16_t index) const
{
    return params_[index];
}

}  // namespace fc
