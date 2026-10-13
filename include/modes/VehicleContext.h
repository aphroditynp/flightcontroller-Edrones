#pragma once

#include <stdint.h>

#include "FC_Config.h"
#include "control/AltitudePitchController.h"
#include "control/AttitudeController.h"
#include "drivers/Airspeed.h"
#include "drivers/Barometer.h"
#include "drivers/Imu.h"
#include "estimation/Ahrs.h"
#if FC_ATTITUDE_ESTIMATOR_MAHONY_ENABLE
#include "estimation/AttitudeMahonyFilter.h"
#endif
#include "modes/Mode.h"
#include "navigation/FuzzyL1Tuner.h"
#include "navigation/GnssFixData.h"
#include "navigation/L1Controller.h"
#include "navigation/Navigation.h"
#include "navigation/Tecs.h"
#include "storage/Params.h"
#include "vehicle/Actuator.h"
#include "vehicle/Battery.h"
#include "vehicle/Buzzer.h"
#include "vehicle/Radio.h"

namespace fc {

/**
 * Bag of references to every subsystem a fixed-wing mode needs, plus the
 * per-loop snapshot data the scheduler refreshes before calling
 * ModeManager::update(). This replaces the legacy code's implicit global
 * access (imu, ahrs, gepees, fw_control, etc. as file-scope singletons) with
 * explicit dependency injection, while keeping ModeBase's virtual
 * enter/update/exit interface argument-free (each mode stores a
 * VehicleContext& instead).
 *
 * Owned and refreshed by the vehicle-integration layer (Phase 8's
 * main.cpp), not by the modes themselves.
 */
struct VehicleContext {
    Imu& imu;
    Barometer& baro;
    Airspeed& airspeed;
    Ahrs& ahrs;
    L1Controller& l1;
    FuzzyL1Tuner& fuzzy_tuner;
    Tecs& tecs;
    Navigation& navigation;
    AttitudeController& attitude;
    // AUTO/LOITER altitude-hold pitch loop, separate from TECS (which is
    // left in charge of auto-throttle only). See docs/altitude-pitch-controller.md.
    AltitudePitchController& altitude_pitch;
    Actuator& actuator;
    Radio& radio;
    Battery& battery;
    Buzzer& buzzer;
    ModeManager& mode_manager;
    Params& params;
#if FC_ATTITUDE_ESTIMATOR_MAHONY_ENABLE
    AttitudeMahonyFilter& attitude_mahony;
#endif

    // Refreshed once per loop by the scheduler, before ModeManager::update():
    GnssFixData gnss{};
    float dt_s = 0.0f;
    uint32_t now_ms = 0;

    // Set by the MAVLink handler (Phase 7) when a companion computer sends a
    // payload-drop command; consumed (reset to false) by Actuator::updatePayload().
    bool payload_drop_command = false;

    bool enableThrottleNudge = true;

    /**
     * The ImuData AttitudeController's roll/pitch feedback loop actually
     * flies on -- BNO055 on-chip fusion (Option 1) unless
     * FC_ATTITUDE_CONTROL_SOURCE_MAHONY is set, in which case roll_deg/
     * pitch_deg are swapped for fc::AttitudeMahonyFilter's (Option 2).
     * gyro/accel/everything-else stays BNO055's regardless -- both
     * estimators consume the same raw gyro, so there's nothing to swap
     * there. Navigation/TECS/Ahrs call sites intentionally keep using
     * imu.data() directly, NOT this -- only the attitude-control feedback
     * loop itself is switchable. See docs/attitude-mahony-filter.md and
     * FC_Config.h's FC_ATTITUDE_CONTROL_SOURCE_MAHONY comment.
     */
    ImuData controlImu() const
    {
#if FC_ATTITUDE_CONTROL_SOURCE_MAHONY
        ImuData source = imu.data();
        const AttitudeMahonyData mahony = attitude_mahony.data();
        source.roll_deg = mahony.roll_deg;
        source.pitch_deg = mahony.pitch_deg;
        return source;
#else
        return imu.data();
#endif
    }
};

}  // namespace fc
