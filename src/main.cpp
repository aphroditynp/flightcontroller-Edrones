// KHAGESWARA-FC-2026 fixed-wing refactor -- integration entry point.
//
// This wires together every module ported across Phases 1-7 into one
// FreeRTOS-scheduled firmware. See docs/*.md for the per-module migration
// notes and docs/main-integration.md for this file's specific wiring
// decisions (task layout, GNSS backend choice, config-dependent object
// lazy-construction order).
//
// GNSS backend: GnssNmea (Radiolink SE100) is the active default, per your
// confirmed decision -- Here4GnssReceiver (DroneCAN) and GnssUbx (M10) are
// fully implemented and ready to swap in once Here4's DSDL signature is
// verified and its node-ID allocation behavior is bench-tested (see
// docs/gnss-here4-dronecan.md).

#include <Arduino.h>
#include <EEPROM.h>

#include "FreeRTOS.h"
#include "task.h"

#include "control/AttitudeController.h"
#include "drivers/Airspeed.h"
#include "drivers/Barometer.h"
#include "drivers/GnssNmea.h"
#include "drivers/Imu.h"
#include "estimation/Ahrs.h"
#include "estimation/AltitudeComplementaryFilter.h"
#include "estimation/AttitudeMahonyFilter.h"
#include "modes/FixedWingModes.h"
#include "modes/Mode.h"
#include "modes/VehicleContext.h"
#include "navigation/FuzzyL1Tuner.h"
#include "navigation/L1Controller.h"
#include "navigation/Navigation.h"
#include "navigation/Tecs.h"
#include "storage/Params.h"
#include "storage/ImuCalibrationStorage.h"
#include "storage/Waypoints.h"
#include "vehicle/Actuator.h"
#include "vehicle/Battery.h"
#include "vehicle/Buzzer.h"
#include "vehicle/Radio.h"
#include "communication/Mavlink.h"
#include "communication/DataLogger.h"
#include "communication/SdLogger.h"
#include "FC_Config.h"

#include <MTP_Teensy.h>

namespace {

// Task periods and serial baud rates live in FC_Config.h now -- one place
// to retune instead of hunting through this file. `using namespace` here
// (rather than qualifying every use as fc::kImuPeriodMs etc.) keeps every
// existing unqualified reference below working unchanged. FC_Config.h also
// defines FC_DEBUG_SERIAL_ENABLE (used directly below via #if, not through
// this using-directive -- it's a preprocessor macro, not a fc:: symbol).
using namespace fc;

DMAMEM uint8_t g_dmaRx2[4096];
DMAMEM uint8_t g_dmaTx2[4096];
DMAMEM uint8_t g_dmaRx7[2048];
DMAMEM uint8_t g_dmaTx7[4096];

// ----------------------------------------------------------------------
// Config-independent subsystems: plain global objects (defaults are
// correct without any EEPROM override).
// ----------------------------------------------------------------------
fc::Imu g_imu(Wire);
fc::Barometer g_baro(Wire1);
fc::Airspeed g_airspeed(Wire2);
fc::GnssNmea g_gnss(Serial1);
fc::Ahrs g_ahrs;
fc::Actuator g_actuator;
fc::Radio g_radio(Serial8);
fc::Battery g_battery;
fc::Buzzer g_buzzer;
fc::ModeManager g_modeManager;
fc::Params g_params;
fc::Mavlink g_mavlink;
fc::SdLogger g_sdLogger;
// PuTTY/serial-terminal CSV logger over the primary USB port -- only
// meaningful when FC_DEBUG_SERIAL_ENABLE=1 (see FC_Config.h); harmless to
// construct either way since it just wraps Serial without touching it
// until begin()/logAttitudeAltitude() are called.
fc::DataLogger g_usbLogger(Serial);

// Option 2 altitude estimator (baro+accel complementary filter), run in
// parallel with fc::Barometer's onboard Kalman filter (Option 1) purely for
// bench/flight comparison -- NOT wired into TECS/Navigation. See
// docs/altitude-complementary-filter.md.
fc::AltitudeComplementaryFilter g_altComplementary;

#if FC_ATTITUDE_ESTIMATOR_MAHONY_ENABLE
// Option 2 attitude estimator (Mahony, gyro+accel+mag), run in parallel
// with BNO055's on-chip NDOF fusion for bench/flight comparison. Excluded
// entirely (not just idle) when FC_ATTITUDE_ESTIMATOR_MAHONY_ENABLE=0 --
// see FC_Config.h. Drives AttitudeController too if
// FC_ATTITUDE_CONTROL_SOURCE_MAHONY is additionally set -- see
// VehicleContext::controlImu() and docs/attitude-mahony-filter.md.
fc::AttitudeMahonyFilter g_attitudeMahony;
#endif

// ----------------------------------------------------------------------
// Config-dependent subsystems: L1Controller/FuzzyL1Tuner/Tecs read their
// tunable fields from Params-owned config structs, and Navigation/
// AttitudeController both need the SAME airspeed envelope Tecs uses. All
// five are constructed in setup(), AFTER params.load() has populated the
// config structs below -- see docs/main-integration.md for why plain
// global objects don't work here (C++ static-init order would construct
// them with un-loaded defaults, before setup() ever runs).
// ----------------------------------------------------------------------
fc::L1ControllerConfig g_l1Config;
fc::FuzzyL1TunerConfig g_fuzzyConfig;
fc::TecsConfig g_tecsConfig;
fc::NavigationConfig g_navigationConfig;
fc::AttitudeControllerConfig g_attitudeConfig;

fc::L1Controller* g_l1 = nullptr;
fc::FuzzyL1Tuner* g_fuzzyTuner = nullptr;
fc::Tecs* g_tecs = nullptr;
fc::Navigation* g_navigation = nullptr;
fc::AttitudeController* g_attitude = nullptr;

fc::VehicleContext* g_ctx = nullptr;

fc::ModeManual* g_modeManual = nullptr;
fc::ModeFbwa* g_modeFbwa = nullptr;
fc::ModeAuto* g_modeAuto = nullptr;
fc::ModeGuided* g_modeGuided = nullptr;
fc::ModeLoiter* g_modeLoiter = nullptr;

fc::ModeId g_activeModeId = fc::ModeId::Manual;

TaskHandle_t g_taskImu = nullptr;
TaskHandle_t g_taskBaro = nullptr;
TaskHandle_t g_taskGps = nullptr;
TaskHandle_t g_taskAirspeed = nullptr;
TaskHandle_t g_taskRadio = nullptr;
TaskHandle_t g_taskBuzzer = nullptr;
TaskHandle_t g_taskControl = nullptr;
TaskHandle_t g_taskMavlink = nullptr;
TaskHandle_t g_taskSdLog = nullptr;
TaskHandle_t g_taskMtp = nullptr;
#if FC_DEBUG_SERIAL_ENABLE
TaskHandle_t g_taskUsbLog = nullptr;
#endif

/** Routes one boot/status diagnostic line to whichever destination
 * FC_DEBUG_SERIAL_ENABLE selects (FC_Config.h) -- compile-time, matches
 * MAVLINK_USB_ENABLE_RX/TX in the same file. mavlink_severity
 * (MAV_SEVERITY_* from common/mavlink.h) is only used when
 * FC_DEBUG_SERIAL_ENABLE==0. */
void reportStatus(uint8_t mavlink_severity, const char* text)
{
#if FC_DEBUG_SERIAL_ENABLE
    (void)mavlink_severity;
    Serial.println(text);
#else
    g_mavlink.sendStatusText(mavlink_severity, text);
#endif
}

/**
 * Maps the four configured RC positions to MANUAL/FBWA/AUTO/GUIDED.
 * LOITER is entered automatically after the final AUTO waypoint or explicitly
 * through MAVLink custom_mode=12; it is intentionally not assigned to one of
 * the existing four RC positions so the radio layout does not change.
 */
void applyRcModeSwitch()
{
    fc::ModeId wanted;
    switch (g_radio.modeNow()) {
        case 1: wanted = fc::ModeId::Manual; break;
        case 2: wanted = fc::ModeId::Fbwa; break;
        case 3: wanted = fc::ModeId::Auto; break;
        case 4: wanted = fc::ModeId::Guided; break;
        default: return;
    }

    if (wanted != g_activeModeId) {
        if (g_modeManager.setMode(wanted)) {
            g_activeModeId = wanted;
        }
    }
}

void waypointReachedTrampoline(uint16_t seq, void* user_data)
{
    static_cast<fc::Mavlink*>(user_data)->notifyWaypointReached(seq);
}

/** Seeds a minimal home+one-waypoint mission if EEPROM has none, so AUTO/
 * GUIDED have somewhere to go on a bench test. Matches legacy wp_setup()'s
 * role, not its specific coordinates (those were site-specific to the
 * original team's flying field). */
void seedDefaultMission(fc::MissionState& mission)
{
    mission.waypoint[0] = Locations(0, 0, 0, Locations::AltFrame::ABOVE_HOME);
    mission.wp_sum = 1;
}

// ----------------------------------------------------------------------
// FreeRTOS tasks
// ----------------------------------------------------------------------

void taskImu(void*)
{
#if FC_ATTITUDE_ESTIMATOR_MAHONY_ENABLE
    uint32_t last_update_us = micros();
#endif
    for (;;) {
        if (g_imu.update()) {
#if FC_ATTITUDE_ESTIMATOR_MAHONY_ENABLE
            // Option 2 attitude estimator (Mahony, gyro+accel+mag), run
            // alongside BNO055's on-chip NDOF fusion (Option 1, g_imu's own
            // roll_deg/pitch_deg/yaw_deg). Logged/telemetered for
            // comparison either way; drives AttitudeController too if
            // FC_ATTITUDE_CONTROL_SOURCE_MAHONY is set (see
            // VehicleContext::controlImu()). See docs/attitude-mahony-filter.md
            // and FC_Config.h.
            const uint32_t now_us = micros();
            float dt_s = (now_us - last_update_us) * 1.0e-6f;
            if (dt_s <= 0.0f || dt_s > 0.1f) {
                dt_s = kImuPeriodMs * 1.0e-3f;  // clamp a startup/overrun gap to the nominal period
            }
            last_update_us = now_us;

            const fc::ImuData& imu_data = g_imu.data();
            g_attitudeMahony.update(
                imu_data.angular_rate_dps.x * kDegToRad, imu_data.angular_rate_dps.y * kDegToRad,
                imu_data.angular_rate_dps.z * kDegToRad, imu_data.acceleration_mss.x,
                imu_data.acceleration_mss.y, imu_data.acceleration_mss.z, imu_data.magnetic_field_ut.x,
                imu_data.magnetic_field_ut.y, imu_data.magnetic_field_ut.z, dt_s);
#endif
        }
        vTaskDelay(pdMS_TO_TICKS(kImuPeriodMs));
    }
}

void taskBaro(void*)
{
    for (;;) {
        g_baro.update();
        vTaskDelay(pdMS_TO_TICKS(kBaroPeriodMs));
    }
}

void taskSdLog(void*)
{
    for (;;) {
        const fc::ImuData& imu = g_imu.data();
#if FC_ATTITUDE_ESTIMATOR_MAHONY_ENABLE
        const fc::AttitudeMahonyData& mahony = g_attitudeMahony.data();
        const float mahony_roll_deg = mahony.roll_deg;
        const float mahony_pitch_deg = mahony.pitch_deg;
        const float mahony_yaw_deg = mahony.yaw_deg;
#else
        // Mahony not compiled in -- SdLogger::logRow()'s CSV schema stays
        // fixed either way (simpler than a conditional column count), so
        // these columns just read 0 instead of being omitted.
        constexpr float mahony_roll_deg = 0.0f;
        constexpr float mahony_pitch_deg = 0.0f;
        constexpr float mahony_yaw_deg = 0.0f;
#endif
        g_sdLogger.logRow(imu.roll_deg, imu.pitch_deg, imu.yaw_deg, g_baro.data().altitude_m,
                          g_modeManager.code4(), g_radio.armed(), g_radio.channelRoll(),
                          g_radio.channelPitch(), g_radio.channelThrottle(), g_radio.channelYaw(),
                          g_radio.channelArmRaw(), mahony_roll_deg, mahony_pitch_deg, mahony_yaw_deg);
        vTaskDelay(pdMS_TO_TICKS(kSdLogPeriodMs));
    }
}

#if FC_DEBUG_SERIAL_ENABLE
// PuTTY-readable CSV twin of taskSdLog, over USB Serial instead of the SD
// card -- full raw IMU sample (not just roll/pitch/yaw) for axis-
// convention/interference diagnosis without pulling the SD card. See
// docs/data-logger-usb.md and docs/imu-bno055.md.
void taskUsbLog(void*)
{
    for (;;) {
        g_usbLogger.logAttitudeAltitude(g_imu.data(), g_imu.calibration(), g_baro.data(),
                                        g_altComplementary.data());
        vTaskDelay(pdMS_TO_TICKS(kUsbLogPeriodMs));
    }
}
#endif

void taskMtp(void*)
{
    for (;;) {
        MTP.loop();
        vTaskDelay(pdMS_TO_TICKS(kMtpPeriodMs));
    }
}

void taskGps(void*)
{
    for (;;) {
        g_gnss.update();
        vTaskDelay(pdMS_TO_TICKS(kGpsPeriodMs));
    }
}

void taskAirspeed(void*)
{
    for (;;) {
        g_airspeed.update();
        vTaskDelay(pdMS_TO_TICKS(kAirspeedPeriodMs));
    }
}

void taskRadio(void*)
{
    for (;;) {
        g_radio.update();
        vTaskDelay(pdMS_TO_TICKS(kRadioPeriodMs));
    }
}

void taskBuzzer(void*)
{
    for (;;) {
        g_buzzer.update(g_radio.armed());
        vTaskDelay(pdMS_TO_TICKS(kBuzzerPeriodMs));
    }
}

void taskControl(void*)
{
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(kControlPeriodMs);
    uint32_t last_update_us = micros();
    for (;;) {
        const uint32_t now_us = micros();
        g_ctx->dt_s = (now_us - last_update_us) * 1.0e-6f;
        if (g_ctx->dt_s > 0.1f) {
            g_ctx->dt_s = kControlPeriodMs * 1.0e-3f;  // clamp a startup/overrun gap to the nominal period
        }
        last_update_us = now_us;

        g_ctx->gnss = g_gnss.data();
        g_ctx->now_ms = millis();

        applyRcModeSwitch();
        g_modeManager.update();  // updates g_ahrs as a side effect (see mode update() bodies)

        // Option 2 altitude estimator: fed from the SAME raw (pre-Kalman) baro
        // sample Option 1 uses, so the two are a fair side-by-side comparison
        // rather than one filtering the other's output. accel_up_mss is
        // negated because acceleration_body_frame_mss.z follows NED (down
        // positive), matching AhrsData's own "climbing UP is negative Z"
        // convention (see Ahrs.cpp).
        g_altComplementary.update(g_baro.data().raw_altitude_m,
                                  -g_ahrs.data().acceleration_body_frame_mss.z,
                                  g_ctx->dt_s);

        if ((xTaskGetTickCount() - last_wake) >= period) {
            last_wake = xTaskGetTickCount();  // overrun: skip catch-up bursts
        }
        vTaskDelayUntil(&last_wake, period);
    }
}

void taskMavlink(void*)
{
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(kMavlinkPeriodMs);
    uint32_t last_battery_ms = 0;
    uint32_t last_imu_calib_ms = 0;
#if FC_ATTITUDE_ESTIMATOR_MAHONY_ENABLE
    uint32_t last_mahony_ms = 0;
#endif
    bool imu_calibration_saved = false;

    for (;;) {
        g_mavlink.handlePorts(*g_ctx);
        g_mavlink.update(*g_ctx);

        const uint32_t now_ms = millis();
        if (now_ms - last_battery_ms >= kBatteryPeriodMs) {
            last_battery_ms = now_ms;
            g_battery.update();
        }

        // 10 Hz, matching MavlinkConfig::attitude_interval_ms's default so
        // the two are comparable at a glance in Mission Planner's Status
        // tab (Option 2 vs the ATTITUDE message's Option 1). See
        // docs/attitude-mahony-filter.md.
#if FC_ATTITUDE_ESTIMATOR_MAHONY_ENABLE
        if (now_ms - last_mahony_ms >= 100) {
            last_mahony_ms = now_ms;
            const fc::AttitudeMahonyData& mahony = g_attitudeMahony.data();
            g_mavlink.sendMahonyAttitude(mahony.roll_deg, mahony.pitch_deg, mahony.yaw_deg);
        }
#endif

        // 1 Hz: poll BNO055 calibration status, and the first time it goes
        // fully calibrated this session, save the offset registers so next
        // boot can restore them instead of re-converging from scratch (see
        // ImuCalibrationStorage.h). Only ever saves once per power cycle.
        if (now_ms - last_imu_calib_ms >= 1000) {
            last_imu_calib_ms = now_ms;
            g_imu.updateCalibration();
            if (FC_IMU_AUTO_SAVE_CALIBRATION_ENABLE && !imu_calibration_saved &&
                g_imu.isFullyCalibrated() && !g_radio.armed()) {
                fc::ImuCalibrationOffsets offsets{};
                if (g_imu.readCalibrationOffsets(offsets)) {
                    fc::ImuCalibrationStorage::save(offsets);
                    imu_calibration_saved = true;
                    Serial.println("[IMU] Fully calibrated -- offsets saved to EEPROM");
                }
            }
        }

        if ((xTaskGetTickCount() - last_wake) >= period) {
            last_wake = xTaskGetTickCount();  // task overran; skip catch-up
        }
        vTaskDelayUntil(&last_wake, period);
    }
}

}  // namespace

void setup()
{
    Serial.begin(kUsbBaud);
    Serial2.begin(kTelemetryBaud);
    Serial2.addMemoryForRead(g_dmaRx2, sizeof(g_dmaRx2));
    Serial2.addMemoryForWrite(g_dmaTx2, sizeof(g_dmaTx2));
    Serial7.begin(kCompanionBaud);
    Serial7.addMemoryForRead(g_dmaRx7, sizeof(g_dmaRx7));
    Serial7.addMemoryForWrite(g_dmaTx7, sizeof(g_dmaTx7));
    delay(100);
#if FC_DEBUG_SERIAL_ENABLE
    g_usbLogger.begin();
#endif

    // --- Parameters: register config-struct fields, then load from EEPROM
    // (or seed defaults if none present) BEFORE constructing anything that
    // reads these configs. ---
    g_params.initFixedWing(g_l1Config, g_fuzzyConfig, g_tecsConfig, g_attitudeConfig, g_imu);
    g_params.load();

    // Fuzzy scales the actual conventional L1 baseline loaded from EEPROM.
    g_fuzzyConfig.base_period_s = g_l1Config.period_s;

    // Keep Navigation/AttitudeController's airspeed envelope consistent
    // with the authoritative TecsConfig values Params just loaded -- see
    // docs/params.md for why these aren't registered as params twice.
    g_navigationConfig.airspeed_min_mps = g_tecsConfig.min_airspeed_mps;
    g_navigationConfig.airspeed_cruise_mps = g_tecsConfig.cruise_airspeed_mps;
    g_attitudeConfig.trim_airspeed_mps = g_tecsConfig.cruise_airspeed_mps;

    g_l1 = new fc::L1Controller(g_l1Config);
    g_fuzzyTuner = new fc::FuzzyL1Tuner(g_fuzzyConfig);
    g_fuzzyTuner->begin();
    g_tecs = new fc::Tecs(g_tecsConfig);
    g_navigation = new fc::Navigation(g_navigationConfig);
    g_attitude = new fc::AttitudeController(g_attitudeConfig);

    // --- Waypoints: load from EEPROM, or seed a minimal default mission. ---
    if (fc::Waypoints::load(g_navigation->state()) != fc::WaypointStorageResult::Success) {
        seedDefaultMission(g_navigation->state());
    }

    g_navigation->setWaypointReachedCallback(&waypointReachedTrampoline, &g_mavlink);

    // --- Vehicle context: bag of references every mode/Mavlink reads from. ---
    g_ctx = new fc::VehicleContext{
        g_imu, g_baro, g_airspeed, g_ahrs, *g_l1, *g_fuzzyTuner, *g_tecs, *g_navigation, *g_attitude,
        g_actuator, g_radio, g_battery, g_buzzer, g_modeManager, g_params,
#if FC_ATTITUDE_ESTIMATOR_MAHONY_ENABLE
        g_attitudeMahony,
#endif
    };

    // --- Modes: construct and register. ---
    g_modeManual = new fc::ModeManual(*g_ctx);
    g_modeFbwa = new fc::ModeFbwa(*g_ctx);
    g_modeAuto = new fc::ModeAuto(*g_ctx);
    g_modeGuided = new fc::ModeGuided(*g_ctx);
    g_modeLoiter = new fc::ModeLoiter(*g_ctx);
    g_modeManager.registerMode(g_modeManual);
    g_modeManager.registerMode(g_modeFbwa);
    g_modeManager.registerMode(g_modeAuto);
    g_modeManager.registerMode(g_modeGuided);
    g_modeManager.registerMode(g_modeLoiter);
    if (g_modeManager.setMode(fc::ModeId::Manual)) {
        g_activeModeId = fc::ModeId::Manual;
    }

    // --- Bring up hardware. ---
    g_buzzer.begin();  // Starts the startup tune; buzzerTask drives it forward.
    g_actuator.begin();
    g_battery.begin();

    if (!g_imu.begin()) {
        reportStatus(MAV_SEVERITY_ERROR, "[SETUP] IMU init failed -- check wiring/I2C address");
    } else {
        // Restore last session's accel/mag/gyro offsets, if any -- BNO055
        // has no non-volatile calibration storage of its own and otherwise
        // re-converges from scratch every boot (see docs/imu-bno055.md).
        if (FC_IMU_RESTORE_CALIBRATION_ENABLE) {
            fc::ImuCalibrationOffsets saved_imu_calibration{};
            if (fc::ImuCalibrationStorage::load(saved_imu_calibration) ==
                fc::ImuCalibrationStorageResult::Success) {
                if (g_imu.writeCalibrationOffsets(saved_imu_calibration)) {
                    reportStatus(MAV_SEVERITY_INFO, "[SETUP] IMU calibration restored from EEPROM");
                } else {
                    reportStatus(MAV_SEVERITY_WARNING, "[SETUP] IMU calibration restore failed");
                }
            }
        }
    }
    if (g_imu.initialized()) {
        // Confirms what's actually configured on the chip (FC_Config.h's
        // FC_BNO055_AXIS_MAP_CONFIG/SIGN_X/Y/Z), so a captured log is
        // self-describing when comparing axis-convention behavior across
        // firmware builds -- see docs/imu-bno055.md.
        char axis_line[96];
        snprintf(axis_line, sizeof(axis_line),
                 "[IMU] BNO055 axis_map=0x%02X sign_x=%u sign_y=%u sign_z=%u",
                 static_cast<unsigned>(FC_BNO055_AXIS_MAP_CONFIG),
                 static_cast<unsigned>(FC_BNO055_AXIS_SIGN_X),
                 static_cast<unsigned>(FC_BNO055_AXIS_SIGN_Y),
                 static_cast<unsigned>(FC_BNO055_AXIS_SIGN_Z));
        reportStatus(MAV_SEVERITY_INFO, axis_line);
    }
    if (!g_baro.begin()) {
        reportStatus(MAV_SEVERITY_ERROR, "[SETUP] Barometer init failed -- check wiring/I2C address");
    } else {
        // One-time PROM dump for diagnosing MS5611-vs-MS5607 chip identity,
        // see docs/barometer-ms5611.md.
        const fc::BarometerPromDump prom = g_baro.promDump();
        char prom_line[96];
        snprintf(prom_line, sizeof(prom_line), "[SETUP] Baro PROM: C1=%u C2=%u C3=%u C4=%u C5=%u C6=%u",
                 prom.c1_sens_t1, prom.c2_off_t1, prom.c3_tcs, prom.c4_tco, prom.c5_tref, prom.c6_tempsense);
        reportStatus(MAV_SEVERITY_INFO, prom_line);
    }
    if (!g_airspeed.begin()) {
        reportStatus(MAV_SEVERITY_ERROR, "[SETUP] Airspeed init failed -- check wiring/I2C address");
    }
    g_gnss.begin();

    // --- SD card logging + MTP (SD card browsable over USB, see
    // platformio.ini's build_flags comment for why this replaced
    // SerialUSB1). MTP.begin() before SD.begin() (inside g_sdLogger.begin()),
    // then addFilesystem() once the card is confirmed mounted -- matches
    // the MTP_Teensy library's own examples' ordering. Deliberately placed
    // BEFORE g_radio.begin()'s blocking safety interlock (2026-08-20): that
    // interlock can spin forever (while(armed_)/while(signal_lost_) with no
    // timeout, see Radio.cpp) if no RC transmitter is bound yet -- e.g.
    // during bench/USB-only testing. With MTP/SD brought up first, the SD
    // card mounts and the host file manager can browse/see its capacity
    // immediately, instead of the whole board looking like an unresponsive
    // empty MTP device (enumerated at the USB-descriptor level via
    // USB_MTPDISK_SERIAL, but never actually answering MTP requests) until
    // someone binds a transmitter. ---
    MTP.begin();
    if (!g_sdLogger.begin()) {
        // Was a buzzer beep pattern (6x trill) through 2026-08-19/20 -- never
        // confirmed audible on the bench, replaced by reportStatus()
        // (FC_DEBUG_SERIAL_ENABLE picks PuTTY-readable text vs GCS Messages
        // tab, see FC_Config.h), see docs/sd-logger-mtp.md.
        reportStatus(MAV_SEVERITY_ERROR, "[SETUP] SD card init failed -- check the card is inserted");
    } else {
        MTP.addFilesystem(SD, "SD Card");
        reportStatus(MAV_SEVERITY_INFO, "[SETUP] SD card + MTP ready");
    }

    // Pre-flight safety interlock: blocks until disarmed and a signal is
    // present. See docs/radio.md.
    g_radio.begin();

    // --- FreeRTOS tasks. Priorities match legacy main.cpp's relative
    // ordering (IMU highest, then control/mavlink/radio, then sensors,
    // buzzer lowest). ---
    xTaskCreate(taskImu, "IMU", 2048, nullptr, 6, &g_taskImu);
    xTaskCreate(taskControl, "Control", 8192, nullptr, 5, &g_taskControl);
    xTaskCreate(taskMavlink, "Mavlink", 8192, nullptr, 5, &g_taskMavlink);
    xTaskCreate(taskRadio, "Radio", 4096, nullptr, 5, &g_taskRadio);
    xTaskCreate(taskGps, "GPS", 4096, nullptr, 4, &g_taskGps);
    xTaskCreate(taskBaro, "BARO", 2048, nullptr, 3, &g_taskBaro);
    xTaskCreate(taskAirspeed, "Airspeed", 2048, nullptr, 3, &g_taskAirspeed);
    xTaskCreate(taskSdLog, "SdLog", 4096, nullptr, 2, &g_taskSdLog);
#if FC_DEBUG_SERIAL_ENABLE
    xTaskCreate(taskUsbLog, "UsbLog", 4096, nullptr, 1, &g_taskUsbLog);
#endif
    xTaskCreate(taskMtp, "Mtp", 4096, nullptr, 2, &g_taskMtp);
    xTaskCreate(taskBuzzer, "Buzzer", 2048, nullptr, 3, &g_taskBuzzer);

    vTaskStartScheduler();
}

void loop()
{
    // Intentionally empty: FreeRTOS scheduler owns execution once
    // vTaskStartScheduler() is called, matching legacy main.cpp.
}
