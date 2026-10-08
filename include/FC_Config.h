#pragma once

// ============================================================
// KHAGESWARA-FC-2026 (fixed-wing) build-time configuration.
//
// Centralizes the constants that used to live as separate constexpr
// definitions inside src/main.cpp's anonymous namespace (task periods,
// serial baud rates, and the boot/status diagnostics output selector) --
// one place to retune them instead of hunting through main.cpp. See
// docs/main-integration.md for the task-layout rationale and
// docs/sd-logger-mtp.md for the status-output selector's rationale.
// ============================================================

#include <stdint.h>

// ---------------- Debug/status output over USB Serial ----------------
// The Teensy 4.1 build only gives this firmware ONE USB Serial port (see
// platformio.ini's build_flags comment -- no USB personality here has a
// second CDC alongside MTP), and that same port is also where fc::Mavlink's
// GCS link lives. Interleaving plain-text status prints with MAVLink's
// binary stream makes both unreadable, so exactly one of them gets the
// port -- a compile-time #define, like FC_ENABLE_*/MAVLINK_*_ENABLE_* in
// the legacy config header, not a runtime prompt.
//
// 1: USB Serial carries plain text -- open a PuTTY (or any serial
//    terminal) session at kUsbBaud and read boot/status diagnostics
//    directly. MAVLINK_USB_ENABLE_RX/TX below are forced to 0 so MAVLink
//    can't read or write that same port (Telemetry/Companion still work).
// 0: USB Serial carries MAVLink as normal (Mission Planner/QGC connect
//    here) -- MAVLINK_USB_ENABLE_RX/TX are forced to 1. Boot/status
//    diagnostics instead go out as MAVLink STATUSTEXT (GCS Messages tab)
//    -- see fc::reportStatus() in main.cpp.
//
// Flip this one line and reflash to switch. See docs/sd-logger-mtp.md.
#ifndef FC_DEBUG_SERIAL_ENABLE
#define FC_DEBUG_SERIAL_ENABLE 0
#endif

// ---------------- Automatic LOITER after AUTO ----------------
// 1: after the final AUTO waypoint, switch to LOITER around that waypoint.
// 0: keep the active mode as AUTO when the mission finishes. LOITER can still
// be selected explicitly through MAVLink custom_mode=12.
//
// Keep this disabled while validating AUTO so a completed mission cannot
// change the flight mode unexpectedly.
#ifndef FC_AUTO_LOITER_ENABLE
#define FC_AUTO_LOITER_ENABLE 0
#endif

#if FC_DEBUG_SERIAL_ENABLE
#define MAVLINK_USB_ENABLE_RX 0
#define MAVLINK_USB_ENABLE_TX 0
#else
#define MAVLINK_USB_ENABLE_RX 1
#define MAVLINK_USB_ENABLE_TX 1
#endif

// ---------------- Attitude estimator selection ----------------
// Two attitude sources exist: BNO055's on-chip NDOF fusion (Option 1,
// always computed -- Imu.cpp reads its Euler registers regardless of these
// flags) and fc::AttitudeMahonyFilter (Option 2, gyro+accel+mag processed
// by this firmware -- see docs/attitude-mahony-filter.md). Two separate
// flags because "build it in" and "fly on it" are different levels of
// commitment:
//
// FC_ATTITUDE_ESTIMATOR_MAHONY_ENABLE: compile fc::AttitudeMahonyFilter in
// at all. 0 excludes it entirely (no object, no update() call in taskImu,
// no SD log columns, no MHN_* MAVLink telemetry) -- smallest/leanest build
// for a production flight where the comparison isn't needed. 1 (default,
// matches this feature's original purpose) builds and runs it in parallel
// at 200 Hz, purely logged/telemetered for comparison -- see the block this
// guards in main.cpp and SdLogger.cpp.
#ifndef FC_ATTITUDE_ESTIMATOR_MAHONY_ENABLE
#define FC_ATTITUDE_ESTIMATOR_MAHONY_ENABLE 1
#endif

// FC_ATTITUDE_CONTROL_SOURCE_MAHONY: make AttitudeController actually fly
// on Mahony's roll/pitch (VehicleContext::controlImu(), consumed by the
// three ctx_.attitude.update() call sites in FixedWingModes.cpp) instead
// of BNO055's. Requires FC_ATTITUDE_ESTIMATOR_MAHONY_ENABLE=1 (enforced by
// the #error below). DEFAULT OFF, and stay off until the bench verification
// procedure in docs/attitude-mahony-filter.md has actually been done --
// Mahony's raw gyro/accel/mag axis convention is unverified as of
// 2026-08-20, so flipping this before that flies the aircraft on data with
// the same class of axis-convention bug already found (and fixed) twice
// this session for other BNO055 code paths. Navigation/TECS/Ahrs are NOT
// affected by this flag -- they keep reading BNO055 (Option 1) regardless,
// only the attitude-control feedback loop itself is switched.
#ifndef FC_ATTITUDE_CONTROL_SOURCE_MAHONY
#define FC_ATTITUDE_CONTROL_SOURCE_MAHONY 0
#endif

#if FC_ATTITUDE_CONTROL_SOURCE_MAHONY && !FC_ATTITUDE_ESTIMATOR_MAHONY_ENABLE
#error "FC_ATTITUDE_CONTROL_SOURCE_MAHONY requires FC_ATTITUDE_ESTIMATOR_MAHONY_ENABLE=1"
#endif

// BNO055 calibration offsets can become orientation-dependent when a bad
// magnetometer snapshot is restored at every boot. Keep restore/auto-save
// disabled until a clean calibration has been verified away from motors,
// ESCs, high-current wiring, steel tables, and reinforced concrete.
#ifndef FC_IMU_RESTORE_CALIBRATION_ENABLE
#define FC_IMU_RESTORE_CALIBRATION_ENABLE 0
#endif

#ifndef FC_IMU_AUTO_SAVE_CALIBRATION_ENABLE
#define FC_IMU_AUTO_SAVE_CALIBRATION_ENABLE 0
#endif

// ---------------- BNO055 axis remap ----------------
// Configures the chip's own AXIS_MAP_CONFIG/AXIS_MAP_SIGN registers
// (applied in Imu::configureSensor()) -- this affects ALL of BNO055's
// output (raw accel/gyro/mag registers AND the on-chip Euler/quaternion
// fusion), unlike ImuConfig's invert_pitch/invert_gyro_y/roll_trim_deg/
// pitch_trim_deg which only patch the values after they leave the chip.
// Default is DEFAULT_AXIS with no sign inversion -- i.e. unchanged factory
// P1 orientation, matching this firmware's behavior before this flag
// existed. See docs/imu-bno055.md for the axis-convention bugs found this
// session and why remapping at the chip is the more correct fix than
// patching afterward.
//
// Valid AXIS_MAP_CONFIG values (from BNO055.h):
// DEFAULT_AXIS        0x24: X=X, Y=Y, Z=Z
// REMAP_X_Y           0x21: X=Y, Y=X, Z=Z
// REMAP_Y_Z           0x18: X=X, Y=Z, Z=Y
// REMAP_Z_X           0x06: X=Z, Y=Y, Z=X
// REMAP_X_Y_Z_TYPE0   0x12: X=Z, Y=X, Z=Y
// REMAP_X_Y_Z_TYPE1   0x09: X=Y, Y=Z, Z=X
//
// Sign values: 0 = normal, 1 = inverted for that remapped output axis.
#ifndef FC_BNO055_AXIS_MAP_CONFIG
#define FC_BNO055_AXIS_MAP_CONFIG DEFAULT_AXIS
#endif

#ifndef FC_BNO055_AXIS_SIGN_X
#define FC_BNO055_AXIS_SIGN_X 0
#endif

#ifndef FC_BNO055_AXIS_SIGN_Y
#define FC_BNO055_AXIS_SIGN_Y 0
#endif

#ifndef FC_BNO055_AXIS_SIGN_Z
#define FC_BNO055_AXIS_SIGN_Z 0
#endif

// ---------------- BNO055 I2C clock ----------------
// 400000 (Fast Mode) is BNO055's max supported I2C clock and this
// firmware's default. Drop to 100000 (Standard Mode) as a troubleshooting
// step if magnetometer readings show sign-flipping noise between
// consecutive samples with implausible magnitude (hundreds of uT, Earth's
// field is ~25-65 uT) -- that pattern looks like I2C bit errors (wiring
// length/quality, pull-up resistors, EMI pickup on SDA/SCL) rather than a
// real external magnetic source, since real motion past a magnet produces
// smooth changes, not sample-to-sample sign flips. See
// docs/imu-bno055.md's 2026-08-22 log analysis entries.
#ifndef FC_BNO055_I2C_CLOCK_HZ
#define FC_BNO055_I2C_CLOCK_HZ 100000
#endif

namespace fc {

// ---------------- Serial baud rates ----------------
// Matches legacy wiring: USB=GCS/debug, Serial2=telemetry radio,
// Serial7=companion computer (RPi). Serial8 (SBUS receiver) is fixed-baud
// and configured inside fc::Radio itself, not listed here.
constexpr uint32_t kUsbBaud = 115200;
constexpr uint32_t kTelemetryBaud = 57600;
constexpr uint32_t kCompanionBaud = 57600;

// ---------------- FreeRTOS task periods (ms) ----------------
constexpr uint32_t kImuPeriodMs = 5;        // 200 Hz
constexpr uint32_t kControlPeriodMs = 5;    // 200 Hz, synced with IMU
constexpr uint32_t kBaroPeriodMs = 10;      // 100 Hz
constexpr uint32_t kGpsPeriodMs = 10;       // 100 Hz
constexpr uint32_t kAirspeedPeriodMs = 50;  // 20 Hz
constexpr uint32_t kRadioPeriodMs = 10;     // 100 Hz
constexpr uint32_t kBuzzerPeriodMs = 10;    // 100 Hz
// 50 Hz -- legacy used vTaskDelayUntil at ~2ms; telemetry itself is
// internally rate-limited by Mavlink::update(), so this only bounds
// parse/dispatch latency.
constexpr uint32_t kMavlinkPeriodMs = 20;
constexpr uint32_t kBatteryPeriodMs = 100;  // 10 Hz
// 20 Hz: enough resolution to reconstruct a flight's attitude/mode/RC
// history for post-flight review without generating an excessive CSV file
// over a multi-minute flight. Independent of kImuPeriodMs/kBaroPeriodMs
// (200/100 Hz sampling into their own filters) -- this only throttles how
// often a logged snapshot is written to the SD card.
constexpr uint32_t kSdLogPeriodMs = 50;
// MTP.loop() just polls for pending USB MTP transactions from the host file
// manager; cheap when idle, so a short period keeps file browsing/copying
// responsive without needing its own high-priority task.
constexpr uint32_t kMtpPeriodMs = 10;
// USB PuTTY/serial-terminal CSV logging (FC_DEBUG_SERIAL_ENABLE=1 only, see
// above) -- 10 Hz keeps the text stream readable without flooding it or
// competing with the 200 Hz IMU/control tasks for CPU. See
// docs/data-logger-usb.md.
constexpr uint32_t kUsbLogPeriodMs = 100;

constexpr float kDegToRad = 3.14159265358979323846f / 180.0f;

}  // namespace fc
