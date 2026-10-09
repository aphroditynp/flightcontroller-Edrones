#pragma once

#include <Arduino.h>
#include <SD.h>
#include <stdint.h>

namespace fc {

/**
 * CSV flight-data logger to the Teensy 4.1's built-in microSD slot
 * (BUILTIN_SDCARD). Companion to fc::DataLogger (which streams a DIFFERENT,
 * barometer-only CSV live over a dedicated USB CDC port) -- this one instead
 * writes to a file so it survives without a laptop connected, and is meant
 * to be pulled after the flight via MTP (see main.cpp's MTP.addFilesystem()
 * wiring, docs/sd-logger-mtp.md): plug the USB cable in, the SD card shows
 * up as a normal drive in the host's file manager, no card removal needed.
 *
 * One row per logRow() call: timestamp, attitude (roll/pitch/yaw), altitude,
 * flight mode, arm state, and the raw RC channels 1-5 -- the fields asked
 * for when this was added (2026-08-19). mahony_roll/pitch/yaw_deg (2026-08-20)
 * log fc::AttitudeMahonyFilter's estimate alongside the BNO055 on-chip one
 * (roll/pitch/yaw_deg above) so the two can be compared post-flight -- see
 * docs/attitude-mahony-filter.md.
 */
class SdLogger final {
public:
    /**
     * Initializes the SD card, picks the next unused LOGnnn.CSV filename (so
     * each boot gets its own file instead of overwriting the last flight's
     * log), opens it, and writes the CSV header row. Safe to call again
     * after a failed begin() (e.g. retry after inserting a card) -- it will
     * just retry SD.begin().
     */
    bool begin();

    /**
     * Appends one CSV row every call, but only flushes to the card at most
     * once per kFlushIntervalMs (throttled internally via millis()) --
     * println() itself just fills SdFat's in-RAM write buffer, cheap and
     * fast, but flush() forces an actual card write, and microSD cards are
     * notorious for occasionally stalling tens to hundreds of ms on a write
     * (internal wear-leveling/garbage collection, not a bug in this code).
     * Bench-diagnosed 2026-08-20: flushing every row at this logger's ~20 Hz
     * call rate caused exactly that -- intermittent RC input "lag"/"lock"
     * during ground testing, tracked down to this call, not the radio or
     * control-law code. Throttling to ~1 Hz trades a worst-case loss of the
     * last ~1s of buffered-but-unflushed rows on a mid-flight power cut for
     * removing 19 out of 20 chances per second of that stall. No-ops if
     * begin() never succeeded.
     */
    void logRow(float roll_deg, float pitch_deg, float yaw_deg, float altitude_m,
               const char* mode_code4, bool armed, uint16_t ch1_roll, uint16_t ch2_pitch,
               uint16_t ch3_throttle, uint16_t ch4_yaw, uint16_t ch5_arm_raw, float mahony_roll_deg,
               float mahony_pitch_deg, float mahony_yaw_deg, float radial_error_m,
               float target_radius_m, float l1_period_s);

    bool isOpen() const;

private:
    static constexpr uint32_t kFlushIntervalMs = 1000;

    File file_{};
    bool open_ = false;
    uint32_t last_flush_ms_ = 0;
};

}  // namespace fc
