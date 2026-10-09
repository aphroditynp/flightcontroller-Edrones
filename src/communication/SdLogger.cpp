#include "communication/SdLogger.h"

#include <stdio.h>

namespace fc {

bool SdLogger::begin()
{
    if (!SD.begin(BUILTIN_SDCARD)) {
        open_ = false;
        return false;
    }

    // Next unused LOGnnn.CSV, 001-999, so every boot gets its own file
    // instead of appending onto (or overwriting) a previous flight's data.
    char name[16];
    uint16_t index = 1;
    for (; index <= 999; ++index) {
        snprintf(name, sizeof(name), "LOG%03u.CSV", index);
        if (!SD.exists(name)) {
            break;
        }
    }
    if (index > 999) {
        open_ = false;
        return false;
    }

    file_ = SD.open(name, FILE_WRITE);
    if (!file_) {
        open_ = false;
        return false;
    }

    file_.println("timestamp_ms,roll_deg,pitch_deg,yaw_deg,altitude_m,mode,armed,"
                  "ch1_roll,ch2_pitch,ch3_throttle,ch4_yaw,ch5_arm_raw,"
                  "mahony_roll_deg,mahony_pitch_deg,mahony_yaw_deg,"
                  "radial_error_m,target_radius_m,l1_period_s");
    file_.flush();
    last_flush_ms_ = millis();
    open_ = true;
    return true;
}

void SdLogger::logRow(float roll_deg, float pitch_deg, float yaw_deg, float altitude_m,
                      const char* mode_code4, bool armed, uint16_t ch1_roll, uint16_t ch2_pitch,
                      uint16_t ch3_throttle, uint16_t ch4_yaw, uint16_t ch5_arm_raw,
                      float mahony_roll_deg, float mahony_pitch_deg, float mahony_yaw_deg,
                      float radial_error_m, float target_radius_m, float l1_period_s)
{
    if (!open_) {
        return;
    }

    char line[260];
    snprintf(line, sizeof(line), "%lu,%.2f,%.2f,%.2f,%.2f,%s,%d,%u,%u,%u,%u,%u,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f",
             static_cast<unsigned long>(millis()), static_cast<double>(roll_deg),
             static_cast<double>(pitch_deg), static_cast<double>(yaw_deg),
             static_cast<double>(altitude_m), mode_code4, armed ? 1 : 0, ch1_roll, ch2_pitch,
             ch3_throttle, ch4_yaw, ch5_arm_raw, static_cast<double>(mahony_roll_deg),
             static_cast<double>(mahony_pitch_deg), static_cast<double>(mahony_yaw_deg),
             static_cast<double>(radial_error_m), static_cast<double>(target_radius_m),
             static_cast<double>(l1_period_s));
    file_.println(line);

    const uint32_t now_ms = millis();
    if (now_ms - last_flush_ms_ >= kFlushIntervalMs) {
        file_.flush();
        last_flush_ms_ = now_ms;
    }
}

bool SdLogger::isOpen() const
{
    return open_;
}

}  // namespace fc
