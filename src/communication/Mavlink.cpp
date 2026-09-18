#include "communication/Mavlink.h"

#include <math.h>
#include <string.h>

#include "FC_Config.h"
#include "storage/ImuCalibrationStorage.h"
#include "storage/Waypoints.h"

namespace {

bool isMissionMessage(uint32_t msgid)
{
    switch (msgid) {
        case MAVLINK_MSG_ID_MISSION_COUNT:
        case MAVLINK_MSG_ID_MISSION_ITEM:
        case MAVLINK_MSG_ID_MISSION_ITEM_INT:
        case MAVLINK_MSG_ID_MISSION_REQUEST:
        case MAVLINK_MSG_ID_MISSION_REQUEST_INT:
        case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
        case MAVLINK_MSG_ID_MISSION_ACK:
        case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:
            return true;
        default:
            return false;
    }
}

}  // namespace

namespace fc {

Mavlink::Mavlink(const MavlinkConfig& config)
    : config_(config)
{
}

void Mavlink::mavWrite(HardwareSerial& port, const mavlink_message_t& msg)
{
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    const uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    port.write(buf, len);
}

void Mavlink::mavWriteUsb(const mavlink_message_t& msg)
{
#if MAVLINK_USB_ENABLE_TX
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    const uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial.write(buf, len);
#else
    (void)msg;  // FC_DEBUG_SERIAL_ENABLE has USB Serial carrying plain text instead -- see FC_Config.h.
#endif
}

void Mavlink::mavSendAll(const mavlink_message_t& msg)
{
    mavWriteUsb(msg);
    mavWrite(Serial2, msg);
    if (mission_upload_state_ != MissionUploadState::Receiving || isMissionMessage(msg.msgid)) {
        mavWrite(Serial7, msg);
    }
}

void Mavlink::replyToSender(const mavlink_message_t& msg)
{
    switch (current_port_) {
        case MavlinkPort::Usb: mavWriteUsb(msg); break;
        case MavlinkPort::Telemetry: mavWrite(Serial2, msg); break;
        case MavlinkPort::Companion: mavWrite(Serial7, msg); break;
    }
}

void Mavlink::handlePorts(VehicleContext& ctx)
{
#if MAVLINK_USB_ENABLE_RX
    // FC_DEBUG_SERIAL_ENABLE==0 (FC_Config.h) required for this: otherwise
    // Serial carries plain-text debug output instead, and read()ing MAVLink
    // bytes here would just be reading and discarding that text stream.
    while (Serial.available()) {
        const uint8_t c = Serial.read();
        if (mavlink_parse_char(MAVLINK_COMM_0, c, &rx_usb_, &status_usb_)) {
            processIncoming(ctx, rx_usb_, MavlinkPort::Usb);
            forwardToOthers(rx_usb_, MavlinkPort::Usb);
        }
    }
#endif

    while (Serial2.available()) {
        const uint8_t c = Serial2.read();
        if (mavlink_parse_char(MAVLINK_COMM_1, c, &rx_tlm_, &status_tlm_)) {
            processIncoming(ctx, rx_tlm_, MavlinkPort::Telemetry);
            forwardToOthers(rx_tlm_, MavlinkPort::Telemetry);
        }
    }

    while (Serial7.available()) {
        const uint8_t c = Serial7.read();
        if (mavlink_parse_char(MAVLINK_COMM_2, c, &rx_rpi_, &status_rpi_)) {
            processIncoming(ctx, rx_rpi_, MavlinkPort::Companion);
            forwardToOthers(rx_rpi_, MavlinkPort::Companion);
        }
    }
}

void Mavlink::processIncoming(VehicleContext& ctx, const mavlink_message_t& msg, MavlinkPort from)
{
    current_port_ = from;

    switch (msg.msgid) {
        case MAVLINK_MSG_ID_COMMAND_LONG: handleCommandLong(ctx, msg); break;
        case MAVLINK_MSG_ID_STATUSTEXT: handleStatusText(ctx, msg); break;
        case MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED: handleSetPositionTargetLocalNed(ctx, msg); break;
        case MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT: handleSetPositionTargetGlobalInt(ctx, msg); break;
        case MAVLINK_MSG_ID_MISSION_REQUEST_LIST: handleMissionRequestList(ctx, msg); break;
        case MAVLINK_MSG_ID_MISSION_REQUEST: handleMissionRequest(ctx, msg); break;
        case MAVLINK_MSG_ID_MISSION_REQUEST_INT: handleMissionRequestInt(ctx, msg); break;
        case MAVLINK_MSG_ID_MISSION_COUNT: handleMissionCount(msg); break;
        case MAVLINK_MSG_ID_MISSION_ITEM_INT: handleMissionItemInt(ctx, msg); break;
        case MAVLINK_MSG_ID_MISSION_ITEM: handleMissionItem(ctx, msg); break;
        case MAVLINK_MSG_ID_MISSION_CLEAR_ALL: handleMissionClearAll(ctx, msg); break;
        case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: handleParamRequestList(ctx); break;
        case MAVLINK_MSG_ID_PARAM_REQUEST_READ: handleParamRequestRead(ctx, msg); break;
        case MAVLINK_MSG_ID_PARAM_SET: handleParamSet(ctx, msg); break;
        default: break;
    }
}

void Mavlink::forwardToOthers(const mavlink_message_t& msg, MavlinkPort from)
{
    switch (msg.msgid) {
        case MAVLINK_MSG_ID_MISSION_COUNT:
        case MAVLINK_MSG_ID_MISSION_ITEM:
        case MAVLINK_MSG_ID_MISSION_ITEM_INT:
        case MAVLINK_MSG_ID_MISSION_REQUEST:
        case MAVLINK_MSG_ID_MISSION_REQUEST_INT:
        case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
        case MAVLINK_MSG_ID_MISSION_ACK:
        case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:
        case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
        case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
        case MAVLINK_MSG_ID_PARAM_SET:
        case MAVLINK_MSG_ID_PARAM_VALUE:
            return;
        default:
            break;
    }

    if (from != MavlinkPort::Telemetry) mavWrite(Serial2, msg);
    if (from != MavlinkPort::Companion) mavWrite(Serial7, msg);
}

void Mavlink::sendCommandAck(uint16_t command, uint8_t result)
{
    mavlink_message_t msg;
    mavlink_msg_command_ack_pack(config_.system_id, config_.component_id, &msg, command, result, 0, 0, 0, 0);
    replyToSender(msg);
}

void Mavlink::sendStatusText(uint8_t severity, const char* text)
{
    mavlink_message_t msg;
    mavlink_msg_statustext_pack(config_.system_id, config_.component_id, &msg, severity, text);
    mavSendAll(msg);
}

void Mavlink::sendNamedValueFloat(const char* name, float value)
{
    mavlink_message_t msg;
    mavlink_msg_named_value_float_pack(config_.system_id, config_.component_id, &msg, millis(), name, value);
    mavSendAll(msg);
}

void Mavlink::sendMahonyAttitude(float roll_deg, float pitch_deg, float yaw_deg)
{
    sendNamedValueFloat("MHN_ROLL", roll_deg);
    sendNamedValueFloat("MHN_PITCH", pitch_deg);
    sendNamedValueFloat("MHN_YAW", yaw_deg);
}

void Mavlink::handleStatusText(VehicleContext& ctx, const mavlink_message_t& msg)
{
    mavlink_statustext_t status;
    mavlink_msg_statustext_decode(&msg, &status);

    if (msg.sysid == config_.system_id && msg.compid == config_.component_id) {
        return;
    }

    if (strncmp(status.text, "DROP", 4) == 0) {
        ctx.payload_drop_command = true;
        sendStatusText(MAV_SEVERITY_INFO, "PAYLOAD DROP TRIGGERED");
    }
}

uint8_t Mavlink::baseModeFor(ModeId id, bool armed) const
{
    uint8_t base = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    switch (id) {
        case ModeId::Manual:
            base |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
            break;
        case ModeId::Fbwa:
            base |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED | MAV_MODE_FLAG_STABILIZE_ENABLED;
            break;
        case ModeId::Guided:
            base |= MAV_MODE_FLAG_GUIDED_ENABLED | MAV_MODE_FLAG_STABILIZE_ENABLED;
            break;
        case ModeId::Auto:
            base |= MAV_MODE_FLAG_AUTO_ENABLED | MAV_MODE_FLAG_STABILIZE_ENABLED;
            break;
        case ModeId::Loiter:
            base |= MAV_MODE_FLAG_AUTO_ENABLED | MAV_MODE_FLAG_STABILIZE_ENABLED;
            break;
        default:
            base |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
            break;
    }
    if (armed) {
        base |= MAV_MODE_FLAG_SAFETY_ARMED;
    }
    return base;
}

uint32_t Mavlink::customModeFor(ModeId id) const
{
    // ArduPlane-compatible custom_mode codes, so GCS mode dropdowns show
    // sensible names: 0=MANUAL, 5=FBWA, 10=AUTO, 4=GUIDED.
    switch (id) {
        case ModeId::Manual: return 0;
        case ModeId::Fbwa: return 5;
        case ModeId::Auto: return 10;
        case ModeId::Guided: return 4;
        case ModeId::Loiter: return 12;
        default: return 0;
    }
}

void Mavlink::handleCommandLong(VehicleContext& ctx, const mavlink_message_t& msg)
{
    mavlink_command_long_t cmd;
    mavlink_msg_command_long_decode(&msg, &cmd);

    if (cmd.target_system != config_.system_id && cmd.target_system != 0) {
        return;
    }

    uint8_t result = MAV_RESULT_UNSUPPORTED;

    switch (cmd.command) {
        case MAV_CMD_COMPONENT_ARM_DISARM: {
            // Arming is RC-only in this build -- see the class-level comment
            // in Mavlink.h. Report unsupported rather than silently no-op,
            // so GCS operators see why nothing happened.
            result = MAV_RESULT_UNSUPPORTED;
            sendStatusText(MAV_SEVERITY_WARNING, "ARM/DISARM: RC transmitter only, not MAVLink");
            break;
        }

        case MAV_CMD_DO_SET_MODE: {
            const uint32_t custom_mode = static_cast<uint32_t>(cmd.param2);
            ModeId new_mode_id;
            const char* mode_name = "UNKNOWN";

            if (custom_mode == 0) { new_mode_id = ModeId::Manual; mode_name = "MANUAL"; }
            else if (custom_mode == 5) { new_mode_id = ModeId::Fbwa; mode_name = "FBWA"; }
            else if (custom_mode == 10) { new_mode_id = ModeId::Auto; mode_name = "AUTO"; }
            else if (custom_mode == 4) { new_mode_id = ModeId::Guided; mode_name = "GUIDED"; }
            else if (custom_mode == 12) { new_mode_id = ModeId::Loiter; mode_name = "LOITER"; }
            else {
                sendStatusText(MAV_SEVERITY_WARNING, "MODE NOT SUPPORTED");
                sendCommandAck(cmd.command, MAV_RESULT_UNSUPPORTED);
                return;
            }

            if (ctx.mode_manager.setMode(new_mode_id)) {
                result = MAV_RESULT_ACCEPTED;
                char buf[32];
                snprintf(buf, sizeof(buf), "MODE: %s", mode_name);
                sendStatusText(MAV_SEVERITY_INFO, buf);
                sendHeartbeat(ctx);  // Immediate heartbeat so GCS mode display updates without delay.
                last_heartbeat_ms_ = millis();
            } else {
                result = MAV_RESULT_FAILED;
                sendStatusText(MAV_SEVERITY_WARNING, "MODE SET FAILED");
            }
            break;
        }

        case MAV_CMD_PREFLIGHT_CALIBRATION: {
            // param5 (accelerometer): 1=full 6-position, 2=board level/trim,
            // 4=simple 1-position -- per MAV_CMD_PREFLIGHT_CALIBRATION's
            // PREFLIGHT_CALIBRATION_ACCELEROMETER enum. Mission Planner's
            // "Calibrate Level" button sends param5=2. Only that one is
            // implemented so far: it needs just one MAVLink round trip and
            // maps directly onto Imu's existing roll_trim_deg/pitch_trim_deg
            // (see docs/imu-bno055.md). "Calibrate Accel" (param5=1, needs a
            // MAV_CMD_ACCELCAL_VEHICLE_POS handshake walking 6 positions) and
            // "Simple Accel Cal" (param5=4, writes BNO055's own accel offset
            // registers) are not implemented yet.
            if (cmd.param5 == 2.0f) {
                if (ctx.radio.armed()) {
                    result = MAV_RESULT_DENIED;
                    sendStatusText(MAV_SEVERITY_WARNING, "LEVEL CAL: disarm first");
                    break;
                }

                const ImuData& imu_data = ctx.imu.data();
                if (!imu_data.valid) {
                    result = MAV_RESULT_TEMPORARILY_REJECTED;
                    sendStatusText(MAV_SEVERITY_WARNING, "LEVEL CAL: no IMU data yet");
                    break;
                }

                // Current roll_deg/pitch_deg already have the old trim
                // subtracted, so adding the residual to the old trim makes
                // the next reading land on zero. See Imu::readData().
                ctx.imu.rollTrimDegRef() += imu_data.roll_deg;
                ctx.imu.pitchTrimDegRef() += imu_data.pitch_deg;
                ctx.params.save();

                result = MAV_RESULT_ACCEPTED;
                char buf[50];
                snprintf(buf, sizeof(buf), "LEVEL CAL: trim R%.1f P%.1f", ctx.imu.rollTrimDegRef(),
                         ctx.imu.pitchTrimDegRef());
                sendStatusText(MAV_SEVERITY_INFO, buf);
                break;
            }

            if (cmd.param5 == 1.0f) {
                // TEMPORARY repurposing of the "Calibrate Accel" button,
                // pending the real 6-position flow (see docs/imu-bno055.md).
                // A bad accel/mag/gyro offset snapshot in
                // storage/ImuCalibrationStorage.h -- captured once, then
                // force-written into the chip on every boot -- can corrupt
                // BNO055's own on-chip fusion badly enough to produce
                // cross-axis coupling (roll/pitch moving during a pure yaw)
                // even through the Euler registers, not just our own
                // quaternion math. This clears the saved snapshot AND
                // zeroes the chip's live offset registers immediately, so
                // the next re-calibration starts clean. See
                // docs/imu-bno055.md's entry for this incident.
                if (ctx.radio.armed()) {
                    result = MAV_RESULT_DENIED;
                    sendStatusText(MAV_SEVERITY_WARNING, "IMU CAL CLEAR: disarm first");
                    break;
                }

                ImuCalibrationStorage::clear();
                const ImuCalibrationOffsets zero_offsets{};
                if (ctx.imu.writeCalibrationOffsets(zero_offsets)) {
                    result = MAV_RESULT_ACCEPTED;
                    sendStatusText(MAV_SEVERITY_INFO, "IMU CAL CLEARED - power-cycle, re-test level");
                } else {
                    result = MAV_RESULT_FAILED;
                    sendStatusText(MAV_SEVERITY_ERROR, "IMU CAL CLEAR: chip write failed");
                }
                break;
            }

            if (cmd.param5 == 4.0f) {
                result = MAV_RESULT_UNSUPPORTED;
                sendStatusText(MAV_SEVERITY_WARNING, "ACCEL CAL: not implemented, use Calibrate Level");
                break;
            }

            // Ground-pressure (baro) calibration -- Mission Planner's
            // "Calibrate Baro" (HUD right-click) and the equivalent QGC
            // action send param3=1. Other sensors (gyro/mag/radio/ESC) have
            // no calibration routine in this build.
            if (cmd.param3 != 1.0f) {
                result = MAV_RESULT_UNSUPPORTED;
                sendStatusText(MAV_SEVERITY_WARNING, "CAL: only baro/level supported");
                break;
            }

            if (ctx.radio.armed()) {
                result = MAV_RESULT_DENIED;
                sendStatusText(MAV_SEVERITY_WARNING, "BARO CAL: disarm first");
                break;
            }

            if (ctx.baro.recalibrateGroundPressure()) {
                result = MAV_RESULT_ACCEPTED;
                sendStatusText(MAV_SEVERITY_INFO, "BARO CAL: ground pressure re-zeroed");
            } else {
                result = MAV_RESULT_FAILED;
                sendStatusText(MAV_SEVERITY_ERROR, "BARO CAL: sensor read failed");
            }
            break;
        }

        default:
            result = MAV_RESULT_UNSUPPORTED;
            break;
    }

    sendCommandAck(cmd.command, result);
}

void Mavlink::handleMissionRequestList(VehicleContext& ctx, const mavlink_message_t& msg)
{
    mavlink_message_t reply;
    mavlink_msg_mission_count_pack(config_.system_id, config_.component_id, &reply, msg.sysid, msg.compid,
                                   ctx.navigation.state().wp_sum, MAV_MISSION_TYPE_MISSION);
    replyToSender(reply);
}

void Mavlink::handleMissionRequest(VehicleContext& ctx, const mavlink_message_t& msg)
{
    mavlink_mission_request_t request;
    mavlink_msg_mission_request_decode(&msg, &request);
    MissionState& mission = ctx.navigation.state();
    if (request.seq >= mission.wp_sum) {
        return;
    }

    mavlink_message_t reply;
    mavlink_msg_mission_item_int_pack(config_.system_id, config_.component_id, &reply, msg.sysid, msg.compid,
                                      request.seq, MAV_FRAME_GLOBAL_RELATIVE_ALT, MAV_CMD_NAV_WAYPOINT, 0, 1, 0,
                                      0, 0, 0, mission.waypoint[request.seq].lat, mission.waypoint[request.seq].lng,
                                      mission.waypoint[request.seq].alt / 100.0f, MAV_MISSION_TYPE_MISSION);
    replyToSender(reply);

    if (request.seq == mission.wp_sum - 1) {
        mavlink_message_t ack;
        mavlink_msg_mission_ack_pack(config_.system_id, config_.component_id, &ack, msg.sysid, msg.compid,
                                     MAV_MISSION_ACCEPTED, MAV_MISSION_TYPE_MISSION);
        replyToSender(ack);
    }
}

void Mavlink::handleMissionRequestInt(VehicleContext& ctx, const mavlink_message_t& msg)
{
    mavlink_mission_request_int_t request;
    mavlink_msg_mission_request_int_decode(&msg, &request);
    MissionState& mission = ctx.navigation.state();
    if (request.seq >= mission.wp_sum) {
        return;
    }

    mavlink_message_t reply;
    mavlink_msg_mission_item_int_pack(config_.system_id, config_.component_id, &reply, msg.sysid, msg.compid,
                                      request.seq, MAV_FRAME_GLOBAL_RELATIVE_ALT, MAV_CMD_NAV_WAYPOINT, 0, 1, 0,
                                      0, 0, 0, mission.waypoint[request.seq].lat, mission.waypoint[request.seq].lng,
                                      mission.waypoint[request.seq].alt / 100.0f, MAV_MISSION_TYPE_MISSION);
    replyToSender(reply);

    if (request.seq == mission.wp_sum - 1) {
        mavlink_message_t ack;
        mavlink_msg_mission_ack_pack(config_.system_id, config_.component_id, &ack, msg.sysid, msg.compid,
                                     MAV_MISSION_ACCEPTED, MAV_MISSION_TYPE_MISSION);
        replyToSender(ack);
    }
}

void Mavlink::handleMissionCount(const mavlink_message_t& msg)
{
    mavlink_mission_count_t count_msg;
    mavlink_msg_mission_count_decode(&msg, &count_msg);
    expected_mission_count_ = count_msg.count;

    if (expected_mission_count_ > MavlinkConfig::kMaxMissionItems) {
        mavlink_message_t ack;
        mavlink_msg_mission_ack_pack(config_.system_id, config_.component_id, &ack, msg.sysid, msg.compid,
                                     MAV_MISSION_ERROR, MAV_MISSION_TYPE_MISSION);
        replyToSender(ack);
        sendStatusText(MAV_SEVERITY_WARNING, "Mission count too large");
        return;
    }

    mission_upload_state_ = MissionUploadState::Receiving;
    current_upload_seq_ = 0;

    mavlink_message_t req;
    mavlink_msg_mission_request_int_pack(config_.system_id, config_.component_id, &req, msg.sysid, msg.compid, 0,
                                         MAV_MISSION_TYPE_MISSION);
    replyToSender(req);
}

void Mavlink::handleMissionItemInt(VehicleContext& ctx, const mavlink_message_t& msg)
{
    if (mission_upload_state_ != MissionUploadState::Receiving) {
        return;
    }

    mavlink_mission_item_int_t item;
    mavlink_msg_mission_item_int_decode(&msg, &item);

    if (item.seq != current_upload_seq_) {
        mavlink_message_t ack;
        mavlink_msg_mission_ack_pack(config_.system_id, config_.component_id, &ack, msg.sysid, msg.compid,
                                     MAV_MISSION_INVALID_SEQUENCE, MAV_MISSION_TYPE_MISSION);
        replyToSender(ack);
        mission_upload_state_ = MissionUploadState::Idle;
        sendStatusText(MAV_SEVERITY_WARNING, "Invalid mission sequence");
        return;
    }

    MissionState& mission = ctx.navigation.state();
    mission.waypoint[item.seq].lat = item.x;
    mission.waypoint[item.seq].lng = item.y;
    mission.waypoint[item.seq].alt = static_cast<int32_t>(item.z * 100.0f);
    current_upload_seq_++;

    if (current_upload_seq_ < expected_mission_count_) {
        mavlink_message_t req;
        mavlink_msg_mission_request_int_pack(config_.system_id, config_.component_id, &req, msg.sysid, msg.compid,
                                             current_upload_seq_, MAV_MISSION_TYPE_MISSION);
        replyToSender(req);
    } else {
        mission.wp_sum = expected_mission_count_;
        mavlink_message_t ack;
        mavlink_msg_mission_ack_pack(config_.system_id, config_.component_id, &ack, msg.sysid, msg.compid,
                                     MAV_MISSION_ACCEPTED, MAV_MISSION_TYPE_MISSION);
        replyToSender(ack);
        mission_upload_state_ = MissionUploadState::Idle;
        sendStatusText(MAV_SEVERITY_INFO, "Mission upload complete");
        Waypoints::save(mission);
    }
}

void Mavlink::handleMissionItem(VehicleContext& ctx, const mavlink_message_t& msg)
{
    if (mission_upload_state_ != MissionUploadState::Receiving) {
        return;
    }

    mavlink_mission_item_t item;
    mavlink_msg_mission_item_decode(&msg, &item);

    if (item.seq != current_upload_seq_) {
        mavlink_message_t ack;
        mavlink_msg_mission_ack_pack(config_.system_id, config_.component_id, &ack, msg.sysid, msg.compid,
                                     MAV_MISSION_INVALID_SEQUENCE, MAV_MISSION_TYPE_MISSION);
        replyToSender(ack);
        mission_upload_state_ = MissionUploadState::Idle;
        sendStatusText(MAV_SEVERITY_WARNING, "Invalid mission sequence");
        return;
    }

    MissionState& mission = ctx.navigation.state();
    mission.waypoint[item.seq].lat = static_cast<int32_t>(item.x * 1e7);
    mission.waypoint[item.seq].lng = static_cast<int32_t>(item.y * 1e7);
    mission.waypoint[item.seq].alt = static_cast<int32_t>(item.z * 100.0f);
    current_upload_seq_++;

    if (current_upload_seq_ < expected_mission_count_) {
        mavlink_message_t req;
        mavlink_msg_mission_request_pack(config_.system_id, config_.component_id, &req, msg.sysid, msg.compid,
                                         current_upload_seq_, MAV_MISSION_TYPE_MISSION);
        replyToSender(req);
    } else {
        mission.wp_sum = expected_mission_count_;
        mavlink_message_t ack;
        mavlink_msg_mission_ack_pack(config_.system_id, config_.component_id, &ack, msg.sysid, msg.compid,
                                     MAV_MISSION_ACCEPTED, MAV_MISSION_TYPE_MISSION);
        replyToSender(ack);
        mission_upload_state_ = MissionUploadState::Idle;
        sendStatusText(MAV_SEVERITY_INFO, "Mission upload complete");
        Waypoints::save(mission);
    }
}

void Mavlink::handleMissionClearAll(VehicleContext& ctx, const mavlink_message_t& msg)
{
    MissionState& mission = ctx.navigation.state();
    mission.wp_sum = 0;
    mission.flag_wp = -1;
    Waypoints::clear();

    mavlink_message_t ack;
    mavlink_msg_mission_ack_pack(config_.system_id, config_.component_id, &ack, msg.sysid, msg.compid,
                                 MAV_MISSION_ACCEPTED, MAV_MISSION_TYPE_MISSION);
    replyToSender(ack);
    sendStatusText(MAV_SEVERITY_INFO, "Missions cleared");
}

void Mavlink::handleSetPositionTargetLocalNed(VehicleContext& ctx, const mavlink_message_t& msg)
{
    mavlink_set_position_target_local_ned_t sp;
    mavlink_msg_set_position_target_local_ned_decode(&msg, &sp);

    constexpr uint16_t kIgnoreX = 1U << 0;
    constexpr uint16_t kIgnoreY = 1U << 1;
    constexpr uint16_t kIgnoreZ = 1U << 2;
    if ((sp.type_mask & (kIgnoreX | kIgnoreY | kIgnoreZ)) != 0) {
        return;
    }

    MissionState& mission = ctx.navigation.state();
    Locations origin = mission.home_wp_loc;
    if (!origin.initialised()) {
        origin = mission.current_loc;
    }

    Locations target = origin;
    target.offset(sp.x, sp.y);
    target.alt = origin.alt - static_cast<int32_t>(lroundf(sp.z * 100.0f));

    mission.rpi_external_setpoint_target = target;
    mission.rpi_external_setpoint_last_ms = millis();
    mission.rpi_external_setpoint_active = true;
}

void Mavlink::handleSetPositionTargetGlobalInt(VehicleContext& ctx, const mavlink_message_t& msg)
{
    mavlink_set_position_target_global_int_t sp;
    mavlink_msg_set_position_target_global_int_decode(&msg, &sp);

    constexpr uint16_t kIgnoreX = 1U << 0;
    constexpr uint16_t kIgnoreY = 1U << 1;
    constexpr uint16_t kIgnoreZ = 1U << 2;
    if ((sp.type_mask & (kIgnoreX | kIgnoreY | kIgnoreZ)) != 0) {
        return;
    }

    Locations target{};
    target.lat = sp.lat_int;
    target.lng = sp.lon_int;
    target.alt = static_cast<int32_t>(lroundf(sp.alt * 100.0f));

    MissionState& mission = ctx.navigation.state();
    mission.rpi_external_setpoint_target = target;
    mission.rpi_external_setpoint_last_ms = millis();
    mission.rpi_external_setpoint_active = true;
}

void Mavlink::sendHeartbeat(const VehicleContext& ctx)
{
    const ModeId mode_id = (ctx.mode_manager.current() != nullptr) ? ctx.mode_manager.current()->id() : ModeId::Manual;
    const bool armed = ctx.radio.armed();

    mavlink_message_t msg;
    const uint8_t system_status = armed ? MAV_STATE_ACTIVE : MAV_STATE_STANDBY;
    mavlink_msg_heartbeat_pack(config_.system_id, config_.component_id, &msg, MAV_TYPE_FIXED_WING,
                              MAV_AUTOPILOT_ARDUPILOTMEGA, baseModeFor(mode_id, armed), customModeFor(mode_id),
                              system_status);
    mavSendAll(msg);
}

void Mavlink::sendAlert(const char* text)
{
    mavlink_message_t msg;
    mavlink_msg_statustext_pack(config_.system_id, config_.component_id, &msg, MAV_SEVERITY_WARNING, text);
    mavSendAll(msg);
}

void Mavlink::sendAltitude(float altitude_m)
{
    mavlink_message_t msg;
    const uint64_t t_usec = static_cast<uint64_t>(millis()) * 1000ULL;
    mavlink_msg_altitude_pack(config_.system_id, config_.component_id, &msg, t_usec, altitude_m, altitude_m, 0,
                             altitude_m, 0, 0.0f);
    mavSendAll(msg);
}

void Mavlink::sendAttitude(float roll_rad, float pitch_rad, float yaw_rad)
{
    mavlink_message_t msg;
    mavlink_msg_attitude_pack(config_.system_id, config_.component_id, &msg, millis(), roll_rad, pitch_rad,
                              yaw_rad, 0, 0, 0);
    mavSendAll(msg);
}

void Mavlink::sendGpsRawInt(const VehicleContext& ctx)
{
    const GnssFixData& gnss = ctx.gnss;
    mavlink_message_t msg;
    const uint64_t t_usec = static_cast<uint64_t>(millis()) * 1000ULL;
    const uint8_t fix_type = static_cast<uint8_t>(gnss.fix_type);
    const uint16_t cog_centi =
        (gnss.fix_type != GnssFixType::NoFix) ? static_cast<uint16_t>(gnss.course_deg * 100.0f) : 65535;

    mavlink_msg_gps_raw_int_pack(config_.system_id, config_.component_id, &msg, t_usec, fix_type,
                                 static_cast<int32_t>(gnss.latitude_deg * 1e7),
                                 static_cast<int32_t>(gnss.longitude_deg * 1e7),
                                 static_cast<int32_t>(ctx.baro.data().altitude_m * 1000.0f),
                                 static_cast<uint16_t>(gnss.hdop * 100.0f), 0,
                                 static_cast<uint16_t>(gnss.ground_speed_mps * 100.0f), cog_centi, gnss.satellites,
                                 0, 0, 0, 0, 0);
    mavSendAll(msg);
}

void Mavlink::sendBatteryStatus(const BatteryData& battery)
{
    mavlink_message_t msg;
    uint16_t voltages[10];
    voltages[0] = static_cast<uint16_t>(battery.voltage_v * 1000.0f);
    for (uint8_t i = 1; i < 10; ++i) {
        voltages[i] = UINT16_MAX;
    }

    mavlink_msg_battery_status_pack(config_.system_id, config_.component_id, &msg, 0, MAV_BATTERY_FUNCTION_ALL,
                                    MAV_BATTERY_TYPE_LIPO, INT16_MAX, voltages, -1, -1, -1,
                                    static_cast<int8_t>(battery.percent), 0, MAV_BATTERY_CHARGE_STATE_OK);
    mavSendAll(msg);
}

void Mavlink::sendNavControllerOutput(const VehicleContext& ctx)
{
    const MissionState& mission = ctx.navigation.state();
    const float aspd_error = mission.target_airspeed_mps - ctx.airspeed.data().velocity_mps;

    mavlink_message_t msg;
    mavlink_msg_nav_controller_output_pack(config_.system_id, config_.component_id, &msg, mission.nav_roll_deg,
                                           mission.nav_pitch_deg, 0, 0, mission.auto_state.distance_next_wp, 0,
                                           aspd_error, ctx.l1.crosstrackError());
    mavSendAll(msg);
}

void Mavlink::sendVfrHud(const VehicleContext& ctx)
{
    mavlink_message_t msg;
    uint16_t throttle_pct =
        static_cast<uint16_t>(constrain(map(ctx.actuator.throttlePwm(), 988, 2012, 0, 100), 0, 100));
    if (!ctx.radio.armed()) {
        throttle_pct = 0;
    }

    const float heading_deg = ctx.imu.data().yaw_deg < 0.0f ? ctx.imu.data().yaw_deg + 360.0f : ctx.imu.data().yaw_deg;

    mavlink_msg_vfr_hud_pack(config_.system_id, config_.component_id, &msg, ctx.airspeed.data().velocity_mps,
                            ctx.gnss.ground_speed_mps, heading_deg, throttle_pct, ctx.baro.data().altitude_m,
                            ctx.baro.data().climb_rate_mps);
    mavSendAll(msg);
}

void Mavlink::sendScaledPressure(const VehicleContext& ctx)
{
    mavlink_message_t msg;
    const float press_abs_hpa = ctx.baro.data().pressure_pa / 100.0f;
    const float press_diff_hpa = ctx.airspeed.data().differential_pressure_psi * 68.94757f;
    const int16_t temp_cdeg = static_cast<int16_t>(ctx.baro.data().temperature_c * 100.0f);

    mavlink_msg_scaled_pressure_pack(config_.system_id, config_.component_id, &msg, millis(), press_abs_hpa,
                                     press_diff_hpa, temp_cdeg);
    mavSendAll(msg);
}

void Mavlink::sendMissionItemReached(uint16_t seq)
{
    mavlink_message_t msg;
    mavlink_msg_mission_item_reached_pack(config_.system_id, config_.component_id, &msg, seq);
    mavSendAll(msg);
}

void Mavlink::notifyWaypointReached(uint16_t seq)
{
    sendMissionItemReached(seq);
}

void Mavlink::sendGlobalPositionInt(const VehicleContext& ctx)
{
    const GnssFixData& gnss = ctx.gnss;
    const bool have_fix = gnss.fix_type != GnssFixType::NoFix;
    const int32_t lat = have_fix ? static_cast<int32_t>(gnss.latitude_deg * 1e7) : 0;
    const int32_t lon = have_fix ? static_cast<int32_t>(gnss.longitude_deg * 1e7) : 0;
    const int32_t alt_mm = static_cast<int32_t>(ctx.baro.data().altitude_m * 1000.0f);
    const float yaw_deg = ctx.imu.data().yaw_deg;
    const uint16_t hdg_cdeg = static_cast<uint16_t>((yaw_deg < 0.0f ? yaw_deg + 360.0f : yaw_deg) * 100.0f);

    mavlink_message_t msg;
    mavlink_msg_global_position_int_pack(config_.system_id, config_.component_id, &msg, millis(), lat, lon, alt_mm,
                                         alt_mm, 0, 0, 0, hdg_cdeg);
    mavSendAll(msg);
}

void Mavlink::sendServoOutputRaw(const VehicleContext& ctx)
{
    mavlink_message_t msg;
    mavlink_msg_servo_output_raw_pack(config_.system_id, config_.component_id, &msg,
                                      static_cast<uint32_t>(millis()), 0, ctx.actuator.aileronLeftPwm(),
                                      ctx.actuator.elevatorPwm(), ctx.actuator.throttlePwm(),
                                      ctx.actuator.rudderLeftPwm(), ctx.actuator.aileronRightPwm(),
                                      ctx.actuator.rudderRightPwm(), 0, ctx.actuator.payloadPwm(), 0, 0, 0, 0, 0, 0,
                                      0, 0);
    mavSendAll(msg);
}

void Mavlink::sendRcChannelsRaw(const VehicleContext& ctx)
{
    mavlink_message_t msg;
    mavlink_msg_rc_channels_raw_pack(config_.system_id, config_.component_id, &msg, static_cast<uint32_t>(millis()),
                                     0, ctx.radio.channelRoll(), ctx.radio.channelPitch(), ctx.radio.channelThrottle(),
                                     ctx.radio.channelYaw(), ctx.radio.channelMode(), ctx.radio.channelModeBackup(),
                                     ctx.radio.channelVehicleMode(), 0, 255);
    mavSendAll(msg);
}

void Mavlink::sendMissionCountToCompanion(const VehicleContext& ctx)
{
    mavlink_message_t msg;
    mavlink_msg_mission_count_pack(config_.system_id, config_.component_id, &msg, 0, MAV_COMP_ID_ALL,
                                   ctx.navigation.state().wp_sum, MAV_MISSION_TYPE_MISSION);
    mavWrite(Serial7, msg);
}

void Mavlink::handleParamRequestList(VehicleContext& ctx)
{
    (void)ctx;
    param_send_index_ = 0;
}

void Mavlink::sendParamValue(const VehicleContext& ctx, uint16_t index)
{
    if (index >= ctx.params.count()) {
        return;
    }
    const ParamEntry& entry = ctx.params.entryAt(index);
    mavlink_message_t msg;
    mavlink_msg_param_value_pack(config_.system_id, config_.component_id, &msg, entry.name, *entry.target,
                                 MAV_PARAM_TYPE_REAL32, ctx.params.count(), index);
    mavSendAll(msg);
}

void Mavlink::handleParamRequestRead(VehicleContext& ctx, const mavlink_message_t& msg)
{
    mavlink_param_request_read_t req;
    mavlink_msg_param_request_read_decode(&msg, &req);

    int idx = -1;
    if (req.param_index >= 0) {
        idx = (req.param_index < ctx.params.count()) ? req.param_index : -1;
    } else {
        for (uint16_t i = 0; i < ctx.params.count(); ++i) {
            if (strncmp(ctx.params.entryAt(i).name, req.param_id, 16) == 0) {
                idx = static_cast<int>(i);
                break;
            }
        }
    }

    if (idx >= 0) {
        sendParamValue(ctx, static_cast<uint16_t>(idx));
    } else {
        sendStatusText(MAV_SEVERITY_WARNING, "PARAM NOT FOUND");
    }
}

void Mavlink::handleParamSet(VehicleContext& ctx, const mavlink_message_t& msg)
{
    mavlink_param_set_t ps;
    mavlink_msg_param_set_decode(&msg, &ps);

    if (ps.target_system != config_.system_id && ps.target_system != 0) {
        return;
    }

    if (!ctx.params.setValue(ps.param_id, ps.param_value, /*save_after_set=*/true)) {
        sendStatusText(MAV_SEVERITY_WARNING, "PARAM SET: NOT FOUND");
        return;
    }

    int idx = -1;
    for (uint16_t i = 0; i < ctx.params.count(); ++i) {
        if (strncmp(ctx.params.entryAt(i).name, ps.param_id, 16) == 0) {
            idx = static_cast<int>(i);
            break;
        }
    }
    if (idx >= 0) {
        sendParamValue(ctx, static_cast<uint16_t>(idx));
    }
}

void Mavlink::pumpParamStream(VehicleContext& ctx)
{
    if (param_send_index_ == UINT16_MAX) {
        return;
    }
    if (param_send_index_ >= ctx.params.count()) {
        param_send_index_ = UINT16_MAX;
        return;
    }
    sendParamValue(ctx, param_send_index_++);
}

void Mavlink::update(VehicleContext& ctx)
{
    const unsigned long now = millis();

    if (now - last_heartbeat_ms_ >= config_.heartbeat_interval_ms) {
        last_heartbeat_ms_ = now;
        sendHeartbeat(ctx);
    }

    if (now - last_altitude_ms_ >= config_.altitude_interval_ms) {
        last_altitude_ms_ = now;
        sendAltitude(ctx.baro.data().altitude_m);
    }

    if (now - last_gps_ms_ >= config_.gps_interval_ms) {
        last_gps_ms_ = now;
        sendGpsRawInt(ctx);
    }

    if (now - last_attitude_ms_ >= config_.attitude_interval_ms) {
        last_attitude_ms_ = now;
        const ImuData& imu_data = ctx.imu.data();
        sendAttitude(imu_data.roll_rad, imu_data.pitch_rad, imu_data.yaw_rad);
    }
    

    const float roll_deg = ctx.imu.data().roll_deg;
    if (fabsf(roll_deg) > config_.roll_alert_threshold_deg && (now - last_alert_ms_ >= config_.roll_alert_debounce_ms)) {
        char buf[50];
        snprintf(buf, sizeof(buf), "ALERT: ROLL LIMIT (%.1f deg)", roll_deg);
        sendAlert(buf);
        last_alert_ms_ = now;
    }

    if (now - last_battery_ms_ >= config_.battery_interval_ms) {
        last_battery_ms_ = now;
        sendBatteryStatus(ctx.battery.data());
    }

    if (now - last_nav_controller_ms_ >= config_.nav_controller_interval_ms) {
        last_nav_controller_ms_ = now;
        sendNavControllerOutput(ctx);
    }

    if (now - last_global_position_ms_ >= config_.global_position_interval_ms) {
        last_global_position_ms_ = now;
        sendGlobalPositionInt(ctx);
    }

    if (now - last_vfr_hud_ms_ >= config_.vfr_hud_interval_ms) {
        last_vfr_hud_ms_ = now;
        sendVfrHud(ctx);
    }

    if (now - last_scaled_pressure_ms_ >= config_.scaled_pressure_interval_ms) {
        last_scaled_pressure_ms_ = now;
        sendScaledPressure(ctx);
    }

    if (now - last_servo_output_ms_ >= config_.servo_output_interval_ms) {
        last_servo_output_ms_ = now;
        sendServoOutputRaw(ctx);
    }

    if (now - last_rc_channels_ms_ >= config_.rc_channels_interval_ms) {
        last_rc_channels_ms_ = now;
        sendRcChannelsRaw(ctx);
    }

    if (now - last_mission_count_to_companion_ms_ >= config_.mission_count_to_companion_interval_ms) {
        last_mission_count_to_companion_ms_ = now;
        sendMissionCountToCompanion(ctx);
    }

    pumpParamStream(ctx);
}

}  // namespace fc
