// CPP-087 slice 6: GCS leftover catalog close (remaining_count 0).

#include <array>
#include <cstdint>
#include <cstring>
#include <span>

#include <catch2/catch_test_macros.hpp>
#include <fwcpp/gcs/gcs.hpp>

using fwcpp::gcs::Attitude;
using fwcpp::gcs::CommandAck;
using fwcpp::gcs::CommandHooks;
using fwcpp::gcs::CommandLong;
using fwcpp::gcs::DecodeError;
using fwcpp::gcs::DispatchKind;
using fwcpp::gcs::Frame;
using fwcpp::gcs::GcsMavlink;
using fwcpp::gcs::Heartbeat;
using fwcpp::gcs::MavResult;
using fwcpp::gcs::MissionCount;
using fwcpp::gcs::MissionItemInt;
using fwcpp::gcs::MissionRequestInt;
using fwcpp::gcs::MissionStore;
using fwcpp::gcs::ParamSet;
using fwcpp::gcs::ParamStore;
using fwcpp::gcs::ParamValue;
using fwcpp::gcs::PlaneAttitudeInputs;
using fwcpp::gcs::PortStatus;
using fwcpp::gcs::attitude_from_frame;
using fwcpp::gcs::command_ack_from_frame;
using fwcpp::gcs::completeness_has;
using fwcpp::gcs::copy_name_to_param_id;
using fwcpp::gcs::copy_param_id_to_name;
using fwcpp::gcs::crc_extra;
using fwcpp::gcs::decode_v2;
using fwcpp::gcs::encode_v2;
using fwcpp::gcs::heartbeat_from_frame;
using fwcpp::gcs::kAttitudeCrcExtra;
using fwcpp::gcs::kAttitudeLen;
using fwcpp::gcs::kCommandAckCrcExtra;
using fwcpp::gcs::kCommandAckLen;
using fwcpp::gcs::kCommandLongCrcExtra;
using fwcpp::gcs::kCommandLongLen;
using fwcpp::gcs::kHeartbeatLen;
using fwcpp::gcs::kMagicForceArmDisarmValue;
using fwcpp::gcs::kMagicForceArmValue;
using fwcpp::gcs::kMavAutopilotArdupilotmega;
using fwcpp::gcs::kMavCmdComponentArmDisarm;
using fwcpp::gcs::kMavCmdDoSetMode;
using fwcpp::gcs::kMavCmdNavWaypoint;
using fwcpp::gcs::kMavFrameGlobalRelAlt;
using fwcpp::gcs::kMavMissionTypeMission;
using fwcpp::gcs::kMavModeFlagCustomModeEnabled;
using fwcpp::gcs::kMavModeFlagDecodePositionSafety;
using fwcpp::gcs::kMavParamTypeReal32;
using fwcpp::gcs::kMavTypeFixedWing;
using fwcpp::gcs::kMavlinkVersion;
using fwcpp::gcs::kMissionCountCrcExtra;
using fwcpp::gcs::kMissionCountLen;
using fwcpp::gcs::kMissionItemIntCrcExtra;
using fwcpp::gcs::kMissionItemIntLen;
using fwcpp::gcs::kMissionRequestIntCrcExtra;
using fwcpp::gcs::kMissionRequestIntLen;
using fwcpp::gcs::kMsgIdAttitude;
using fwcpp::gcs::kMsgIdCommandAck;
using fwcpp::gcs::kMsgIdCommandLong;
using fwcpp::gcs::kMsgIdHeartbeat;
using fwcpp::gcs::kMsgIdMissionCount;
using fwcpp::gcs::kMsgIdMissionItemInt;
using fwcpp::gcs::kMsgIdMissionRequestInt;
using fwcpp::gcs::kMsgIdParamRequestList;
using fwcpp::gcs::kMsgIdParamSet;
using fwcpp::gcs::kMsgIdParamValue;
using fwcpp::gcs::kParamIdLen;
using fwcpp::gcs::kParamIndexSetAck;
using fwcpp::gcs::kParamNameCapacity;
using fwcpp::gcs::kParamRequestListCrcExtra;
using fwcpp::gcs::kParamRequestListLen;
using fwcpp::gcs::kParamSetCrcExtra;
using fwcpp::gcs::kParamSetLen;
using fwcpp::gcs::kParamValueCrcExtra;
using fwcpp::gcs::kParamValueLen;
using fwcpp::gcs::kStxV2;
using fwcpp::gcs::leftover_completeness_size;
using fwcpp::gcs::leftover_send_attitude;
using fwcpp::gcs::make_frame;
using fwcpp::gcs::make_mission_count;
using fwcpp::gcs::mission_item_int_from_frame;
using fwcpp::gcs::mission_store_insert;
using fwcpp::gcs::on_main_count;
using fwcpp::gcs::out_of_scope_count;
using fwcpp::gcs::pack_attitude;
using fwcpp::gcs::pack_command_ack;
using fwcpp::gcs::pack_command_long;
using fwcpp::gcs::pack_heartbeat;
using fwcpp::gcs::pack_mission_count;
using fwcpp::gcs::pack_mission_item_int;
using fwcpp::gcs::pack_mission_request_int;
using fwcpp::gcs::pack_param_request_list;
using fwcpp::gcs::pack_param_set;
using fwcpp::gcs::pack_param_value;
using fwcpp::gcs::param_store_find;
using fwcpp::gcs::param_store_insert;
using fwcpp::gcs::param_value_from_frame;
using fwcpp::gcs::plane_heartbeat;
using fwcpp::gcs::remaining_count;
using fwcpp::gcs::this_slice_count;
using fwcpp::gcs::unpack_attitude;
using fwcpp::gcs::unpack_command_ack;
using fwcpp::gcs::unpack_command_long;
using fwcpp::gcs::unpack_heartbeat;
using fwcpp::gcs::unpack_mission_count;
using fwcpp::gcs::unpack_mission_item_int;
using fwcpp::gcs::unpack_mission_request_int;
using fwcpp::gcs::unpack_param_value;

namespace {

struct FakeVehicle {
    bool armed = false;
    bool arm_ok = true;
    bool disarm_ok = true;
    bool set_mode_ok = true;
    bool safety_on_ok = true;
    bool last_arm_checks = true;
    bool last_disarm_checks = true;
    std::uint32_t last_custom_mode = 0;
    int safety_on_calls = 0;
    int safety_off_calls = 0;
};

bool fake_is_armed(void* ctx) { return static_cast<FakeVehicle*>(ctx)->armed; }

bool fake_arm(void* ctx, bool do_arming_checks) {
    auto* v = static_cast<FakeVehicle*>(ctx);
    v->last_arm_checks = do_arming_checks;
    if (!v->arm_ok) {
        return false;
    }
    v->armed = true;
    return true;
}

bool fake_disarm(void* ctx, bool do_disarm_checks) {
    auto* v = static_cast<FakeVehicle*>(ctx);
    v->last_disarm_checks = do_disarm_checks;
    if (!v->disarm_ok) {
        return false;
    }
    v->armed = false;
    return true;
}

bool fake_set_mode(void* ctx, std::uint32_t custom_mode) {
    auto* v = static_cast<FakeVehicle*>(ctx);
    v->last_custom_mode = custom_mode;
    return v->set_mode_ok;
}

bool fake_safety_on(void* ctx) {
    auto* v = static_cast<FakeVehicle*>(ctx);
    ++v->safety_on_calls;
    return v->safety_on_ok;
}

void fake_safety_off(void* ctx) {
    auto* v = static_cast<FakeVehicle*>(ctx);
    ++v->safety_off_calls;
}

CommandHooks make_hooks(FakeVehicle& v) {
    CommandHooks h{};
    h.is_armed = fake_is_armed;
    h.arm = fake_arm;
    h.disarm = fake_disarm;
    h.set_mode = fake_set_mode;
    h.force_safety_on = fake_safety_on;
    h.force_safety_off = fake_safety_off;
    h.ctx = &v;
    return h;
}

CommandLong make_cmd(std::uint16_t command, float p1, float p2) {
    CommandLong cmd{};
    cmd.param1 = p1;
    cmd.param2 = p2;
    cmd.command = command;
    cmd.target_system = 1;
    cmd.target_component = 1;
    return cmd;
}

Frame frame_command_long(const CommandLong& cmd, std::uint8_t sysid = 255, std::uint8_t compid = 190) {
    std::array<std::uint8_t, kCommandLongLen> payload{};
    REQUIRE(pack_command_long(cmd, payload) == kCommandLongLen);
    Frame frame{};
    REQUIRE(make_frame(0, sysid, compid, kMsgIdCommandLong, payload, frame));
    return frame;
}

bool seed_store(ParamStore& store) {
    return param_store_insert(store, "SYSID_THISMAV", 1.0f, kMavParamTypeReal32) &&
           param_store_insert(store, "ARSPD_ENABLE", 0.0f, kMavParamTypeReal32) &&
           param_store_insert(store, "TRIM_PITCH", 0.0f, kMavParamTypeReal32);
}

Frame frame_param_request_list(std::uint8_t target_system = 1, std::uint8_t target_component = 1) {
    fwcpp::gcs::ParamRequestList req{};
    req.target_system = target_system;
    req.target_component = target_component;
    std::array<std::uint8_t, kParamRequestListLen> payload{};
    REQUIRE(pack_param_request_list(req, payload) == kParamRequestListLen);
    Frame frame{};
    REQUIRE(make_frame(0, 255, 190, kMsgIdParamRequestList, payload, frame));
    return frame;
}

Frame frame_param_set(const char* name, float value, std::uint8_t target_system = 1) {
    ParamSet set{};
    set.param_value = value;
    set.target_system = target_system;
    set.target_component = 1;
    copy_name_to_param_id(set.param_id, name);
    set.param_type = kMavParamTypeReal32;
    std::array<std::uint8_t, kParamSetLen> payload{};
    REQUIRE(pack_param_set(set, payload) == kParamSetLen);
    Frame frame{};
    REQUIRE(make_frame(0, 255, 190, kMsgIdParamSet, payload, frame));
    return frame;
}

MissionItemInt make_waypoint(std::uint16_t seq, std::int32_t lat_e7, std::int32_t lon_e7, float alt) {
    MissionItemInt item{};
    item.param1 = 0.0f;
    item.param2 = 0.0f;
    item.param3 = 0.0f;
    item.param4 = 0.0f;
    item.x = lat_e7;
    item.y = lon_e7;
    item.z = alt;
    item.seq = seq;
    item.command = kMavCmdNavWaypoint;
    item.target_system = 1;
    item.target_component = 1;
    item.frame = kMavFrameGlobalRelAlt;
    item.current = seq == 0 ? 1 : 0;
    item.autocontinue = 1;
    item.mission_type = kMavMissionTypeMission;
    return item;
}

Frame frame_mission_request_int(std::uint16_t seq, std::uint8_t target_system = 1) {
    MissionRequestInt req{};
    req.seq = seq;
    req.target_system = target_system;
    req.target_component = 1;
    req.mission_type = kMavMissionTypeMission;
    std::array<std::uint8_t, kMissionRequestIntLen> payload{};
    REQUIRE(pack_mission_request_int(req, payload) == kMissionRequestIntLen);
    Frame frame{};
    REQUIRE(make_frame(0, 255, 190, kMsgIdMissionRequestInt, payload, frame));
    return frame;
}

}  // namespace

TEST_CASE("HEARTBEAT payload and MAVLink2 frame round-trip", "[gcs][heartbeat]") {
    const Heartbeat hb = plane_heartbeat(kMavTypeFixedWing, 0x81, 12, 4);
    std::array<std::uint8_t, kHeartbeatLen> payload{};
    REQUIRE(pack_heartbeat(hb, payload) == kHeartbeatLen);

    Heartbeat unpacked{};
    REQUIRE(unpack_heartbeat(payload, unpacked));
    REQUIRE(unpacked.custom_mode == 12);
    REQUIRE(unpacked.type == kMavTypeFixedWing);
    REQUIRE(unpacked.autopilot == kMavAutopilotArdupilotmega);
    REQUIRE(unpacked.base_mode == 0x81);
    REQUIRE(unpacked.system_status == 4);
    REQUIRE(unpacked.mavlink_version == kMavlinkVersion);

    Frame frame{};
    REQUIRE(make_frame(7, 1, 1, kMsgIdHeartbeat, payload, frame));
    std::array<std::uint8_t, 32> wire{};
    const std::size_t n = encode_v2(frame, wire);
    REQUIRE(n == 21);
    REQUIRE(wire[0] == kStxV2);

    auto decoded = decode_v2(std::span<const std::uint8_t>(wire.data(), n));
    REQUIRE(decoded.has_value());
    REQUIRE(decoded.value().seq == 7);
    REQUIRE(decoded.value().sysid == 1);
    REQUIRE(decoded.value().compid == 1);
    REQUIRE(decoded.value().msgid == kMsgIdHeartbeat);

    Heartbeat from_frame{};
    REQUIRE(heartbeat_from_frame(decoded.value(), from_frame));
    REQUIRE(from_frame.custom_mode == 12);
    REQUIRE(from_frame.type == kMavTypeFixedWing);
    REQUIRE(from_frame.autopilot == kMavAutopilotArdupilotmega);
    REQUIRE(from_frame.mavlink_version == kMavlinkVersion);
}

TEST_CASE("MAVLink2 rejects bad CRC", "[gcs][framing]") {
    const Heartbeat hb = plane_heartbeat(kMavTypeFixedWing, 0x81, 12, 4);
    std::array<std::uint8_t, kHeartbeatLen> payload{};
    REQUIRE(pack_heartbeat(hb, payload) == kHeartbeatLen);
    Frame frame{};
    REQUIRE(make_frame(0, 1, 1, kMsgIdHeartbeat, payload, frame));
    std::array<std::uint8_t, 32> wire{};
    const std::size_t n = encode_v2(frame, wire);
    REQUIRE(n >= 2);
    wire[n - 2] = static_cast<std::uint8_t>(wire[n - 2] ^ 0x01);

    auto decoded = decode_v2(std::span<const std::uint8_t>(wire.data(), n));
    REQUIRE_FALSE(decoded.has_value());
    REQUIRE(decoded.error() == DecodeError::kBadCrc);
}

TEST_CASE("unknown msgid is not dispatched", "[gcs][dispatch]") {
    Frame frame{};
    REQUIRE(make_frame(0, 255, 190, 253, {}, frame));
    GcsMavlink gcs;
    const auto d = gcs.handle_message(frame, 0);
    REQUIRE(d.kind == DispatchKind::kUnknown);
    REQUIRE(d.msgid == 253);
    REQUIRE(gcs.last_gcs_heartbeat_ms() == 0);
}

TEST_CASE("send_heartbeat dispatches as msgid 0", "[gcs][dispatch]") {
    GcsMavlink sender;
    std::array<std::uint8_t, 32> wire{};
    const std::size_t n = sender.send_heartbeat(wire, kMavTypeFixedWing, 0x81, 12, 4);
    REQUIRE(n > 0);

    auto decoded = decode_v2(std::span<const std::uint8_t>(wire.data(), n));
    REQUIRE(decoded.has_value());
    REQUIRE(decoded.value().msgid == kMsgIdHeartbeat);

    GcsMavlink peer;
    const auto d = peer.handle_message(decoded.value(), 1000);
    REQUIRE(d.kind == DispatchKind::kHeartbeat);
    REQUIRE_FALSE(d.from_gcs);
    REQUIRE(d.heartbeat.custom_mode == 12);
    REQUIRE(d.heartbeat.autopilot == kMavAutopilotArdupilotmega);
    REQUIRE(peer.last_gcs_heartbeat_ms() == 0);
}

TEST_CASE("leftover catalog this slice vs remaining", "[gcs][leftover]") {
    REQUIRE(remaining_count() == 0);
    REQUIRE(this_slice_count() == 1);
    REQUIRE(on_main_count() == 7);
    REQUIRE(out_of_scope_count() == 1);
    REQUIRE(leftover_completeness_size() ==
            on_main_count() + this_slice_count() + remaining_count() + out_of_scope_count());
    REQUIRE(completeness_has("MAVLink 2 framing", PortStatus::kOnMain));
    REQUIRE(completeness_has("HEARTBEAT pack/unpack", PortStatus::kOnMain));
    REQUIRE(completeness_has("msgid dispatch stub", PortStatus::kOnMain));
    REQUIRE(completeness_has("leftover catalog", PortStatus::kThisSlice));
    REQUIRE(completeness_has("COMMAND_LONG", PortStatus::kOnMain));
    REQUIRE(completeness_has("PARAM protocol", PortStatus::kOnMain));
    REQUIRE(completeness_has("MISSION", PortStatus::kOnMain));
    REQUIRE(completeness_has("Plane/Copter vehicle handlers", PortStatus::kOnMain));
    REQUIRE(completeness_has("XML dialect generation", PortStatus::kOutOfScope));
}

TEST_CASE("COMMAND_LONG pack/unpack round-trip", "[gcs][command_long]") {
    const CommandLong cmd = make_cmd(kMavCmdComponentArmDisarm, 1.0f, kMagicForceArmValue);
    std::array<std::uint8_t, kCommandLongLen> payload{};
    REQUIRE(pack_command_long(cmd, payload) == kCommandLongLen);

    CommandLong unpacked{};
    REQUIRE(unpack_command_long(payload, unpacked));
    REQUIRE(unpacked.param1 == 1.0f);
    REQUIRE(unpacked.param2 == kMagicForceArmValue);
    REQUIRE(unpacked.command == kMavCmdComponentArmDisarm);
    REQUIRE(unpacked.target_system == 1);
    REQUIRE(unpacked.target_component == 1);
    REQUIRE(unpacked.confirmation == 0);
}

TEST_CASE("COMMAND_ACK pack/unpack size-sorted v2", "[gcs][command_ack]") {
    CommandAck ack{};
    ack.command = kMavCmdDoSetMode;
    ack.result = static_cast<std::uint8_t>(MavResult::kAccepted);
    ack.progress = 0;
    ack.result_param2 = 0;
    ack.target_system = 255;
    ack.target_component = 190;
    std::array<std::uint8_t, kCommandAckLen> payload{};
    REQUIRE(pack_command_ack(ack, payload) == kCommandAckLen);
    REQUIRE(payload[4] == static_cast<std::uint8_t>(kMavCmdDoSetMode));
    REQUIRE(payload[5] == static_cast<std::uint8_t>(kMavCmdDoSetMode >> 8));

    CommandAck unpacked{};
    REQUIRE(unpack_command_ack(payload, unpacked));
    REQUIRE(unpacked.command == kMavCmdDoSetMode);
    REQUIRE(unpacked.result == static_cast<std::uint8_t>(MavResult::kAccepted));
    REQUIRE(unpacked.progress == 0);
    REQUIRE(unpacked.result_param2 == 0);
    REQUIRE(unpacked.target_system == 255);
    REQUIRE(unpacked.target_component == 190);
}

TEST_CASE("crc extra for COMMAND_LONG and COMMAND_ACK", "[gcs][framing]") {
    std::uint8_t extra = 0;
    REQUIRE(crc_extra(kMsgIdCommandLong, extra));
    REQUIRE(extra == kCommandLongCrcExtra);
    REQUIRE(extra == 152);
    REQUIRE(crc_extra(kMsgIdCommandAck, extra));
    REQUIRE(extra == kCommandAckCrcExtra);
    REQUIRE(extra == 143);
    REQUIRE_FALSE(crc_extra(253, extra));
}

TEST_CASE("crc extra for PARAM_REQUEST_LIST PARAM_VALUE PARAM_SET", "[gcs][framing][param]") {
    std::uint8_t extra = 0;
    REQUIRE(crc_extra(kMsgIdParamRequestList, extra));
    REQUIRE(extra == kParamRequestListCrcExtra);
    REQUIRE(extra == 159);
    REQUIRE(crc_extra(kMsgIdParamValue, extra));
    REQUIRE(extra == kParamValueCrcExtra);
    REQUIRE(extra == 220);
    REQUIRE(crc_extra(kMsgIdParamSet, extra));
    REQUIRE(extra == kParamSetCrcExtra);
    REQUIRE(extra == 168);
}

TEST_CASE("ARM/DISARM and force-magic", "[gcs][arm]") {
    FakeVehicle veh;
    GcsMavlink gcs;
    gcs.set_hooks(make_hooks(veh));

    SECTION("arm when disarmed ACCEPTED and runs checks") {
        const auto d = gcs.handle_command_long_frame(
            frame_command_long(make_cmd(kMavCmdComponentArmDisarm, 1.0f, 0.0f)));
        REQUIRE(d.kind == DispatchKind::kCommandLong);
        REQUIRE(d.command_result == MavResult::kAccepted);
        REQUIRE(veh.armed);
        REQUIRE(veh.last_arm_checks);
        REQUIRE(d.command_ack.result == static_cast<std::uint8_t>(MavResult::kAccepted));
    }

    SECTION("already armed ACCEPTED without calling arm") {
        veh.armed = true;
        veh.arm_ok = false;
        const auto d = gcs.handle_command_long_frame(
            frame_command_long(make_cmd(kMavCmdComponentArmDisarm, 1.0f, 0.0f)));
        REQUIRE(d.command_result == MavResult::kAccepted);
        REQUIRE(veh.armed);
    }

    SECTION("force arm 2989 skips checks") {
        const auto d = gcs.handle_command_long_frame(
            frame_command_long(make_cmd(kMavCmdComponentArmDisarm, 1.0f, kMagicForceArmValue)));
        REQUIRE(d.command_result == MavResult::kAccepted);
        REQUIRE_FALSE(veh.last_arm_checks);
        REQUIRE(veh.armed);
    }

    SECTION("force arm 21196 skips checks") {
        const auto d = gcs.handle_command_long_frame(
            frame_command_long(make_cmd(kMavCmdComponentArmDisarm, 1.0f, kMagicForceArmDisarmValue)));
        REQUIRE(d.command_result == MavResult::kAccepted);
        REQUIRE_FALSE(veh.last_arm_checks);
    }

    SECTION("arm fail FAILED") {
        veh.arm_ok = false;
        const auto d = gcs.handle_command_long_frame(
            frame_command_long(make_cmd(kMavCmdComponentArmDisarm, 1.0f, 0.0f)));
        REQUIRE(d.command_result == MavResult::kFailed);
        REQUIRE_FALSE(veh.armed);
    }

    SECTION("disarm when armed ACCEPTED with checks") {
        veh.armed = true;
        const auto d = gcs.handle_command_long_frame(
            frame_command_long(make_cmd(kMavCmdComponentArmDisarm, 0.0f, 0.0f)));
        REQUIRE(d.command_result == MavResult::kAccepted);
        REQUIRE_FALSE(veh.armed);
        REQUIRE(veh.last_disarm_checks);
    }

    SECTION("already disarmed ACCEPTED") {
        veh.armed = false;
        veh.disarm_ok = false;
        const auto d = gcs.handle_command_long_frame(
            frame_command_long(make_cmd(kMavCmdComponentArmDisarm, 0.0f, 0.0f)));
        REQUIRE(d.command_result == MavResult::kAccepted);
    }

    SECTION("force disarm 21196 skips checks") {
        veh.armed = true;
        const auto d = gcs.handle_command_long_frame(
            frame_command_long(make_cmd(kMavCmdComponentArmDisarm, 0.0f, kMagicForceArmDisarmValue)));
        REQUIRE(d.command_result == MavResult::kAccepted);
        REQUIRE_FALSE(veh.last_disarm_checks);
        REQUIRE_FALSE(veh.armed);
    }

    SECTION("disarm fail FAILED") {
        veh.armed = true;
        veh.disarm_ok = false;
        const auto d = gcs.handle_command_long_frame(
            frame_command_long(make_cmd(kMavCmdComponentArmDisarm, 0.0f, 0.0f)));
        REQUIRE(d.command_result == MavResult::kFailed);
        REQUIRE(veh.armed);
    }

    SECTION("other param1 UNSUPPORTED") {
        const auto d = gcs.handle_command_long_frame(
            frame_command_long(make_cmd(kMavCmdComponentArmDisarm, 2.0f, 0.0f)));
        REQUIRE(d.command_result == MavResult::kUnsupported);
    }
}

TEST_CASE("DO_SET_MODE custom vs safety switch", "[gcs][set_mode]") {
    FakeVehicle veh;
    GcsMavlink gcs;
    gcs.set_hooks(make_hooks(veh));

    SECTION("custom mode enabled calls set_mode ACCEPTED") {
        const auto d = gcs.handle_command_long_frame(
            frame_command_long(make_cmd(kMavCmdDoSetMode, static_cast<float>(kMavModeFlagCustomModeEnabled), 12.0f)));
        REQUIRE(d.command_result == MavResult::kAccepted);
        REQUIRE(veh.last_custom_mode == 12);
    }

    SECTION("custom mode fail FAILED") {
        veh.set_mode_ok = false;
        const auto d = gcs.handle_command_long_frame(
            frame_command_long(make_cmd(kMavCmdDoSetMode, static_cast<float>(kMavModeFlagCustomModeEnabled), 5.0f)));
        REQUIRE(d.command_result == MavResult::kFailed);
    }

    SECTION("safety off custom 0 ACCEPTED") {
        const auto d = gcs.handle_command_long_frame(frame_command_long(
            make_cmd(kMavCmdDoSetMode, static_cast<float>(kMavModeFlagDecodePositionSafety), 0.0f)));
        REQUIRE(d.command_result == MavResult::kAccepted);
        REQUIRE(veh.safety_off_calls == 1);
        REQUIRE(veh.safety_on_calls == 0);
    }

    SECTION("safety on custom 1 ACCEPTED") {
        const auto d = gcs.handle_command_long_frame(frame_command_long(
            make_cmd(kMavCmdDoSetMode, static_cast<float>(kMavModeFlagDecodePositionSafety), 1.0f)));
        REQUIRE(d.command_result == MavResult::kAccepted);
        REQUIRE(veh.safety_on_calls == 1);
    }

    SECTION("safety on custom 1 fail FAILED") {
        veh.safety_on_ok = false;
        const auto d = gcs.handle_command_long_frame(frame_command_long(
            make_cmd(kMavCmdDoSetMode, static_cast<float>(kMavModeFlagDecodePositionSafety), 1.0f)));
        REQUIRE(d.command_result == MavResult::kFailed);
    }

    SECTION("safety other custom DENIED") {
        const auto d = gcs.handle_command_long_frame(frame_command_long(
            make_cmd(kMavCmdDoSetMode, static_cast<float>(kMavModeFlagDecodePositionSafety), 2.0f)));
        REQUIRE(d.command_result == MavResult::kDenied);
    }

    SECTION("no custom bit DENIED") {
        const auto d =
            gcs.handle_command_long_frame(frame_command_long(make_cmd(kMavCmdDoSetMode, 0.0f, 12.0f)));
        REQUIRE(d.command_result == MavResult::kDenied);
        REQUIRE(veh.last_custom_mode == 0);
    }
}

TEST_CASE("unknown command UNSUPPORTED", "[gcs][command_long]") {
    FakeVehicle veh;
    GcsMavlink gcs;
    gcs.set_hooks(make_hooks(veh));
    const auto d = gcs.handle_command_long_frame(frame_command_long(make_cmd(22, 0.0f, 0.0f)));
    REQUIRE(d.kind == DispatchKind::kCommandLong);
    REQUIRE(d.command_result == MavResult::kUnsupported);
    REQUIRE(d.command_ack.result == static_cast<std::uint8_t>(MavResult::kUnsupported));
    REQUIRE(d.command_ack.command == 22);
}

TEST_CASE("COMMAND_ACK is framed after COMMAND_LONG", "[gcs][command_ack]") {
    FakeVehicle veh;
    GcsMavlink gcs;
    gcs.set_hooks(make_hooks(veh));
    const Frame in = frame_command_long(make_cmd(kMavCmdComponentArmDisarm, 1.0f, 0.0f), 255, 190);
    std::array<std::uint8_t, 48> wire{};
    const std::size_t n = gcs.handle_command_long(in, wire);
    REQUIRE(n == 10 + 10 + 2);
    REQUIRE(wire[0] == kStxV2);

    auto decoded = decode_v2(std::span<const std::uint8_t>(wire.data(), n));
    REQUIRE(decoded.has_value());
    REQUIRE(decoded.value().msgid == kMsgIdCommandAck);
    REQUIRE(decoded.value().sysid == 1);
    REQUIRE(decoded.value().compid == 1);

    CommandAck ack{};
    REQUIRE(command_ack_from_frame(decoded.value(), ack));
    REQUIRE(ack.command == kMavCmdComponentArmDisarm);
    REQUIRE(ack.result == static_cast<std::uint8_t>(MavResult::kAccepted));
    REQUIRE(ack.progress == 0);
    REQUIRE(ack.result_param2 == 0);
    REQUIRE(ack.target_system == 255);
    REQUIRE(ack.target_component == 190);
}

TEST_CASE("PARAM_VALUE pack/unpack size-sorted v2", "[gcs][param]") {
    ParamValue value{};
    copy_name_to_param_id(value.param_id, "ARSPD_ENABLE");
    value.param_value = 1.0f;
    value.param_type = kMavParamTypeReal32;
    value.param_count = 3;
    value.param_index = 1;
    std::array<std::uint8_t, kParamValueLen> payload{};
    REQUIRE(pack_param_value(value, payload) == kParamValueLen);
    REQUIRE(payload[4] == 3);
    REQUIRE(payload[5] == 0);
    REQUIRE(payload[6] == 1);
    REQUIRE(payload[7] == 0);
    REQUIRE(payload[24] == kMavParamTypeReal32);

    ParamValue unpacked{};
    REQUIRE(unpack_param_value(payload, unpacked));
    char name[kParamNameCapacity]{};
    copy_param_id_to_name(name, unpacked.param_id);
    REQUIRE(std::strcmp(name, "ARSPD_ENABLE") == 0);
    REQUIRE(unpacked.param_value == 1.0f);
    REQUIRE(unpacked.param_count == 3);
    REQUIRE(unpacked.param_index == 1);
    REQUIRE(unpacked.param_type == kMavParamTypeReal32);

    Frame frame{};
    REQUIRE(make_frame(0, 1, 1, kMsgIdParamValue, payload, frame));
    std::array<std::uint8_t, 48> wire{};
    const std::size_t n = encode_v2(frame, wire);
    REQUIRE(n == 10 + kParamValueLen + 2);
    auto decoded = decode_v2(std::span<const std::uint8_t>(wire.data(), n));
    REQUIRE(decoded.has_value());
    REQUIRE(decoded.value().msgid == kMsgIdParamValue);
    ParamValue from_frame{};
    REQUIRE(param_value_from_frame(decoded.value(), from_frame));
    REQUIRE(from_frame.param_count == 3);
    REQUIRE(from_frame.param_index == 1);
}

TEST_CASE("PARAM_REQUEST_LIST emits N PARAM_VALUE", "[gcs][param]") {
    ParamStore store;
    REQUIRE(seed_store(store));
    GcsMavlink gcs;
    gcs.set_param_store(store);

    const Frame in = frame_param_request_list();
    const auto d = gcs.handle_message(in, 0);
    REQUIRE(d.kind == DispatchKind::kParamRequestList);
    REQUIRE(d.param_count == 3);

    std::array<ParamValue, 8> values{};
    const std::size_t n = gcs.handle_param_request_list(in, values);
    REQUIRE(n == 3);
    REQUIRE(values[0].param_count == 3);
    REQUIRE(values[0].param_index == 0);
    REQUIRE(values[1].param_index == 1);
    REQUIRE(values[2].param_index == 2);
    char n0[kParamNameCapacity]{};
    char n1[kParamNameCapacity]{};
    char n2[kParamNameCapacity]{};
    copy_param_id_to_name(n0, values[0].param_id);
    copy_param_id_to_name(n1, values[1].param_id);
    copy_param_id_to_name(n2, values[2].param_id);
    REQUIRE(std::strcmp(n0, "SYSID_THISMAV") == 0);
    REQUIRE(std::strcmp(n1, "ARSPD_ENABLE") == 0);
    REQUIRE(std::strcmp(n2, "TRIM_PITCH") == 0);
    REQUIRE(values[0].param_value == 1.0f);
    REQUIRE(values[1].param_value == 0.0f);
    REQUIRE(values[1].param_type == kMavParamTypeReal32);
}

TEST_CASE("PARAM_SET updates store and acks PARAM_VALUE", "[gcs][param]") {
    ParamStore store;
    REQUIRE(seed_store(store));
    GcsMavlink gcs;
    gcs.set_param_store(store);

    const auto d = gcs.handle_param_set_frame(frame_param_set("ARSPD_ENABLE", 1.0f));
    REQUIRE(d.kind == DispatchKind::kParamSet);
    REQUIRE(d.param_applied);
    REQUIRE(d.param_value.param_index == kParamIndexSetAck);
    REQUIRE(d.param_value.param_count == 3);
    REQUIRE(d.param_value.param_value == 1.0f);
    const auto* entry = param_store_find(store, "ARSPD_ENABLE");
    REQUIRE(entry != nullptr);
    REQUIRE(entry->value == 1.0f);

    std::array<std::uint8_t, 48> wire{};
    const std::size_t n = gcs.handle_param_set(frame_param_set("ARSPD_ENABLE", 2.0f), wire);
    REQUIRE(n == 10 + kParamValueLen + 2);
    auto decoded = decode_v2(std::span<const std::uint8_t>(wire.data(), n));
    REQUIRE(decoded.has_value());
    REQUIRE(decoded.value().msgid == kMsgIdParamValue);
    ParamValue ack{};
    REQUIRE(param_value_from_frame(decoded.value(), ack));
    char name[kParamNameCapacity]{};
    copy_param_id_to_name(name, ack.param_id);
    REQUIRE(std::strcmp(name, "ARSPD_ENABLE") == 0);
    REQUIRE(ack.param_value == 2.0f);
    REQUIRE(ack.param_index == kParamIndexSetAck);
    REQUIRE(param_store_find(store, "ARSPD_ENABLE")->value == 2.0f);
}

TEST_CASE("PARAM_SET unknown name does not crash", "[gcs][param]") {
    ParamStore store;
    REQUIRE(seed_store(store));
    GcsMavlink gcs;
    gcs.set_param_store(store);

    const auto d = gcs.handle_param_set_frame(frame_param_set("NO_SUCH_PARAM", 4.0f));
    REQUIRE(d.kind == DispatchKind::kParamSet);
    REQUIRE_FALSE(d.param_applied);
    REQUIRE(param_store_find(store, "NO_SUCH_PARAM") == nullptr);
    REQUIRE(param_store_find(store, "ARSPD_ENABLE")->value == 0.0f);

    std::array<std::uint8_t, 48> wire{};
    REQUIRE(gcs.handle_param_set(frame_param_set("NO_SUCH_PARAM", 4.0f), wire) == 0);
}

TEST_CASE("crc extra for MISSION_REQUEST_INT ITEM_INT COUNT", "[gcs][framing][mission]") {
    std::uint8_t extra = 0;
    REQUIRE(crc_extra(kMsgIdMissionRequestInt, extra));
    REQUIRE(extra == kMissionRequestIntCrcExtra);
    REQUIRE(extra == 196);
    REQUIRE(crc_extra(kMsgIdMissionItemInt, extra));
    REQUIRE(extra == kMissionItemIntCrcExtra);
    REQUIRE(extra == 38);
    REQUIRE(crc_extra(kMsgIdMissionCount, extra));
    REQUIRE(extra == kMissionCountCrcExtra);
    REQUIRE(extra == 221);
}

TEST_CASE("MISSION_REQUEST_INT pack/unpack size-sorted v2", "[gcs][mission]") {
    MissionRequestInt req{};
    req.seq = 0;
    req.target_system = 1;
    req.target_component = 1;
    req.mission_type = kMavMissionTypeMission;
    std::array<std::uint8_t, kMissionRequestIntLen> payload{};
    REQUIRE(pack_mission_request_int(req, payload) == kMissionRequestIntLen);
    REQUIRE(payload[0] == 0);
    REQUIRE(payload[1] == 0);
    REQUIRE(payload[2] == 1);
    REQUIRE(payload[3] == 1);
    REQUIRE(payload[4] == kMavMissionTypeMission);

    MissionRequestInt unpacked{};
    REQUIRE(unpack_mission_request_int(payload, unpacked));
    REQUIRE(unpacked.seq == 0);
    REQUIRE(unpacked.target_system == 1);
    REQUIRE(unpacked.target_component == 1);
    REQUIRE(unpacked.mission_type == kMavMissionTypeMission);
}

TEST_CASE("MISSION_ITEM_INT pack/unpack size-sorted v2", "[gcs][mission]") {
    const MissionItemInt item = make_waypoint(0, 377749000, -1224194000, 100.0f);
    std::array<std::uint8_t, kMissionItemIntLen> payload{};
    REQUIRE(pack_mission_item_int(item, payload) == kMissionItemIntLen);
    REQUIRE(payload[28] == 0);
    REQUIRE(payload[29] == 0);
    REQUIRE(payload[30] == static_cast<std::uint8_t>(kMavCmdNavWaypoint));
    REQUIRE(payload[31] == 0);
    REQUIRE(payload[32] == 1);
    REQUIRE(payload[37] == kMavMissionTypeMission);

    MissionItemInt unpacked{};
    REQUIRE(unpack_mission_item_int(payload, unpacked));
    REQUIRE(unpacked.seq == 0);
    REQUIRE(unpacked.command == kMavCmdNavWaypoint);
    REQUIRE(unpacked.x == 377749000);
    REQUIRE(unpacked.y == -1224194000);
    REQUIRE(unpacked.z == 100.0f);
    REQUIRE(unpacked.frame == kMavFrameGlobalRelAlt);
    REQUIRE(unpacked.current == 1);
    REQUIRE(unpacked.autocontinue == 1);

    Frame frame{};
    REQUIRE(make_frame(0, 1, 1, kMsgIdMissionItemInt, payload, frame));
    std::array<std::uint8_t, 64> wire{};
    const std::size_t n = encode_v2(frame, wire);
    REQUIRE(n == 10 + kMissionItemIntLen + 2);
    auto decoded = decode_v2(std::span<const std::uint8_t>(wire.data(), n));
    REQUIRE(decoded.has_value());
    REQUIRE(decoded.value().msgid == kMsgIdMissionItemInt);
    MissionItemInt from_frame{};
    REQUIRE(mission_item_int_from_frame(decoded.value(), from_frame));
    REQUIRE(from_frame.x == 377749000);
    REQUIRE(from_frame.command == kMavCmdNavWaypoint);
}

TEST_CASE("MISSION_COUNT stub pack/unpack", "[gcs][mission]") {
    MissionStore store;
    REQUIRE(mission_store_insert(store, make_waypoint(0, 1, 2, 10.0f)));
    const MissionCount count = make_mission_count(store, 255, 190, kMavMissionTypeMission);
    REQUIRE(count.count == 1);
    std::array<std::uint8_t, kMissionCountLen> payload{};
    REQUIRE(pack_mission_count(count, payload) == kMissionCountLen);
    MissionCount unpacked{};
    REQUIRE(unpack_mission_count(payload, unpacked));
    REQUIRE(unpacked.count == 1);
    REQUIRE(unpacked.target_system == 255);
    REQUIRE(unpacked.target_component == 190);
    REQUIRE(unpacked.mission_type == kMavMissionTypeMission);

    Frame frame{};
    REQUIRE(make_frame(0, 1, 1, kMsgIdMissionCount, payload, frame));
    std::array<std::uint8_t, 32> wire{};
    const std::size_t n = encode_v2(frame, wire);
    REQUIRE(n == 10 + kMissionCountLen + 2);
    auto decoded = decode_v2(std::span<const std::uint8_t>(wire.data(), n));
    REQUIRE(decoded.has_value());
    REQUIRE(decoded.value().msgid == kMsgIdMissionCount);
}

TEST_CASE("MISSION_REQUEST_INT replies with MISSION_ITEM_INT", "[gcs][mission]") {
    MissionStore store;
    REQUIRE(mission_store_insert(store, make_waypoint(0, 377749000, -1224194000, 100.0f)));
    GcsMavlink gcs;
    gcs.set_mission_store(store);

    const Frame in = frame_mission_request_int(0);
    const auto d = gcs.handle_message(in, 0);
    REQUIRE(d.kind == DispatchKind::kMissionRequestInt);
    REQUIRE(d.mission_item_found);
    REQUIRE(d.mission_item_int.seq == 0);
    REQUIRE(d.mission_item_int.x == 377749000);
    REQUIRE(d.mission_item_int.command == kMavCmdNavWaypoint);

    std::array<std::uint8_t, 64> wire{};
    const std::size_t n = gcs.handle_mission_request_int(in, wire);
    REQUIRE(n == 10 + kMissionItemIntLen + 2);
    auto decoded = decode_v2(std::span<const std::uint8_t>(wire.data(), n));
    REQUIRE(decoded.has_value());
    REQUIRE(decoded.value().msgid == kMsgIdMissionItemInt);
    MissionItemInt item{};
    REQUIRE(mission_item_int_from_frame(decoded.value(), item));
    REQUIRE(item.seq == 0);
    REQUIRE(item.y == -1224194000);
    REQUIRE(item.z == 100.0f);
}

TEST_CASE("MISSION_REQUEST_INT unknown seq does not crash", "[gcs][mission]") {
    MissionStore store;
    REQUIRE(mission_store_insert(store, make_waypoint(0, 1, 2, 10.0f)));
    GcsMavlink gcs;
    gcs.set_mission_store(store);

    const auto d = gcs.handle_mission_request_int_frame(frame_mission_request_int(9));
    REQUIRE(d.kind == DispatchKind::kMissionRequestInt);
    REQUIRE_FALSE(d.mission_item_found);

    std::array<std::uint8_t, 64> wire{};
    REQUIRE(gcs.handle_mission_request_int(frame_mission_request_int(9), wire) == 0);
}

TEST_CASE("ATTITUDE pack/unpack round-trip", "[gcs][attitude]") {
    Attitude a{};
    a.time_boot_ms = 123456789u;
    a.roll = 0.1f;
    a.pitch = -0.2f;
    a.yaw = 1.5f;
    a.rollspeed = 0.01f;
    a.pitchspeed = -0.02f;
    a.yawspeed = 0.03f;

    std::array<std::uint8_t, kAttitudeLen> payload{};
    REQUIRE(pack_attitude(a, payload) == kAttitudeLen);

    Attitude unpacked{};
    REQUIRE(unpack_attitude(payload, unpacked));
    REQUIRE(unpacked.time_boot_ms == 123456789u);
    REQUIRE(unpacked.roll == 0.1f);
    REQUIRE(unpacked.pitch == -0.2f);
    REQUIRE(unpacked.yaw == 1.5f);
    REQUIRE(unpacked.rollspeed == 0.01f);
    REQUIRE(unpacked.pitchspeed == -0.02f);
    REQUIRE(unpacked.yawspeed == 0.03f);
}

TEST_CASE("crc extra for ATTITUDE msgid 30", "[gcs][framing][attitude]") {
    std::uint8_t extra = 0;
    REQUIRE(crc_extra(kMsgIdAttitude, extra));
    REQUIRE(extra == kAttitudeCrcExtra);
    REQUIRE(extra == 39);
    REQUIRE(kMsgIdAttitude == 30);
}

TEST_CASE("leftover_send_attitude frames ATTITUDE with valid CRC", "[gcs][attitude]") {
    PlaneAttitudeInputs in{};
    in.time_boot_ms = 42;
    in.roll = 0.25f;
    in.pitch = -0.125f;
    in.yaw = 3.0f;
    in.rollspeed = 0.5f;
    in.pitchspeed = -0.25f;
    in.yawspeed = 0.125f;

    std::array<std::uint8_t, 64> wire{};
    const std::size_t n = leftover_send_attitude(in, wire, /*seq=*/7, /*sysid=*/1, /*compid=*/1);
    REQUIRE(n == 10 + kAttitudeLen + 2);
    REQUIRE(wire[0] == kStxV2);

    auto decoded = decode_v2(std::span<const std::uint8_t>(wire.data(), n));
    REQUIRE(decoded.has_value());
    REQUIRE(decoded.value().msgid == kMsgIdAttitude);
    REQUIRE(decoded.value().seq == 7);
    REQUIRE(decoded.value().payload_len == kAttitudeLen);

    Attitude out{};
    REQUIRE(attitude_from_frame(decoded.value(), out));
    REQUIRE(out.time_boot_ms == 42);
    REQUIRE(out.roll == 0.25f);
    REQUIRE(out.pitch == -0.125f);
    REQUIRE(out.yaw == 3.0f);
    REQUIRE(out.rollspeed == 0.5f);
    REQUIRE(out.pitchspeed == -0.25f);
    REQUIRE(out.yawspeed == 0.125f);
}

TEST_CASE("leftover_send_attitude short buffer returns 0", "[gcs][attitude]") {
    PlaneAttitudeInputs in{};
    std::array<std::uint8_t, 8> too_small{};
    REQUIRE(leftover_send_attitude(in, too_small) == 0);
}

TEST_CASE("COMMAND_INT fixture decodes with the common dialect CRC extra", "[gcs][framing]") {
    // Independent pymavlink-generated v2 packet: cmd176, sys7/comp1,
    // frame6, p1=1, p2=6; trailing current/autocontinue zeros truncated.
    std::array<std::uint8_t,45> wire = {0xfd,0x21,0x00,0x00,0x00,0xff,0xbe,0x4b,0x00,0x00,0x00,0x00,0x80,0x3f,0x00,0x00,0xc0,0x40,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xb0,0x00,0x07,0x01,0x06,0x35,0x18};
    auto decoded = decode_v2(wire);
    REQUIRE(decoded.has_value());
    REQUIRE(decoded.value().msgid == 75);
    REQUIRE(fwcpp::gcs::read_u16_le(decoded.value().payload.data()+28) == 176);
    REQUIRE(decoded.value().payload[30] == 7);
    REQUIRE(decoded.value().payload[31] == 1);
    REQUIRE(decoded.value().payload[32] == 6);
    wire.back() ^= 1;
    auto corrupt = decode_v2(wire);
    REQUIRE_FALSE(corrupt.has_value());
    REQUIRE(corrupt.error() == DecodeError::kBadCrc);
}
