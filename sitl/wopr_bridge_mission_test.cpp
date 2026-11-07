// Exercise conversion through the real bridge, without a second live vehicle.
#define main wopr_bridge_server_main
#include "wopr_bridge.cpp"
#undef main

int main(int argc, char** argv) {
    Bridge uninitialized;
    if (uninitialized.gcs_clear_mission()) return 18;
    Bridge bridge;
    InitPayload init{};
    init.lat_1e7 = 299675007;
    init.lng_1e7 = -816663971;
    init.alt_m = -4.3f;
    if (bridge.init(init) != Status::kOk) return 1;
    std::array<Bridge::GcsGeoItem, 4> rows{};
    for (std::size_t i = 0; i < rows.size(); ++i) {
        rows[i].seq = static_cast<std::uint16_t>(i);
        rows[i].command = 16;
        rows[i].frame = 3;
        rows[i].lat_1e7 = init.lat_1e7 + static_cast<int>(i) * 10000;
        rows[i].lng_1e7 = init.lng_1e7;
        rows[i].z = 120.0f;
    }
    rows[0].frame = 0;
    rows[1].command = 22;
    rows[3].command = 21;
    if (bridge.gcs_mission_load(rows.data(), rows.size()) != 3) return 2;
    const auto original_lat = bridge.plane().mission.current()->loc.lat;
    rows[1].lat_1e7 += 100000;
    for (const std::uint16_t unsupported : {17,18,19,20,31,178}) {
        rows[3].command = unsupported;
        if (bridge.gcs_mission_load(rows.data(), rows.size()) != -3) return 3;
        StateReply state{};
        bridge.fill_state(state);
        if (state.mission_count != 3) return 4;
        if (bridge.plane().mission.current()->loc.lat != original_lat) return 7;
    }
    rows[3].command = 21;
    rows[3].frame = 10;
    if (bridge.gcs_mission_load(rows.data(), rows.size()) != -2) return 5;
    rows[3].frame = 6;
    if (bridge.gcs_mission_load(rows.data(), rows.size()) != 3) return 6;
    // Replacing a running mission must apply its first NAV immediately. A
    // repeated AUTO request itself remains idempotent, as in ArduPlane.
    if (bridge.init(init) != Status::kOk) return 8;
    MissionItemWire waypoint{};
    waypoint.east_m = 20000.0f;
    waypoint.up_m = 500.0f;
    const ModePayload auto_mode{static_cast<std::uint8_t>(BridgeMode::kAuto)};
    if (bridge.set_mission(&waypoint, 1) != Status::kOk ||
        bridge.set_mode(auto_mode) != Status::kOk ||
        bridge.plane().next_WP_loc.alt != 50000) return 9;
    waypoint.up_m = 600.0f;
    const auto home_before_reload = bridge.plane().home;
    const_cast<fwcpp::vehicle::Plane&>(bridge.plane()).current_loc.lat = 222222;
    if (bridge.set_mission(&waypoint, 1) != Status::kOk ||
        bridge.plane().next_WP_loc.alt != 60000) {
        std::fputs("host replacement did not activate new NAV altitude\n", stderr);
        return 10;
    }
    if (bridge.plane().home.lat != home_before_reload.lat ||
        bridge.plane().home.lng != home_before_reload.lng ||
        bridge.plane().home.alt != home_before_reload.alt) return 17;
    // Test-only sentinel on the non-const fixture; no mutable production API.
    const_cast<fwcpp::vehicle::Plane&>(bridge.plane()).prev_WP_loc.lat = 123456;
    if (bridge.set_mode(auto_mode) != Status::kOk ||
        bridge.plane().prev_WP_loc.lat != 123456) return 11;

    std::array<Bridge::GcsGeoItem, 2> replacement{};
    replacement[0] = rows[0];
    replacement[1] = rows[1];
    replacement[1].command = 16;
    replacement[1].frame = 3;
    replacement[1].z = 700.0f;
    if (bridge.gcs_mission_load(replacement.data(), replacement.size()) != 1 ||
        bridge.plane().next_WP_loc.alt != 70000) return 12;
    replacement[1].frame = 10;
    if (bridge.gcs_mission_load(replacement.data(), replacement.size()) != -2 ||
        bridge.plane().next_WP_loc.alt != 70000 ||
        bridge.plane().mission.current()->loc.alt != 70000) return 13;

    const ModePayload manual_mode{static_cast<std::uint8_t>(BridgeMode::kManual)};
    if (bridge.set_mode(manual_mode) != Status::kOk) return 14;
    replacement[1].frame = 3;
    replacement[1].z = 800.0f;
    if (bridge.gcs_mission_load(replacement.data(), replacement.size()) != 1 ||
        bridge.current_mode() != BridgeMode::kManual) return 15;
    if (bridge.set_mode(auto_mode) != Status::kOk ||
        bridge.plane().next_WP_loc.alt != 80000) return 16;
    if (!bridge.gcs_clear_mission() || !bridge.plane().mission.empty() ||
        bridge.current_mode() != BridgeMode::kRtl) return 19;
    StateReply cleared{};
    bridge.fill_state(cleared);
    if (cleared.mission_count != 0 || cleared.mission_index != 0 || cleared.armed != 0) return 20;
    if (!bridge.gcs_clear_mission() || bridge.current_mode() != BridgeMode::kRtl) return 21;
    for (const auto mode : {BridgeMode::kManual, BridgeMode::kLoiter, BridgeMode::kFbwb}) {
        if (bridge.set_mission(&waypoint, 1) != Status::kOk ||
            bridge.set_mode(ModePayload{static_cast<std::uint8_t>(mode)}) != Status::kOk ||
            !bridge.gcs_clear_mission() || !bridge.plane().mission.empty() ||
            bridge.current_mode() != mode) return 22;
    }
    if (argc != 2 || std::strlen(argv[1]) >= sizeof(init.model_json)) return 23;
    std::strcpy(init.model_json, argv[1]);
    if (bridge.init(init) != Status::kOk) return 24;
    for (const auto mode : {BridgeMode::kQhover, BridgeMode::kQland}) {
        if (bridge.set_mission(&waypoint, 1) != Status::kOk ||
            bridge.set_mode(ModePayload{static_cast<std::uint8_t>(mode)}) != Status::kOk ||
            !bridge.gcs_clear_mission() || !bridge.plane().mission.empty() ||
            bridge.current_mode() != mode) return 25;
    }
    std::puts("PASS: mission validation, reload and empty-plan control ownership");
    return 0;
}
