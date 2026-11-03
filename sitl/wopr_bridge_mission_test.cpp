// Exercise conversion through the real bridge, without a second live vehicle.
#define main wopr_bridge_server_main
#include "wopr_bridge.cpp"
#undef main

int main() {
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
    std::puts("PASS: supported mission loads; unsupported commands/frames reject atomically");
    return 0;
}
