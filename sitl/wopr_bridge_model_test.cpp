// Exercise the real INIT/model loader, including re-INIT defaults. No network
// listener or Catch2 installation is needed for this bounded reader regression.
#define main wopr_bridge_server_main
#include "wopr_bridge.cpp"
#undef main

namespace {
struct ModelLoaderTestAccess {
    static bool overrides(Bridge& b) {
        return b.sim_.prop_lapse_speed == 59.0f &&
               b.sim_.ground_brake_mu == 0.4f &&
               b.plane_.tecs.get_land_airspeed() == 18.0f &&
               b.plane_.aparm.ground_steer_alt == 7.0f &&
               b.plane_.mode_takeoff.ground_pitch == 2.0f &&
               b.plane_.aparm.flare_alt_m == 17.5f &&
               b.plane_.aparm.flare_sec == 4.5f &&
               b.plane_.tecs.mutable_gains().land_sink == 2.25f &&
               b.plane_.tecs.mutable_gains().land_sink_rate_change == 0.125f;
    }
    static bool defaults(Bridge& b) {
        return b.sim_.prop_lapse_speed == 0.0f &&
               b.sim_.ground_brake_mu == 0.0f &&
               b.sim_.wheel_brake == 0.0f &&
               b.plane_.tecs.get_land_airspeed() == -1.0f &&
               b.plane_.aparm.ground_steer_alt == 0.0f &&
               b.plane_.mode_takeoff.ground_pitch == 5.0f &&
               b.plane_.aparm.flare_alt_m == 3.0f &&
               b.plane_.aparm.flare_sec == 2.0f &&
               b.plane_.tecs.mutable_gains().land_sink == 0.25f &&
               b.plane_.tecs.mutable_gains().land_sink_rate_change == 0.0f;
    }
};
}

int main(int argc, char** argv) {
    if (argc != 2 || std::strlen(argv[1]) >= sizeof(InitPayload::model_json)) {
        return 2;
    }
    Bridge b;
    InitPayload p{};
    p.lat_1e7 = 348997800;
    p.lng_1e7 = -1178864000;
    p.alt_m = 700.0f;
    if (b.init(p) != Status::kOk || !ModelLoaderTestAccess::defaults(b)) {
        std::fprintf(stderr, "initial defaults differ\n");
        return 1;
    }
    std::strcpy(p.model_json, argv[1]);
    if (b.init(p) != Status::kOk || !ModelLoaderTestAccess::overrides(b)) {
        std::fprintf(stderr, "model keys did not reach their consumers after INIT\n");
        return 1;
    }
    p.model_json[0] = '\0';
    if (b.init(p) != Status::kOk || !ModelLoaderTestAccess::defaults(b)) {
        std::fprintf(stderr, "re-INIT retained stale model overrides\n");
        return 1;
    }
    // The same tyre setting must affect rollout but never airborne motion.
    for (bool airborne : {false, true}) {
        fwcpp::sim::SimPlane coast, brake;
        for (auto* sim : {&coast, &brake}) {
            sim->ground_level = 0;
            sim->home.alt = 0;
            sim->position.z = airborne ? -100.0f : 0.0f;
            sim->velocity_ef.x = 20.0f;
            sim->ground_friction = 0;
            sim->ground_brake_mu = 0.4f;
        }
        brake.wheel_brake = 1.0f;
        coast.update(0,0,0,0,0.02f);
        brake.update(0,0,0,0,0.02f);
        if (airborne ? brake.velocity_ef.x != coast.velocity_ef.x
                     : !(brake.velocity_ef.x < coast.velocity_ef.x && brake.velocity_ef.x >= 0)) {
            std::fprintf(stderr, "wheel brake contact gating failed\n"); return 1;
        }
    }
    std::puts("PASS: model overrides/reset and grounded-only wheel braking");
    return 0;
}
