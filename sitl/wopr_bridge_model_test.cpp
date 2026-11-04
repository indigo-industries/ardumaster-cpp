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
               b.sim_.ground_contact_drop_limit_m == 0.5f &&
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
               b.sim_.ground_contact_drop_limit_m == 0.0f &&
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
    for (int scenario = 0; scenario < 6; ++scenario) {
        fwcpp::sim::SimPlane sim;
        sim.ground_behavior = fwcpp::sim::GroundBehavior::kFwdOnly;
        sim.ground_contact_drop_limit_m = scenario == 5 ? 0.0f : 0.5f;
        sim.home.alt = 0; sim.ground_level = 0; sim.frame_height = 0;
        sim.position = {10,20, scenario == 1 ? -10.0f : 0.0f};
        sim.velocity_ef = {30,0, scenario == 2 ? -0.01f : 0.0f};
        const auto old_pos = sim.position; const auto old_vel = sim.velocity_ef;
        const float surface = scenario == 3 ? -1.0f : scenario == 4 ? 0.2f : -0.2f;
        sim.set_terrain_height(surface);
        const float expected_z = scenario == 0 ? 0.2f : old_pos.z;
        if (sim.position.z != expected_z || sim.position.x != old_pos.x || sim.position.y != old_pos.y ||
            sim.velocity_ef.x != old_vel.x || sim.velocity_ef.z != old_vel.z || sim.ground_level != surface ||
            (scenario == 0 && !sim.on_ground())) {
            std::fprintf(stderr, "terrain support gating failed: scenario %d\n",scenario); return 1;
        }
    }
    std::puts("PASS: model overrides/reset, ground-only brakes and bounded terrain support");
    return 0;
}
