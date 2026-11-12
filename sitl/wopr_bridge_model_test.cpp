// Exercise the real INIT/model loader, including re-INIT defaults. No network
// listener or Catch2 installation is needed for this bounded reader regression.
#define main wopr_bridge_server_main
#include "wopr_bridge.cpp"
#undef main

namespace {
struct ModelLoaderTestAccess {
    static bool rudder_mix(Bridge& b, float expected) {
        return b.initialized_ && b.plane_.aparm.kff_rudder_mix == expected;
    }
    static bool uninitialized(Bridge& b) { return !b.initialized_; }
    static bool host_true_airspeed(Bridge& b) {
        // At altitude with wind, TAS differs from both EAS and groundspeed.
        // Exercise the host packet boundary without changing controller EAS.
        b.sim_.airspeed = 100.0f;
        b.sim_.eas2tas = 1.3f;
        b.sim_.velocity_ef = {150.0f, 0.0f, 0.0f};
        StateReply reply{};
        b.fill_state(reply);
        if (std::fabs(reply.airspeed_mps - 130.0f) > .001f || b.sim_.airspeed != 100.0f) {
            return false;
        }
        b.sim_.eas2tas = 1.0f;
        b.fill_state(reply);
        return reply.airspeed_mps == 100.0f;
    }
    static bool overrides(Bridge& b) {
        return rudder_mix(b, .25f) &&
               b.plane_.aparm.loiter_radius == 4000.0f && b.plane_.aparm.rtl_radius == -4500.0f &&
               b.plane_.pitch_controller.rate_pid().kP() == .2f &&
               b.plane_.pitch_controller.rate_pid().kI() == .16f &&
               b.plane_.pitch_controller.rate_pid().kD() == .01f &&
               b.plane_.pitch_controller.rate_pid().ff() == .4f &&
               b.plane_.tecs.mutable_gains().land_time_const == 3.25f &&
               b.plane_.tecs.mutable_gains().land_damp == .8f &&
               b.plane_.tecs.mutable_gains().land_spd_weight == .4f &&
               b.plane_.aparm.landing_rollout_pitch_rate == 1.0f &&
               b.plane_.aparm.landing_reverse_throttle_pct == 30.0f &&
               b.sim_.ground_brake_pitch_free == 1.0f &&
               b.sim_.prop_lapse_speed == 59.0f &&
               b.sim_.ground_brake_mu == 0.4f &&
               b.sim_.ground_steering_wheelbase_m == 9.5f &&
               b.sim_.ground_steering_max_deg == 65.0f &&
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
        return rudder_mix(b, .5f) &&
               b.plane_.aparm.loiter_radius == 60.0f && b.plane_.aparm.rtl_radius == 0.0f &&
               b.plane_.pitch_controller.rate_pid().kP() == .04f &&
               b.plane_.pitch_controller.rate_pid().kI() == .15f &&
               b.plane_.pitch_controller.rate_pid().kD() == 0.0f &&
               b.plane_.pitch_controller.rate_pid().ff() == .345f &&
               b.plane_.tecs.mutable_gains().land_time_const == 2.0f &&
               b.plane_.tecs.mutable_gains().land_damp == .5f &&
               b.plane_.tecs.mutable_gains().land_spd_weight == -1.0f &&
               b.plane_.aparm.landing_rollout_pitch_rate == 0.0f &&
               b.plane_.aparm.landing_reverse_throttle_pct == 0.0f &&
               b.sim_.ground_brake_pitch_free == 0.0f &&
               b.sim_.prop_lapse_speed == 0.0f &&
               b.sim_.ground_brake_mu == 0.0f &&
               b.sim_.ground_steering_wheelbase_m == 0.0f &&
               !b.sim_.ground_steering_active &&
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
    if (!ModelLoaderTestAccess::host_true_airspeed(b)) {
        std::fprintf(stderr, "host speed must be TAS, not EAS or groundspeed\n");
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
    const std::string fixture_path = argv[1];
    const std::string fixture_dir = fixture_path.substr(0, fixture_path.find_last_of("/\\") + 1);
    // Exercise the real loader with endpoints and a fractional override,
    // restoring both an omitted-key model and model-less defaults each time.
    auto init_fixture = [&](const char* file) {
        const std::string path = fixture_dir + file;
        if (path.size() >= sizeof(p.model_json)) { return Status::kBadModel; }
        std::strcpy(p.model_json, path.c_str());
        return b.init(p);
    };
    for (const auto& entry : {std::pair{"wopr_rudder_zero.json", 0.0f},
                              std::pair{"wopr_bridge_model_overrides.json", .25f},
                              std::pair{"wopr_rudder_one.json", 1.0f}}) {
        if (init_fixture(entry.first) != Status::kOk ||
            !ModelLoaderTestAccess::rudder_mix(b, entry.second)) {
            std::fprintf(stderr, "rudder mix did not reach consumer: %s\n", entry.first); return 1;
        }
        if (init_fixture("wopr_rudder_omitted.json") != Status::kOk ||
            !ModelLoaderTestAccess::defaults(b)) {
            std::fprintf(stderr, "omitted rudder key retained stale override\n"); return 1;
        }
        if (init_fixture(entry.first) != Status::kOk) { return 1; }
        p.model_json[0] = '\0';
        if (b.init(p) != Status::kOk || !ModelLoaderTestAccess::defaults(b)) { return 1; }
    }
    for (const char* file : {"wopr_rudder_negative.json", "wopr_rudder_above_one.json",
                             "wopr_rudder_nonnumeric.json", "wopr_rudder_overflow.json",
                             "wopr_rudder_float_overflow.json", "wopr_rudder_null.json",
                             "wopr_rudder_bool.json", "wopr_rudder_near_above_one.json",
                             "wopr_rudder_near_negative.json"}) {
        if (init_fixture("wopr_bridge_model_overrides.json") != Status::kOk) { return 1; }
        if (init_fixture(file) != Status::kBadModel || !ModelLoaderTestAccess::uninitialized(b)) {
            std::fprintf(stderr, "invalid rudder mix accepted or bridge still initialized: %s\n", file);
            return 1;
        }
        StepPayload step{};
        if (b.step(step) != Status::kNotInitialized) {
            std::fprintf(stderr, "STEP accepted after bad rudder model: %s\n", file); return 1;
        }
        if (init_fixture("wopr_rudder_omitted.json") != Status::kOk ||
            !ModelLoaderTestAccess::defaults(b)) {
            std::fprintf(stderr, "clean re-INIT failed after invalid rudder model: %s\n", file); return 1;
        }
    }
    for (const char* file : {"wopr_pitch_negative.json", "wopr_pitch_nonnumeric.json", "wopr_pitch_overflow.json", "wopr_radius_nonnumeric.json", "wopr_radius_overflow.json", "wopr_radius_range.json"}) {
        const std::string invalid = fixture_dir + file;
        if (invalid.size() >= sizeof(p.model_json)) { return 2; }
        std::strcpy(p.model_json, invalid.c_str());
        if (b.init(p) != Status::kBadModel) {
            std::fprintf(stderr, "invalid pitch gain accepted: %s\n", file); return 1;
        }
    }
    p.model_json[0] = '\0';
    if (b.init(p) != Status::kOk || !ModelLoaderTestAccess::defaults(b)) { return 1; }
    // No fake vertical velocity on an elevated first tick or restart gap.
    fwcpp::tecs::Tecs seeded({}, {});
    fwcpp::tecs::TecsInputs seed;
    seed.now_us=20000; seed.now_ms=20;
    seed.relative_position_d_home_m=-1400; seed.baro_altitude_m=1400;
    seed.accel_ef_z=-fwcpp::sim::kGravityMss;
    seed.using_airspeed_sensor=true; seed.airspeed_eas_valid=true; seed.airspeed_eas=120;
    for (int i=0;i<100;++i) {
        seeded.update_50hz(seed);
        if (std::fabs(seeded.get_climb_rate()) > .001f ||
            std::fabs(seeded.get_tas_state()-120) > .001f) { return 1; }
        seed.now_us+=20000; seed.now_ms+=20;
    }
    seed.now_us+=2000000;seed.now_ms+=2000;
    seed.relative_position_d_home_m=-800;seed.baro_altitude_m=800;
    for (int i=0;i<100;++i) {
        seeded.update_50hz(seed);
        if (std::fabs(seeded.get_climb_rate()) > .001f) { return 1; }
        seed.now_us+=20000;seed.now_ms+=20;
    }
    seed.velocity_ned_valid=true;seed.velocity_down_ms=2.5f;
    seeded.update_50hz(seed);
    if (std::fabs(seeded.get_climb_rate()+2.5f) > .001f) { return 1; }
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
    // Contact projection residuals must not turn sub-micrometre/s upward
    // roundoff into an airborne state on the next small surface drop.
    {
        fwcpp::sim::SimPlane sim;
        sim.ground_behavior=fwcpp::sim::GroundBehavior::kFwdOnly;
        sim.home.alt=0;sim.ground_level=0;sim.frame_height=0;sim.position={};
        sim.ground_contact_drop_limit_m=.5f;sim.velocity_ef={30,0,-4.7683716e-7f};
        sim.set_terrain_height(-.02f);
        if (!sim.on_ground() || sim.velocity_ef.z!=0 || std::fabs(sim.position.z-.02f)>1e-6f) {return 1;}
    }
    // Real base dynamics must retain pitch momentum only for opted-in braking.
    for (int mode = 0; mode < 4; ++mode) {
        fwcpp::sim::SimPlane sim;
        sim.home.alt = 0; sim.ground_level = 0; sim.frame_height = 0;
        sim.ground_behavior = fwcpp::sim::GroundBehavior::kFwdOnly;
        sim.ground_brake_pitch_free = mode == 0 ? 0.0f : 1.0f;
        sim.wheel_brake = mode == 1 ? 0.0f : 1.0f;
        sim.dcm.from_euler(0, mode == 3 ? 0.0f : .05f, 0);
        sim.velocity_ef = {40,0,0}; sim.gyro = {0,-.1f,0};
        sim.update_dynamics({}, .02f);
        if (mode == 2 ? !(sim.gyro.y < 0) : sim.gyro.y != 0) {
            std::fprintf(stderr,"braking pitch dynamics gate failed %d\n",mode);return 1;
        }
    }
    // Identical non-flare inputs retain identical control outputs despite
    // different flare settings. Final uses its own TC, then exits cleanly.
    fwcpp::tecs::Tecs reference({}, {}), changed({}, {});
    changed.mutable_gains().land_time_const = 4;
    changed.mutable_gains().land_damp = 1;
    fwcpp::tecs::TecsInputs in;
    in.rotation_body_to_ned.identity(); in.eas2tas = 1;
    in.using_airspeed_sensor = in.airspeed_eas_valid = true;
    in.airspeed_eas = 12; in.velocity_ned_valid = true;
    in.relative_position_d_home_m = -10;
    for (unsigned i=1;i<=20;++i) {
        in.now_us=i*20000; in.now_ms=i*20;
        for (auto* t : {&reference,&changed}) {
            t->update_50hz(in); t->update_pitch_throttle(1000,1200,10,1,in);
        }
        if (reference.get_pitch_demand()!=changed.get_pitch_demand() ||
            reference.get_throttle_demand()!=changed.get_throttle_demand()) {return 1;}
    }
    fwcpp::tecs::TecsLandingInputs land;
    land.is_flaring = true; land.height_above_land_valid = true; land.height_above_land_m=10;
    in.now_us+=20000;in.now_ms+=20;changed.update_50hz(in);
    changed.update_pitch_throttle(0,1200,10,1,in,land);
    if (changed.debug_state().active_time_const != 4) {return 1;}
    changed.reset_landing();
    if (changed.debug_state().active_time_const != 5) {return 1;}
    // A target100m above home must blend at the same actual clearance as
    // target0. The old home-height substitution produced different rates.
    fwcpp::tecs::Tecs low({}, {}), high({}, {});
    for (int i=1;i<=150;++i) {
        const float h = 10.0f - i*.06f;
        if (i == 75) { high.offset_altitude(20); }
        for (auto pair : {std::pair{&low,0.0f},std::pair{&high,i >= 75 ? 80.0f : 100.0f}}) {
            auto v=in;v.now_us=i*20000;v.now_ms=i*20;
            v.relative_position_d_home_m=-(h+pair.second);v.velocity_down_ms=3;
            land.height_above_land_m=h;
            pair.first->update_50hz(v);
            pair.first->update_pitch_throttle(static_cast<int>(pair.second*100),1200,h+pair.second,1,v,land);
        }
        if (std::fabs(low.get_height_rate_demand()-high.get_height_rate_demand()) > .0001f) {return 1;}
    }
    auto& plane = const_cast<fwcpp::vehicle::Plane&>(b.plane());
    plane.landing_stage = fwcpp::vehicle::Plane::LandingStage::Final;
    fwcpp::vehicle::StabilizeInputs contact; contact.dt=.02f;
    contact.on_ground=true; contact.on_ground_valid=true;
    plane.update_landing_rollout(contact);
    if (plane.landing_stage != fwcpp::vehicle::Plane::LandingStage::Final) {return 1;}
    plane.aparm.landing_rollout_pitch_rate=1;
    contact.on_ground_valid=false;plane.update_landing_rollout(contact);
    if (plane.landing_stage != fwcpp::vehicle::Plane::LandingStage::Final) {return 1;}
    contact.on_ground_valid=true;plane.update_landing_rollout(contact);
    if (plane.landing_stage != fwcpp::vehicle::Plane::LandingStage::Rollout) {return 1;}
    plane.landing_rollout_pitch_deg=3;contact.on_ground=false;
    plane.update_landing_rollout(contact);
    if (!(plane.nav_pitch_cd>=297 && plane.nav_pitch_cd<=298)) {return 1;}
    if (!plane.set_mode(plane.mode_fbwb) ||
        plane.landing_stage != fwcpp::vehicle::Plane::LandingStage::Normal) {return 1;}
    plane.aparm.landing_reverse_throttle_pct=30;
    plane.armed=true;plane.landing_stage=fwcpp::vehicle::Plane::LandingStage::Rollout;
    contact.true_velocity_ned={30,0,0};contact.on_ground=true;contact.on_ground_valid=true;
    if (std::fabs(plane.landing_reverse_throttle(contact)+.4f)>.0001f) {return 1;}
    for (int i=0;i<100;++i) {plane.landing_reverse_throttle(contact);}
    if (plane.landing_reverse_output_pct!=30) {return 1;}
    contact.on_ground=false;
    if (plane.landing_reverse_throttle(contact)!=0) {return 1;}
    contact.on_ground=true;contact.true_velocity_ned={4,0,0};
    if (plane.landing_reverse_throttle(contact)!=0) {return 1;}
    contact.true_velocity_ned={30,0,0};plane.armed=false;
    if (plane.landing_reverse_throttle(contact)!=0) {return 1;}
    plane.armed=true;plane.landing_stage=fwcpp::vehicle::Plane::LandingStage::Final;
    if (plane.landing_reverse_throttle(contact)!=0) {return 1;}
    // Signed throttle reaches the actual plant and reduces forward momentum.
    fwcpp::sim::SimPlane idle, reverse;
    for (auto* sim : {&idle,&reverse}) {
        sim->home.alt=0;sim->ground_level=0;sim->frame_height=0;
        sim->position={};sim->velocity_ef={20,0,0};sim->ground_friction=0;
    }
    idle.update(0,0,0,0,.02f);reverse.update(0,0,0,-.3f,.02f);
    if (!(reverse.velocity_ef.x < idle.velocity_ef.x && reverse.velocity_ef.x > 0)) {return 1;}
    // Taxi steering changes heading only with rolling contact and explicit
    // opt-in; the same input must not rotate a parked or airborne aircraft.
    for (int scenario=0;scenario<5;++scenario) {
        fwcpp::sim::SimPlane sim;
        sim.home.alt=0; sim.ground_level=0; sim.frame_height=0;
        sim.ground_behavior=fwcpp::sim::GroundBehavior::kFwdOnly;
        sim.position={0,0,scenario==1 ? -100.0f : 0.0f};
        sim.velocity_ef={scenario==2 ? 0.0f : 3.0f,0,0};
        sim.ground_steering_wheelbase_m=scenario==3 ? 0.0f : 9.5f;
        sim.ground_steering_max_deg=65;sim.ground_steering_input=1;
        sim.ground_steering_active=scenario!=4;
        sim.update(0,0,0,0,.02f);
        float r,p,y;sim.dcm.to_euler(&r,&p,&y);
        if (scenario==0 ? !(y>.01f && y<.02f) : std::fabs(y)>1e-6f) {
            std::fprintf(stderr,"taxi steering gate failed %d yaw=%f\n",scenario,y);return 1;
        }
    }
    // A contact-tolerance upward residual must be arrested by downforce;
    // actual upward lift must still release the wheels (no sticky landing).
    for (bool lifting : {false, true}) {
        fwcpp::sim::Aircraft sim;
        sim.ground_behavior=fwcpp::sim::GroundBehavior::kFwdOnly;
        sim.ground_pitch_rate_enabled=true;
        sim.velocity_ef={80,0,-.001f};
        for (int i=0;i<10;++i) {
            sim.accel_body={0,0,-fwcpp::sim::kGravityMss+(lifting ? -2.0f : 2.0f)};
            sim.update_dynamics({},.02f);
        }
        if (lifting ? sim.on_ground() : (!sim.on_ground() || sim.velocity_ef.z!=0)) {
            std::fprintf(stderr,"contact force release/regain failed\n");return 1;
        }
    }
    // 50 Hz integration at the transfer flight's 485 km offset must preserve
    // net travel; wire positions may quantize, but every tick must not do so.
    fwcpp::sim::Aircraft distant;
    distant.position={18000,-485000,-1000};
    distant.velocity_ef={78,-10.6f,0};
    for (int i=0;i<5000;++i) {
        distant.accel_body={0,0,-fwcpp::sim::kGravityMss};
        distant.update_dynamics({},.02f);
    }
    if (std::fabs(distant.position.x-25800)>.02f ||
        std::fabs(distant.position.y+486060)>.04f) {
        std::fprintf(stderr,"distant integration loses travel\n");return 1;
    }
    distant.position={1,2,-1000};distant.velocity_ef={0,0,0};
    distant.accel_body={0,0,-fwcpp::sim::kGravityMss};distant.update_dynamics({},.02f);
    if (distant.position.x!=1 || distant.position.y!=2) {return 1;}
    std::puts("PASS: model/reset, taxi, brakes, terrain, flare, contact release and distant integration");
    return 0;
}
