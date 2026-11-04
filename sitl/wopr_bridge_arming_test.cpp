// Exercise the real bridge/controller/plant boundary, without a live editor.
#define main wopr_bridge_server_main
#include "wopr_bridge.cpp"
#undef main

#define CHECK(x) do { if (!(x)) { std::fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #x); return 1; } } while (0)

int main() {
    Bridge b;
    InitPayload init{};
    init.lat_1e7=299666143; init.lng_1e7=-816627354;
    CHECK(b.init(init)==Status::kOk);
    std::array<Bridge::GcsGeoItem,3> rows{};
    for (unsigned i=0;i<rows.size();++i) {
        auto& r=rows[i]; r.seq=i; r.command=i==1 ? 22 : 16;
        r.frame=i==0 ? 0 : 3; r.lat_1e7=init.lat_1e7+100000*i;
        r.lng_1e7=init.lng_1e7; r.z=i==0 ? 0 : 120; r.p1=10;
    }
    CHECK(b.gcs_mission_load(rows.data(),rows.size())==2);
    CHECK(b.gcs_set_mode_custom(10));
    b.gcs_arm(true); b.gcs_arm(false); b.gcs_arm(false);
    CHECK(b.plane().hal.rc_output.safety_state()==fwcpp::hal::SafetyState::kDisarmed);
    rows[2].lat_1e7+=10000;
    CHECK(b.gcs_mission_load(rows.data(),rows.size())==2);
    StepPayload step{}; step.n_steps=1;
    float demand=0;
    for(int i=0;i<1500;++i) {
        CHECK(b.step(step)==Status::kOk);
        demand=std::max(demand,b.plane().srv_channels.get_output_scaled(fwcpp::srv::Function::kThrottle));
        StateReply s{};b.fill_state(s);
        CHECK(!s.armed && s.on_ground && s.servo_norm[3]==0 && s.mission_count==2);
        CHECK(std::abs(s.pos_ned_m[0])<.001f && std::abs(s.pos_ned_m[1])<.001f);
    }
    CHECK(demand>10); // AUTO is actively asking for thrust: not a vacuous idle test.
    b.gcs_arm(true);
    for(int i=0;i<100;++i) CHECK(b.step(step)==Status::kOk);
    StateReply moving{};b.fill_state(moving);
    CHECK(moving.armed && moving.servo_norm[3]>0 && moving.airspeed_mps>1);
    b.gcs_arm(false); CHECK(b.step(step)==Status::kOk);
    StateReply coast{};b.fill_state(coast);
    CHECK(!coast.armed && coast.servo_norm[3]==0 && coast.sim_time_ms>moving.sim_time_ms);
    CHECK(coast.pos_ned_m[0]!=moving.pos_ned_m[0] || coast.pos_ned_m[1]!=moving.pos_ned_m[1]);

    // Manual high-stick must obey the same gate on fresh boot and after disarm.
    for(int cycle=0;cycle<2;++cycle) {
        CHECK(b.init(init)==Status::kOk);
        if(cycle) { b.gcs_arm(true); b.gcs_arm(false); }
        step.rc_pwm[2]=1900;
        for(int i=0;i<1500;++i) CHECK(b.step(step)==Status::kOk);
        StateReply s{};b.fill_state(s);
        CHECK(!s.armed && s.servo_norm[3]==0 && s.airspeed_mps==0);
        CHECK(s.pos_ned_m[0]==0 && s.pos_ned_m[1]==0);
        b.gcs_arm(true);
        for(int i=0;i<100;++i) CHECK(b.step(step)==Status::kOk);
        b.fill_state(s);CHECK(s.servo_norm[3]>.5f && s.airspeed_mps>1);
    }

    // Signed gate preserves reverse when armed, blocks both signs otherwise.
    fwcpp::vehicle::Plane p;
    for(float demand_pct : {-30.0f,70.0f}) {
        p.srv_channels.set_output_scaled(fwcpp::srv::Function::kThrottle,demand_pct);
        for(bool armed : {false,true}) for(bool safety_off : {false,true}) {
            p.armed=armed;
            if(safety_off) p.hal.rc_output.force_safety_off(); else p.hal.rc_output.force_safety_on();
            CHECK(p.effective_throttle_pct()==(armed && safety_off ? demand_pct : 0.0f));
        }
    }
    std::puts("PASS: disarmed AUTO reload/manual cannot propel; arm accelerates; disarm coasts; signed safety gate");
}
