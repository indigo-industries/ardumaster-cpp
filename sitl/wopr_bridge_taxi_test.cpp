// Real RC -> bridge -> plant regression, without an editor or UDP listener.
#define main wopr_bridge_server_main
#include "wopr_bridge.cpp"
#undef main

#define CHECK(x) do { if (!(x)) { std::fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #x); return 1; } } while (0)

namespace {
struct ModelLoaderTestAccess {
    static void seed(Bridge& b, bool airborne, float speed=3.0f) {
        b.sim_.position.z = airborne ? -100.0f : 0.0f;
        b.sim_.velocity_ef = {speed,0,0};
    }
};
float heading(const fwcpp::sim::SimPlane& sim) {
    float roll,pitch,yaw; sim.dcm.to_euler(&roll,&pitch,&yaw); return yaw;
}
}

int main(int argc, char** argv) {
    CHECK(argc==2 && std::strlen(argv[1])<sizeof(InitPayload::model_json));
    InitPayload init{};
    init.lat_1e7=299665823; init.lng_1e7=-816627719;
    std::strcpy(init.model_json,argv[1]);
    StepPayload rc{}; rc.n_steps=1; rc.rc_pwm[2]=800; rc.rc_pwm[3]=1900;

    // Braking is a below-idle RC axis only for the opted-in manual profile.
    // Test controller mode gating as well as physical off-ground force gating.
    for (BridgeMode mode : {BridgeMode::kManual,BridgeMode::kFbwa,BridgeMode::kAuto}) {
        for(bool airborne : {false,true}) {
            Bridge b; CHECK(b.init(init)==Status::kOk);
            ModelLoaderTestAccess::seed(b,airborne);
            ModePayload m{};m.mode=static_cast<std::uint8_t>(mode);
            CHECK(b.set_mode(m)==Status::kOk);
            CHECK(b.step(rc)==Status::kOk);
            const bool manual=mode==BridgeMode::kManual;
            CHECK(b.sim().ground_steering_active==manual);
            CHECK(b.sim().wheel_brake==(manual ? 1.0f : 0.0f));
            CHECK((b.sim().last_brake_accel>0)==(manual && !airborne));
            CHECK(!manual || b.plane().effective_throttle_pct()==0);
            if(manual && !airborne) CHECK(heading(b.sim())>.01f && heading(b.sim())<.02f);
        }
    }
    // Armed throttle below idle remains zero thrust; brake range and release.
    Bridge b; CHECK(b.init(init)==Status::kOk);b.gcs_arm(true);
    for(auto pwm : {800,950,1100,1200}) {
        ModelLoaderTestAccess::seed(b,false);
        rc.rc_pwm[2]=static_cast<std::uint16_t>(pwm);rc.rc_pwm[3]=1500;
        CHECK(b.step(rc)==Status::kOk);
        const float fraction=pwm<=1100 ? (1100-pwm)/300.0f : 0.0f;
        CHECK(std::fabs(b.sim().wheel_brake-fraction)<1e-6f);
        CHECK(pwm>1100 || b.plane().effective_throttle_pct()==0);
    }
    // Model disabled: neither steering nor manual brake may leak in.
    init.model_json[0]='\0'; CHECK(b.init(init)==Status::kOk);
    ModelLoaderTestAccess::seed(b,false);rc.rc_pwm[2]=800;rc.rc_pwm[3]=1900;
    CHECK(b.step(rc)==Status::kOk);
    CHECK(!b.sim().ground_steering_active && b.sim().wheel_brake==0 && b.sim().last_brake_accel==0);

    // Compare physical yaw against the unchanged plant when airborne or
    // above taxi speed. This catches accidental suppression of rudder torque.
    for(bool airborne : {false,true}) {
        fwcpp::sim::SimPlane old, taxi;
        for(auto* sim : {&old,&taxi}) {
            sim->ground_behavior=fwcpp::sim::GroundBehavior::kFwdOnly;
            sim->position.z=airborne ? -100.0f : 0.0f;
            sim->velocity_ef={20,0,0};sim->velocity_air_bf={20,0,0};sim->airspeed=20;
            sim->moment_inertia={1000,1000,1000};
        }
        taxi.ground_steering_active=true;taxi.ground_steering_input=1;
        taxi.ground_steering_wheelbase_m=9.5f;taxi.ground_steering_max_deg=65;
        old.update(0,0,1,0,.02f);taxi.update(0,0,1,0,.02f);
        CHECK(heading(old)==heading(taxi));
        CHECK((old.gyro_for_sensors()-taxi.gyro_for_sensors()).length()<1e-6f);
    }

    // Sustained rolling 180-degree turn: true integration, no pose reset.
    // Heavy model coasts through the turn in the taxi speed band.
    fwcpp::sim::SimPlane turn;
    turn.ground_behavior=fwcpp::sim::GroundBehavior::kFwdOnly;
    turn.mass=55000;turn.moment_inertia={1492857,1492857,2828571};
    turn.ground_friction=.0035f;turn.ground_brake_mu=.45f;
    turn.ground_steering_active=true;turn.ground_steering_input=1;
    turn.ground_steering_wheelbase_m=9.5f;turn.ground_steering_max_deg=65;
    turn.velocity_ef={3,0,0};
    float total_yaw=0,previous=0,max_radius=0;
    int ticks=0;
    while(total_yaw<fwcpp::math::radians(180.0f) && ticks++<1000) {
        turn.update(0,0,0,0,.02f);
        const float h=heading(turn);float delta=h-previous;
        if(delta< -3)delta+=fwcpp::math::radians(360.0f);
        total_yaw+=delta;previous=h;
        max_radius=std::fmax(max_radius,std::sqrt(turn.position.x*turn.position.x+turn.position.y*turn.position.y));
        CHECK(turn.on_ground());
    }
    CHECK(ticks<1000 && max_radius<12.0f && turn.position.y>8.0f && turn.position.y<11.0f);
    turn.ground_steering_input=0;turn.wheel_brake=1;
    for(int i=0;i<150;++i)turn.update(0,0,0,0,.02f);
    CHECK(turn.velocity_ef.length()<.01f);
    // Exercise sensors/controller as well as truth during the turnaround.
    std::strcpy(init.model_json,argv[1]);CHECK(b.init(init)==Status::kOk);
    ModelLoaderTestAccess::seed(b,false);b.gcs_arm(true);
    rc.rc_pwm[2]=1100;rc.rc_pwm[3]=1900;
    float bridge_yaw=0,last_heading=heading(b.sim());int bridge_ticks=0;
    while(bridge_yaw<fwcpp::math::radians(180.0f) && bridge_ticks++<1000) {
        CHECK(b.step(rc)==Status::kOk);
        const float h=heading(b.sim());float delta=h-last_heading;
        if(delta< -3)delta+=fwcpp::math::radians(360.0f);
        bridge_yaw+=delta;last_heading=h;
    }
    float heading_error=b.plane().ahrs->get_yaw()-heading(b.sim());
    while(heading_error>fwcpp::math::radians(180.0f))heading_error-=fwcpp::math::radians(360.0f);
    while(heading_error<fwcpp::math::radians(-180.0f))heading_error+=fwcpp::math::radians(360.0f);
    std::printf("Bridge turnaround %.2fs, AHRS/truth yaw error %.3fdeg\n",bridge_ticks*.02f,fwcpp::math::degrees(heading_error));
    CHECK(bridge_ticks<1000 && std::fabs(heading_error)<fwcpp::math::radians(5.0f));
    rc.rc_pwm[2]=800;rc.rc_pwm[3]=1500;CHECK(b.step(rc)==Status::kOk);
    CHECK(b.sim().ground_steering_sensor_yaw_rate==0);
    ModePayload exit{};exit.mode=static_cast<std::uint8_t>(BridgeMode::kFbwa);
    CHECK(b.set_mode(exit)==Status::kOk && b.step(rc)==Status::kOk);
    CHECK(!b.sim().ground_steering_active && b.sim().wheel_brake==0 &&
        b.sim().ground_steering_sensor_yaw_rate==0);
    std::printf("PASS: taxi RC/mode/contact/default gates, high-speed yaw parity; 180deg turn %.2fs, max displacement %.3fm, braked stop\n",ticks*.02f,max_radius);
}
