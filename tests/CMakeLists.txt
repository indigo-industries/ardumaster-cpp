# Parity/unit tests land here as modules are ported, one target per module
# under test - mirrors fw-rust's per-crate tests/ directories.
#
# Framework: Catch2 3.x (system package `catch2`, found via find_package
# below - no FetchContent, no network access needed to configure or build).
# Chosen over doctest/GoogleTest for BDD-style SECTION blocks, which suit the
# scenario-table fixture format tools/parity already emits for fw-rust and
# that this effort intends to read directly where a fixture is
# upstream-behavior data rather than Rust- or C++-specific (see the fw-cpp
# effort charter). Amendable per-ticket if a module's test shape argues
# otherwise; record the reason in that ticket's notes if so.

# Fixture locations, so no test hardcodes one machine's absolute paths
# (they used to point at /srv/ardumaster/... and only resolved on one dev box).
#
#   FWCPP_SOURCE_DIR         - this repo, for fixtures committed alongside the
#                              code; those must exist, so tests REQUIRE them.
#   FWCPP_RUST_FIXTURES_DIR  - ardumaster-rust's shared parity data. That is a
#                              SEPARATE repo which may not be checked out, so
#                              tests reading it SKIP when it is missing rather
#                              than fail. Defaults to the sibling layout the
#                              README describes; override with -D at configure
#                              time or the same-named env var at run time.
set(FWCPP_RUST_FIXTURES_DIR "${CMAKE_SOURCE_DIR}/../ardumaster-rust/fixtures"
    CACHE PATH "ardumaster-rust fixtures dir (cross-repo parity data; tests skip if absent)")
add_compile_definitions(
    FWCPP_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
    FWCPP_RUST_FIXTURES_DIR="${FWCPP_RUST_FIXTURES_DIR}")

find_package(Catch2 3 REQUIRED)
include(CTest)
include(Catch)

add_executable(result_test result_test.cpp)
target_link_libraries(result_test PRIVATE fwcpp-result Catch2::Catch2WithMain)
catch_discover_tests(result_test)

add_executable(internal_error_test internal_error_test.cpp)
target_link_libraries(internal_error_test PRIVATE ap-internal-error Catch2::Catch2WithMain)
catch_discover_tests(internal_error_test)

add_executable(scalar_test scalar_test.cpp)
target_link_libraries(scalar_test PRIVATE ap-math Catch2::Catch2WithMain)
catch_discover_tests(scalar_test)

add_executable(matrix_alg_test matrix_alg_test.cpp)
target_link_libraries(matrix_alg_test PRIVATE ap-math Catch2::Catch2WithMain)
catch_discover_tests(matrix_alg_test)

add_executable(vector2_test vector2_test.cpp)
target_link_libraries(vector2_test PRIVATE ap-math Catch2::Catch2WithMain)
catch_discover_tests(vector2_test)

add_executable(vector3_test vector3_test.cpp)
target_link_libraries(vector3_test PRIVATE ap-math Catch2::Catch2WithMain)
catch_discover_tests(vector3_test)

add_executable(matrix3_test matrix3_test.cpp)
target_link_libraries(matrix3_test PRIVATE ap-math Catch2::Catch2WithMain)
catch_discover_tests(matrix3_test)

add_executable(quaternion_test quaternion_test.cpp)
target_link_libraries(quaternion_test PRIVATE ap-math Catch2::Catch2WithMain)
catch_discover_tests(quaternion_test)

add_executable(bitmask_test bitmask_test.cpp)
target_link_libraries(bitmask_test PRIVATE ap-common Catch2::Catch2WithMain)
catch_discover_tests(bitmask_test)

add_executable(location_test location_test.cpp)
target_link_libraries(location_test PRIVATE ap-common Catch2::Catch2WithMain)
catch_discover_tests(location_test)

add_executable(float16_test float16_test.cpp)
target_link_libraries(float16_test PRIVATE ap-common Catch2::Catch2WithMain)
catch_discover_tests(float16_test)

add_executable(low_pass_filter_2p_test low_pass_filter_2p_test.cpp)
target_link_libraries(low_pass_filter_2p_test PRIVATE ap-filter Catch2::Catch2WithMain)
catch_discover_tests(low_pass_filter_2p_test)

add_executable(low_pass_filter_test low_pass_filter_test.cpp)
target_link_libraries(low_pass_filter_test PRIVATE ap-filter Catch2::Catch2WithMain)
catch_discover_tests(low_pass_filter_test)

add_executable(slew_limiter_test slew_limiter_test.cpp)
target_link_libraries(slew_limiter_test PRIVATE ap-filter Catch2::Catch2WithMain)
catch_discover_tests(slew_limiter_test)

add_executable(ac_pid_test ac_pid_test.cpp)
target_link_libraries(ac_pid_test PRIVATE ap-pid Catch2::Catch2WithMain)
catch_discover_tests(ac_pid_test)

add_executable(l1_control_test l1_control_test.cpp)
# CPP-048: l1_control.hpp now also includes fwcpp/param/* headers (the
# new top-level AP_Param Info[] table for L1Control::Gains) - ap-nav's
# own CMakeLists.txt does not link ap-param (out of this ticket's touch
# scope: only l1_control.hpp, its own new test file, and this file are
# allowed), so ap-param is linked directly here instead, for this
# pre-existing test target to keep resolving those headers.
target_link_libraries(l1_control_test PRIVATE ap-nav ap-param Catch2::Catch2WithMain)
catch_discover_tests(l1_control_test)

# CPP-048: storage/persistence integration test for L1Control::Gains's
# real top-level AP_Param Info[] table - no ap-sim needed (not a
# closed-loop test, see the ticket's own "NOT a SimPlane closed-loop
# test" instruction, matching CPP-043's own plane_aparm_param_test).

add_executable(l1_control_param_test l1_control_param_test.cpp)
target_link_libraries(l1_control_param_test PRIVATE ap-nav ap-param Catch2::Catch2WithMain)
catch_discover_tests(l1_control_param_test)

add_executable(notch_filter_test notch_filter_test.cpp)
target_link_libraries(notch_filter_test PRIVATE ap-filter Catch2::Catch2WithMain)
catch_discover_tests(notch_filter_test)

add_executable(vector2_geometry_test vector2_geometry_test.cpp)
target_link_libraries(vector2_geometry_test PRIVATE ap-math Catch2::Catch2WithMain)
catch_discover_tests(vector2_geometry_test)

add_executable(rotations_test rotations_test.cpp)
target_link_libraries(rotations_test PRIVATE ap-math Catch2::Catch2WithMain)
catch_discover_tests(rotations_test)

add_executable(quaternion_rotation_test quaternion_rotation_test.cpp)
target_link_libraries(quaternion_rotation_test PRIVATE ap-math Catch2::Catch2WithMain)
catch_discover_tests(quaternion_rotation_test)

add_executable(vector3_matops_test vector3_matops_test.cpp)
target_link_libraries(vector3_matops_test PRIVATE ap-math Catch2::Catch2WithMain)
catch_discover_tests(vector3_matops_test)

add_executable(postype_test postype_test.cpp)
target_link_libraries(postype_test PRIVATE ap-math Catch2::Catch2WithMain)
catch_discover_tests(postype_test)

add_executable(storage_test storage_test.cpp)
target_link_libraries(storage_test PRIVATE ap-param Catch2::Catch2WithMain)
catch_discover_tests(storage_test)

add_executable(param_test param_test.cpp)
target_link_libraries(param_test PRIVATE ap-param Catch2::Catch2WithMain)
catch_discover_tests(param_test)

add_executable(group_info_test group_info_test.cpp)
target_link_libraries(group_info_test PRIVATE ap-param Catch2::Catch2WithMain)
catch_discover_tests(group_info_test)

add_executable(name_lookup_test name_lookup_test.cpp)
target_link_libraries(name_lookup_test PRIVATE ap-param Catch2::Catch2WithMain)
catch_discover_tests(name_lookup_test)

add_executable(persistence_test persistence_test.cpp)
target_link_libraries(persistence_test PRIVATE ap-param Catch2::Catch2WithMain)
catch_discover_tests(persistence_test)

add_executable(defaults_test defaults_test.cpp)
target_link_libraries(defaults_test PRIVATE ap-param Catch2::Catch2WithMain)
catch_discover_tests(defaults_test)

add_executable(top_level_test top_level_test.cpp)
target_link_libraries(top_level_test PRIVATE ap-param Catch2::Catch2WithMain)
catch_discover_tests(top_level_test)

add_executable(native_value_test native_value_test.cpp)
target_link_libraries(native_value_test PRIVATE ap-param Catch2::Catch2WithMain)
catch_discover_tests(native_value_test)

add_executable(notch_filter_params_test notch_filter_params_test.cpp)
target_link_libraries(notch_filter_params_test PRIVATE ap-filter ap-param Catch2::Catch2WithMain)
catch_discover_tests(notch_filter_params_test)

add_executable(filter_registry_test filter_registry_test.cpp)
target_link_libraries(filter_registry_test PRIVATE ap-filter Catch2::Catch2WithMain)
catch_discover_tests(filter_registry_test)

add_executable(scheduler_test scheduler_test.cpp)
target_link_libraries(scheduler_test PRIVATE ap-scheduler Catch2::Catch2WithMain)
catch_discover_tests(scheduler_test)

add_executable(rc_test rc_test.cpp)
target_link_libraries(rc_test PRIVATE ap-hal Catch2::Catch2WithMain)
catch_discover_tests(rc_test)

add_executable(rc_channel_test rc_channel_test.cpp)
target_link_libraries(rc_channel_test PRIVATE ap-rc-channel Catch2::Catch2WithMain)
catch_discover_tests(rc_channel_test)

add_executable(uart_driver_test uart_driver_test.cpp)
target_link_libraries(uart_driver_test PRIVATE ap-hal Catch2::Catch2WithMain)
catch_discover_tests(uart_driver_test)

add_executable(analog_in_test analog_in_test.cpp)
target_link_libraries(analog_in_test PRIVATE ap-hal Catch2::Catch2WithMain)
catch_discover_tests(analog_in_test)

add_executable(srv_channel_test srv_channel_test.cpp)
target_link_libraries(srv_channel_test PRIVATE ap-srv-channel Catch2::Catch2WithMain)
catch_discover_tests(srv_channel_test)

add_executable(hal_context_test hal_context_test.cpp)
target_link_libraries(hal_context_test PRIVATE ap-hal Catch2::Catch2WithMain)
catch_discover_tests(hal_context_test)

add_executable(rc_channels_test rc_channels_test.cpp)
target_link_libraries(rc_channels_test PRIVATE ap-rc-channel Catch2::Catch2WithMain)
catch_discover_tests(rc_channels_test)

add_executable(srv_channels_test srv_channels_test.cpp)
target_link_libraries(srv_channels_test PRIVATE ap-srv-channel Catch2::Catch2WithMain)
catch_discover_tests(srv_channels_test)

add_executable(ahrs_dcm_test ahrs_dcm_test.cpp)
target_link_libraries(ahrs_dcm_test PRIVATE ap-ahrs Catch2::Catch2WithMain)
catch_discover_tests(ahrs_dcm_test)

# CPP-028 leftover closer: healthy/pre_arm_check/groundspeed/airspeed/
# relative-position production stubs + leftover-complete catalog.
# Links ap-airspeed (real sensor feeding airspeed_EAS) and ap-ekf
# (EkfCoreBackend must implement the widened AhrsBackend surface).

add_executable(ahrs_leftover_test ahrs_leftover_test.cpp)
target_link_libraries(ahrs_leftover_test PRIVATE ap-ahrs ap-airspeed ap-ekf Catch2::Catch2WithMain)
catch_discover_tests(ahrs_leftover_test)

add_executable(sim_plane_test sim_plane_test.cpp)
target_link_libraries(sim_plane_test PRIVATE ap-sim Catch2::Catch2WithMain)
# CPP-094: absolute path to this test's real, byte-for-byte upstream fixture
# (tests/fixtures/skywalker_2013.json) - no established fixture-path
# convention existed anywhere else in this tree to reuse, so this
# CMAKE_CURRENT_SOURCE_DIR-derived compile definition is the new one;
# an absolute path avoids any dependence on ctest's/the test binary's
# working directory.
target_compile_definitions(sim_plane_test PRIVATE
    FWCPP_SIM_PLANE_FIXTURES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/fixtures")
catch_discover_tests(sim_plane_test)

add_executable(sim_multicopter_test sim_multicopter_test.cpp)
target_link_libraries(sim_multicopter_test PRIVATE ap-sim Catch2::Catch2WithMain)
catch_discover_tests(sim_multicopter_test)

# CCP-067: Frame::load_frame_params() round-trip tests against real upstream
# Callisto.json/freestyle.json fixtures - same fixture-path convention
# CPP-094 established for sim_plane_test above (an absolute,
# CMAKE_CURRENT_SOURCE_DIR-derived compile definition, so the test binary
# does not depend on ctest's/the binary's own working directory).
add_executable(sim_frame_test sim_frame_test.cpp)
target_link_libraries(sim_frame_test PRIVATE ap-sim Catch2::Catch2WithMain)
target_compile_definitions(sim_frame_test PRIVATE
    FWCPP_SIM_FRAME_FIXTURES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/fixtures")
catch_discover_tests(sim_frame_test)

add_executable(sim_leftover_test sim_leftover_test.cpp)
target_link_libraries(sim_leftover_test PRIVATE ap-sim Catch2::Catch2WithMain)
catch_discover_tests(sim_leftover_test)

add_executable(sim_plants_test sim_plants_test.cpp)
target_link_libraries(sim_plants_test PRIVATE ap-sim Catch2::Catch2WithMain)
catch_discover_tests(sim_plants_test)

add_executable(sim_gps_protocols_test sim_gps_protocols_test.cpp)
target_link_libraries(sim_gps_protocols_test PRIVATE ap-sim Catch2::Catch2WithMain)
catch_discover_tests(sim_gps_protocols_test)

add_executable(sim_rf_protocols_test sim_rf_protocols_test.cpp)
target_link_libraries(sim_rf_protocols_test PRIVATE ap-sim Catch2::Catch2WithMain)
catch_discover_tests(sim_rf_protocols_test)

add_executable(sim_periph_test sim_periph_test.cpp)
target_link_libraries(sim_periph_test PRIVATE ap-sim Catch2::Catch2WithMain)
catch_discover_tests(sim_periph_test)

add_executable(sim_i2c_vehicles_test sim_i2c_vehicles_test.cpp)
target_link_libraries(sim_i2c_vehicles_test PRIVATE ap-sim Catch2::Catch2WithMain)
catch_discover_tests(sim_i2c_vehicles_test)

add_executable(tecs_test tecs_test.cpp)
# ap-param added by CPP-049: tecs.hpp itself now #includes several
# fwcpp/param/ headers for its own "CPP-049 ADDENDUM" (tecs_group_info/
# tecs_param_info/apply_tecs_defaults/load_tecs_parameters/
# save_tecs_parameters) - any translation unit that includes tecs.hpp,
# including this pre-existing test, needs ap-param's include path too.
# modules/ap-tecs/CMakeLists.txt itself is intentionally left untouched
# (out of this ticket's scope - see tecs_param_test's own target below
# for the fuller note); adding the dependency here, at every consuming
# test target, is the fix available within this ticket's touched files.
target_link_libraries(tecs_test PRIVATE ap-tecs ap-param Catch2::Catch2WithMain)
catch_discover_tests(tecs_test)

# CPP-049: storage/persistence integration test for Tecs::Gains (TECS_
# prefix) - no ap-sim needed (not a closed-loop test, see the ticket's
# own "NOT a SimPlane closed-loop test" instruction). Links ap-param
# directly (in addition to ap-tecs) because tecs_param_test.cpp itself,
# not just tecs.hpp, uses fwcpp::param:: types (GroupInfo, ParamHeader,
# storage::StorageAccess, top_level::find) directly - ap-tecs's own
# CMakeLists.txt is intentionally left untouched by this ticket (see the
# ticket's own scope note restricting changes to tecs.hpp/this test
# file/this CMakeLists.txt only), so this test target picks up
# fwcpp/param/'s include path via its own direct ap-param link instead.

add_executable(tecs_param_test tecs_param_test.cpp)
target_link_libraries(tecs_param_test PRIVATE ap-tecs ap-param Catch2::Catch2WithMain)
catch_discover_tests(tecs_param_test)

add_executable(tecs_leftover_test tecs_leftover_test.cpp)
target_link_libraries(tecs_leftover_test PRIVATE ap-tecs ap-param Catch2::Catch2WithMain)
catch_discover_tests(tecs_leftover_test)

add_executable(fw_control_test fw_control_test.cpp)
# ap-param: CPP-044 added real AP_Param Info-table support directly to
# roll_controller.hpp (included here), which now unconditionally
# #includes fwcpp/param/* headers - ap-fw-control itself doesn't link
# ap-param (see roll_controller_param_test's own link line below for why
# that dependency isn't added to the module's own CMakeLists), so this
# test target needs the include path directly.
target_link_libraries(fw_control_test PRIVATE ap-fw-control ap-param Catch2::Catch2WithMain)
catch_discover_tests(fw_control_test)

# ap-sim is a TEST-ONLY dependency here (CPP-031's closed-loop
# integration test) - ap-vehicle itself does not link it, matching how
# ap-sim deliberately shares no code with ap-ahrs (see sim_plane.hpp's
# own file banner).

add_executable(vehicle_test vehicle_test.cpp)
target_link_libraries(vehicle_test PRIVATE ap-vehicle ap-sim Catch2::Catch2WithMain)
catch_discover_tests(vehicle_test)

# CPP-043: storage/persistence integration test for Plane::aparm - no
# ap-sim needed (not a closed-loop test, see the ticket's own "NOT a
# SimPlane closed-loop test" instruction).

add_executable(plane_aparm_param_test plane_aparm_param_test.cpp)
target_link_libraries(plane_aparm_param_test PRIVATE ap-vehicle Catch2::Catch2WithMain)
catch_discover_tests(plane_aparm_param_test)

# CPP-044: storage/persistence integration test for
# fw_control::RollController::Gains - no ap-sim needed (not a closed-loop
# test, matching CPP-043's own plane_aparm_param_test above). Links
# ap-param directly (in addition to ap-fw-control) for the same reason
# fw_control_test does above.

add_executable(roll_controller_param_test roll_controller_param_test.cpp)
target_link_libraries(roll_controller_param_test PRIVATE ap-fw-control ap-param Catch2::Catch2WithMain)
catch_discover_tests(roll_controller_param_test)

add_executable(gps_test gps_test.cpp)
target_link_libraries(gps_test PRIVATE ap-gps Catch2::Catch2WithMain)
catch_discover_tests(gps_test)

add_executable(compass_test compass_test.cpp)
target_link_libraries(compass_test PRIVATE ap-compass Catch2::Catch2WithMain)
catch_discover_tests(compass_test)

# CPP-082 phase 1: AirspeedSensor's own read()-formula unit tests - no
# ap-sim/ap-vehicle needed (not a closed-loop test, matching CPP-043's
# own plane_aparm_param_test precedent above). The genuine closed-loop
# test proving tick()-driven sensor output changes real vehicle behavior
# lives in vehicle_test.cpp instead (ap-sim is already a test-only
# dependency there).

add_executable(airspeed_sensor_test airspeed_sensor_test.cpp)
target_link_libraries(airspeed_sensor_test PRIVATE ap-airspeed Catch2::Catch2WithMain)
catch_discover_tests(airspeed_sensor_test)

add_executable(baro_test baro_test.cpp)
target_link_libraries(baro_test PRIVATE ap-baro ap-common Catch2::Catch2WithMain)
catch_discover_tests(baro_test)

add_executable(sim_baro_model_test sim_baro_model_test.cpp)
target_link_libraries(sim_baro_model_test PRIVATE ap-sim Catch2::Catch2WithMain)
catch_discover_tests(sim_baro_model_test)

add_executable(steer_controller_test steer_controller_test.cpp)
target_link_libraries(steer_controller_test PRIVATE ap-steer-control Catch2::Catch2WithMain)
catch_discover_tests(steer_controller_test)

# CPP-045: storage/persistence integration test for
# PitchController::Gains - no ap-sim needed (not a closed-loop test, see
# the ticket's own "NOT a SimPlane closed-loop test" instruction).
# ap-fw-control's own transitive link chain (ap-pid -> ap-filter -> a
# PUBLIC ap-param dependency) already brings in ap-param's include path,
# so no module CMakeLists.txt needs editing for this ticket.

add_executable(pitch_controller_param_test pitch_controller_param_test.cpp)
target_link_libraries(pitch_controller_param_test PRIVATE ap-fw-control Catch2::Catch2WithMain)
catch_discover_tests(pitch_controller_param_test)

# CPP-046: storage/persistence integration test for
# fw_control::YawController::Gains - no ap-sim/ap-vehicle needed (not a
# closed-loop test, see the ticket's own "NOT a SimPlane closed-loop
# test" instruction); ap-fw-control alone (which transitively pulls in
# ap-param via ap-pid -> ap-filter -> ap-param) is enough.

add_executable(yaw_controller_param_test yaw_controller_param_test.cpp)
target_link_libraries(yaw_controller_param_test PRIVATE ap-fw-control Catch2::Catch2WithMain)
catch_discover_tests(yaw_controller_param_test)

# CPP-047: storage/persistence integration test for
# steer_control::SteerController::Gains - no ap-sim needed (not a
# closed-loop test, matching CPP-043's own plane_aparm_param_test
# precedent above). Links ap-param directly (in addition to
# ap-steer-control, whose own module CMakeLists.txt does not depend on
# ap-param) for the same reason notch_filter_params_test does above.

add_executable(steer_controller_param_test steer_controller_param_test.cpp)
target_link_libraries(steer_controller_param_test PRIVATE ap-steer-control ap-param Catch2::Catch2WithMain)
catch_discover_tests(steer_controller_param_test)

# CPP-052 phase 1: strapdown INS mechanization + covariance prediction
# unit tests, no fusion (see fwcpp/ekf/ekf_core.hpp's scope banner).

add_executable(ekf_core_test ekf_core_test.cpp)
target_link_libraries(ekf_core_test PRIVATE ap-ekf Catch2::Catch2WithMain)
catch_discover_tests(ekf_core_test)

# CPP-056 phase 2: fuse_direct_state_observation() primitive + GPS
# velocity/position fusion unit tests (see fwcpp/ekf/ekf_core.hpp's
# "CPP-056, PHASE 2" scope banner section).

add_executable(ekf_fusion_test ekf_fusion_test.cpp)
target_link_libraries(ekf_fusion_test PRIVATE ap-ekf Catch2::Catch2WithMain)
catch_discover_tests(ekf_fusion_test)

# CPP-059 phase 5: 3-axis magnetometer fusion (fuse_magnetometer()) unit
# tests (see fwcpp/ekf/ekf_core.hpp's "CPP-059, PHASE 5" scope banner
# section).

add_executable(ekf_mag_fusion_test ekf_mag_fusion_test.cpp)
target_link_libraries(ekf_mag_fusion_test PRIVATE ap-ekf Catch2::Catch2WithMain)
catch_discover_tests(ekf_mag_fusion_test)

# CPP-061 phase 7: closed-loop EkfCore validation against SimPlane's
# ground truth (see fwcpp/ekf/ekf_core.hpp's phase banners for phases
# 1-6 - this is the first test to run the WHOLE assembled pipeline
# together, continuously, against a realistic simulated flight). ap-sim,
# ap-compass, and ap-vehicle are TEST-ONLY dependencies here (same
# precedent as vehicle_test.cpp's own ap-sim dependency) - ap-ekf itself
# links none of them.

add_executable(ekf_closed_loop_test ekf_closed_loop_test.cpp)
target_link_libraries(ekf_closed_loop_test PRIVATE ap-ekf ap-sim ap-compass ap-vehicle Catch2::Catch2WithMain)
catch_discover_tests(ekf_closed_loop_test)

# CPP-063 phase 9: true airspeed / wind velocity fusion (fuse_airspeed())
# unit tests (see fwcpp/ekf/ekf_core.hpp's "CPP-063, PHASE 9" scope banner
# section). A new file, not ekf_fusion_test.cpp/ekf_mag_fusion_test.cpp -
# same reasoning phase 5 (CPP-059) used for its own new file: a genuinely
# independent, self-contained fusion mechanism, not built on
# fuse_direct_state_observation().

add_executable(ekf_airspeed_fusion_test ekf_airspeed_fusion_test.cpp)
target_link_libraries(ekf_airspeed_fusion_test PRIVATE ap-ekf Catch2::Catch2WithMain)
catch_discover_tests(ekf_airspeed_fusion_test)

# CPP-066 phase 12: standalone generic ring-buffer infrastructure
# (fwcpp::ekf::ObsBuffer<T,N> / fwcpp::ekf::ImuBuffer<T,N>, see
# fwcpp/ekf/ekf_buffer.hpp's own file banner) - the FIRST step of the
# delay-buffer/observation-time-horizon fusion epic. Header-only, but
# still links ap-ekf (same include path/namespace as ekf_core.hpp) rather
# than being a bare Catch2 executable - NOT wired into EkfCore at all,
# zero change to any existing ap-ekf file.

add_executable(ekf_buffer_test ekf_buffer_test.cpp)
target_link_libraries(ekf_buffer_test PRIVATE ap-ekf Catch2::Catch2WithMain)
catch_discover_tests(ekf_buffer_test)

# CPP-071 phase 17: EkfCore::tick() - IMU history buffering and
# delayed-state mechanization (see fwcpp/ekf/ekf_core.hpp's "CPP-071,
# PHASE 17" scope banner). A NEW, standalone test file, touching nothing
# in ekf_core_test.cpp - update_strapdown_equations_ned()/
# covariance_prediction()'s own direct-call API and every existing test
# using it are completely unaffected by tick()'s existence.

add_executable(ekf_tick_test ekf_tick_test.cpp)
target_link_libraries(ekf_tick_test PRIVATE ap-ekf Catch2::Catch2WithMain)
catch_discover_tests(ekf_tick_test)

# CPP-080: EkfCoreBackend, the adapter making EkfCore implement
# fwcpp::ahrs::AhrsBackend (see fwcpp/ekf/ekf_backend.hpp's own file
# banner for the full design writeup). Links ap-ahrs directly (in
# addition to ap-ekf) because this test file itself, not just
# ekf_backend.hpp, uses fwcpp::ahrs:: sample types (GyroSample/
# AccelSample/CompassSample/GpsSample) directly - same precedent as
# tecs_param_test's own direct ap-param link above.

add_executable(ekf_backend_test ekf_backend_test.cpp)
target_link_libraries(ekf_backend_test PRIVATE ap-ekf ap-ahrs Catch2::Catch2WithMain)
catch_discover_tests(ekf_backend_test)

# CPP-084: SitlHarness (fwcpp::hal_sitl) - a real, reusable class
# generalizing vehicle_test.cpp's own hand-rolled closed-loop sensor-
# synthesis pattern (run_biased_closed_loop() etc). ap-hal-sitl already
# links ap-sim publicly (SitlHarness's constructor takes a SimPlane&
# directly, unlike ap-vehicle - see modules/ap-hal-sitl/CMakeLists.txt),
# so this test target does not need to link ap-sim/ap-vehicle a second
# time, but this test file itself uses fwcpp::vehicle:: and fwcpp::sim::
# types directly (not just through ap-hal-sitl's own header), so both are
# linked directly too - same precedent as ekf_backend_test's own direct
# ap-ahrs link above.

add_executable(sitl_harness_test sitl_harness_test.cpp)
target_link_libraries(sitl_harness_test PRIVATE ap-hal-sitl ap-vehicle ap-sim Catch2::Catch2WithMain)
catch_discover_tests(sitl_harness_test)

# CCP-001 (copter-cpp effort): fwcpp::motors::MotorsMatrix - AP_MotorsMatrix
# core factor storage/arithmetic (add_motor/add_motor_raw's angle-to-factor
# conversion, remove_motor, normalise_rpy_factors). First test target for
# the new ap-motors module - see modules/ap-motors/include/fwcpp/motors/
# motors_matrix.hpp's own file banner for full scope.

add_executable(motors_matrix_test motors_matrix_test.cpp)
target_link_libraries(motors_matrix_test PRIVATE ap-motors Catch2::Catch2WithMain)
catch_discover_tests(motors_matrix_test)

# CCP-010 (copter-cpp effort, AP_Motors output-stage phase):
# fwcpp::motors::ThrustLinearization - thrust curve linearization plus
# battery-voltage/air-density compensation
# (AP_Motors_Thrust_Linearization.{h,cpp}) - see
# modules/ap-motors/include/fwcpp/motors/thrust_linearization.hpp's own
# file banner for full scope.

add_executable(thrust_linearization_test thrust_linearization_test.cpp)
target_link_libraries(thrust_linearization_test PRIVATE ap-motors Catch2::Catch2WithMain)
catch_discover_tests(thrust_linearization_test)

# CCP-018 (copter-cpp effort, AC_AttitudeControl phase): fwcpp::control's
# attitude_kinematics.hpp - AC_AttitudeControl frame-conversion/limit
# primitives (euler_derivative_to_body/body_to_euler_derivative/
# ang_vel_limit/body_to_euler_limit). First test target for the new
# ap-control module - see modules/ap-control/include/fwcpp/control/
# attitude_kinematics.hpp's own file banner for full scope.

add_executable(attitude_kinematics_test attitude_kinematics_test.cpp)
target_link_libraries(attitude_kinematics_test PRIVATE ap-control Catch2::Catch2WithMain)
catch_discover_tests(attitude_kinematics_test)

# CCP-021 (copter-cpp effort, AC_AttitudeControl phase): fwcpp::math's
# control.hpp - the scalar half of AP_Math/control.cpp (update_vel_accel/
# update_pos_vel_accel/shape_accel/shape_vel_accel/shape_pos_vel_accel/
# shape_angle_vel_accel/sqrt_controller/inv_sqrt_controller/
# sqrt_controller_accel/stopping_distance). Header-only addition to the
# existing ap-math module (no new .cpp, matching matrix3.hpp/postype.hpp/
# rotations.hpp's own precedent - see modules/ap-math/CMakeLists.txt,
# unchanged by this ticket), so only this new test target is registered.

add_executable(control_test control_test.cpp)
target_link_libraries(control_test PRIVATE ap-math Catch2::Catch2WithMain)
catch_discover_tests(control_test)

# CCP-027: ap-poscontrol slice 1.

add_executable(pos_control_lean_test pos_control_lean_test.cpp)
target_link_libraries(pos_control_lean_test PRIVATE ap-poscontrol Catch2::Catch2WithMain)
catch_discover_tests(pos_control_lean_test)

add_executable(pos_control_leftover_test pos_control_leftover_test.cpp)
target_link_libraries(pos_control_leftover_test PRIVATE ap-poscontrol Catch2::Catch2WithMain)
catch_discover_tests(pos_control_leftover_test)

add_executable(pos_control_ne_update_test pos_control_ne_update_test.cpp)
target_link_libraries(pos_control_ne_update_test PRIVATE ap-poscontrol Catch2::Catch2WithMain)
catch_discover_tests(pos_control_ne_update_test)

add_executable(pos_control_ne_limits_test pos_control_ne_limits_test.cpp)
target_link_libraries(pos_control_ne_limits_test PRIVATE ap-poscontrol Catch2::Catch2WithMain)
catch_discover_tests(pos_control_ne_limits_test)

add_executable(pos_control_d_update_test pos_control_d_update_test.cpp)
target_link_libraries(pos_control_d_update_test PRIVATE ap-poscontrol Catch2::Catch2WithMain)
catch_discover_tests(pos_control_d_update_test)

add_executable(pos_control_accessors_test pos_control_accessors_test.cpp)
target_link_libraries(pos_control_accessors_test PRIVATE ap-poscontrol Catch2::Catch2WithMain)
catch_discover_tests(pos_control_accessors_test)

add_executable(pos_control_class_test pos_control_class_test.cpp)
target_link_libraries(pos_control_class_test PRIVATE ap-poscontrol Catch2::Catch2WithMain)
catch_discover_tests(pos_control_class_test)

add_executable(pos_control_path_test pos_control_path_test.cpp)
target_link_libraries(pos_control_path_test PRIVATE ap-poscontrol Catch2::Catch2WithMain)
catch_discover_tests(pos_control_path_test)

add_executable(pos_control_d_limits_test pos_control_d_limits_test.cpp)
target_link_libraries(pos_control_d_limits_test PRIVATE ap-poscontrol Catch2::Catch2WithMain)
catch_discover_tests(pos_control_d_limits_test)



# CCP-028 (copter-cpp effort): fwcpp::wpnav — AC_WPNav first slice
# (wp_and_spline_init_m + distance/bearing helpers). See
# modules/ap-wpnav/include/fwcpp/wpnav/wpnav.hpp file banner.

add_executable(wp_and_spline_init_test wp_and_spline_init_test.cpp)
target_link_libraries(wp_and_spline_init_test PRIVATE ap-wpnav Catch2::Catch2WithMain)
catch_discover_tests(wp_and_spline_init_test)
# CCP-028 slice 2: set_wp_destination / update_wpnav

add_executable(set_wp_destination_test set_wp_destination_test.cpp)
target_link_libraries(set_wp_destination_test PRIVATE ap-wpnav Catch2::Catch2WithMain)
catch_discover_tests(set_wp_destination_test)

add_executable(update_wpnav_test update_wpnav_test.cpp)
target_link_libraries(update_wpnav_test PRIVATE ap-wpnav Catch2::Catch2WithMain)
catch_discover_tests(update_wpnav_test)

# CCP-028 slice 3: advance_wp_target_along_track / update_track_with_speed_accel_limits

add_executable(advance_wp_target_test advance_wp_target_test.cpp)
target_link_libraries(advance_wp_target_test PRIVATE ap-wpnav Catch2::Catch2WithMain)
catch_discover_tests(advance_wp_target_test)

# CCP-028 slice 4: AC_Loiter init_target_m / init_target / update leftovers

add_executable(loiter_init_test loiter_init_test.cpp)
target_link_libraries(loiter_init_test PRIVATE ap-wpnav Catch2::Catch2WithMain)
catch_discover_tests(loiter_init_test)

add_executable(quadplane_setup_test quadplane_setup_test.cpp)
target_link_libraries(quadplane_setup_test PRIVATE ap-quadplane Catch2::Catch2WithMain)
catch_discover_tests(quadplane_setup_test)

add_executable(quadplane_options_test quadplane_options_test.cpp)

add_executable(quadplane_poscontrol_test quadplane_poscontrol_test.cpp)

add_executable(quadplane_leftover_test quadplane_leftover_test.cpp)
target_link_libraries(quadplane_options_test PRIVATE ap-quadplane Catch2::Catch2WithMain)
target_link_libraries(quadplane_poscontrol_test PRIVATE ap-quadplane Catch2::Catch2WithMain)
target_link_libraries(quadplane_leftover_test PRIVATE ap-quadplane Catch2::Catch2WithMain)
catch_discover_tests(quadplane_options_test)
catch_discover_tests(quadplane_poscontrol_test)
catch_discover_tests(quadplane_leftover_test)

add_executable(quadplane_pilot_input_test quadplane_pilot_input_test.cpp)
target_link_libraries(quadplane_pilot_input_test PRIVATE ap-quadplane Catch2::Catch2WithMain)
catch_discover_tests(quadplane_pilot_input_test)

add_executable(quadplane_land_detector_test quadplane_land_detector_test.cpp)
target_link_libraries(quadplane_land_detector_test PRIVATE ap-quadplane Catch2::Catch2WithMain)
catch_discover_tests(quadplane_land_detector_test)

add_executable(quadplane_takeoff_controller_test quadplane_takeoff_controller_test.cpp)
target_link_libraries(quadplane_takeoff_controller_test PRIVATE ap-quadplane Catch2::Catch2WithMain)
catch_discover_tests(quadplane_takeoff_controller_test)

add_executable(quadplane_control_auto_test quadplane_control_auto_test.cpp)
target_link_libraries(quadplane_control_auto_test PRIVATE ap-quadplane Catch2::Catch2WithMain)
catch_discover_tests(quadplane_control_auto_test)

add_executable(quadplane_stabilize_test quadplane_stabilize_test.cpp)
target_link_libraries(quadplane_stabilize_test PRIVATE ap-quadplane Catch2::Catch2WithMain)
catch_discover_tests(quadplane_stabilize_test)

add_executable(quadplane_xy_controller_test quadplane_xy_controller_test.cpp)
target_link_libraries(quadplane_xy_controller_test PRIVATE ap-quadplane Catch2::Catch2WithMain)
catch_discover_tests(quadplane_xy_controller_test)

add_executable(quadplane_throttle_test quadplane_throttle_test.cpp)
target_link_libraries(quadplane_throttle_test PRIVATE ap-quadplane Catch2::Catch2WithMain)
catch_discover_tests(quadplane_throttle_test)

add_executable(quadplane_guided_test quadplane_guided_test.cpp)
target_link_libraries(quadplane_guided_test PRIVATE ap-quadplane Catch2::Catch2WithMain)
catch_discover_tests(quadplane_guided_test)

add_executable(quadplane_landing_test quadplane_landing_test.cpp)
target_link_libraries(quadplane_landing_test PRIVATE ap-quadplane Catch2::Catch2WithMain)
catch_discover_tests(quadplane_landing_test)

add_executable(quadplane_assist_test quadplane_assist_test.cpp)
target_link_libraries(quadplane_assist_test PRIVATE ap-quadplane Catch2::Catch2WithMain)
catch_discover_tests(quadplane_assist_test)

add_executable(quadplane_motors_output_test quadplane_motors_output_test.cpp)
target_link_libraries(quadplane_motors_output_test PRIVATE ap-quadplane Catch2::Catch2WithMain)
catch_discover_tests(quadplane_motors_output_test)

add_executable(quadplane_motor_test_test quadplane_motor_test_test.cpp)
target_link_libraries(quadplane_motor_test_test PRIVATE ap-quadplane Catch2::Catch2WithMain)
catch_discover_tests(quadplane_motor_test_test)

add_executable(quadplane_subsystems_test quadplane_subsystems_test.cpp)
target_link_libraries(quadplane_subsystems_test PRIVATE ap-quadplane Catch2::Catch2WithMain)
catch_discover_tests(quadplane_subsystems_test)

add_executable(quadplane_vtol_subsystems_test quadplane_vtol_subsystems_test.cpp)
target_link_libraries(quadplane_vtol_subsystems_test PRIVATE ap-quadplane Catch2::Catch2WithMain)
catch_discover_tests(quadplane_vtol_subsystems_test)

add_executable(quadplane_setup_channels_test quadplane_setup_channels_test.cpp)
target_link_libraries(quadplane_setup_channels_test PRIVATE ap-quadplane Catch2::Catch2WithMain)
catch_discover_tests(quadplane_setup_channels_test)

add_executable(quadplane_setup_navigators_test quadplane_setup_navigators_test.cpp)
target_link_libraries(quadplane_setup_navigators_test PRIVATE ap-quadplane Catch2::Catch2WithMain)
catch_discover_tests(quadplane_setup_navigators_test)





# VCP-002 slice 1: ap-vtol-assist enable/check + hysteresis + should_assist stubs.

add_executable(quadplane_update_test quadplane_update_test.cpp)
target_link_libraries(quadplane_update_test PRIVATE ap-quadplane Catch2::Catch2WithMain)
catch_discover_tests(quadplane_update_test)
add_executable(quadplane_air_mode_test quadplane_air_mode_test.cpp)
add_executable(quadplane_mode_predicates_test quadplane_mode_predicates_test.cpp)
add_executable(quadplane_vtol_position_controller_test quadplane_vtol_position_controller_test.cpp)
target_link_libraries(quadplane_air_mode_test PRIVATE ap-quadplane Catch2::Catch2WithMain)
target_link_libraries(quadplane_mode_predicates_test PRIVATE ap-quadplane Catch2::Catch2WithMain)
target_link_libraries(quadplane_vtol_position_controller_test PRIVATE ap-quadplane Catch2::Catch2WithMain)
catch_discover_tests(quadplane_air_mode_test)
catch_discover_tests(quadplane_mode_predicates_test)
catch_discover_tests(quadplane_vtol_position_controller_test)
add_executable(quadplane_auto_vtol_mission_test quadplane_auto_vtol_mission_test.cpp)
target_link_libraries(quadplane_auto_vtol_mission_test PRIVATE ap-quadplane Catch2::Catch2WithMain)
catch_discover_tests(quadplane_auto_vtol_mission_test)
add_executable(quadplane_verify_vtol_land_test quadplane_verify_vtol_land_test.cpp)
target_link_libraries(quadplane_verify_vtol_land_test PRIVATE ap-quadplane Catch2::Catch2WithMain)
catch_discover_tests(quadplane_verify_vtol_land_test)

add_executable(quadplane_tecs_mixing_test quadplane_tecs_mixing_test.cpp)
target_link_libraries(quadplane_tecs_mixing_test PRIVATE ap-quadplane Catch2::Catch2WithMain)
catch_discover_tests(quadplane_tecs_mixing_test)








# VCP-002 slice 1: ap-vtol-assist enable/check + hysteresis + should_assist stubs.

add_executable(vtol_assist_enable_test vtol_assist_enable_test.cpp)
target_link_libraries(vtol_assist_enable_test PRIVATE ap-vtol-assist Catch2::Catch2WithMain)
catch_discover_tests(vtol_assist_enable_test)

add_executable(vtol_assist_hysteresis_test vtol_assist_hysteresis_test.cpp)
target_link_libraries(vtol_assist_hysteresis_test PRIVATE ap-vtol-assist Catch2::Catch2WithMain)
catch_discover_tests(vtol_assist_hysteresis_test)

add_executable(vtol_assist_should_assist_test vtol_assist_should_assist_test.cpp)
target_link_libraries(vtol_assist_should_assist_test PRIVATE ap-vtol-assist Catch2::Catch2WithMain)
catch_discover_tests(vtol_assist_should_assist_test)

add_executable(vtol_assist_completeness_test vtol_assist_completeness_test.cpp)
target_link_libraries(vtol_assist_completeness_test PRIVATE ap-vtol-assist Catch2::Catch2WithMain)
catch_discover_tests(vtol_assist_completeness_test)

add_executable(vtol_assist_alt_angle_test vtol_assist_alt_angle_test.cpp)
target_link_libraries(vtol_assist_alt_angle_test PRIVATE ap-vtol-assist Catch2::Catch2WithMain)
catch_discover_tests(vtol_assist_alt_angle_test)

# VCP-002 slice 3: VTOL fw recovery FSM + spin surface outputs.

add_executable(vtol_assist_recovery_test vtol_assist_recovery_test.cpp)
target_link_libraries(vtol_assist_recovery_test PRIVATE ap-vtol-assist Catch2::Catch2WithMain)
catch_discover_tests(vtol_assist_recovery_test)

# VCP-003 slice 1: ap-quadplane-transition FSM scaffold.

add_executable(vtol_assist_gcs_getters_test vtol_assist_gcs_getters_test.cpp)
target_link_libraries(vtol_assist_gcs_getters_test PRIVATE ap-vtol-assist Catch2::Catch2WithMain)
catch_discover_tests(vtol_assist_gcs_getters_test)

add_executable(vtol_assist_recovery_options_test vtol_assist_recovery_options_test.cpp)
target_link_libraries(vtol_assist_recovery_options_test PRIVATE ap-vtol-assist Catch2::Catch2WithMain)
catch_discover_tests(vtol_assist_recovery_options_test)

# VCP-003 slice 1: ap-quadplane-transition FSM scaffold.

add_executable(transition_fsm_test transition_fsm_test.cpp)
target_link_libraries(transition_fsm_test PRIVATE ap-quadplane-transition Catch2::Catch2WithMain)
catch_discover_tests(transition_fsm_test)

add_executable(transition_completeness_test transition_completeness_test.cpp)
target_link_libraries(transition_completeness_test PRIVATE ap-quadplane-transition Catch2::Catch2WithMain)
catch_discover_tests(transition_completeness_test)
# VCP-004 slice 1: ap-q-modes QSTABILIZE / QHOVER / QACRO scaffold.


# CCP-028 slice 7: loiter/circle/wpnav leftover parity
add_executable(circle_leftover_test circle_leftover_test.cpp)
target_link_libraries(circle_leftover_test PRIVATE ap-wpnav ap-common Catch2::Catch2WithMain)
catch_discover_tests(circle_leftover_test)

add_executable(loiter_pilot_accel_test loiter_pilot_accel_test.cpp)
target_link_libraries(loiter_pilot_accel_test PRIVATE ap-wpnav Catch2::Catch2WithMain)
catch_discover_tests(loiter_pilot_accel_test)

add_executable(wpnav_leftover_test wpnav_leftover_test.cpp)
target_link_libraries(wpnav_leftover_test PRIVATE ap-wpnav ap-common Catch2::Catch2WithMain)
catch_discover_tests(wpnav_leftover_test)

add_executable(circle_init_test circle_init_test.cpp)
target_link_libraries(circle_init_test PRIVATE ap-wpnav ap-common Catch2::Catch2WithMain)
catch_discover_tests(circle_init_test)

add_executable(set_spline_destination_test set_spline_destination_test.cpp)
target_link_libraries(set_spline_destination_test PRIVATE ap-wpnav Catch2::Catch2WithMain)
catch_discover_tests(set_spline_destination_test)

add_executable(set_wp_destination_next_test set_wp_destination_next_test.cpp)
target_link_libraries(set_wp_destination_next_test PRIVATE ap-wpnav Catch2::Catch2WithMain)
catch_discover_tests(set_wp_destination_next_test)

add_executable(transition_update_test transition_update_test.cpp)
target_link_libraries(transition_update_test PRIVATE ap-quadplane-transition Catch2::Catch2WithMain)
catch_discover_tests(transition_update_test)

add_executable(mode_q_run_test mode_q_run_test.cpp)
target_link_libraries(mode_q_run_test PRIVATE ap-q-modes ap-poscontrol ap-pid Catch2::Catch2WithMain)
catch_discover_tests(mode_q_run_test)

add_executable(q_modes_completeness_test q_modes_completeness_test.cpp)
target_link_libraries(q_modes_completeness_test PRIVATE ap-q-modes Catch2::Catch2WithMain)
catch_discover_tests(q_modes_completeness_test)


# VCP-006 slice 1: ap-qrtl QRTL mode enter scaffold.

add_executable(transition_paths_test transition_paths_test.cpp)
target_link_libraries(transition_paths_test PRIVATE ap-quadplane-transition Catch2::Catch2WithMain)
catch_discover_tests(transition_paths_test)

add_executable(transition_base_test transition_base_test.cpp)
target_link_libraries(transition_base_test PRIVATE ap-quadplane-transition Catch2::Catch2WithMain)
catch_discover_tests(transition_base_test)

add_executable(transition_update_loop_test transition_update_loop_test.cpp)
target_link_libraries(transition_update_loop_test PRIVATE ap-quadplane-transition Catch2::Catch2WithMain)
catch_discover_tests(transition_update_loop_test)

add_executable(qrtl_geometry_test qrtl_geometry_test.cpp)
target_link_libraries(qrtl_geometry_test PRIVATE ap-qrtl Catch2::Catch2WithMain)
catch_discover_tests(qrtl_geometry_test)


add_executable(mode_qrtl_run_test mode_qrtl_run_test.cpp)
target_link_libraries(mode_qrtl_run_test PRIVATE ap-qrtl Catch2::Catch2WithMain)
catch_discover_tests(mode_qrtl_run_test)
add_executable(mode_qrtl_update_test mode_qrtl_update_test.cpp)
target_link_libraries(mode_qrtl_update_test PRIVATE ap-qrtl Catch2::Catch2WithMain)
catch_discover_tests(mode_qrtl_update_test)


add_executable(mode_qrtl_enter_test mode_qrtl_enter_test.cpp)
target_link_libraries(mode_qrtl_enter_test PRIVATE ap-qrtl Catch2::Catch2WithMain)
catch_discover_tests(mode_qrtl_enter_test)

add_executable(qrtl_completeness_test qrtl_completeness_test.cpp)
target_link_libraries(qrtl_completeness_test PRIVATE ap-qrtl Catch2::Catch2WithMain)
catch_discover_tests(qrtl_completeness_test)

# VCP-005 slice 1: ap-q-loiter QLOITER QLAND scaffold.

add_executable(q_loiter_meta_test q_loiter_meta_test.cpp)
target_link_libraries(q_loiter_meta_test PRIVATE ap-q-loiter Catch2::Catch2WithMain)
catch_discover_tests(q_loiter_meta_test)

add_executable(mode_qloiter_enter_test mode_qloiter_enter_test.cpp)
target_link_libraries(mode_qloiter_enter_test PRIVATE ap-q-loiter Catch2::Catch2WithMain)
catch_discover_tests(mode_qloiter_enter_test)

add_executable(mode_qland_enter_test mode_qland_enter_test.cpp)
target_link_libraries(mode_qland_enter_test PRIVATE ap-q-loiter Catch2::Catch2WithMain)
catch_discover_tests(mode_qland_enter_test)

add_executable(mode_qloiter_run_test mode_qloiter_run_test.cpp)
target_link_libraries(mode_qloiter_run_test PRIVATE ap-q-loiter Catch2::Catch2WithMain)
catch_discover_tests(mode_qloiter_run_test)

add_executable(loiter_alt_qland_test loiter_alt_qland_test.cpp)
target_link_libraries(loiter_alt_qland_test PRIVATE ap-q-loiter Catch2::Catch2WithMain)
catch_discover_tests(loiter_alt_qland_test)

add_executable(q_loiter_completeness_test q_loiter_completeness_test.cpp)
target_link_libraries(q_loiter_completeness_test PRIVATE ap-q-loiter Catch2::Catch2WithMain)
catch_discover_tests(q_loiter_completeness_test)

add_executable(mode_qloiter_slice3_test mode_qloiter_slice3_test.cpp)
target_link_libraries(mode_qloiter_slice3_test PRIVATE ap-q-loiter Catch2::Catch2WithMain)
catch_discover_tests(mode_qloiter_slice3_test)

# VCP-007 slice 1: ap-tailsitter scaffold.

add_executable(tailsitter_enable_test tailsitter_enable_test.cpp)
target_link_libraries(tailsitter_enable_test PRIVATE ap-tailsitter Catch2::Catch2WithMain)
catch_discover_tests(tailsitter_enable_test)

add_executable(tailsitter_input_type_test tailsitter_input_type_test.cpp)
target_link_libraries(tailsitter_input_type_test PRIVATE ap-tailsitter Catch2::Catch2WithMain)
catch_discover_tests(tailsitter_input_type_test)

add_executable(tailsitter_completeness_test tailsitter_completeness_test.cpp)
target_link_libraries(tailsitter_completeness_test PRIVATE ap-tailsitter Catch2::Catch2WithMain)
catch_discover_tests(tailsitter_completeness_test)

add_executable(tailsitter_transition_test tailsitter_transition_test.cpp)
target_link_libraries(tailsitter_transition_test PRIVATE ap-tailsitter Catch2::Catch2WithMain)
catch_discover_tests(tailsitter_transition_test)

add_executable(tailsitter_control_test tailsitter_control_test.cpp)
target_link_libraries(tailsitter_control_test PRIVATE ap-tailsitter Catch2::Catch2WithMain)
catch_discover_tests(tailsitter_control_test)

add_executable(tailsitter_output_test tailsitter_output_test.cpp)
target_link_libraries(tailsitter_output_test PRIVATE ap-tailsitter Catch2::Catch2WithMain)
catch_discover_tests(tailsitter_output_test)

add_executable(tailsitter_setup_flags_test tailsitter_setup_flags_test.cpp)
target_link_libraries(tailsitter_setup_flags_test PRIVATE ap-tailsitter ap-quadplane ap-vtol-assist Catch2::Catch2WithMain)
catch_discover_tests(tailsitter_setup_flags_test)

add_executable(tailsitter_speed_scaling_test tailsitter_speed_scaling_test.cpp)
target_link_libraries(tailsitter_speed_scaling_test PRIVATE ap-tailsitter Catch2::Catch2WithMain)
catch_discover_tests(tailsitter_speed_scaling_test)

# VCP-008 slice 1: ap-tiltrotor scaffold.

add_executable(tiltrotor_enable_test tiltrotor_enable_test.cpp)
target_link_libraries(tiltrotor_enable_test PRIVATE ap-tiltrotor Catch2::Catch2WithMain)
catch_discover_tests(tiltrotor_enable_test)

add_executable(tiltrotor_predicates_test tiltrotor_predicates_test.cpp)
target_link_libraries(tiltrotor_predicates_test PRIVATE ap-tiltrotor Catch2::Catch2WithMain)
catch_discover_tests(tiltrotor_predicates_test)

add_executable(tiltrotor_completeness_test tiltrotor_completeness_test.cpp)
target_link_libraries(tiltrotor_completeness_test PRIVATE ap-tiltrotor Catch2::Catch2WithMain)
catch_discover_tests(tiltrotor_completeness_test)

# VCP-009 slice 1: ap-qautotune scaffold.
add_executable(qautotune_init_internals_test qautotune_init_internals_test.cpp)
target_link_libraries(qautotune_init_internals_test PRIVATE ap-qautotune Catch2::Catch2WithMain)
catch_discover_tests(qautotune_init_internals_test)

add_executable(qautotune_log_pids_test qautotune_log_pids_test.cpp)
target_link_libraries(qautotune_log_pids_test PRIVATE ap-qautotune Catch2::Catch2WithMain)
catch_discover_tests(qautotune_log_pids_test)


add_executable(qautotune_enable_test qautotune_enable_test.cpp)
target_link_libraries(qautotune_enable_test PRIVATE ap-qautotune Catch2::Catch2WithMain)
catch_discover_tests(qautotune_enable_test)

add_executable(qautotune_init_test qautotune_init_test.cpp)
target_link_libraries(qautotune_init_test PRIVATE ap-qautotune Catch2::Catch2WithMain)
catch_discover_tests(qautotune_init_test)

add_executable(mode_qautotune_run_test mode_qautotune_run_test.cpp)
target_link_libraries(mode_qautotune_run_test PRIVATE ap-qautotune Catch2::Catch2WithMain)
catch_discover_tests(mode_qautotune_run_test)

add_executable(qautotune_completeness_test qautotune_completeness_test.cpp)
target_link_libraries(qautotune_completeness_test PRIVATE ap-qautotune Catch2::Catch2WithMain)
catch_discover_tests(qautotune_completeness_test)
# VCP-008 slice 2: ap-tiltrotor control/transition.

add_executable(tiltrotor_control_test tiltrotor_control_test.cpp)
target_link_libraries(tiltrotor_control_test PRIVATE ap-tiltrotor Catch2::Catch2WithMain)
catch_discover_tests(tiltrotor_control_test)

add_executable(tiltrotor_transition_test tiltrotor_transition_test.cpp)
target_link_libraries(tiltrotor_transition_test PRIVATE ap-tiltrotor Catch2::Catch2WithMain)
catch_discover_tests(tiltrotor_transition_test)

# VCP-008 slice 5: leftover Tiltrotor::setup() motor scan / SRV / callback.
add_executable(tiltrotor_setup_flags_test tiltrotor_setup_flags_test.cpp)
target_link_libraries(tiltrotor_setup_flags_test PRIVATE ap-tiltrotor Catch2::Catch2WithMain)
catch_discover_tests(tiltrotor_setup_flags_test)

# VCP-008 slice 6: bicopter_output + get_forward_throttle.
add_executable(tiltrotor_output_test tiltrotor_output_test.cpp)
target_link_libraries(tiltrotor_output_test PRIVATE ap-tiltrotor Catch2::Catch2WithMain)
catch_discover_tests(tiltrotor_output_test)

# CPP-090 slice 1: AP_Logger memory backend WriteBlock.
add_executable(logger_backend_test logger_backend_test.cpp)
target_link_libraries(logger_backend_test PRIVATE ap-logger Catch2::Catch2WithMain)
catch_discover_tests(logger_backend_test)

add_executable(logger_completeness_test logger_completeness_test.cpp)
target_link_libraries(logger_completeness_test PRIVATE ap-logger Catch2::Catch2WithMain)
catch_discover_tests(logger_completeness_test)

# CCP-035: ap-copter scheduler table + leftover ticks (rc_loop, motors_output,
# read_AHRS, throttle_loop, run_rate_controller_main, read_inertia,
# check_ekf_reset, update_flight_mode, update_home_from_EKF,
# update_land_and_crash_detectors, update_rangefinder_terrain_offset,
# update_batt_compass, update_altitude, run_nav_updates).
add_executable(copter_loop_test copter_loop_test.cpp)
target_link_libraries(copter_loop_test PRIVATE ap-copter Catch2::Catch2WithMain)
catch_discover_tests(copter_loop_test)

# CPP-087 slice 6: ap-gcs MAVLink2 framing + HEARTBEAT + COMMAND_LONG + PARAM + MISSION + Plane ATTITUDE; catalog closed.
add_executable(gcs_test gcs_test.cpp)
target_link_libraries(gcs_test PRIVATE ap-gcs Catch2::Catch2WithMain)
catch_discover_tests(gcs_test)

# CPP-088 slice 1: SITL GPIO and Semaphore.
add_executable(gpio_test gpio_test.cpp)
target_link_libraries(gpio_test PRIVATE ap-hal Catch2::Catch2WithMain)
catch_discover_tests(gpio_test)

add_executable(semaphore_test semaphore_test.cpp)
target_link_libraries(semaphore_test PRIVATE ap-hal Catch2::Catch2WithMain)
catch_discover_tests(semaphore_test)

add_executable(hal_leftover_test hal_leftover_test.cpp)
target_link_libraries(hal_leftover_test PRIVATE ap-hal Catch2::Catch2WithMain)
catch_discover_tests(hal_leftover_test)

# CPP-088 slice 2: SITL I2C/SPI Device register access.
add_executable(i2c_device_test i2c_device_test.cpp)
target_link_libraries(i2c_device_test PRIVATE ap-hal Catch2::Catch2WithMain)
catch_discover_tests(i2c_device_test)

add_executable(spi_device_test spi_device_test.cpp)
target_link_libraries(spi_device_test PRIVATE ap-hal Catch2::Catch2WithMain)
catch_discover_tests(spi_device_test)

# CPP-088 slice 3: SITL Util soft_armed / safety_switch / system_id.
add_executable(util_test util_test.cpp)
target_link_libraries(util_test PRIVATE ap-hal Catch2::Catch2WithMain)
catch_discover_tests(util_test)

# CPP-088 slice 4: in-memory CANIface send/receive.
add_executable(can_test can_test.cpp)
target_link_libraries(can_test PRIVATE ap-hal Catch2::Catch2WithMain)
catch_discover_tests(can_test)

# CCP-036 slice 1: Mode::Number, ModeReason, Mode base, set_mode checks.
add_executable(mode_test mode_test.cpp)
target_link_libraries(mode_test PRIVATE ap-copter Catch2::Catch2WithMain)
catch_discover_tests(mode_test)

# CPP-090 slice 6: max-files rotation + erase_next (in-memory slot table).
add_executable(logger_rotation_test logger_rotation_test.cpp)
target_link_libraries(logger_rotation_test PRIVATE ap-logger Catch2::Catch2WithMain)
catch_discover_tests(logger_rotation_test)

# CPP-090 slice 7: DataFlash RAM page map BufferToPage / PageToBuffer.
add_executable(logger_dataflash_test logger_dataflash_test.cpp)
target_link_libraries(logger_dataflash_test PRIVATE ap-logger Catch2::Catch2WithMain)
catch_discover_tests(logger_dataflash_test)

add_executable(logger_transfer_test logger_transfer_test.cpp)
target_link_libraries(logger_transfer_test PRIVATE ap-logger Catch2::Catch2WithMain)
catch_discover_tests(logger_transfer_test)

# CPP-088 slice 5: in-memory WSPIDevice command header and transfer.
add_executable(wspi_test wspi_test.cpp)
target_link_libraries(wspi_test PRIVATE ap-hal Catch2::Catch2WithMain)
catch_discover_tests(wspi_test)

# CCP-037 slice 1: Copter pilot lean / yaw / throttle expo helpers.
add_executable(pilot_input_test pilot_input_test.cpp)
target_link_libraries(pilot_input_test PRIVATE ap-copter Catch2::Catch2WithMain)
catch_discover_tests(pilot_input_test)

# CPP-089 slice 1: BoardKind + compile-only Watchdog stub.
add_executable(hw_backend_test hw_backend_test.cpp)
target_link_libraries(hw_backend_test PRIVATE ap-hal Catch2::Catch2WithMain)
catch_discover_tests(hw_backend_test)

# CCP-039 slice 1: ModeStabilize run via input_euler_angle.
add_executable(mode_stabilize_test mode_stabilize_test.cpp)
target_link_libraries(mode_stabilize_test PRIVATE ap-copter Catch2::Catch2WithMain)
catch_discover_tests(mode_stabilize_test)

# CCP-039 slice 2: ModeAcro run via input_rate_bf.
add_executable(mode_acro_test mode_acro_test.cpp)
target_link_libraries(mode_acro_test PRIVATE ap-copter Catch2::Catch2WithMain)
catch_discover_tests(mode_acro_test)

# CCP-037 slice 3: AutoYaw get_heading PILOT_RATE vs HOLD.
add_executable(autoyaw_test autoyaw_test.cpp)
target_link_libraries(autoyaw_test PRIVATE ap-copter Catch2::Catch2WithMain)
catch_discover_tests(autoyaw_test)

# CCP-039 slice 3: ModeAltHold run via climb_rate and input_euler_angle.
add_executable(mode_althold_test mode_althold_test.cpp)
target_link_libraries(mode_althold_test PRIVATE ap-copter Catch2::Catch2WithMain)
catch_discover_tests(mode_althold_test)

# CCP-038 slice 1: AP_Arming_Copter pre_arm leftover scaffold.
add_executable(arming_test arming_test.cpp)
target_link_libraries(arming_test PRIVATE ap-copter Catch2::Catch2WithMain)
catch_discover_tests(arming_test)

# CCP-040 slice 2: ModeLoiter leftover_run flags + catalog.
add_executable(mode_loiter_test mode_loiter_test.cpp)
target_link_libraries(mode_loiter_test PRIVATE ap-copter Catch2::Catch2WithMain)
catch_discover_tests(mode_loiter_test)

# CCP-041 slice 1: land_detector leftover scaffold + catalog.
add_executable(land_detector_test land_detector_test.cpp)
target_link_libraries(land_detector_test PRIVATE ap-copter Catch2::Catch2WithMain)
catch_discover_tests(land_detector_test)

# CCP-041 slice 2: do_user_takeoff_U_m leftover gates + catalog.
add_executable(takeoff_test takeoff_test.cpp)
target_link_libraries(takeoff_test PRIVATE ap-copter Catch2::Catch2WithMain)
catch_discover_tests(takeoff_test)

# CPP-023 slice 1–5: convert_old_parameters + width + REVERSE/FORCE + convert_class +
# convert_g2/toplevel + catalog close (remaining_count 0).
add_executable(param_conversion_test param_conversion_test.cpp)
target_link_libraries(param_conversion_test PRIVATE ap-param Catch2::Catch2WithMain)
catch_discover_tests(param_conversion_test)

# CCP-040 slice 5: ModePosHold leftover_run thin flags + catalog close.
add_executable(mode_poshold_test mode_poshold_test.cpp)
target_link_libraries(mode_poshold_test PRIVATE ap-copter Catch2::Catch2WithMain)
catch_discover_tests(mode_poshold_test)

# CCP-042 slice 4: GCS failsafe leftover + catalog close (remaining_count 0).
add_executable(failsafe_test failsafe_test.cpp)
target_link_libraries(failsafe_test PRIVATE ap-copter Catch2::Catch2WithMain)
catch_discover_tests(failsafe_test)

# CCP-040 slice 4: ModeDrift leftover scaffold + catalog.
add_executable(mode_drift_test mode_drift_test.cpp)
target_link_libraries(mode_drift_test PRIVATE ap-copter Catch2::Catch2WithMain)
catch_discover_tests(mode_drift_test)

# CCP-043 slice 1: SitlCopterHarness leftover scaffold + catalog.
add_executable(sitl_copter_harness_test sitl_copter_harness_test.cpp)
target_link_libraries(sitl_copter_harness_test PRIVATE ap-hal-sitl ap-copter ap-sim Catch2::Catch2WithMain)
catch_discover_tests(sitl_copter_harness_test)

# CCP-044 slice 2: copter_sitl_run leftover aero + arm/takeoff/hold/land catalog close.
add_executable(copter_sitl_run_test copter_sitl_run_test.cpp)
target_link_libraries(copter_sitl_run_test PRIVATE ap-hal-sitl ap-copter ap-sim Catch2::Catch2WithMain)
catch_discover_tests(copter_sitl_run_test)

# CCP-056: remaining in-tree SITL long-tail (vehicles, I2C, serial, SPI, heli swash)
add_executable(sim_longtail_test sim_longtail_test.cpp)
target_link_libraries(sim_longtail_test PRIVATE ap-sim ap-motors Catch2::Catch2WithMain)
catch_discover_tests(sim_longtail_test)

# VCP-011: SitlQuadPlaneHarness + quadplane_sitl_run hover/transition catalog.
add_executable(sitl_quadplane_harness_test sitl_quadplane_harness_test.cpp)
target_link_libraries(sitl_quadplane_harness_test PRIVATE ap-hal-sitl ap-vehicle ap-sim ap-quadplane Catch2::Catch2WithMain)
# VCP-013: reuses the exact fixture-path convention CPP-094/CCP-067 already
# established (an absolute, CMAKE_CURRENT_SOURCE_DIR-derived compile
# definition) so this target can load the same real tests/fixtures/
# Callisto.json fixture CCP-067 added, instead of duplicating it.
target_compile_definitions(sitl_quadplane_harness_test PRIVATE
    FWCPP_SIM_FRAME_FIXTURES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/fixtures")
catch_discover_tests(sitl_quadplane_harness_test)
