# CPP-085: this port's first real, standalone runnable executable (not a
# Catch2 test) - a genuine main() driving Plane+SimPlane via SitlHarness
# (CPP-084). Mirrors modules/'s and tests/'s own top-level convention: one
# new directory, wired into the root CMakeLists.txt via add_subdirectory.
#
# ap-hal-sitl already links ap-vehicle and ap-sim publicly (see
# modules/ap-hal-sitl/CMakeLists.txt), so linking it alone is enough for
# main.cpp's own direct use of fwcpp::vehicle::/fwcpp::sim:: types - same
# precedent tests/CMakeLists.txt's own sitl_harness_test target follows.
add_executable(sitl_run main.cpp)
target_link_libraries(sitl_run PRIVATE ap-hal-sitl)

# CCP-044 slice 1: Copter analogue of sitl_run (CPP-085). Same
# ap-hal-sitl link — SitlCopterHarness + LeftoverCopter come through
# ap-hal-sitl's public ap-copter dependency (CCP-043).
add_executable(copter_sitl_run copter_main.cpp)
target_link_libraries(copter_sitl_run PRIVATE ap-hal-sitl)

# CCP-068: general-purpose arbitrary-waypoint Copter mission runner test —
# proves copter_auto_mission.hpp against a mission the fixed CCP-065 demo
# (copter_sitl_run) cannot express (multiple waypoints at different
# altitudes). Same ap-hal-sitl link.
add_executable(copter_auto_mission_run copter_auto_mission_main.cpp)
target_link_libraries(copter_auto_mission_run PRIVATE ap-hal-sitl)

# VCP-011: QuadPlane analogue of sitl_run / copter_sitl_run.
add_executable(quadplane_sitl_run quadplane_main.cpp)
target_link_libraries(quadplane_sitl_run PRIVATE ap-hal-sitl)

# WOPR-BRIDGE: lockstep UDP bridge around SitlHarness (fixed-wing) and
# SitlQuadPlaneHarness (VTOL models) for the WOPR5000 Unreal sim
# (external-autopilot mode). Same ap-hal-sitl link as sitl_run, plus the
# MAVLink framer and the platform socket library. Linked statically so the
# executable can be spawned by a host process without the llvm-mingw
# runtime DLLs (libc++.dll/libunwind.dll/libwinpthread-1.dll) on PATH.
add_executable(wopr_bridge wopr_bridge.cpp)

# Standalone model-reader regression, available on SITL-only hosts without Catch2.
option(FWCPP_BRIDGE_MODEL_TEST "Build the standalone WOPR model-reader regression" OFF)
if(FWCPP_BRIDGE_MODEL_TEST)
    add_executable(wopr_bridge_model_test wopr_bridge_model_test.cpp)
    target_link_libraries(wopr_bridge_model_test PRIVATE ap-hal-sitl ap-gcs)
    if(WIN32)
        target_link_libraries(wopr_bridge_model_test PRIVATE ws2_32)
        target_link_options(wopr_bridge_model_test PRIVATE -static)
    endif()
    add_test(NAME wopr_bridge_model COMMAND wopr_bridge_model_test
        ${CMAKE_SOURCE_DIR}/tests/fixtures/wopr_bridge_model_overrides.json)
    add_executable(wopr_bridge_mission_test wopr_bridge_mission_test.cpp)
    target_link_libraries(wopr_bridge_mission_test PRIVATE ap-hal-sitl ap-gcs)
    if(WIN32)
        target_link_libraries(wopr_bridge_mission_test PRIVATE ws2_32)
        target_link_options(wopr_bridge_mission_test PRIVATE -static)
    endif()
    add_test(NAME wopr_bridge_mission COMMAND wopr_bridge_mission_test
        ${CMAKE_SOURCE_DIR}/tests/fixtures/wopr_bridge_vtol.json)
    add_executable(wopr_bridge_arming_test wopr_bridge_arming_test.cpp)
    target_link_libraries(wopr_bridge_arming_test PRIVATE ap-hal-sitl ap-gcs)
    if(WIN32)
        target_link_libraries(wopr_bridge_arming_test PRIVATE ws2_32)
        target_link_options(wopr_bridge_arming_test PRIVATE -static)
    endif()
    add_test(NAME wopr_bridge_arming COMMAND wopr_bridge_arming_test)
    add_executable(wopr_bridge_taxi_test wopr_bridge_taxi_test.cpp)
    target_link_libraries(wopr_bridge_taxi_test PRIVATE ap-hal-sitl ap-gcs)
    if(WIN32)
        target_link_libraries(wopr_bridge_taxi_test PRIVATE ws2_32)
        target_link_options(wopr_bridge_taxi_test PRIVATE -static)
    endif()
    add_test(NAME wopr_bridge_taxi COMMAND wopr_bridge_taxi_test
        ${CMAKE_SOURCE_DIR}/tests/fixtures/wopr_bridge_model_overrides.json)
endif()
target_link_libraries(wopr_bridge PRIVATE ap-hal-sitl ap-gcs)

# Opt-in real-process MAVLink tests require the Python GCS dependency.
option(FWCPP_BRIDGE_MAVLINK_TEST "Enable WOPR MAVLink wire regressions (pymavlink)" OFF)
if(FWCPP_BRIDGE_MAVLINK_TEST)
    find_package(Python3 COMPONENTS Interpreter REQUIRED)
    execute_process(COMMAND ${Python3_EXECUTABLE} -c "import pymavlink"
        RESULT_VARIABLE FWCPP_PYMAVLINK_IMPORT_RESULT)
    if(NOT FWCPP_PYMAVLINK_IMPORT_RESULT EQUAL 0)
        message(FATAL_ERROR "FWCPP_BRIDGE_MAVLINK_TEST requires pymavlink in Python3_EXECUTABLE")
    endif()
    add_test(NAME wopr_bridge_mission_roundtrip
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/wopr_bridge_mission_roundtrip_test.py
            --exe $<TARGET_FILE:wopr_bridge>
            --model ${CMAKE_SOURCE_DIR}/tests/fixtures/wopr_bridge_model_overrides.json)
    set_tests_properties(wopr_bridge_mission_roundtrip PROPERTIES RUN_SERIAL TRUE TIMEOUT 30)
    add_test(NAME wopr_bridge_parameter_truth
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/wopr_bridge_parameter_test.py
            --exe $<TARGET_FILE:wopr_bridge>)
    set_tests_properties(wopr_bridge_parameter_truth PROPERTIES RUN_SERIAL TRUE TIMEOUT 30)
    add_test(NAME wopr_bridge_invalid_mode
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/wopr_bridge_invalid_mode_test.py
            --exe $<TARGET_FILE:wopr_bridge>
            --model ${CMAKE_SOURCE_DIR}/tests/fixtures/wopr_bridge_model_overrides.json)
    set_tests_properties(wopr_bridge_invalid_mode PROPERTIES RUN_SERIAL TRUE TIMEOUT 30)
    add_test(NAME wopr_bridge_command_component
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/wopr_bridge_component_test.py
            --exe $<TARGET_FILE:wopr_bridge>
            --model ${CMAKE_SOURCE_DIR}/tests/fixtures/wopr_bridge_model_overrides.json)
    set_tests_properties(wopr_bridge_command_component PROPERTIES RUN_SERIAL TRUE TIMEOUT 30)
    add_test(NAME wopr_bridge_mission_component
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/wopr_bridge_mission_component_test.py
            --exe $<TARGET_FILE:wopr_bridge>
            --model ${CMAKE_SOURCE_DIR}/tests/fixtures/wopr_bridge_model_overrides.json)
    set_tests_properties(wopr_bridge_mission_component PROPERTIES RUN_SERIAL TRUE TIMEOUT 30)
    add_test(NAME wopr_bridge_item_component
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/wopr_bridge_item_component_test.py
            --exe $<TARGET_FILE:wopr_bridge>
            --model ${CMAKE_SOURCE_DIR}/tests/fixtures/wopr_bridge_model_overrides.json)
    set_tests_properties(wopr_bridge_item_component PROPERTIES RUN_SERIAL TRUE TIMEOUT 30)
    add_test(NAME wopr_bridge_invalid_arm
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/wopr_bridge_invalid_arm_test.py
            --exe $<TARGET_FILE:wopr_bridge>
            --model ${CMAKE_SOURCE_DIR}/tests/fixtures/wopr_bridge_model_overrides.json)
    set_tests_properties(wopr_bridge_invalid_arm PROPERTIES RUN_SERIAL TRUE TIMEOUT 30)
    foreach(radius_case fallback boundary)
        add_test(NAME wopr_bridge_radius_${radius_case}
            COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/wopr_bridge_mission_roundtrip_test.py
                --exe $<TARGET_FILE:wopr_bridge>
                --model ${CMAKE_SOURCE_DIR}/tests/fixtures/wopr_radius_${radius_case}.json)
        set_tests_properties(wopr_bridge_radius_${radius_case} PROPERTIES RUN_SERIAL TRUE TIMEOUT 30)
    endforeach()
endif()
if(WIN32)
    target_link_libraries(wopr_bridge PRIVATE ws2_32)
    target_link_options(wopr_bridge PRIVATE -static)
endif()
