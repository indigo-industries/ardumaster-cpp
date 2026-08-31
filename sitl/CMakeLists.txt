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

add_executable(quadplane_sitl_run quadplane_main.cpp)
target_link_libraries(quadplane_sitl_run PRIVATE ap-sim)

# WOPR-BRIDGE: lockstep UDP bridge around SitlHarness for the WOPR5000
# Unreal sim (external-autopilot mode). Same ap-hal-sitl link as sitl_run,
# plus the platform socket library. Linked statically so the executable can
# be spawned by a host process without the llvm-mingw runtime DLLs
# (libc++.dll/libunwind.dll/libwinpthread-1.dll) on PATH.
add_executable(wopr_bridge wopr_bridge.cpp)
target_link_libraries(wopr_bridge PRIVATE ap-hal-sitl ap-gcs)
if(WIN32)
    target_link_libraries(wopr_bridge PRIVATE ws2_32)
    target_link_options(wopr_bridge PRIVATE -static)
endif()
