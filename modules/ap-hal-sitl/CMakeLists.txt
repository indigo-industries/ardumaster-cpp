add_library(ap-hal-sitl INTERFACE)
target_include_directories(ap-hal-sitl INTERFACE include)
target_compile_features(ap-hal-sitl INTERFACE cxx_std_20)
# ap-vehicle+ap-sim: SitlHarness (CPP-084). ap-copter: SitlCopterHarness (CCP-043).
target_link_libraries(ap-hal-sitl INTERFACE ap-vehicle ap-sim ap-copter ap-motors ap-quadplane ap-q-modes ap-wpnav)
