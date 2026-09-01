add_library(ap-baro INTERFACE)
target_include_directories(ap-baro INTERFACE include)
target_compile_features(ap-baro INTERFACE cxx_std_20)

# No dependencies: Baro's interface is plain floats/bools and its
# implementation needs only <cmath>/<cstdint>, the same reasoning
# ap-airspeed's own CMakeLists records.
