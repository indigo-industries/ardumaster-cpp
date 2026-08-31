cmake_minimum_required(VERSION 3.25)

project(plane_fw_cpp
    VERSION 0.1.0
    DESCRIPTION "ArduPlane fixed-wing port to modern C++ (CPP effort). See ADR-0012."
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "" FORCE)
endif()

# ADR-0012 decision 7: precision is a build option, not a template parameter.
option(FWCPP_EKF_DOUBLE "Use double precision for ftype (matches upstream HAL_WITH_EKF_DOUBLE)" OFF)

# Same ADR-0012 decision 7 pattern, for a different upstream precision
# switch: postype_t/Vector2p/Vector3p (AP_Math/control.h). Upstream:
# `#if HAL_WITH_POSTYPE_DOUBLE` where HAL_WITH_POSTYPE_DOUBLE is itself
# `HAL_PROGRAM_SIZE_LIMIT_KB > 1024` - true for every board with more than
# 1MB of flash. This port's target is SITL (see ADR/memory: "SITL target,
# full fidelity"), which has no such limit, so ON is the value that
# actually matches what this port runs as - not an arbitrary default.
option(FWCPP_POSTYPE_DOUBLE "Use double precision for postype_t (matches upstream HAL_WITH_POSTYPE_DOUBLE, true for SITL)" ON)

# Warnings apply everywhere, tests included - these don't affect literal
# typing or numeric behavior.
add_compile_options(-Wall -Wextra)

# ADR-0012 decision 1's upstream-matching, BEHAVIOR-affecting flags
# (-fsingle-precision-constant above all: it changes what a bare literal
# *means*, an f32/f64 rather than a warning). This is an INTERFACE target,
# not a global add_compile_options, and every modules/* target links it -
# see modules/CMakeLists.txt's fwcpp_upstream_flags usage.
#
# Deliberately NOT applied to tests/*. A test author writing `1e-10` to probe
# a specific double-precision code path needs that literal to actually BE a
# double; discovered the hard way when CPP-004's own is_equal test silently
# exercised the float overload instead of the double one it was written to
# test, because the flag was originally global and coerced the test's own
# literal. The port's fidelity to upstream's literal-typing behavior belongs
# in the port's code (modules/*), verified by tests written with ordinary
# C++ literal semantics - not baked into the tests themselves.
add_library(fwcpp_upstream_flags INTERFACE)
target_compile_options(fwcpp_upstream_flags INTERFACE
    -fno-threadsafe-statics
    -fsingle-precision-constant
    -fno-signed-zeros
    -fno-trapping-math
    -fno-math-errno
)

# Native Windows builds: expose POSIX bits the Linux SITL host provides
# (<strings.h>, M_PI). No-op on Linux.
if(WIN32)
    target_include_directories(fwcpp_upstream_flags INTERFACE
        ${CMAKE_SOURCE_DIR}/cmake/win-compat)
    target_compile_definitions(fwcpp_upstream_flags INTERFACE
        _USE_MATH_DEFINES
        _CRT_SECURE_NO_WARNINGS)
    # Header-only consumers of sitl/ (and modules that don't link the
    # INTERFACE flags) still need the compat include path.
    include_directories(${CMAKE_SOURCE_DIR}/cmake/win-compat)
    add_compile_definitions(_USE_MATH_DEFINES _CRT_SECURE_NO_WARNINGS)
endif()

enable_testing()

# Catch2 is a system package on the Linux port host. Allow SITL-only builds
# on machines that have a C++ toolchain but no Catch2 (e.g. Windows).
option(FWCPP_BUILD_TESTS "Build Catch2 unit/parity tests" ON)

add_subdirectory(modules)
if(FWCPP_BUILD_TESTS)
    add_subdirectory(tests)
endif()
add_subdirectory(sitl)
