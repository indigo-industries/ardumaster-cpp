#pragma once

// CCP-045: port of libraries/SITL/SIM_Frame.h + SIM_Frame.cpp (Copter-4.7.0).
// Frame templates (motor angle / yaw_factor / servo index) and
// Frame::init / calculate_forces transcribed from original source.
//
// Disclosed leftovers vs original:
//   - CCP-067: the "JSON model load_frame_params is not ported" claim that
//     used to be here was stale - Frame::load_frame_params() (below) is a
//     real, working port of SIM_Frame.cpp's own load_frame_params()/
//     Frame::init() ":file.json" suffix mechanism (real lines 458/580-588),
//     round-trip tested against real upstream Callisto.json/freestyle.json
//     fixtures (see tests/sim_frame_test.cpp). Corrected in passing.
//   - Battery is a constant maxVoltage (no SIM_Battery drain / AP_Param
//     SIM_BATT_*). Short SITL missions are insensitive; current is still
//     summed from motors.
//   - AP::sitl()->vibe_motor RPM coupling is omitted (no SITL singleton).
//   - get_air_density uses troposphere ISA (0-11 km), not the full 1976
//     layered table in AP_Baro_atmosphere.cpp. SITL hover/refAlt=593 m
//     is inside that layer.
// Motor instances are copied into the Frame so two SimMulticopter objects
// do not share static motor slew state (original points at process-lifetime
// static arrays).

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>

#include <fwcpp/math/matrix3.hpp>
#include <fwcpp/math/scalar.hpp>
#include <fwcpp/math/vector3.hpp>
#include <fwcpp/sim/sim_atmosphere.hpp>
#include <fwcpp/sim/sim_json.hpp>
#include <fwcpp/sim/sim_sitl.hpp>
#include <fwcpp/sim/sim_motor.hpp>

namespace fwcpp::sim {

inline constexpr std::uint8_t kSimFrameMaxActuators = 32;

[[nodiscard]] inline float air_density_for_alt_amsl(float alt_amsl) {
    return get_air_density_for_alt_amsl(alt_amsl);
}

[[nodiscard]] inline bool frame_name_matches(const char* name, const char* prefix) {
    const std::size_t n = std::strlen(prefix);
    for (std::size_t i = 0; i < n; ++i) {
        char a = name[i];
        char b = prefix[i];
        if (a == '\0') {
            return false;
        }
        if (a >= 'A' && a <= 'Z') {
            a = static_cast<char>(a - 'A' + 'a');
        }
        if (b >= 'A' && b <= 'Z') {
            b = static_cast<char>(b - 'A' + 'a');
        }
        if (a != b) {
            return false;
        }
    }
    return true;
}

inline Motor quad_plus_motors[] = {
    Motor(kMot1, 90.0f, kMotorsYawFactorCcw, 2),
    Motor(kMot2, -90.0f, kMotorsYawFactorCcw, 4),
    Motor(kMot3, 0.0f, kMotorsYawFactorCw, 1),
    Motor(kMot4, 180.0f, kMotorsYawFactorCw, 3),
};

inline Motor quad_x_motors[] = {
    Motor(kMot1, 45.0f, kMotorsYawFactorCcw, 1),
    Motor(kMot2, -135.0f, kMotorsYawFactorCcw, 3),
    Motor(kMot3, -45.0f, kMotorsYawFactorCw, 4),
    Motor(kMot4, 135.0f, kMotorsYawFactorCw, 2),
};

inline Motor quad_bf_x_motors[] = {
    Motor(kMot1, 135.0f, kMotorsYawFactorCw, 2),
    Motor(kMot2, 45.0f, kMotorsYawFactorCcw, 1),
    Motor(kMot3, -135.0f, kMotorsYawFactorCcw, 3),
    Motor(kMot4, -45.0f, kMotorsYawFactorCw, 4),
};

inline Motor quad_bf_x_rev_motors[] = {
    Motor(kMot1, 135.0f, kMotorsYawFactorCcw, 2),
    Motor(kMot2, 45.0f, kMotorsYawFactorCw, 1),
    Motor(kMot3, -135.0f, kMotorsYawFactorCw, 3),
    Motor(kMot4, -45.0f, kMotorsYawFactorCcw, 4),
};

inline Motor quad_dji_x_motors[] = {
    Motor(kMot1, 45.0f, kMotorsYawFactorCcw, 1),
    Motor(kMot2, -45.0f, kMotorsYawFactorCw, 4),
    Motor(kMot3, -135.0f, kMotorsYawFactorCcw, 3),
    Motor(kMot4, 135.0f, kMotorsYawFactorCw, 2),
};

inline Motor quad_cw_x_motors[] = {
    Motor(kMot1, 45.0f, kMotorsYawFactorCcw, 1),
    Motor(kMot2, 135.0f, kMotorsYawFactorCw, 2),
    Motor(kMot3, -135.0f, kMotorsYawFactorCcw, 3),
    Motor(kMot4, -45.0f, kMotorsYawFactorCw, 4),
};

inline Motor dotriaconta_octaquad_x_motors[] = {
    Motor(kMot1, 45.0f, kMotorsYawFactorCcw, 1),
    Motor(kMot2, -135.0f, kMotorsYawFactorCcw, 17),
    Motor(kMot3, -45.0f, kMotorsYawFactorCw, 25),
    Motor(kMot4, 135.0f, kMotorsYawFactorCw, 9),
    Motor(kMot5, 45.0f, kMotorsYawFactorCw, 2),
    Motor(kMot6, -135.0f, kMotorsYawFactorCw, 18),
    Motor(kMot7, -45.0f, kMotorsYawFactorCcw, 26),
    Motor(kMot8, 135.0f, kMotorsYawFactorCcw, 10),
    Motor(kMot9, 45.0f, kMotorsYawFactorCcw, 3),
    Motor(kMot10, -135.0f, kMotorsYawFactorCcw, 19),
    Motor(kMot11, -45.0f, kMotorsYawFactorCw, 27),
    Motor(kMot12, 135.0f, kMotorsYawFactorCw, 11),
    Motor(kMot13, 45.0f, kMotorsYawFactorCw, 4),
    Motor(kMot14, -135.0f, kMotorsYawFactorCw, 20),
    Motor(kMot15, -45.0f, kMotorsYawFactorCcw, 28),
    Motor(kMot16, 135.0f, kMotorsYawFactorCcw, 12),
    Motor(kMot17, 45.0f, kMotorsYawFactorCcw, 5),
    Motor(kMot18, -135.0f, kMotorsYawFactorCcw, 21),
    Motor(kMot19, -45.0f, kMotorsYawFactorCw, 29),
    Motor(kMot20, 135.0f, kMotorsYawFactorCw, 13),
    Motor(kMot21, 45.0f, kMotorsYawFactorCw, 6),
    Motor(kMot22, -135.0f, kMotorsYawFactorCw, 22),
    Motor(kMot23, -45.0f, kMotorsYawFactorCcw, 30),
    Motor(kMot24, 135.0f, kMotorsYawFactorCcw, 14),
    Motor(kMot25, 45.0f, kMotorsYawFactorCcw, 7),
    Motor(kMot26, -135.0f, kMotorsYawFactorCcw, 23),
    Motor(kMot27, -45.0f, kMotorsYawFactorCw, 31),
    Motor(kMot28, 135.0f, kMotorsYawFactorCw, 15),
    Motor(kMot29, 45.0f, kMotorsYawFactorCw, 8),
    Motor(kMot30, -135.0f, kMotorsYawFactorCw, 24),
    Motor(kMot31, -45.0f, kMotorsYawFactorCcw, 32),
    Motor(kMot32, 135.0f, kMotorsYawFactorCcw, 16),
};

inline Motor tiltquad_h_vectored_motors[] = {
    Motor(kMot1, 45.0f, kMotorsYawFactorCw, 1, -1, 0.0f, 0.0f, kMot8, 10.0f, -90.0f),
    Motor(kMot2, -135.0f, kMotorsYawFactorCw, 3, -1, 0.0f, 0.0f, kMot9, 10.0f, -90.0f),
    Motor(kMot3, -45.0f, kMotorsYawFactorCcw, 4, -1, 0.0f, 0.0f, kMot9, 10.0f, -90.0f),
    Motor(kMot4, 135.0f, kMotorsYawFactorCcw, 2, -1, 0.0f, 0.0f, kMot8, 10.0f, -90.0f),
};

inline Motor tiltquad[] = {
    Motor(kMot1, 45.0f, kMotorsYawFactorCcw, 1, -1, 0.0f, 0.0f, kMot8, 10.0f, -90.0f),
    Motor(kMot2, -135.0f, kMotorsYawFactorCcw, 3),
    Motor(kMot3, -45.0f, kMotorsYawFactorCw, 4, -1, 0.0f, 0.0f, kMot9, 10.0f, -90.0f),
    Motor(kMot4, 135.0f, kMotorsYawFactorCw, 2),
};

inline Motor hexa_motors[] = {
    Motor(kMot1, 0.0f, kMotorsYawFactorCw, 1),
    Motor(kMot2, 180.0f, kMotorsYawFactorCcw, 4),
    Motor(kMot3, -120.0f, kMotorsYawFactorCw, 5),
    Motor(kMot4, 60.0f, kMotorsYawFactorCcw, 2),
    Motor(kMot5, -60.0f, kMotorsYawFactorCcw, 6),
    Motor(kMot6, 120.0f, kMotorsYawFactorCw, 3),
};

inline Motor hexax_motors[] = {
    Motor(kMot1, 90.0f, kMotorsYawFactorCw, 2),
    Motor(kMot2, -90.0f, kMotorsYawFactorCcw, 5),
    Motor(kMot3, -30.0f, kMotorsYawFactorCw, 6),
    Motor(kMot4, 150.0f, kMotorsYawFactorCcw, 3),
    Motor(kMot5, 30.0f, kMotorsYawFactorCcw, 1),
    Motor(kMot6, -150.0f, kMotorsYawFactorCw, 4),
};

inline Motor hexa_dji_x_motors[] = {
    Motor(kMot1, 30.0f, kMotorsYawFactorCcw, 1),
    Motor(kMot2, -30.0f, kMotorsYawFactorCw, 6),
    Motor(kMot3, -90.0f, kMotorsYawFactorCcw, 5),
    Motor(kMot4, -150.0f, kMotorsYawFactorCw, 4),
    Motor(kMot5, 150.0f, kMotorsYawFactorCcw, 3),
    Motor(kMot6, 90.0f, kMotorsYawFactorCw, 2),
};

inline Motor hexa_cw_x_motors[] = {
    Motor(kMot1, 30.0f, kMotorsYawFactorCcw, 1),
    Motor(kMot2, 90.0f, kMotorsYawFactorCw, 2),
    Motor(kMot3, 150.0f, kMotorsYawFactorCcw, 3),
    Motor(kMot4, -150.0f, kMotorsYawFactorCw, 4),
    Motor(kMot5, -90.0f, kMotorsYawFactorCcw, 5),
    Motor(kMot6, -30.0f, kMotorsYawFactorCw, 6),
};

inline Motor octa_motors[] = {
    Motor(kMot1, 0.0f, kMotorsYawFactorCw, 1),
    Motor(kMot2, 180.0f, kMotorsYawFactorCw, 5),
    Motor(kMot3, 45.0f, kMotorsYawFactorCcw, 2),
    Motor(kMot4, 135.0f, kMotorsYawFactorCcw, 4),
    Motor(kMot5, -45.0f, kMotorsYawFactorCcw, 8),
    Motor(kMot6, -135.0f, kMotorsYawFactorCcw, 6),
    Motor(kMot7, -90.0f, kMotorsYawFactorCw, 7),
    Motor(kMot8, 90.0f, kMotorsYawFactorCw, 3),
};

inline Motor octa_dji_x_motors[] = {
    Motor(kMot1, 22.5f, kMotorsYawFactorCcw, 1),
    Motor(kMot2, -22.5f, kMotorsYawFactorCw, 8),
    Motor(kMot3, -67.5f, kMotorsYawFactorCcw, 7),
    Motor(kMot4, -112.5f, kMotorsYawFactorCw, 6),
    Motor(kMot5, -157.5f, kMotorsYawFactorCcw, 5),
    Motor(kMot6, 157.5f, kMotorsYawFactorCw, 4),
    Motor(kMot7, 112.5f, kMotorsYawFactorCcw, 3),
    Motor(kMot8, 67.5f, kMotorsYawFactorCw, 2),
};

inline Motor octa_cw_x_motors[] = {
    Motor(kMot1, 22.5f, kMotorsYawFactorCcw, 1),
    Motor(kMot2, 67.5f, kMotorsYawFactorCw, 2),
    Motor(kMot3, 112.5f, kMotorsYawFactorCcw, 3),
    Motor(kMot4, 157.5f, kMotorsYawFactorCw, 4),
    Motor(kMot5, -157.5f, kMotorsYawFactorCcw, 5),
    Motor(kMot6, -112.5f, kMotorsYawFactorCw, 6),
    Motor(kMot7, -67.5f, kMotorsYawFactorCcw, 7),
    Motor(kMot8, -22.5f, kMotorsYawFactorCw, 8),
};

inline Motor octa_quad_motors[] = {
    Motor(kMot1, 45.0f, kMotorsYawFactorCcw, 1),
    Motor(kMot2, -45.0f, kMotorsYawFactorCw, 7),
    Motor(kMot3, -135.0f, kMotorsYawFactorCcw, 5),
    Motor(kMot4, 135.0f, kMotorsYawFactorCw, 3),
    Motor(kMot5, -45.0f, kMotorsYawFactorCcw, 8),
    Motor(kMot6, 45.0f, kMotorsYawFactorCw, 2),
    Motor(kMot7, 135.0f, kMotorsYawFactorCcw, 4),
    Motor(kMot8, -135.0f, kMotorsYawFactorCw, 6),
};

inline Motor octa_quad_corotating_motors[] = {
    Motor(kMot1, 45.0f, kMotorsYawFactorCcw, 1),
    Motor(kMot2, -45.0f, kMotorsYawFactorCw, 7),
    Motor(kMot3, -135.0f, kMotorsYawFactorCcw, 5),
    Motor(kMot4, 135.0f, kMotorsYawFactorCw, 3),
    Motor(kMot5, -45.0f, kMotorsYawFactorCw, 8),
    Motor(kMot6, 45.0f, kMotorsYawFactorCw, 2),
    Motor(kMot7, 135.0f, kMotorsYawFactorCw, 4),
    Motor(kMot8, -135.0f, kMotorsYawFactorCw, 6),
};

inline Motor octa_quad_cw_corotating_motors[] = {
    Motor(kMot1, 45.0f, kMotorsYawFactorCcw, 1),
    Motor(kMot2, 45.0f, kMotorsYawFactorCcw, 2),
    Motor(kMot3, 135.0f, kMotorsYawFactorCw, 3),
    Motor(kMot4, 135.0f, kMotorsYawFactorCw, 4),
    Motor(kMot5, -135.0f, kMotorsYawFactorCcw, 5),
    Motor(kMot6, -135.0f, kMotorsYawFactorCcw, 6),
    Motor(kMot7, -45.0f, kMotorsYawFactorCw, 7),
    Motor(kMot8, -45.0f, kMotorsYawFactorCw, 8),
};

inline Motor octa_quad_cw_x_motors[] = {
    Motor(kMot1, 45.0f, kMotorsYawFactorCcw, 1),
    Motor(kMot2, 45.0f, kMotorsYawFactorCw, 2),
    Motor(kMot3, 135.0f, kMotorsYawFactorCcw, 3),
    Motor(kMot4, 135.0f, kMotorsYawFactorCw, 4),
    Motor(kMot5, -135.0f, kMotorsYawFactorCcw, 5),
    Motor(kMot6, -135.0f, kMotorsYawFactorCw, 6),
    Motor(kMot7, -45.0f, kMotorsYawFactorCcw, 7),
    Motor(kMot8, -45.0f, kMotorsYawFactorCw, 8),
};

inline Motor dodeca_hexa_motors[] = {
    Motor(kMot1, 30.0f, kMotorsYawFactorCcw, 1),
    Motor(kMot2, 30.0f, kMotorsYawFactorCw, 2),
    Motor(kMot3, 90.0f, kMotorsYawFactorCw, 3),
    Motor(kMot4, 90.0f, kMotorsYawFactorCcw, 4),
    Motor(kMot5, 150.0f, kMotorsYawFactorCcw, 5),
    Motor(kMot6, 150.0f, kMotorsYawFactorCw, 6),
    Motor(kMot7, -150.0f, kMotorsYawFactorCw, 7),
    Motor(kMot8, -150.0f, kMotorsYawFactorCcw, 8),
    Motor(kMot9, -90.0f, kMotorsYawFactorCcw, 9),
    Motor(kMot10, -90.0f, kMotorsYawFactorCw, 10),
    Motor(kMot11, -30.0f, kMotorsYawFactorCw, 11),
    Motor(kMot12, -30.0f, kMotorsYawFactorCcw, 12),
};

inline Motor hexadeca_octa_motors[] = {
    Motor(kMot1, 0.0f, kMotorsYawFactorCw, 1),
    Motor(kMot2, 0.0f, kMotorsYawFactorCcw, 2),
    Motor(kMot3, 45.0f, kMotorsYawFactorCcw, 3),
    Motor(kMot4, 45.0f, kMotorsYawFactorCw, 4),
    Motor(kMot5, 90.0f, kMotorsYawFactorCw, 5),
    Motor(kMot6, 90.0f, kMotorsYawFactorCcw, 6),
    Motor(kMot7, 135.0f, kMotorsYawFactorCcw, 7),
    Motor(kMot8, 135.0f, kMotorsYawFactorCw, 8),
    Motor(kMot9, 180.0f, kMotorsYawFactorCw, 9),
    Motor(kMot10, 180.0f, kMotorsYawFactorCcw, 10),
    Motor(kMot11, -135.0f, kMotorsYawFactorCcw, 11),
    Motor(kMot12, -135.0f, kMotorsYawFactorCw, 12),
    Motor(kMot13, -90.0f, kMotorsYawFactorCw, 13),
    Motor(kMot14, -90.0f, kMotorsYawFactorCcw, 14),
    Motor(kMot15, -45.0f, kMotorsYawFactorCcw, 15),
    Motor(kMot16, -45.0f, kMotorsYawFactorCw, 16),
};

inline Motor hexadeca_octa_cw_x_motors[] = {
    Motor(kMot1, 22.5f, kMotorsYawFactorCw, 1),
    Motor(kMot2, 22.5f, kMotorsYawFactorCcw, 2),
    Motor(kMot3, 67.5f, kMotorsYawFactorCcw, 3),
    Motor(kMot4, 67.5f, kMotorsYawFactorCw, 4),
    Motor(kMot5, 112.5f, kMotorsYawFactorCw, 5),
    Motor(kMot6, 112.5f, kMotorsYawFactorCcw, 6),
    Motor(kMot7, 157.5f, kMotorsYawFactorCcw, 7),
    Motor(kMot8, 157.5f, kMotorsYawFactorCw, 8),
    Motor(kMot9, -157.5f, kMotorsYawFactorCw, 9),
    Motor(kMot10, -157.5f, kMotorsYawFactorCcw, 10),
    Motor(kMot11, -112.5f, kMotorsYawFactorCcw, 11),
    Motor(kMot12, -112.5f, kMotorsYawFactorCw, 12),
    Motor(kMot13, -67.5f, kMotorsYawFactorCw, 13),
    Motor(kMot14, -67.5f, kMotorsYawFactorCcw, 14),
    Motor(kMot15, -22.5f, kMotorsYawFactorCcw, 15),
    Motor(kMot16, -22.5f, kMotorsYawFactorCw, 16),
};

inline Motor deca_motors[] = {
    Motor(kMot1, 0.0f, kMotorsYawFactorCcw, 1),
    Motor(kMot2, 36.0f, kMotorsYawFactorCw, 2),
    Motor(kMot3, 72.0f, kMotorsYawFactorCcw, 3),
    Motor(kMot4, 108.0f, kMotorsYawFactorCw, 4),
    Motor(kMot5, 144.0f, kMotorsYawFactorCcw, 5),
    Motor(kMot6, 180.0f, kMotorsYawFactorCw, 6),
    Motor(kMot7, -144.0f, kMotorsYawFactorCcw, 7),
    Motor(kMot8, -108.0f, kMotorsYawFactorCw, 8),
    Motor(kMot9, -72.0f, kMotorsYawFactorCcw, 9),
    Motor(kMot10, -36.0f, kMotorsYawFactorCw, 10),
};

inline Motor deca_cw_x_motors[] = {
    Motor(kMot1, 18.0f, kMotorsYawFactorCcw, 1),
    Motor(kMot2, 54.0f, kMotorsYawFactorCw, 2),
    Motor(kMot3, 90.0f, kMotorsYawFactorCcw, 3),
    Motor(kMot4, 126.0f, kMotorsYawFactorCw, 4),
    Motor(kMot5, 162.0f, kMotorsYawFactorCcw, 5),
    Motor(kMot6, -162.0f, kMotorsYawFactorCw, 6),
    Motor(kMot7, -126.0f, kMotorsYawFactorCcw, 7),
    Motor(kMot8, -90.0f, kMotorsYawFactorCw, 8),
    Motor(kMot9, -54.0f, kMotorsYawFactorCcw, 9),
    Motor(kMot10, -18.0f, kMotorsYawFactorCw, 10),
};

inline Motor tri_motors[] = {
    Motor(kMot1, 60.0f, kMotorsYawFactorCcw, 1),
    Motor(kMot2, -60.0f, kMotorsYawFactorCw, 3),
    Motor(kMot4, 180.0f, kMotorsYawFactorCcw, 2, kMot7, 60.0f, -60.0f, -1, 0.0f, 0.0f),
};

inline Motor tilttri_motors[] = {
    Motor(kMot1, 60.0f, kMotorsYawFactorCcw, 1, -1, 0.0f, 0.0f, kMot8, 0.0f, -90.0f),
    Motor(kMot2, -60.0f, kMotorsYawFactorCw, 3, -1, 0.0f, 0.0f, kMot8, 0.0f, -90.0f),
    Motor(kMot4, 180.0f, kMotorsYawFactorCcw, 2, kMot7, 60.0f, -60.0f, -1, 0.0f, 0.0f),
};

inline Motor tilttri_vectored_motors[] = {
    Motor(kMot1, 60.0f, kMotorsYawFactorCcw, 1, -1, 0.0f, 0.0f, kMot8, 10.0f, -90.0f),
    Motor(kMot2, -60.0f, kMotorsYawFactorCw, 3, -1, 0.0f, 0.0f, kMot9, 10.0f, -90.0f),
    Motor(kMot4, 180.0f, kMotorsYawFactorCcw, 2),
};

inline Motor y6_motors[] = {
    Motor(kMot1, 60.0f, kMotorsYawFactorCcw, 2),
    Motor(kMot2, -60.0f, kMotorsYawFactorCw, 5),
    Motor(kMot3, -60.0f, kMotorsYawFactorCcw, 6),
    Motor(kMot4, 180.0f, kMotorsYawFactorCw, 4),
    Motor(kMot5, 60.0f, kMotorsYawFactorCw, 1),
    Motor(kMot6, 180.0f, kMotorsYawFactorCcw, 3),
};

inline Motor firefly_motors[] = {
    Motor(kMot1, 180.0f, kMotorsYawFactorCcw, 3),
    Motor(kMot2, 60.0f, kMotorsYawFactorCcw, 1, -1, 0.0f, 0.0f, kMot7, 0.0f, -90.0f),
    Motor(kMot3, -60.0f, kMotorsYawFactorCcw, 5, -1, 0.0f, 0.0f, kMot7, 0.0f, -90.0f),
    Motor(kMot4, 180.0f, kMotorsYawFactorCw, 4),
    Motor(kMot5, 60.0f, kMotorsYawFactorCw, 2, -1, 0.0f, 0.0f, kMot7, 0.0f, -90.0f),
    Motor(kMot6, -60.0f, kMotorsYawFactorCw, 6, -1, 0.0f, 0.0f, kMot7, 0.0f, -90.0f),
};

struct FrameTemplate {
    const char* name;
    std::uint8_t num_motors;
    Motor* motors;
};

inline FrameTemplate kSupportedFrameTemplates[] = {
    {"+", static_cast<std::uint8_t>(4), quad_plus_motors},
    {"quad", static_cast<std::uint8_t>(4), quad_plus_motors},
    {"copter", static_cast<std::uint8_t>(4), quad_plus_motors},
    {"x", static_cast<std::uint8_t>(4), quad_x_motors},
    {"bfxrev", static_cast<std::uint8_t>(4), quad_bf_x_rev_motors},
    {"bfx", static_cast<std::uint8_t>(4), quad_bf_x_motors},
    {"dotriaconta", static_cast<std::uint8_t>(32), dotriaconta_octaquad_x_motors},
    {"djix", static_cast<std::uint8_t>(4), quad_dji_x_motors},
    {"cwx", static_cast<std::uint8_t>(4), quad_cw_x_motors},
    {"tilthvec", static_cast<std::uint8_t>(4), tiltquad_h_vectored_motors},
    {"hexadeca-octa", static_cast<std::uint8_t>(16), hexadeca_octa_motors},
    {"hexadeca-octa-cwx", static_cast<std::uint8_t>(16), hexadeca_octa_cw_x_motors},
    {"hexax", static_cast<std::uint8_t>(6), hexax_motors},
    {"hexa-cwx", static_cast<std::uint8_t>(6), hexa_cw_x_motors},
    {"hexa-dji", static_cast<std::uint8_t>(6), hexa_dji_x_motors},
    {"hexa", static_cast<std::uint8_t>(6), hexa_motors},
    {"octa-cwx", static_cast<std::uint8_t>(8), octa_cw_x_motors},
    {"octa-dji", static_cast<std::uint8_t>(8), octa_dji_x_motors},
    {"octa-quad-cwx", static_cast<std::uint8_t>(8), octa_quad_cw_x_motors},
    {"octa-quad-cor", static_cast<std::uint8_t>(8), octa_quad_corotating_motors},
    {"octa-quad-cw-cor", static_cast<std::uint8_t>(8), octa_quad_cw_corotating_motors},
    {"octa-quad", static_cast<std::uint8_t>(8), octa_quad_motors},
    {"octa", static_cast<std::uint8_t>(8), octa_motors},
    {"deca", static_cast<std::uint8_t>(10), deca_motors},
    {"deca-cwx", static_cast<std::uint8_t>(10), deca_cw_x_motors},
    {"dodeca-hexa", static_cast<std::uint8_t>(12), dodeca_hexa_motors},
    {"tri", static_cast<std::uint8_t>(3), tri_motors},
    {"tilttrivec", static_cast<std::uint8_t>(3), tilttri_vectored_motors},
    {"tilttri", static_cast<std::uint8_t>(3), tilttri_motors},
    {"y6", static_cast<std::uint8_t>(6), y6_motors},
    {"firefly", static_cast<std::uint8_t>(6), firefly_motors},
    {"tilt", static_cast<std::uint8_t>(4), tiltquad}
};

struct FrameModel {
    float mass = 3.0f;
    float diagonal_size = 0.35f;
    float refSpd = 15.08f;
    float refAngle = 45.0f;
    float refVoltage = 12.09f;
    float refCurrent = 29.3f;
    float refAlt = 593.0f;
    float refTempC = 25.0f;
    float refBatRes = 0.01f;
    float maxVoltage = 4.2f * 3.0f;
    float battCapacityAh = 0.0f;
    float hoverThrOut = 0.39f;
    float propExpo = 0.65f;
    float refRotRate = 120.0f;
    float pwmMin = 1000.0f;
    float pwmMax = 2000.0f;
    float spin_min = 0.15f;
    float spin_max = 0.95f;
    float slew_max = 150.0f;
    float disc_area = 0.385f;
    float mdrag_coef = 0.2f;
    float bbdrag_coef = 1.0f;
    math::Vector3f moment_of_inertia{};
    math::Vector3f motor_pos[kSimFrameMaxActuators]{};
    math::Vector3f motor_thrust_vec[kSimFrameMaxActuators]{};
    float yaw_factor[kSimFrameMaxActuators]{};
    float num_motors = 4.0f;
};

class Frame {
public:
    const char* name{nullptr};
    std::uint8_t num_motors{0};
    Motor* motors{nullptr};
    float terminal_velocity{0.0f};
    float terminal_rotation_rate{0.0f};
    std::uint8_t motor_offset{0};

    Frame() = default;

    Frame(const char* n, std::uint8_t nmot, const Motor* src) : name(n), num_motors(nmot) {
        for (std::uint8_t i = 0; i < nmot && i < kSimFrameMaxActuators; ++i) {
            motor_store_[i] = src[i];
        }
        rebind_motors();
    }

    // motor_store_ is the owned Motor array. `motors` must always point at
    // THIS object's store. Default copy/move copies the raw pointer, which
    // dangles after Frame::create_frame() returns (SimMulticopter assigns
    // that temporary). Tests sometimes survived on leftover stack; copter_main
    // did not — PWM mixed into the wrong servo map and calculate_forces saw
    // garbage Motor state (no thrust).
    Frame(const Frame& other) { *this = other; }
    Frame(Frame&& other) noexcept { *this = other; }
    Frame& operator=(const Frame& other) {
        if (this == &other) {
            return *this;
        }
        name = other.name;
        num_motors = other.num_motors;
        terminal_velocity = other.terminal_velocity;
        terminal_rotation_rate = other.terminal_rotation_rate;
        motor_offset = other.motor_offset;
        mass_scale = other.mass_scale;
        sitl = other.sitl;
        rpm_out = other.rpm_out;
        battery_dirty = other.battery_dirty;
        motor_store_ = other.motor_store_;
        model = other.model;
        area_cd_ = other.area_cd_;
        mass_ = other.mass_;
        battery_voltage_ = other.battery_voltage_;
        rebind_motors();
        return *this;
    }
    Frame& operator=(Frame&& other) noexcept {
        *this = static_cast<const Frame&>(other);
        return *this;
    }

    static Frame create_frame(const char* frame_name) {
        for (auto& tplate : kSupportedFrameTemplates) {
            if (frame_name_matches(frame_name, tplate.name)) {
                return Frame(tplate.name, tplate.num_motors, tplate.motors);
            }
        }
        return Frame();
    }

    [[nodiscard]] bool valid() const { return motors != nullptr && num_motors > 0; }

    void init(const char* frame_str) {
        model = FrameModel{};
        if (frame_str != nullptr) {
            const char* colon = std::strchr(frame_str, ':');
            const std::size_t slen = std::strlen(frame_str);
            if (colon != nullptr && slen > 5 && std::strcmp(&frame_str[slen - 5], ".json") == 0) {
                load_frame_params(colon + 1);
            }
        }
        mass_ = model.mass * mass_scale;

        const float drag_force = model.mass * kGravityMss * std::tan(math::radians(model.refAngle));
        const float cos_tilt = std::cos(math::radians(model.refAngle));
        const float airspeed_bf = model.refSpd * cos_tilt;
        const float ref_thrust = model.mass * kGravityMss / cos_tilt;
        const float ref_air_density = air_density_for_alt_amsl(model.refAlt);

        const float momentum_drag =
            cos_tilt * model.mdrag_coef * airspeed_bf * std::sqrt(ref_thrust * ref_air_density * model.disc_area);

        if (momentum_drag > drag_force) {
            model.mdrag_coef *= drag_force / momentum_drag;
            area_cd_ = 0.0f;
        } else {
            area_cd_ = model.bbdrag_coef * (drag_force - momentum_drag) / (0.5f * ref_air_density * model.refSpd * model.refSpd);
        }

        terminal_rotation_rate = model.refRotRate;

        const float hover_thrust = mass_ * kGravityMss;
        const float hover_power = model.refCurrent * model.refVoltage;
        const float hover_velocity_out = 2.0f * hover_power / hover_thrust;
        const float effective_disc_area =
            hover_thrust / (0.5f * ref_air_density * hover_velocity_out * hover_velocity_out);
        const float velocity_max = hover_velocity_out / std::sqrt(model.hoverThrOut);
        const float effective_prop_area = effective_disc_area / static_cast<float>(num_motors);
        const float true_prop_area = model.disc_area / static_cast<float>(num_motors);
        const float power_factor = hover_power / hover_thrust;

        battery_voltage_ = model.maxVoltage;
        battery_dirty = true;

        for (std::uint8_t i = 0; i < num_motors; ++i) {
            motors[i].setup_params(static_cast<std::uint16_t>(model.pwmMin), static_cast<std::uint16_t>(model.pwmMax),
                                   model.spin_min, model.spin_max, model.propExpo, model.slew_max, model.diagonal_size,
                                   power_factor, model.maxVoltage, effective_prop_area, velocity_max,
                                   model.motor_pos[i], model.motor_thrust_vec[i], model.yaw_factor[i], true_prop_area,
                                   model.mdrag_coef);
        }

        if (math::is_zero(model.moment_of_inertia.x) || math::is_zero(model.moment_of_inertia.y) ||
            math::is_zero(model.moment_of_inertia.z)) {
            model.moment_of_inertia.x = model.mass * 0.25f * (model.diagonal_size * 0.5f) * (model.diagonal_size * 0.5f);
            model.moment_of_inertia.y = model.moment_of_inertia.x;
            model.moment_of_inertia.z = model.mass * 0.5f * (model.diagonal_size * 0.5f) * (model.diagonal_size * 0.5f);
        }
    }

    void calculate_forces(const math::Matrix3f& dcm, const math::Vector3f& velocity_ef, const math::Vector3f& gyro,
                          float alt_amsl, const SitlInput& input, math::Vector3f& rot_accel, math::Vector3f& body_accel,
                          float gross_mass, bool use_drag, std::uint64_t time_us) {
        math::Vector3f thrust;
        math::Vector3f torque;

        const float air_density = air_density_for_alt_amsl(alt_amsl);
        const math::Vector3f vel_air_bf = dcm.transposed() * velocity_ef;

        for (std::uint8_t i = 0; i < num_motors; ++i) {
            math::Vector3f mtorque;
            math::Vector3f mthrust;
            motors[i].calculate_forces(input, motor_offset, mtorque, mthrust, vel_air_bf, gyro, air_density,
                                       battery_voltage_, use_drag, time_us);
            torque += mtorque;
            thrust += mthrust;
            if (sitl != nullptr && !math::is_zero(sitl->vibe_motor) && rpm_out != nullptr) {
                rpm_out[motor_offset + i] = motors[i].get_command() * sitl->vibe_motor * 60.0f;
            }
        }

        rot_accel.x = torque.x / model.moment_of_inertia.x;
        rot_accel.y = torque.y / model.moment_of_inertia.y;
        rot_accel.z = torque.z / model.moment_of_inertia.z;

        if (terminal_rotation_rate > 0.0f) {
            rot_accel.x -= gyro.x * math::radians(400.0f) / terminal_rotation_rate;
            rot_accel.y -= gyro.y * math::radians(400.0f) / terminal_rotation_rate;
            rot_accel.z -= gyro.z * math::radians(400.0f) / terminal_rotation_rate;
        }

        if (use_drag) {
            math::Vector3f drag_bf;
            drag_bf.x = area_cd_ * 0.5f * air_density * vel_air_bf.x * vel_air_bf.x;
            if (math::is_negative(vel_air_bf.x)) {
                drag_bf.x = -drag_bf.x;
            }
            drag_bf.y = area_cd_ * 0.5f * air_density * vel_air_bf.y * vel_air_bf.y;
            if (math::is_negative(vel_air_bf.y)) {
                drag_bf.y = -drag_bf.y;
            }
            drag_bf.z = area_cd_ * 0.5f * air_density * vel_air_bf.z * vel_air_bf.z;
            if (math::is_negative(vel_air_bf.z)) {
                drag_bf.z = -drag_bf.z;
            }
            thrust -= drag_bf;
        }

        body_accel = thrust / gross_mass;
    }

    void current_and_voltage(float& voltage, float& current) const {
        voltage = battery_voltage_;
        current = 0.0f;
        for (std::uint8_t i = 0; i < num_motors; ++i) {
            current += motors[i].get_current();
        }
    }

    [[nodiscard]] float get_mass() const { return mass_; }
    void set_mass(float new_mass) { mass_ = new_mass; }
    [[nodiscard]] float hover_thr_out() const { return model.hoverThrOut; }

    // Command that yields hover_velocity_out through calc_thrust expo:
    // (1-e)*c + e*c^2 = hoverThrOut. Original copter control finds this
    // operating point; leftover mission uses it as the hold collective.
    [[nodiscard]] float hover_command() const {
        const float e = model.propExpo;
        const float h = model.hoverThrOut;
        if (e <= 1.0e-6f) {
            return h;
        }
        const float disc = (1.0f - e) * (1.0f - e) + 4.0f * e * h;
        return (-(1.0f - e) + std::sqrt(disc)) / (2.0f * e);
    }
    [[nodiscard]] float battery_voltage() const { return battery_voltage_; }
    void set_battery_voltage(float v) { battery_voltage_ = v; }
    [[nodiscard]] const FrameModel& get_model() const { return model; }

    [[nodiscard]] std::uint16_t command_to_pwm(float command) const {
        if (num_motors == 0) {
            return 1000;
        }
        return motors[0].command_to_pwm(command);
    }

    void set_equal_command(SitlInput& input, float command) const {
        const std::uint16_t pwm = command_to_pwm(command);
        for (std::uint8_t i = 0; i < num_motors; ++i) {
            input.servos[motor_offset + motors[i].servo] = pwm;
        }
    }


    float mass_scale{1.0f};
    SitlParams* sitl{nullptr};
    float* rpm_out{nullptr};
    bool battery_dirty{false};

    void set_mass_scale(float scale) { mass_scale = scale; }
    void set_sitl(SitlParams* s) { sitl = s; }

    [[nodiscard]] float get_model_batt_max_voltage() const { return model.maxVoltage; }
    [[nodiscard]] float get_model_batt_capacity_ah() const { return model.battCapacityAh; }
    [[nodiscard]] float get_model_batt_resistance_ohm() const { return model.refBatRes; }
    [[nodiscard]] float get_current_amp() const {
        float current = 0.0f;
        for (std::uint8_t i = 0; i < num_motors; ++i) {
            current += motors[i].get_current();
        }
        return current;
    }
    bool battery_changed() {
        const bool ret = battery_dirty;
        battery_dirty = false;
        return ret;
    }

    bool load_frame_params(const char* model_json) {
        JsonValue obj;
        std::string err;
        if (!load_json_file(model_json, obj, err)) {
            return false;
        }
        json_get_float(obj, "mass", model.mass);
        json_get_float(obj, "diagonal_size", model.diagonal_size);
        json_get_float(obj, "refSpd", model.refSpd);
        json_get_float(obj, "refAngle", model.refAngle);
        json_get_float(obj, "refVoltage", model.refVoltage);
        json_get_float(obj, "refCurrent", model.refCurrent);
        json_get_float(obj, "refAlt", model.refAlt);
        json_get_float(obj, "maxVoltage", model.maxVoltage);
        json_get_float(obj, "battCapacityAh", model.battCapacityAh);
        json_get_float(obj, "refBatRes", model.refBatRes);
        json_get_float(obj, "propExpo", model.propExpo);
        json_get_float(obj, "refRotRate", model.refRotRate);
        json_get_float(obj, "hoverThrOut", model.hoverThrOut);
        json_get_float(obj, "pwmMin", model.pwmMin);
        json_get_float(obj, "pwmMax", model.pwmMax);
        json_get_float(obj, "spin_min", model.spin_min);
        json_get_float(obj, "spin_max", model.spin_max);
        json_get_float(obj, "slew_max", model.slew_max);
        json_get_float(obj, "disc_area", model.disc_area);
        json_get_float(obj, "mdrag_coef", model.mdrag_coef);
        json_get_float(obj, "bbdrag_coef", model.bbdrag_coef);
        json_get_float(obj, "refTempC", model.refTempC);
        json_get_float(obj, "num_motors", model.num_motors);
        json_get_vector3(obj, "moment_inertia", model.moment_of_inertia);
        char label[32];
        for (std::uint8_t j = 0; j < kSimFrameMaxActuators; ++j) {
            std::snprintf(label, sizeof(label), "motor%u_position", static_cast<unsigned>(j + 1));
            json_get_vector3(obj, label, model.motor_pos[j]);
            std::snprintf(label, sizeof(label), "motor%u_vector", static_cast<unsigned>(j + 1));
            json_get_vector3(obj, label, model.motor_thrust_vec[j]);
            std::snprintf(label, sizeof(label), "motor%u_yaw", static_cast<unsigned>(j + 1));
            json_get_float(obj, label, model.yaw_factor[j]);
        }
        return true;
    }

    void rebind_motors() {
        motors = (num_motors > 0) ? motor_store_.data() : nullptr;
    }

private:
    std::array<Motor, kSimFrameMaxActuators> motor_store_{};
    FrameModel model{};
    float area_cd_{0.0f};
    float mass_{3.0f};
    float battery_voltage_{12.6f};
};

}  // namespace fwcpp::sim
