#pragma once

// This port's stand-in for AP_AHRS::get_EAS2TAS(), which upstream delegates to
// AP_Baro::get_EAS2TAS() -- pressure and temperature from the barometer, then
// EAS2TAS = sqrt(rho_ssl / rho).
//
// There is no AP_Baro here. StabilizeInputs::current_altitude_m is this port's
// established barometric-altitude substitute (see plane.hpp's own "BAROMETRIC
// ALTITUDE SUBSTITUTION" banner, which every FBWB/AUTO/RTL/LOITER slice already
// follows), so the factor is derived from that altitude through the same 1976
// U.S. Standard Atmosphere model AP_Baro uses.
//
// WHY THIS EXISTS AT ALL: the SITL harness previously read the PLANT's own
// eas2tas field. That is simulator truth, not an estimate -- the vehicle was
// being handed a number no real airframe could know. Deriving it here from the
// vehicle's own altitude signal keeps the conversion on the firmware side of
// the boundary, which is where upstream has it.
//
// HONEST LIMIT: with no barometer there is no pressure/temperature measurement
// and no sensor noise, bias or lag, so this is still a cleaner signal than a
// real aircraft would have. It is a standard-day model of the vehicle's own
// altitude, not a measurement. A real AP_Baro subsystem remains future work;
// when it lands, this function is the single place that changes.

#include <fwcpp/atmosphere.hpp>

namespace fwcpp::ahrs {

/** EAS -> TAS factor for an altitude expressed as metres AMSL.
 *  Upstream: AP_AHRS::get_EAS2TAS() / AP_Baro::get_EAS2TAS_for_alt_amsl(). */
[[nodiscard]] inline float get_eas2tas_for_alt_amsl(float alt_amsl_m) {
    return atmosphere::get_eas2tas_for_alt_amsl(alt_amsl_m);
}

/** Convenience for callers holding the port's usual pair: an altitude above
 *  the vehicle's fixed start point (StabilizeInputs::current_altitude_m) plus
 *  that start point's own AMSL elevation. Split out so the AMSL offset is
 *  named at the call site rather than being an unexplained addition -- getting
 *  it wrong silently biases every airspeed demand. */
[[nodiscard]] inline float get_eas2tas_above_origin(float alt_above_origin_m, float origin_amsl_m) {
    return get_eas2tas_for_alt_amsl(alt_above_origin_m + origin_amsl_m);
}

}  // namespace fwcpp::ahrs
