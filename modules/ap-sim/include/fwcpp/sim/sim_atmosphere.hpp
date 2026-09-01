#pragma once

// Forwarding header. The 1976 U.S. Standard Atmosphere model itself now lives
// in ap-common as <fwcpp/atmosphere.hpp>, because upstream it is AP_Baro --
// firmware code, not simulator code -- and the AHRS needs it to ESTIMATE
// eas2tas rather than have the harness read the plant's own field. Leaving it
// under ap-sim would have made the firmware depend on the simulator.
//
// Every fwcpp::sim:: name it used to define is re-exported unchanged, so the
// plant call sites that predate the move need no edit.

#include <fwcpp/atmosphere.hpp>
#include <fwcpp/sim/sim_sitl_input.hpp>

namespace fwcpp::sim {

using atmosphere::AtmosphereLayer;
using atmosphere::kAtmospheric1976;
using atmosphere::kAtmospheric1976Size;
using atmosphere::kGravityMss;
using atmosphere::kRadiusEarth;
using atmosphere::kRSpecific;
using atmosphere::kSslAirDensity;

using atmosphere::find_atmosphere_layer_by_altitude;
using atmosphere::geometric_alt_to_geopotential;
using atmosphere::geopotential_alt_to_geometric;
using atmosphere::get_air_density_for_alt_amsl;
using atmosphere::get_eas2tas_for_alt_amsl;
using atmosphere::get_pressure_temperature_for_alt_amsl;
using atmosphere::get_temperature_by_altitude_layer;

}  // namespace fwcpp::sim
