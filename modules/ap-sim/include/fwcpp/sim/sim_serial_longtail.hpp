#pragma once

// Facade: original-source SIM serial parsers live in dedicated headers.
// CCP-062: VectorNav / MicroStrain / InertialLabs / SensAItion / gimbals /
// power / proximity. CCP-063: FrSky D / CRSF / ELRS / Volz.

#include <fwcpp/sim/sim_crsf.hpp>
#include <fwcpp/sim/sim_elrs.hpp>
#include <fwcpp/sim/sim_frsky.hpp>
#include <fwcpp/sim/sim_gimbals.hpp>
#include <fwcpp/sim/sim_inertiallabs.hpp>
#include <fwcpp/sim/sim_microstrain.hpp>
#include <fwcpp/sim/sim_power_serial.hpp>
#include <fwcpp/sim/sim_proximity.hpp>
#include <fwcpp/sim/sim_sensation.hpp>
#include <fwcpp/sim/sim_serial_device.hpp>
#include <fwcpp/sim/sim_vectornav.hpp>
#include <fwcpp/sim/sim_volz.hpp>

namespace fwcpp::sim {
}  // namespace fwcpp::sim
