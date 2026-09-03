#pragma once

// Port of libraries/SITL/SIM_Mount.h.
#include <cstdint>
#include <fwcpp/sim/sim_aircraft.hpp>
#include <fwcpp/sim/sim_gimbal.hpp>

namespace fwcpp::sim {

class SimMount {
public:
    virtual ~SimMount() = default;
    virtual void update(const Aircraft& aircraft) = 0;
    virtual void set_instance(std::uint8_t instance) { instance_ = instance; }
    [[nodiscard]] std::uint8_t instance() const { return instance_; }
protected:
    std::uint8_t instance_{0};
};

class SimMountGimbal : public SimMount {
public:
    SimGimbal gimbal;
    void update(const Aircraft& aircraft) override { gimbal.update(aircraft, now_us_); }
    void set_now_us(std::uint32_t now_us) { now_us_ = now_us; }
private:
    std::uint32_t now_us_{0};
};

}  // namespace fwcpp::sim
