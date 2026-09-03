#pragma once

#include <fwcpp/q_modes/mode_qstabilize.hpp>

namespace fwcpp::q_loiter {

struct QLoiterUpdateResult {
    bool delegate_qstabilize_update{true};
    fwcpp::q_modes::QStabilizeUpdateResult qstabilize{};
};

/// Port of ModeQLoiter::update — `plane.mode_qstabilize.update()`.
[[nodiscard]] inline QLoiterUpdateResult qloiter_update(
    const fwcpp::q_modes::QStabilizeUpdateInputs& in = {}) {
    QLoiterUpdateResult out{};
    out.delegate_qstabilize_update = true;
    out.qstabilize = fwcpp::q_modes::qstabilize_update(in);
    return out;
}

/// Port of ModeQLand::update — also `plane.mode_qstabilize.update()`.
[[nodiscard]] inline QLoiterUpdateResult qland_update(
    const fwcpp::q_modes::QStabilizeUpdateInputs& in = {}) {
    return qloiter_update(in);
}

}  // namespace fwcpp::q_loiter
