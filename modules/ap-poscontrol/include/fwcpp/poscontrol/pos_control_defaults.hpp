#pragma once

// Upstream POSCONTROL_* defaults from AC_PosControl.h (Plane-4.7.0).

namespace fwcpp::poscontrol {

inline constexpr float kPoscontrolAccelNeMss = 1.0f;
inline constexpr float kPoscontrolJerkNeMsss = 5.0f;
inline constexpr float kPoscontrolStoppingDistUpMaxM = 3.0f;
inline constexpr float kPoscontrolStoppingDistDownMaxM = 2.0f;
inline constexpr float kPoscontrolSpeedMs = 5.0f;
inline constexpr float kPoscontrolSpeedDownMs = 1.5f;
inline constexpr float kPoscontrolSpeedUpMs = 2.5f;
inline constexpr float kPoscontrolAccelDMss = 2.5f;
inline constexpr float kPoscontrolJerkDMsss = 5.0f;
inline constexpr float kPoscontrolThrottleCutoffFreqHz = 2.0f;
inline constexpr float kPoscontrolOverspeedGainU = 2.0f;
inline constexpr float kPoscontrolRelaxTc = 0.16f;

}  // namespace fwcpp::poscontrol
