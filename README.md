# ardumaster-cpp

ArduPilot vehicle firmware ported to modern C++ (Plane, Copter, and QuadPlane/VTOL),
pinned to `Plane-4.7.0` / matching Copter tag — same baseline as
[`ardumaster-rust`](../ardumaster-rust). See the `fw-cpp`, `copter-cpp`, and `vtol-cpp`
efforts in the tracker (`../../tracker/efforts/`) for charters, and `ADR-0012` for
conventions.

This is a second, independent implementation, not a fork of `ardumaster-rust` and not a
translation of it. It reads only from the shared, read-only `upstream/plane-4.7.0`
worktree and does not depend on anything in `ports/ardumaster-rust`.

## Building

```
cmake -B build -G Ninja
cmake --build build
ctest --test-dir build
```

## Layout

```
modules/    one directory per ported module, mirrors ardumaster-rust's crates/
sitl/       Plane / Copter / QuadPlane SITL entrypoints
tests/      parity and unit tests, one target per module under test
```

## Status

Check `python3 ../../tracker/tracker.py status --effort fw-cpp -v` from the tracker
directory, or see `../../tracker/STATUS.md`.

## WOPR model reader regression

The bridge consumes landing `flare_alt_m`, `flare_sec`, `land_sink` and `land_src`
in addition to ground steering, propeller lapse and takeoff ground pitch. It also
reads `land_time_const`, `land_damp`, `land_spd_weight`,
`landing_rollout_pitch_rate`, `landing_reverse_throttle_pct` and
`ground_brake_pitch_free`. To verify overrides
through the real INIT loader, re-INIT defaults, final-flare state/datum handling,
and the production ground-dynamics pitch gates:

```
cmake -B build -G Ninja -DFWCPP_BUILD_TESTS=OFF -DFWCPP_BRIDGE_MODEL_TEST=ON
cmake --build build --target wopr_bridge wopr_bridge_model_test
ctest --test-dir build -R '^wopr_bridge_model$' --output-on-failure
```

This bounded test needs no Catch2 installation. It establishes reader behavior,
not aircraft calibration or flight certification.

## WOPR final flare and tricycle rollout

Landing time constant/damping and speed-weight fade apply in FINAL only. Approach
retains its previous control law. This is a scoped subset of
[Plane-4.7.0 TECS](https://github.com/ArduPilot/ardupilot/blob/Plane-4.7.0/libraries/AP_TECS/AP_TECS.cpp),
not full automatic-landing parity. Flare blending uses height above the landing
target; changing home altitude does not change that relative datum. Leaving the
mode or starting another landing resets the landing state.

`landing_rollout_pitch_rate > 0` enables a tricycle rollout stage after valid
weight-on-wheels. It slews the pitch command toward level through the normal
elevator controller. `ground_brake_pitch_free > 0` retains pitch momentum in
forward-only ground dynamics while braking, so that command can lower the nose.
Both default off. Rollout latches after contact; it is not a live contact sensor.

`landing_reverse_throttle_pct` defaults to0. A positive value enables negative
propeller thrust only while armed, in AUTO landing rollout, with current valid
wheel contact. It ramps at20percentage-points/s, fades from10to5m/s ground speed,
and cuts at5m/s or immediately when contact is lost. This is explicit model
calibration, not a real-aircraft stopping-distance guarantee. The signed command
uses the existing servo-cache/harness/plant force path. VFR_HUD's unsigned throttle
is clamped to0..100; reverse is observable separately.

For models using bounded sampled-surface support, upward velocity projection
residues of at most10micrometres/s no longer break support on small terrain drops.
The already-grounded, forward-only and configured drop-limit gates remain; actual
upward motion and large drops retain free flight.

The GCS stream includes MAVLink `NAMED_VALUE_FLOAT` at10Hz: LAND_ACT, FLARE,
ROLLOUT, NAV_PITCH (deg), LAND_HGT (m above landing target), SINK_EST/SINK_DEM
(m/s positive down), PITCH_DEM (TECS deg before rollout override), TECS_TC (s).
LAND_ACT gives validity context. Controller diagnostics are computed before the
plant step, while ATTITUDE/GLOBAL_POSITION_INT follow it: allow a20ms phase offset
despite matching time_boot_ms. Additional fields are AHRS_PITCH (estimated deg),
ELEVATOR (normalized command), WHEEL_BRAK (brake fraction), BRAKE_ACC (last plant
step's wheel deceleration,m/s²), REV_THR (signed reverse command,percent,otherwise0),
and ON_GROUND (post-step plant contact). Pre-step commands may disagree with the
post-step contact bit during the transition tick. Assess actual pitch, vertical
velocity and ground contact as well as demands when judging a flare.
