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

### Replacing a mission during AUTO

Both the private host mission upload and the MAVLink mission upload replace the
stored mission and reset its index. When the fixed-wing controller is already
in AUTO, a successful replacement also activates the new first NAV command
from the current position. Rejected uploads preserve the active mission.
Uploading in MANUAL or another mode does not start a flight. Ordinary repeated
AUTO requests remain idempotent, and replacement does not reinitialize home.
An accepted upload is not proof of altitude/track convergence; observe telemetry.

The bridge consumes landing `flare_alt_m`, `flare_sec`, `land_sink` and `land_src`
in addition to ground steering, propeller lapse and takeoff ground pitch. It also
reads `land_time_const`, `land_damp`, `land_spd_weight`,
`landing_rollout_pitch_rate`, `landing_reverse_throttle_pct` and
`ground_brake_pitch_free`. Optional `pitch_rate_p`, `pitch_rate_i`,
`pitch_rate_d`, and `pitch_rate_ff` configure the active inner pitch-rate PID.
Omitted values retain upstream defaults; nonnumeric, negative and nonfinite
values reject the model. Re-INIT resets overrides. Startup output reports
consumed gains, and MAVLink `HAGL` reports unrounded plant ground clearance.
To verify overrides
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

## WOPR manual taxi extension

Models may opt into a simulation-only nosewheel constraint with
`ground_steering_wheelbase_m > 0` and `ground_steering_max_deg > 0` (loader bounds:
0..30 m and 0..65 degrees). Both default to zero. This is a bicycle approximation
at the aircraft origin, not validated C130 wheel geometry or full tyre dynamics.

In MANUAL, rudder PWM 1100..1900 commands left..right steering, with 1500 centered.
The steering angle fades between 8 and 15 m/s; yaw is limited to 0.3 g lateral
acceleration. It requires forward-only rolling ground contact, produces no yaw
at rest, and leaves aerodynamic yaw unchanged at 15 m/s and above or in flight.
AUTO takeoff and landing do not use this extension.
The applied ground yaw rate is retained separately for the next AHRS sensor
sample, because forward-only contact clears the physical integration gyro.
This sensor rate resets on the next update and is not added again after liftoff.
The binary STATE reply's `gyro_rps` still exposes the plant integration gyro,
which the ground clamp clears; it can read zero during a constrained taxi turn.
AHRS consumes `gyro_for_sensors()` instead. Assess taxi rotation using actual
heading over time, not that integration-state diagnostic alone.

On a model with positive steering wheelbase, MANUAL throttle PWM 800..1100 is
also a brake axis: 1100 releases, 950 is half, and 800 is full braking. Propulsion
remains at idle below 1100. Brakes require `ground_brake_mu > 0`; physical tyre
force is applied only on contact and cannot reverse the aircraft. Braking works
while disarmed; propulsion still requires arming and safety-off. Switching out
of MANUAL clears this brake command; the existing AUTO final-landing brake path
is unchanged. `WHEEL_BRAK` is the command and `BRAKE_ACC` is the applied force.
The bridge retains the raw brake PWM for tyre force and sends idle (1100) to
Plane's RC throttle channel in this range, so holding the brakes does not trip
the low-throttle RC failsafe. Values below 800 remain failsafe inputs; other
modes/models and the missing-RC-frame timeout retain their existing behavior.
Taxi control follows the controller's actual mode, including failsafe changes.

With `FWCPP_BRIDGE_MODEL_TEST=ON`, build `wopr_bridge_taxi_test` and run
`ctest --test-dir build -R '^wopr_bridge_(model|mission|arming|taxi)$' --output-on-failure`.
The taxi test exercises real bridge RC/mode handling, disabled models, airborne
force gating, idle thrust, eight-second brake holds armed/disarmed, low/absent
RC failsafe preservation, high-speed yaw parity, and an integrated 180-degree
turn followed by a stop. It does not prove live terrain contact or obstacle
clearance; validate wheel tracks and the full aircraft swept footprint separately.
