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

## WOPR mission replacement during AUTO

Both the private host mission upload and the MAVLink mission upload replace the
stored mission and reset its index. When the fixed-wing controller is already
in AUTO, a successful replacement also activates the new first NAV command
from the current position. Rejected uploads preserve the active mission.
Uploading in MANUAL or another mode does not start a flight. Ordinary repeated
AUTO requests remain idempotent, and replacement does not reinitialize home.
An accepted upload is not proof of altitude/track convergence; observe telemetry.

Regression: `wopr_bridge_mission_test` covers both upload paths, rejected-plan
preservation, MANUAL upload, home preservation and repeated-AUTO idempotence.
An actual WOPR C130 MAVLink comparison at 4000m HAE replaced its AUTO mission
with a +100m plan: before the fix it still held 3996.30m after 80s against a
4096.28m target; after the fix the final sampled-window mean was 4096.55m.
Both uploads/AUTO commands ACKed; the fixed run also verified mission readback.
This checks command activation, not certified performance or every flight mode.

## WOPR model reader regression

The bridge consumes landing `flare_alt_m`, `flare_sec`, `land_sink` and `land_src`
in addition to ground steering, propeller lapse and takeoff ground pitch. It also
reads `land_time_const`, `land_damp`, `land_spd_weight`,
`landing_rollout_pitch_rate`, `landing_reverse_throttle_pct` and
`ground_brake_pitch_free`. Optional `pitch_rate_p`, `pitch_rate_i`,
`pitch_rate_d`, and `pitch_rate_ff` configure the active inner pitch-rate PID.
Omitted values retain upstream defaults; nonnumeric, negative and nonfinite
values reject the model. Re-INIT resets overrides. Startup output reports
consumed gains, and MAVLink `HAGL` reports unrounded plant ground clearance.

Optional `kff_rudder_mix` configures the aileron-to-rudder feedforward mix.
The bridge requires a numeric, finite value in `[0,1]`, validated before float
conversion. Omission restores the existing `0.5` default on each INIT, including
an INIT without a model. Invalid values reject INIT and leave STEP disabled
until a successful INIT. Successful INIT output reports the consumed mix and
model path. This enables model-specific calibration; it does not change the
default or select a tune for any aircraft.

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

The GCS stream includes MAVLink `NAMED_VALUE_FLOAT` at approximately10Hz wall time while stepping: LAND_ACT, FLARE,
ROLLOUT, NAV_PITCH (deg), LAND_HGT (m above landing target), SINK_EST/SINK_DEM
(m/s positive down), PITCH_DEM (TECS deg before rollout override), TECS_TC (s).
HDM_IN/HDM_LTD/HDM_ACT expose local-frame height references in metres:
conditioned TECS input before averaging/slew, climb/sink slew-limited demand,
and effective height used by the potential-energy controller, respectively.
HDM_IN is not the raw waypoint target (TGT_UP); saturation can hold its input.
These are demands, not measured HAE/MSL altitude. In modes that do not update
TECS (such as FBWA), values can remain held while message timestamps advance;
the timestamp records observation time, not controller-calculation freshness.
LAND_ACT gives landing validity context. Controller diagnostics are computed before the
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

### WOPR MAVLink mission metadata round-trip

Accepted `MISSION_ITEM_INT` rows retain `param3` and `param4` in the GCS
upload/download cache, including a NaN yaw sentinel. This preserves the
uploaded plan for review and subsequent downloads; it does not implement
pass-by or yaw flight behavior in the current Plane mission vocabulary.
Rejected replacements leave the accepted download cache intact.

Wire regression (Python with `pymavlink`, an existing model JSON):

```sh
python sitl/wopr_bridge_mission_roundtrip_test.py --exe build/sitl/wopr_bridge.exe --model /path/to/c130.json
```

The regression starts and cleans up its own bridge process on ports9317/10317
(overridable with `--port` and `--mavport`). It covers positive/negative optional
values, NaN yaw and download preservation after a rejected replacement.

For CTest/CI, configure with -DFWCPP_BRIDGE_MAVLINK_TEST=ON and run
ctest --test-dir build -R wopr_bridge_mission_roundtrip --output-on-failure.
The option requires pymavlink in the selected Python3_EXECUTABLE and fails
configuration if that dependency is missing; default builds do not require Python.

### Active mission upload target isolation

`MISSION_ITEM_INT` rows addressed to another system are ignored before they
can change an active upload's sequence or plan. Rows addressed to this bridge
or broadcast system0 remain accepted, consistent with `MISSION_COUNT`.
The optional MAVLink wire CTest also injects a wrong-system NAV row during an
active upload, checks that no request/ACK advances, then completes the original
plan using broadcast rows and verifies its download.

### Empty mission upload

A flight-plan `MISSION_COUNT` of zero clears both the executable plan and the
GCS download cache before acknowledging success, as required by the
[MAVLink mission upload protocol](https://mavlink.io/en/services/mission.html#upload-a-plan-to-the-vehicle).
If the actual controller is in AUTO, the bridge enters RTL using its existing
mission-complete policy; RTL is a bridge policy, not a protocol requirement.
Other modes retain control, including active Qhover/Qland. Clearing does not
arm the vehicle. Only queued mission landing metadata is reset.

Fence and rally `MISSION_COUNT` uploads are unsupported and return an error
with the requested mission type, preserving the independent flight plan and
any partial flight upload. `MISSION_CLEAR_ALL` is not added by this change.
The mission native/wire CTests cover empty storage, control-mode ownership,
repeated clear, re-upload and cross-plan rejection.

### Model navigation radii and applied diagnostics

Optional `loiter_radius` (WP_LOITER_RAD) and `rtl_radius` (RTL_RADIUS) are
signed metres, finite and within[-65535,65535]. Negative selects counterclockwise.
Omitted keys retain60/0 and re-INIT resets omitted overrides. Existing controller
integer-metre consumption/fallback is preserved: RTL absolute values truncate to
uint16;0/1 fall back to general loiter radius; general absolute values<=1 use60m.
These are control demands, not bank-feasible radius guarantees. No stock model
radius is changed by this feature. Test feasible values before operational use.

At1Hz, timestamped MAVLink NAMED_VALUE_FLOAT diagnostics report:

| Name | Meaning |
|---|---|
| LOIT_RAD / RTL_RAD | Configured signed radii, metres |
| RAD_BASE | Controller last-consumed radius, metres; zero outside RTL/LOITER |
| RAD_EFF | That radius scaled by current L1 EAS/TAS inputs; zero before consumption |
| EAS2TAS | Current plant conversion factor |
| LOIT_DIR | Controller direction(-1/+1), zero outside RTL/LOITER |
| NAV_ROLL | Controller roll demand, degrees |
| HOME_UP / TGT_UP | Autopilot home/target altitude above the bridge start datum, metres |
| WP_DIST | Controller distance to next waypoint/loiter centre, metres |

RAD_EFF is not measured orbit radius or evidence of capture. HOME_UP/TGT_UP are
local values, not HAE/MSL. Drain telemetry promptly and check timestamps before
using diagnostics to assess response. Native and wire regressions cover reader
consumers/default reset, invalid input, signed boundary/fractional fallback,
direction, and the diagnostic fields. Aircraft tuning requires separate flight
validation; these settings do not automatically compensate for an infeasible orbit.

### Explicit origin home ownership

AUTO preserves a home supplied through `Plane::set_home`, including an all-zero
local origin when the aircraft enters AUTO elsewhere or airborne. Setter-call
provenance is separate from coordinate values. If the setter was never called
and public home remains zero, AUTO retains its existing current-location fallback;
legacy direct nonzero home assignments are also preserved. `arm()` still sets
home intentionally. A newly constructed Plane resets setter provenance; this is
not a GPS-validity/home-lock subsystem.

The WOPR bridge explicitly configures home at its start-frame origin. Entering
AUTO therefore keeps HOME_UP0 and RTL targets start-relative100m under the default
RTL altitude, rather than silently moving home to the airborne entry point.
This does not change HAE/MSL or mission-frame conversion, arming, tuning, or radius.
Native regression and MAVLink HOME_UP/TGT_UP readback cover this distinction.
