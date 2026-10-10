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
in addition to ground steering, propeller lapse and takeoff ground pitch. To
verify seven distinct overrides through the real INIT loader, then verify that
empty-model re-INIT restores defaults:

```
cmake -B build -G Ninja -DFWCPP_BUILD_TESTS=OFF -DFWCPP_BRIDGE_MODEL_TEST=ON
cmake --build build --target wopr_bridge wopr_bridge_model_test
ctest --test-dir build -R '^wopr_bridge_model$' --output-on-failure
```

This bounded test needs no Catch2 installation. It establishes reader behavior,
not aircraft calibration or flight certification.
