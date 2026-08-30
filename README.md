# NavCore

A portable C++17 navigation kernel for Saarthi — built as a completely separate repository from
the [mobile app](https://github.com/Dharaneesh7299/ai-dead-reckoning-system-mobile), which links
this as a git submodule to run it on Android via JNI. NavCore itself has no dependency on
Android, JNI, or the mobile app at all — everything here builds and tests standalone, on a
desktop, against synthetic data.

## What's built so far

**In-Vehicle Alignment & Calibration (`HacfAligner`)** — solves the fixed rotation between a
phone's own frame and the vehicle it's mounted in, automatically, with zero user action:

- **Pitch & roll** from a low-pass-filtered gravity vector — runs continuously, no driving needed.
- **Yaw** from PCA on the vehicle's own dynamic (accelerating/braking) motion, since gravity alone
  can never reveal yaw. Needs a few real accelerate/brake events to converge; stays honestly
  unconverged during steady-speed cruising or while stationary.
- A narrow GNSS-bearing hook (`addGnssBearing()`) for sign disambiguation only — never a fusion
  input.

**Not yet built**: Kalman filtering, GNSS fusion, ZUPT/NHC, a mount classifier, or any
model/inference code — see `include/navcore/align/hacf_aligner.hpp` and
`src/align/hacf_aligner.cpp`'s own doc comments for the exact scope boundary and design notes.

## 1. Clone with submodules

Eigen (header-only, used for the PCA eigendecomposition) is a nested git submodule.

Cloning fresh:

```
git clone --recurse-submodules https://github.com/Dharaneesh7299/NavCore-.git
```

Already cloned without that flag:

```
git submodule update --init --recursive
```

## 2. Prerequisites

- CMake 3.16+
- A C++17 compiler
- An internet connection the first time you configure (GoogleTest is pulled in fresh via
  `FetchContent`, not vendored)

## 3. Build

```
cmake -B build
cmake --build build
```

Builds the `navcore` static library, the `calib_cli` tool, and (default `ON`, toggle with
`-DNAVCORE_BUILD_TESTS=OFF`) the unit test suite.

## 4. Run the tests

```
./build/navcore_unit_tests
```

Three tests, all against synthetic IMU data with a known ground-truth rotation: recovers a known
pitch/roll/yaw within 2° and converges within ~15-20s of simulated normal driving; stays honestly
unconverged through pure steady-speed cruising with no accel/brake events; detects a simulated
shock + sustained yaw change and re-converges on the new value.

## 5. Try the CLI

The manual "does this actually look right" check — feeds a CSV of IMU samples through
`HacfAligner` and prints its state once per simulated second. Two fixture files are included:

```
./build/calib_cli tools/fixtures/normal_drive.csv    # yaw_sigma should fall from ~14° to ~1-2°,
                                                       # converged flips true within ~15-20s
./build/calib_cli tools/fixtures/steady_cruise.csv   # stays unconverged the whole way through —
                                                       # correct, expected behaviour, not a bug
```

CSV format is one line per `ImuSample`, no header: `t_ns,ax,ay,az,gx,gy,gz` (nanosecond
timestamp; accel in m/s², raw/uncalibrated; gyro in rad/s, raw/uncalibrated). Point it at a real
recorded session in this same format once one exists — nothing about the tool is
synthetic-data-specific.

## Using this from the mobile app

The mobile app's `modules/saarthi-navcore/android/src/main/cpp/navcore_capi_real.cpp` wraps
`HacfAligner` behind NavCore's frozen C ABI (`capi.h`, defined on the mobile app's side) and
compiles this repo's `include/` + `src/align/hacf_aligner.cpp` directly into the Android native
build via the submodule — see that repo's own README/CLAUDE.md for the full integration and how
to rebuild the app after pulling changes here.
