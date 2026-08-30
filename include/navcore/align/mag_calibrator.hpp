#pragma once

#include <cstdint>

namespace navcore {

// Passive hard-iron magnetometer calibration + tilt-compensated compass heading.
//
// Deliberately standalone from HacfAligner: HacfAligner's own yaw is a FIXED
// phone-to-vehicle mounting offset that must not respond to hand rotation or in-place
// turning (see hacf_aligner.hpp's own doc comment) — this class answers a different
// question entirely ("which way is the phone physically facing, right now"), so its
// output is never fed into HacfAligner and HacfAligner's yaw is never fed into this.
// The only thing borrowed from HacfAligner is its already-solved pitch/roll, passed in
// by the caller at query time, purely to tilt-compensate the heading — no coupling
// beyond that one read.
//
// Hard-iron only, not soft-iron/scale: a running per-axis min/max estimates the
// constant offset added by nearby ferrous material (the phone chassis, a mount
// bracket), which is the dominant, correctable-without-a-guided-routine error for a
// phone-mounted compass. It does not correct for soft-iron distortion (direction-
// dependent scale/shear) — a known, accepted limitation of a passive scheme, not an
// oversight; a full ellipsoid fit would need a deliberate guided motion this class
// intentionally doesn't require.
//
// "Passive" means no guided figure-8 gesture: the bias estimate simply refines itself
// from whatever rotation the phone naturally goes through (mounting it, picking it up,
// ordinary handling) as time passes. The min/max bounds only ever widen — they do NOT
// decay/relax over idle time. An earlier version relaxed them continuously so a
// remounted phone could recalibrate on its own, but that eroded a good calibration
// during ordinary stationary use too (nothing stops the reading from being a "new
// extreme" while idle, so the relax step just ate into real, already-earned range),
// which surfaced on-device as the heading swinging wildly even with the phone
// perfectly still — see mag_calibrator.cpp for the full account. Accepted trade-off:
// a remounted phone no longer recalibrates within a session on its own.
class MagCalibrator {
 public:
  MagCalibrator();

  // mx, my, mz: raw/uncalibrated magnetometer reading, µT, phone frame (matches
  // ImuSample's own mx/my/mz — same units and axis convention). t_ns: same
  // elapsedRealtimeNanos timebase every other NavCore timestamp uses.
  void update(float mx, float my, float mz, int64_t t_ns);

  struct State {
    float heading_deg;  // compass bearing, 0=North/90=East, increasing clockwise —
                         // NaN until at least one sample has been seen
    bool calibrated;     // true once enough axis range has been observed to trust bias
  };

  // pitch_rad/roll_rad: the device's current tilt (radians) — this class has no tilt
  // estimate of its own, by design (see the class-level doc comment above).
  State state(float pitch_rad, float roll_rad) const;

 private:
  float min_[3];
  float max_[3];
  // Low-pass filtered raw reading (see mag_calibrator.cpp) — heading is computed from
  // this, not the single latest sample. Raw magnetometer noise alone is a few degrees
  // of sample-to-sample jitter; every other sensor-derived value in this codebase
  // (GravityFilter, AttitudeEstimator, ...) filters before deriving an angle, and this
  // class was the one that originally skipped that step.
  float smoothed_[3];
  bool have_sample_ = false;
  int64_t last_t_ns_ = 0;
};

}  // namespace navcore
