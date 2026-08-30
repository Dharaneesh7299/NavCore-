#pragma once

#include <cstdint>

namespace navcore {

// One combined accel+gyro reading, phone frame, uncalibrated/raw. This is a frozen
// contract other components (and the mobile Android adapter's capi.h) already assume —
// do not rename or reorder fields without updating every consumer.
struct ImuSample {
  int64_t t_ns;
  float ax, ay, az;  // m/s^2, raw, phone frame
  float gx, gy, gz;  // rad/s, raw, phone frame
  float mx, my, mz;  // µT, raw/uncalibrated, phone frame — NaN if unavailable, same
                      // "NaN, not a companion bool" convention as temp_c below
  float temp_c;       // NaN if unavailable
};

// One GNSS fix. Also a frozen contract — see ImuSample's note above. Field order and
// names deliberately mirror the mobile app's own capi.h GnssObs exactly (that struct is
// the actual live adapter contract; this one isn't wired to anything yet — see the scope
// note below — so it must match capi.h's shape now rather than silently drifting until
// whoever wires GNSS fusion in discovers the mismatch the hard way).
struct GnssObs {
  int64_t t_ns;
  double lat, lon;
  float alt_m;
  float h_acc_m, v_acc_m;
  float speed_mps, bearing_deg;
  int sat_count;
  float hdop;
  bool has_raw;  // true only if a GnssMeasurementsEvent was attached to this fix
};

// HacfAligner's current output: the rotation that takes a sample from the phone's frame
// into the vehicle's frame, plus how confident that estimate is.
struct AlignState {
  float pitch_deg, roll_deg, yaw_deg;
  float yaw_sigma_deg;  // uncertainty — starts high, falls as it converges
  bool converged;
  float progress;  // 0..1
};

}  // namespace navcore
