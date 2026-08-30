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
  float temp_c;      // NaN if unavailable
};

// One GNSS fix. Also a frozen contract — see ImuSample's note above.
struct GnssObs {
  int64_t t_ns;
  double lat, lon;
  float alt_m;
  float h_acc_m, v_acc_m;
  float speed_mps, bearing_deg;
  bool has_bearing;
  int sat_count;
  float hdop;
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
