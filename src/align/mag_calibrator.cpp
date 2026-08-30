#include "navcore/align/mag_calibrator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace navcore {

namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr float kRad2Deg = 180.0f / kPi;

// Earth's total field magnitude varies roughly 25-65 uT by location — this is a
// reasonable global-average stand-in, not a per-location measurement (same "honest
// approximation" framing as capi.h's own hdop-from-h_acc_m comment). Used only to judge
// how much of a fully-calibrated axis's swing has actually been observed so far.
constexpr float kExpectedFieldUt = 45.0f;

// Once the observed per-axis range averages at least this fraction of a fully-swept
// axis (2x the expected field magnitude), the bias estimate is trusted.
constexpr float kCalibratedQualityThreshold = 0.6f;

// Low-pass time constant for the raw reading heading is computed from. Without this,
// heading is derived from a single raw sample and visibly jitters several degrees at
// rest — raw magnetometer noise, not real motion.
constexpr float kSmoothingTimeConstS = 0.75f;

float wrapDeg360(float deg) {
  float d = std::fmod(deg, 360.0f);
  if (d < 0.0f) d += 360.0f;
  return d;
}

}  // namespace

MagCalibrator::MagCalibrator() : min_{0, 0, 0}, max_{0, 0, 0}, smoothed_{0, 0, 0} {}

void MagCalibrator::update(float mx, float my, float mz, int64_t t_ns) {
  if (!have_sample_) {
    min_[0] = max_[0] = smoothed_[0] = mx;
    min_[1] = max_[1] = smoothed_[1] = my;
    min_[2] = max_[2] = smoothed_[2] = mz;
    have_sample_ = true;
    last_t_ns_ = t_ns;
    return;
  }

  const float dt_s = static_cast<float>(t_ns - last_t_ns_) / 1e9f;
  last_t_ns_ = t_ns;

  // Guard against a clock jump producing a nonsensical or negative dt — skip the
  // smoothing step for this sample rather than let the filter lurch.
  if (dt_s > 0.0f && dt_s < 60.0f) {
    const float smoothAlpha = dt_s / (kSmoothingTimeConstS + dt_s);
    smoothed_[0] += smoothAlpha * (mx - smoothed_[0]);
    smoothed_[1] += smoothAlpha * (my - smoothed_[1]);
    smoothed_[2] += smoothAlpha * (mz - smoothed_[2]);
  }

  // Bounds only ever widen — never decay/shrink over idle time. An earlier version
  // continuously relaxed min/max toward their own midpoint so a remounted phone could
  // eventually recalibrate, but that ran on every sample regardless of whether
  // anything had actually changed: while the phone sat still, the reading never set a
  // new extreme, so the "relax" step just ate into bounds already earned from real
  // rotation. Given enough idle time, min and max converged toward each other and
  // toward whatever the phone happened to be reading — at which point the bias-
  // corrected vector (reading - bias) shrank toward zero, and atan2 on a near-zero
  // vector produced a heading that swung wildly even though the phone hadn't moved.
  // Confirmed on-device as the actual cause of "drifting rapidly while stationary."
  // Trade-off accepted: a remounted phone no longer recalibrates on its own within a
  // session (bounds only grow) — a strictly worse-idle-behavior decay was not an
  // acceptable price for that.
  min_[0] = std::min(min_[0], mx);
  max_[0] = std::max(max_[0], mx);
  min_[1] = std::min(min_[1], my);
  max_[1] = std::max(max_[1], my);
  min_[2] = std::min(min_[2], mz);
  max_[2] = std::max(max_[2], mz);
}

MagCalibrator::State MagCalibrator::state(float pitch_rad, float roll_rad) const {
  if (!have_sample_) {
    return State{std::numeric_limits<float>::quiet_NaN(), false};
  }

  const float bias0 = 0.5f * (min_[0] + max_[0]);
  const float bias1 = 0.5f * (min_[1] + max_[1]);
  const float bias2 = 0.5f * (min_[2] + max_[2]);

  const float cx = smoothed_[0] - bias0;
  const float cy = smoothed_[1] - bias1;
  const float cz = smoothed_[2] - bias2;

  // Tilt-compensated heading — direct port of the mobile app's own
  // src/engine/live/headingFallback.ts::tiltCompensatedHeading(), same body-frame
  // convention (Y = phone's forward/top axis, X = right edge — fixed by Android's
  // sensor API) and the same atan2(-xh, yh) form (not the more commonly copy-pasted
  // atan2(-yh, xh), which assumes X is forward).
  const float cosPitch = std::cos(pitch_rad);
  const float sinPitch = std::sin(pitch_rad);
  const float cosRoll = std::cos(roll_rad);
  const float sinRoll = std::sin(roll_rad);

  const float xh = cx * cosPitch + cz * sinPitch;
  const float yh = cx * sinRoll * sinPitch + cy * cosRoll - cz * sinRoll * cosPitch;

  const float heading_deg = wrapDeg360(std::atan2(-xh, yh) * kRad2Deg);

  float quality = 0.0f;
  for (int i = 0; i < 3; ++i) {
    const float range = max_[i] - min_[i];
    quality += std::min(1.0f, range / (2.0f * kExpectedFieldUt));
  }
  quality /= 3.0f;

  return State{heading_deg, quality >= kCalibratedQualityThreshold};
}

}  // namespace navcore
