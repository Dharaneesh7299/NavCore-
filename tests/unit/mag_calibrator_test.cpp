#include "navcore/align/mag_calibrator.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr float kDeg2Rad = kPi / 180.0f;

float AngleDiffDeg(float a_deg, float b_deg) {
  float d = std::fmod(a_deg - b_deg + 180.0f, 360.0f);
  if (d < 0) d += 360.0f;
  return d - 180.0f;
}

}  // namespace

// Before any sample: honest "nothing to report," not a fabricated 0 heading.
TEST(MagCalibratorTest, UncalibratedBeforeAnySample) {
  navcore::MagCalibrator cal;
  const auto s = cal.state(0.0f, 0.0f);
  EXPECT_FALSE(s.calibrated);
  EXPECT_TRUE(std::isnan(s.heading_deg));
}

// A level device rotated through a full circle, with a fixed injected hard-iron bias on
// every axis, should converge to `calibrated == true` and, once held steady, recover
// the true heading.
TEST(MagCalibratorTest, PassiveCalibrationConvergesAndRecoversHeading) {
  constexpr float kFieldUt = 45.0f;   // level-device horizontal field magnitude
  constexpr float kBiasX = 12.0f;     // injected hard-iron offset, uT
  constexpr float kBiasY = -8.0f;
  constexpr float kBiasZ = 5.0f;
  constexpr float kVerticalUt = -20.0f;  // arbitrary fixed vertical component (not exercised by tilt here)

  navcore::MagCalibrator cal;
  int64_t t_ns = 0;

  auto feed = [&](float heading_deg) {
    const float heading_rad = heading_deg * kDeg2Rad;
    // Body-frame field for a level phone facing `heading_rad`: forward (Y) component
    // is cos(heading), right (X) component is sin(heading) — the exact inverse of this
    // class's own atan2(-xh, yh) convention (level device: xh=mx, yh=my).
    const float mx = kFieldUt * std::sin(heading_rad) + kBiasX;
    const float my = kFieldUt * std::cos(heading_rad) + kBiasY;
    const float mz = kVerticalUt + kBiasZ;
    t_ns += 10'000'000;  // 100Hz
    cal.update(mx, my, mz, t_ns);
  };

  // Sweep the full circle several times, slowly relative to the class's own ~0.75s
  // smoothing time constant (each revolution here takes 7.2s), so the smoothed reading
  // has a chance to actually track the rotation rather than permanently lag behind an
  // unrealistically fast synthetic sweep — a passive calibrator should not need a
  // single deliberate gesture, just enough ordinary, human-paced rotation.
  for (int sweep = 0; sweep < 3; ++sweep) {
    for (int deg = 0; deg < 360; deg += 1) feed(static_cast<float>(deg));
  }

  // Hold steady for several smoothing time constants before reading — this is what
  // "once held steady" means for a filtered value; asserting immediately after motion
  // stops would be checking the filter's lag, not its converged accuracy.
  for (int i = 0; i < 400; ++i) feed(90.0f);

  const auto s = cal.state(0.0f, 0.0f);
  EXPECT_TRUE(s.calibrated);
  EXPECT_NEAR(AngleDiffDeg(s.heading_deg, 90.0f), 0.0f, 1.0f);
}
