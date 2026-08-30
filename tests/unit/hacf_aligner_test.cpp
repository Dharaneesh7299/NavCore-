#include "navcore/align/hacf_aligner.hpp"

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <optional>
#include <random>
#include <vector>

namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr float kDeg2Rad = kPi / 180.0f;
constexpr float kGravityMps2 = 9.81f;
constexpr double kSampleRateHz = 100.0;

// Smallest signed difference between two angles in degrees, in [-180, 180] — used so
// assertions near the wraparound boundary don't get a false failure.
float AngleDiffDeg(float a_deg, float b_deg) {
  float d = std::fmod(a_deg - b_deg + 180.0f, 360.0f);
  if (d < 0) d += 360.0f;
  return d - 180.0f;
}

// One leg of a synthetic drive: a constant forward acceleration (vehicle frame,
// m/s^2 — 0 for cruising/stationary, positive for accelerating, negative for braking)
// held for a duration.
struct DrivePhase {
  double duration_s;
  float forward_accel_mps2;
};

// Builds a synthetic phone-frame IMU stream for a KNOWN, fixed (pitch, roll, yaw)
// misalignment and a described driving profile, starting at t_ns_start. Mirrors
// HacfAligner's own rotation model exactly (v_phone = Rx(roll)*Ry(pitch)*Rz(yaw) *
// v_vehicle, vehicle frame = forward/left/up) so recovering the same (pitch,roll,yaw)
// back out is a meaningful, self-consistent check — see hacf_aligner.cpp's design
// comments for the derivation.
//
// Small zero-mean Gaussian noise is added to accel/gyro (a fixed-seed RNG threaded
// through by the caller, so tests stay deterministic) — without it every PCA solve on
// noise-free data would agree exactly and convergence would look instant regardless of
// how much driving time had actually elapsed, which wouldn't meaningfully exercise the
// ~15-20s convergence behaviour the task brief describes.
std::vector<navcore::ImuSample> GenerateDrive(float pitch_deg, float roll_deg, float yaw_deg,
                                               const std::vector<DrivePhase>& phases,
                                               int64_t t_ns_start, std::mt19937& rng) {
  const Eigen::AngleAxisf roll_r(roll_deg * kDeg2Rad, Eigen::Vector3f::UnitX());
  const Eigen::AngleAxisf pitch_r(pitch_deg * kDeg2Rad, Eigen::Vector3f::UnitY());
  const Eigen::AngleAxisf yaw_r(yaw_deg * kDeg2Rad, Eigen::Vector3f::UnitZ());
  const Eigen::Matrix3f rot = (roll_r * pitch_r * yaw_r).toRotationMatrix();

  std::normal_distribution<float> accel_noise(0.0f, 0.05f);
  std::normal_distribution<float> gyro_noise(0.0f, 0.01f);

  std::vector<navcore::ImuSample> out;
  const double dt_s = 1.0 / kSampleRateHz;
  int64_t t_ns = t_ns_start;

  for (const auto& phase : phases) {
    const int n = static_cast<int>(phase.duration_s * kSampleRateHz);
    // Ramp up over the first ~0.4s and back down over the last ~0.4s (a smoothstep,
    // not a hard step) rather than an instant jump to a constant value — this matters
    // for more than realism: PCA finds the direction of maximum VARIANCE, and a
    // constant-plus-noise signal has no real variance *along* the true axis at all, just
    // whatever shape the noise itself happens to have — the direction from the origin to
    // an (almost) fixed point tells PCA nothing. A real pedal press/release traces out an
    // actual line from near-zero up to peak and back, which is what gives PCA a genuine
    // signal to find the axis from — exactly what pressing and releasing a real
    // accelerator or brake pedal does, and exactly what a constant-value phase does not.
    const int ramp_n = std::min(n / 2, static_cast<int>(0.4 * kSampleRateHz));
    for (int i = 0; i < n; ++i) {
      float envelope = 1.0f;
      if (i < ramp_n) {
        envelope = static_cast<float>(i) / static_cast<float>(ramp_n);
      } else if (i >= n - ramp_n) {
        envelope = static_cast<float>(n - 1 - i) / static_cast<float>(ramp_n);
      }
      const Eigen::Vector3f v_vehicle{phase.forward_accel_mps2 * envelope, 0.0f, kGravityMps2};
      const Eigen::Vector3f v_phone = rot * v_vehicle;

      navcore::ImuSample s{};
      s.t_ns = t_ns;
      s.ax = v_phone.x() + accel_noise(rng);
      s.ay = v_phone.y() + accel_noise(rng);
      s.az = v_phone.z() + accel_noise(rng);
      s.gx = gyro_noise(rng);
      s.gy = gyro_noise(rng);
      s.gz = gyro_noise(rng);
      s.temp_c = 25.0f;
      out.push_back(s);

      t_ns += static_cast<int64_t>(dt_s * 1e9);
    }
  }
  return out;
}

// A brief burst of large gyro readings simulating the phone physically getting bumped —
// enough to cross HacfAligner's shock-detection threshold for a few samples.
std::vector<navcore::ImuSample> GenerateShockBurst(int64_t t_ns_start) {
  std::vector<navcore::ImuSample> out;
  const double dt_s = 1.0 / kSampleRateHz;
  int64_t t_ns = t_ns_start;
  for (int i = 0; i < 5; ++i) {
    navcore::ImuSample s{};
    s.t_ns = t_ns;
    s.ax = 0.0f;
    s.ay = 0.0f;
    s.az = kGravityMps2;
    s.gx = 0.0f;
    s.gy = 0.0f;
    s.gz = 6.0f;  // rad/s — well above the shock threshold
    s.temp_c = 25.0f;
    out.push_back(s);
    t_ns += static_cast<int64_t>(dt_s * 1e9);
  }
  return out;
}

// Feeds a sample stream through the aligner, calling addGnssBearing roughly once per
// simulated second (a constant bearing — this synthetic setup never actually turns, so
// a fixed value is self-consistent; see HacfAligner's GNSS-bearing-bridge doc comment
// for why the bearing *value* barely matters here — it's the presence of a call that
// resolves sign_resolved for `converged`, with the real sign-fixing done by the
// quiet-start heuristic). Returns the elapsed simulated time (seconds from t0) at which
// `converged` first became true, or nullopt if it never did.
std::optional<double> FeedAndTrackConvergence(navcore::HacfAligner& aligner,
                                               const std::vector<navcore::ImuSample>& samples,
                                               int64_t t0_ns, float bearing_deg = 90.0f) {
  std::optional<double> converged_at_s;
  int64_t last_bearing_t_ns = t0_ns;
  for (const auto& s : samples) {
    aligner.addImuSample(s);
    if (s.t_ns - last_bearing_t_ns >= 1'000'000'000LL) {
      aligner.addGnssBearing(bearing_deg, s.t_ns);
      last_bearing_t_ns = s.t_ns;
    }
    if (!converged_at_s.has_value() && aligner.state().converged) {
      converged_at_s = static_cast<double>(s.t_ns - t0_ns) * 1e-9;
    }
  }
  return converged_at_s;
}

}  // namespace

TEST(HacfAligner, RecoversKnownRotationAndConvergesWithinExpectedTime) {
  constexpr float kTruePitch = 12.0f;
  constexpr float kTrueRoll = 4.0f;
  constexpr float kTrueYaw = 30.0f;
  constexpr int64_t kT0 = 0;

  // Stationary, then a normal mix of accelerate/cruise/brake events — starts from rest
  // so the quiet-start sign heuristic gets a clean first shot, same as a real drive
  // pulling away from a stop. Totals 20s.
  const std::vector<DrivePhase> phases = {
      {2.0, 0.0f},   {3.0, 2.5f}, {2.0, 0.0f}, {2.0, -3.0f},
      {3.0, 0.0f},   {3.0, 2.0f}, {2.0, -2.5f}, {3.0, 0.0f},
  };

  std::mt19937 rng(42);
  const auto samples = GenerateDrive(kTruePitch, kTrueRoll, kTrueYaw, phases, kT0, rng);

  navcore::HacfAligner aligner;
  const auto converged_at_s = FeedAndTrackConvergence(aligner, samples, kT0);

  const navcore::AlignState final_state = aligner.state();

  EXPECT_LT(std::fabs(AngleDiffDeg(final_state.pitch_deg, kTruePitch)), 2.0f)
      << "pitch=" << final_state.pitch_deg;
  EXPECT_LT(std::fabs(AngleDiffDeg(final_state.roll_deg, kTrueRoll)), 2.0f)
      << "roll=" << final_state.roll_deg;
  EXPECT_LT(std::fabs(AngleDiffDeg(final_state.yaw_deg, kTrueYaw)), 2.0f)
      << "yaw=" << final_state.yaw_deg;

  ASSERT_TRUE(converged_at_s.has_value()) << "never converged within the 20s drive";
  EXPECT_LE(*converged_at_s, 20.5) << "converged at " << *converged_at_s << "s, expected <=~20s";
  EXPECT_TRUE(final_state.converged);
}

TEST(HacfAligner, SteadyCruiseNeverConverges) {
  // Pure constant-speed cruising: zero net forward acceleration the whole time, so no
  // dynamic window is ever detected. This is the expected, correct behaviour, not a
  // bug — HacfAligner has no basis to solve yaw without at least one accel/brake event.
  constexpr float kTruePitch = 5.0f;
  constexpr float kTrueRoll = 2.0f;
  constexpr float kTrueYaw = 15.0f;
  constexpr int64_t kT0 = 0;

  const std::vector<DrivePhase> phases = {{20.0, 0.0f}};

  std::mt19937 rng(7);
  const auto samples = GenerateDrive(kTruePitch, kTrueRoll, kTrueYaw, phases, kT0, rng);

  navcore::HacfAligner aligner;
  for (const auto& s : samples) {
    aligner.addImuSample(s);
  }

  const navcore::AlignState final_state = aligner.state();
  EXPECT_FALSE(final_state.converged);
  EXPECT_LT(final_state.progress, 0.3f) << "progress=" << final_state.progress;

  // Stage 1 has no dependency on dynamic events at all — pitch/roll should still be
  // correctly recovered from gravity alone even though yaw never converges.
  EXPECT_LT(std::fabs(AngleDiffDeg(final_state.pitch_deg, kTruePitch)), 2.0f);
  EXPECT_LT(std::fabs(AngleDiffDeg(final_state.roll_deg, kTrueRoll)), 2.0f);
}

TEST(HacfAligner, ReconvergesAfterShockAndSustainedYawChange) {
  constexpr float kPitch = 8.0f;
  constexpr float kRoll = 3.0f;
  constexpr float kYawBefore = 20.0f;
  constexpr float kYawAfter = kYawBefore + 25.0f;  // sustained 25deg change
  constexpr int64_t kT0 = 0;

  const std::vector<DrivePhase> phases_before = {
      {2.0, 0.0f}, {3.0, 2.5f}, {2.0, 0.0f}, {2.0, -3.0f}, {3.0, 0.0f}, {3.0, 2.0f},
  };

  std::mt19937 rng(99);
  auto samples = GenerateDrive(kPitch, kRoll, kYawBefore, phases_before, kT0, rng);

  navcore::HacfAligner aligner;
  const auto converged_before_s = FeedAndTrackConvergence(aligner, samples, kT0);
  ASSERT_TRUE(converged_before_s.has_value()) << "never converged on the pre-shock segment";
  EXPECT_LT(std::fabs(AngleDiffDeg(aligner.state().yaw_deg, kYawBefore)), 2.0f)
      << "pre-shock yaw=" << aligner.state().yaw_deg;

  const int64_t t_after_before_segment = samples.back().t_ns + 10'000'000LL;
  const auto shock = GenerateShockBurst(t_after_before_segment);
  for (const auto& s : shock) aligner.addImuSample(s);

  // New orientation (yaw only — pitch/roll unaffected, matching a bump that changes the
  // phone's yaw against the mount, not a full re-seat). Starts from rest again so the
  // quiet-start heuristic can re-resolve the new sign cleanly.
  const std::vector<DrivePhase> phases_after = {
      {1.0, 0.0f}, {3.0, 2.5f}, {2.0, 0.0f}, {2.0, -3.0f}, {3.0, 0.0f},
      {3.0, 2.0f}, {2.0, -2.5f}, {3.0, 0.0f},
  };
  const int64_t t_after_shock = shock.back().t_ns + 10'000'000LL;
  const auto samples_after = GenerateDrive(kPitch, kRoll, kYawAfter, phases_after, t_after_shock, rng);

  const auto converged_after_s = FeedAndTrackConvergence(aligner, samples_after, t_after_shock);
  ASSERT_TRUE(converged_after_s.has_value()) << "never re-converged after the shock";

  const navcore::AlignState final_state = aligner.state();
  EXPECT_TRUE(final_state.converged);
  EXPECT_LT(std::fabs(AngleDiffDeg(final_state.yaw_deg, kYawAfter)), 2.0f)
      << "post-shock yaw=" << final_state.yaw_deg << ", expected ~" << kYawAfter;
  EXPECT_LT(std::fabs(AngleDiffDeg(final_state.pitch_deg, kPitch)), 2.0f);
  EXPECT_LT(std::fabs(AngleDiffDeg(final_state.roll_deg, kRoll)), 2.0f);
}
