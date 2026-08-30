#include "navcore/align/hacf_aligner.hpp"

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <memory>

namespace navcore {

namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr float kDeg2Rad = kPi / 180.0f;
constexpr float kRad2Deg = 180.0f / kPi;
constexpr float kGravityMps2 = 9.81f;

// Stage 1 low-pass cutoff (~0.25Hz) — isolates the gravity component from short-term
// dynamic acceleration. RC = 1/(2*pi*fc); alpha = dt/(dt+RC) per-sample, so a long gap
// between samples naturally lets the filter snap to the new sample instead of needing a
// separate clamp.
constexpr float kGravityLpfCutoffHz = 0.25f;
constexpr float kGravityLpfRC = 1.0f / (2.0f * kPi * kGravityLpfCutoffHz);

// Stage 2 tuning. All somewhat arbitrary, physically-reasoned constants — see the class
// doc comment for the full design; adjust here if convergence timing needs retuning.
constexpr float kDynamicAccelThresholdMps2 = 0.8f;      // horizontal accel magnitude, leveled frame
constexpr int64_t kDynWindowSpanNs = 8'000'000'000LL;   // rolling ~8s buffer of dynamic samples
constexpr int kMinDynSamplesForSolve = 40;              // don't trust a covariance from too few points
constexpr int kDynSamplesPerSolve = 30;                 // re-solve every ~this many new dynamic samples
constexpr size_t kMaxRecentEstimates = 6;               // window-to-window history for yaw_sigma_deg
constexpr float kYawSigmaStartDeg = 14.0f;
constexpr float kYawSigmaFloorDeg = 1.0f;
constexpr float kConvergedSigmaThresholdDeg = 2.0f;
constexpr float kShockGyroThresholdRadS = 3.0f;   // sudden spike, not ordinary driving/turning
constexpr float kReAlignDisagreementDeg = 5.0f;
constexpr int kReAlignPersistWindows = 2;         // disagreement must persist, not be a single blip
constexpr float kBearingSignFlipThresholdDeg = 90.0f;

// Brake/throttle sign disambiguation (see solvePcaWindow()'s own use of these). An
// extreme has to clear this magnitude to count as a real event at all — well above the
// kDynamicAccelThresholdMps2 noise floor that just gates buffer entry, since a routine
// small fluctuation shouldn't be mistaken for a genuine accel/brake spike.
constexpr float kBrakeThrottleMinExtremeMps2 = 1.5f;
// Ordinary vehicles brake harder than they accelerate — the larger-magnitude extreme in
// a window has to exceed the other by at least this ratio before it's trusted as "that
// one's the brake event," not just ordinary variation between two roughly-equal events.
constexpr float kBrakeThrottleAsymmetryFactor = 1.3f;

float wrapDeg180(float deg) {
  float d = std::fmod(deg + 180.0f, 360.0f);
  if (d < 0) d += 360.0f;
  return d - 180.0f;
}

// Reduces an angle to [-90, 90) — i.e. mod 180deg, not mod 360deg. Used to collapse PCA's
// inherent axis (not direction) ambiguity to a single canonical representative before any
// GNSS-informed 180deg branch choice is applied on top — see solvePcaWindow()'s doc
// comment on why this specific ambiguity needs collapsing first, not resolving per-window.
float wrapHalfTurnDeg(float deg) {
  float d = std::fmod(deg + 90.0f, 180.0f);
  if (d < 0) d += 180.0f;
  return d - 90.0f;
}

float wrapDeg360(float deg) {
  float d = std::fmod(deg, 360.0f);
  if (d < 0) d += 360.0f;
  return d;
}

// Circular mean/stddev of a set of angles (degrees) — avoids the classic wraparound bug
// a naive linear mean/variance would have near the +-180 boundary.
struct CircularStats {
  float mean_deg;
  float sigma_deg;  // 0 if fewer than 2 samples
};

CircularStats circularStats(const std::deque<float>& angles_deg) {
  if (angles_deg.empty()) return {0.0f, kYawSigmaStartDeg};
  float sum_cos = 0.0f, sum_sin = 0.0f;
  for (float a : angles_deg) {
    sum_cos += std::cos(a * kDeg2Rad);
    sum_sin += std::sin(a * kDeg2Rad);
  }
  const float n = static_cast<float>(angles_deg.size());
  const float mean_cos = sum_cos / n;
  const float mean_sin = sum_sin / n;
  const float mean_deg = std::atan2(mean_sin, mean_cos) * kRad2Deg;
  if (angles_deg.size() < 2) return {mean_deg, kYawSigmaStartDeg};
  const float r = std::sqrt(mean_cos * mean_cos + mean_sin * mean_sin);
  // Standard circular-standard-deviation formula. r is a mean resultant length in (0,1];
  // clamp away from exactly 0 to avoid log(0) — that only happens with near-uniformly
  // scattered angles, i.e. wildly inconsistent estimates, which should read as "high
  // sigma" anyway, so clamping to a tiny epsilon (=> a large but finite sigma) is correct.
  const float r_clamped = std::max(r, 1e-6f);
  const float sigma_rad = std::sqrt(-2.0f * std::log(r_clamped));
  return {mean_deg, std::min(sigma_rad * kRad2Deg, kYawSigmaStartDeg)};
}

}  // namespace

// ---------------------------------------------------------------------------------
// Rotation model (referenced as "the class doc comment" by several comments below)
// ---------------------------------------------------------------------------------
// Vehicle frame: x=forward, y=left, z=up. Phone frame: x=right, y=up (top of phone in
// portrait), z=out of the screen — matching Android's own sensor axis convention, so a
// real accelerometer reading is directly comparable to what this model predicts. A
// stationary phone lying flat, screen up, reads accel=(0,0,+g): the sensor measures the
// reaction force holding it up against gravity, not gravity's own direction.
//
// The phone's fixed mounting misalignment (pitch, roll, yaw — exactly what this class
// solves for) is modeled as one rotation carrying a vehicle-frame vector into the
// phone's frame:
//
//     v_phone = Rx(roll) * Ry(pitch) * Rz(yaw) * v_vehicle
//
// with Rx/Ry/Rz the standard right-handed rotation matrices about each axis, applied
// yaw-then-pitch-then-roll (yaw innermost). Two consequences this code leans on
// directly:
//
//  1. For a level, non-accelerating vehicle, v_vehicle = (0, 0, g) (pure gravity
//     reaction, no horizontal component). Rz(yaw) leaves a purely-vertical vector
//     unchanged, so yaw has NO effect on gravity at all — gravity alone can only ever
//     recover pitch and roll, never yaw. That's the entire reason Stage 2 (PCA on
//     dynamic acceleration) exists as a separate mechanism, not an oversight in Stage 1.
//     Solving v_phone = (g sin(pitch), -g sin(roll)cos(pitch), g cos(roll)cos(pitch))
//     for the two unknowns gives Stage 1's exact recovery formulas:
//       pitch = atan2(ax, sqrt(ay^2 + az^2)),  roll = atan2(-ay, az)
//
//  2. "Leveling" a phone-frame sample means undoing roll then pitch, in that reverse
//     order: v_leveled = Ry(-pitch) * Rx(-roll) * v_phone. Given accurate pitch/roll,
//     this exactly cancels Rx(roll)*Ry(pitch), leaving v_leveled = Rz(yaw) * v_vehicle
//     — i.e. whatever yaw does to a vehicle-frame vector, with pitch/roll fully removed.
//     For a pure along-forward-axis event (accelerating or braking, v_vehicle=(a,0,0)),
//     this reduces to v_leveled_xy = (a*cos(yaw), a*sin(yaw)) — a line through the
//     origin at angle yaw, whose direction PCA's principal eigenvector recovers via
//     atan2(y,x). See solvePcaWindow()'s own comment for why that raw angle still needs
//     collapsing to a canonical half-turn before it means anything reliable.
struct HacfAligner::Impl {
  // ---- Stage 1: pitch & roll from gravity ----
  bool have_gravity = false;
  Eigen::Vector3f gravity_lpf{0.0f, 0.0f, kGravityMps2};
  int64_t last_imu_t_ns = 0;
  bool have_last_imu_t = false;

  float pitch_rad = 0.0f;
  float roll_rad = 0.0f;

  // ---- Stage 2: yaw via PCA on dynamic-window leveled horizontal accel ----
  struct DynSample {
    int64_t t_ns;
    float x, y;
  };
  std::deque<DynSample> dyn_buffer;
  int samples_since_last_solve = 0;

  std::deque<float> recent_estimates_deg;  // branch-corrected, for circular sigma/mean

  float yaw_deg = 0.0f;
  float yaw_sigma_deg = kYawSigmaStartDeg;
  bool converged = false;
  float progress = 0.0f;

  // Shock -> persistent-disagreement -> re-converge tracking.
  bool shock_pending = false;
  int disagree_count = 0;

  // GNSS-bearing sign disambiguation. solvePcaWindow() already collapses PCA's raw
  // axis-vs-direction ambiguity AND the accel-vs-brake sign difference into one
  // canonical, stable half-turn value (see its own doc comment) — what's left is a
  // single, genuine, irreducible question: is the true yaw this canonical value, or
  // exactly 180deg from it? yaw_branch_deg (0 or 180) answers that, globally, applied
  // uniformly to every solve. Without any GNSS bearing ever arriving, it just stays at
  // its default (0) — the aligner still converges on the canonical value with no
  // external confirmation, same as any single-antenna INS would report its best
  // estimate; this bridge exists to CORRECT that guess if it turns out backwards, not to
  // gate whether a confident-but-unconfirmed estimate can be reported at all.
  //
  // The bridge itself works via relative gyro-z integration between bearing samples —
  // never absolute (there's no magnetometer to anchor absolute heading, and there
  // doesn't need to be: a rigid phone-to-vehicle mount means the phone's OWN rotation
  // tracks the vehicle's rotation exactly, whatever the fixed misalignment is, so two
  // bearing samples bridged by the phone's own measured rotation should agree if the
  // current branch choice is correct — and disagree by ~180deg if it's not). This is a
  // deliberately simple heuristic, not a rigorous absolute-heading solve: bridging a
  // *first* bearing sample to a body-frame estimate with no prior anchor is genuinely
  // underdetermined by bearing+gyro alone, so the first-ever call just anchors and
  // trusts the current branch as-is; only a later bearing that disagrees with the
  // gyro-projected prediction flips it.
  //
  // LIMITATION, worth being explicit about: on a perfectly straight-line drive (no
  // turning at all), gyro_yaw_integral_deg barely changes, so every bearing sample
  // agrees with the projected prediction *regardless of whether the branch is actually
  // right* — there is nothing to disagree with. This bridge can only detect and correct
  // a wrong branch once some real heading change (an actual turn) happens somewhere
  // during the drive to calibrate against; it cannot validate the very first guess on
  // its own from bearing+gyro alone.
  //
  // solvePcaWindow() now covers the straight-line case this bridge can't: brake/throttle
  // asymmetry picks yaw_branch_deg from accel/brake event magnitudes whenever
  // has_bearing_anchor is still false, deferring to this GNSS bridge the moment it has
  // anything to say. That's a real fix for the common case (a drive with at least one
  // clearly-asymmetric accel/brake pair before GNSS ever weighs in — see
  // BrakeThrottleAsymmetryResolvesSignWithNoGnss), not a complete one: a drive with no
  // clear accel/brake asymmetry AND no turn AND no GNSS bearing before the first solve is
  // still a coin flip on PCA's raw eigensolver sign. A magnetometer (NavCore's
  // MagCalibrator exists now, but is deliberately standalone from this class — see its
  // own doc comment for why) could close that residual gap if it's ever wired in here.
  float gyro_yaw_integral_deg = 0.0f;
  bool has_bearing_anchor = false;
  float bearing_anchor_deg = 0.0f;
  float gyro_yaw_at_anchor_deg = 0.0f;
  float yaw_branch_deg = 0.0f;  // 0 or 180 — added to every raw PCA solve

  void addImuSample(const ImuSample& s) {
    // Gyro-yaw integral is bookkeeping for the GNSS-bearing sign bridge only (see the
    // has_bearing_anchor doc comment below) — never fed into pitch/roll/yaw themselves.
    // Computed from the *previous* sample's timestamp, so this must happen before
    // updateGravityAndAttitude() below overwrites last_imu_t_ns with the current one.
    if (have_last_imu_t) {
      const float dt = static_cast<float>(s.t_ns - last_imu_t_ns) * 1e-9f;
      if (dt > 0.0f && dt < 1.0f) {
        gyro_yaw_integral_deg += s.gz * kRad2Deg * dt;
      }
    }

    // Gate the gravity LPF's update on whether THIS sample looks dynamic, using the
    // attitude estimate from before this sample — a single-pole ~0.25Hz filter only
    // rejects genuinely brief dynamic bursts; a sustained multi-second acceleration
    // (ordinary accelerating-from-a-stop, not a bump) is slow enough, relative to the
    // filter's own ~0.6s time constant, to leak into "gravity" and bias pitch/roll for
    // as long as it lasts. Freezing the filter during dynamic samples — updating it only
    // "during quiet or steady-speed intervals," per the task brief's own words — keeps
    // pitch/roll accurate throughout an event instead of drifting with it.
    // Called twice deliberately, not a missed optimization: this first call uses
    // pitch/roll from BEFORE this sample to decide whether to trust it for the LPF
    // update below; the second (after, once pitch/roll may have just changed) is what
    // Stage 2 actually buffers. They coincide when the sample is dynamic (attitude
    // stays frozen either way) and differ only slightly otherwise.
    const bool dynamic_before_update = levelHorizontal(s.ax, s.ay, s.az).norm() > kDynamicAccelThresholdMps2;
    updateGravityAndAttitude(s, !dynamic_before_update);

    const Eigen::Vector2f leveled = levelHorizontal(s.ax, s.ay, s.az);
    const float horiz_mag = leveled.norm();

    const bool shock = std::sqrt(s.gx * s.gx + s.gy * s.gy + s.gz * s.gz) > kShockGyroThresholdRadS;
    if (shock && !shock_pending) {
      // Rising edge of a shock: any dynamic samples already buffered may reflect a now-
      // stale phone orientation, mixing old and new axes if left in place — drop them
      // and start the rolling buffer fresh from this point.
      dyn_buffer.clear();
      samples_since_last_solve = 0;
    }
    if (shock) shock_pending = true;

    if (horiz_mag > kDynamicAccelThresholdMps2) {
      dyn_buffer.push_back({s.t_ns, leveled.x(), leveled.y()});
      while (!dyn_buffer.empty() && (s.t_ns - dyn_buffer.front().t_ns) > kDynWindowSpanNs) {
        dyn_buffer.pop_front();
      }
      ++samples_since_last_solve;

      if (static_cast<int>(dyn_buffer.size()) >= kMinDynSamplesForSolve &&
          samples_since_last_solve >= kDynSamplesPerSolve) {
        solvePcaWindow();
        samples_since_last_solve = 0;
      }
    }
  }

  void addGnssBearing(float bearing_deg, int64_t /*t_ns*/) {
    if (!has_bearing_anchor) {
      bearing_anchor_deg = wrapDeg360(bearing_deg);
      gyro_yaw_at_anchor_deg = gyro_yaw_integral_deg;
      has_bearing_anchor = true;
      return;
    }
    const float predicted =
        wrapDeg360(bearing_anchor_deg + (gyro_yaw_integral_deg - gyro_yaw_at_anchor_deg));
    const float diff = wrapDeg180(bearing_deg - predicted);
    if (std::fabs(diff) > kBearingSignFlipThresholdDeg) {
      yaw_branch_deg = (yaw_branch_deg == 0.0f) ? 180.0f : 0.0f;
    }
    bearing_anchor_deg = wrapDeg360(bearing_deg);
    gyro_yaw_at_anchor_deg = gyro_yaw_integral_deg;
  }

  AlignState state() const {
    return AlignState{
        pitch_rad * kRad2Deg,
        roll_rad * kRad2Deg,
        yaw_deg,
        yaw_sigma_deg,
        converged,
        progress,
    };
  }

  void updateGravityAndAttitude(const ImuSample& s, bool allow_update) {
    const Eigen::Vector3f raw{s.ax, s.ay, s.az};
    if (!have_gravity) {
      // Bootstrap unconditionally even on a dynamic first sample — an initial guess,
      // however rough, beats having no attitude estimate at all to gate anything on.
      gravity_lpf = raw;
      have_gravity = true;
    } else if (allow_update) {
      float dt = 0.01f;
      if (have_last_imu_t) {
        dt = static_cast<float>(s.t_ns - last_imu_t_ns) * 1e-9f;
        if (dt <= 0.0f) dt = 0.001f;
      }
      const float alpha = dt / (dt + kGravityLpfRC);
      gravity_lpf = gravity_lpf + alpha * (raw - gravity_lpf);
    }
    // else: a dynamic sample — leave gravity_lpf frozen at its last quiet-period value.
    last_imu_t_ns = s.t_ns;
    have_last_imu_t = true;

    // Phone frame: x=right, y=up(top of phone), z=out of the screen. A stationary phone
    // lying flat, screen up, reads (0,0,+g) — the accelerometer measures the reaction
    // force holding it up against gravity, not gravity itself. See the class doc comment
    // for the full rotation-model derivation these two formulas come from.
    const float gx = gravity_lpf.x();
    const float gy = gravity_lpf.y();
    const float gz = gravity_lpf.z();
    pitch_rad = std::atan2(gx, std::sqrt(gy * gy + gz * gz));
    roll_rad = std::atan2(-gy, gz);
  }

  // Undoes roll then pitch (inverse order) from a phone-frame accel sample, leaving only
  // the yaw-dependent horizontal component: v_leveled = Ry(-pitch) * Rx(-roll) * v_phone.
  Eigen::Vector2f levelHorizontal(float ax, float ay, float az) const {
    const Eigen::Vector3f v_phone{ax, ay, az};
    const Eigen::AngleAxisf undo_roll(-roll_rad, Eigen::Vector3f::UnitX());
    const Eigen::AngleAxisf undo_pitch(-pitch_rad, Eigen::Vector3f::UnitY());
    const Eigen::Vector3f v_leveled = undo_pitch * (undo_roll * v_phone);
    return {v_leveled.x(), v_leveled.y()};
  }

  void solvePcaWindow() {
    const int n = static_cast<int>(dyn_buffer.size());
    Eigen::Vector2f mean = Eigen::Vector2f::Zero();
    for (const auto& d : dyn_buffer) mean += Eigen::Vector2f(d.x, d.y);
    mean /= static_cast<float>(n);

    Eigen::Matrix2f cov = Eigen::Matrix2f::Zero();
    for (const auto& d : dyn_buffer) {
      const Eigen::Vector2f c = Eigen::Vector2f(d.x, d.y) - mean;
      cov += c * c.transpose();
    }
    cov /= static_cast<float>(n - 1);

    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> solver(cov);
    // Eigen returns eigenvalues in ASCENDING order — the last column is the principal
    // (largest-eigenvalue) eigenvector: the estimated forward axis in the leveled frame.
    //
    // This is ambiguous mod 180deg in TWO compounding ways, not one: Eigen only fixes
    // the eigenvector up to sign (an axis, not a direction) to begin with, and separately
    // a braking event's samples point the opposite way along that same true axis from an
    // accelerating event's (same physical line, opposite sign of force) — so a window
    // solved during braking and one solved during accelerating are equally "valid" raw
    // answers 180deg apart, through no fault of Eigen's. Trying to locally guess which
    // one is "really" forward per-window (an earlier version of this code did, via each
    // window's own mean projection) doesn't work: that guess is only physically justified
    // for an accelerating event, and is actively wrong for a braking one — there's no way
    // to tell the two apart from one window's own data alone. The fix is to not try:
    // reduce every solve to a canonical half-turn range first, which collapses BOTH
    // sources of mod-180 ambiguity into one, leaving only a single, genuine, irreducible
    // "is it this or the exact opposite" question — answered once, globally, via GNSS
    // bearing below (yaw_branch_deg), not re-litigated per window.
    const Eigen::Vector2f principal = solver.eigenvectors().col(1);
    const float raw_yaw_deg = std::atan2(principal.y(), principal.x()) * kRad2Deg;
    const float canonical_deg = wrapHalfTurnDeg(raw_yaw_deg);

    // Brake/throttle asymmetry: a second, independent way to pick yaw_branch_deg,
    // alongside the GNSS-bearing bridge above. Exists specifically for the gap that
    // bridge cannot cover on its own (see has_bearing_anchor's own doc comment): a
    // straight-line drive with no GNSS bearing yet gives it nothing to disagree with,
    // so the very first branch choice would otherwise be a coin flip on whichever raw
    // sign PCA's eigensolver happened to return. Deliberately deferred to the GNSS
    // bridge the instant it has anything to say (has_bearing_anchor) — that mechanism
    // is corrected by a real observed turn, which is strictly more reliable than a
    // magnitude heuristic, so this never re-litigates a branch GNSS has already spoken
    // on.
    if (!has_bearing_anchor) {
      const float canonical_rad = canonical_deg * kDeg2Rad;
      const Eigen::Vector2f axis(std::cos(canonical_rad), std::sin(canonical_rad));
      float max_proj = -std::numeric_limits<float>::infinity();
      float min_proj = std::numeric_limits<float>::infinity();
      for (const auto& d : dyn_buffer) {
        const float proj = d.x * axis.x() + d.y * axis.y();
        max_proj = std::max(max_proj, proj);
        min_proj = std::min(min_proj, proj);
      }
      // Only draw a conclusion when the window shows real evidence of BOTH a
      // throttle-like and a brake-like event — a window dominated by only one kind of
      // event has an absent opposite extreme, not a small one, and concluding
      // anything from that would be exactly the per-window guessing this class's own
      // design already rejected once (see the comment above solvePcaWindow's PCA
      // solve).
      const float brakeMag = -min_proj;
      if (max_proj > kBrakeThrottleMinExtremeMps2 && brakeMag > kBrakeThrottleMinExtremeMps2) {
        if (brakeMag > max_proj * kBrakeThrottleAsymmetryFactor) {
          // The negative extreme is the dominant one -> it's the brake event -> the
          // canonical (+axis) direction really is forward.
          yaw_branch_deg = 0.0f;
        } else if (max_proj > brakeMag * kBrakeThrottleAsymmetryFactor) {
          // The positive extreme dominates instead -> IT's actually the brake event ->
          // true forward is the opposite of canonical.
          yaw_branch_deg = 180.0f;
        }
        // else: too close to call apart from noise this window — leave yaw_branch_deg
        // exactly as it was rather than guess.
      }
    }

    const float candidate_deg = wrapDeg180(canonical_deg + yaw_branch_deg);

    if (converged) {
      const float diff = std::fabs(wrapDeg180(candidate_deg - yaw_deg));
      if (shock_pending && diff > kReAlignDisagreementDeg) {
        ++disagree_count;
        if (disagree_count >= kReAlignPersistWindows) {
          // Persistent, shock-confirmed disagreement — the phone genuinely moved.
          // Re-converge from here, silently: no error, no crash, just start over.
          recent_estimates_deg.clear();
          converged = false;
          disagree_count = 0;
          shock_pending = false;
        }
      } else {
        disagree_count = 0;
      }
    }

    recent_estimates_deg.push_back(candidate_deg);
    while (recent_estimates_deg.size() > kMaxRecentEstimates) recent_estimates_deg.pop_front();

    const CircularStats stats = circularStats(recent_estimates_deg);
    yaw_deg = stats.mean_deg;
    yaw_sigma_deg = std::max(stats.sigma_deg, kYawSigmaFloorDeg);

    const float span = kYawSigmaStartDeg - kYawSigmaFloorDeg;
    progress = span > 0.0f
                   ? std::clamp((kYawSigmaStartDeg - yaw_sigma_deg) / span, 0.0f, 1.0f)
                   : 0.0f;

    if (yaw_sigma_deg < kConvergedSigmaThresholdDeg) {
      converged = true;
      progress = 1.0f;
    }
  }
};

HacfAligner::HacfAligner() : impl_(std::make_unique<Impl>()) {}
HacfAligner::~HacfAligner() = default;
HacfAligner::HacfAligner(HacfAligner&&) noexcept = default;
HacfAligner& HacfAligner::operator=(HacfAligner&&) noexcept = default;

// Deliberately not using the magnetometer anywhere in this pass: it's unreliable inside
// a vehicle body (steel frame, motor, wiring all distort it) — if a later change wants
// to add it, that's a deliberate reconsideration, not an oversight to "fix."
void HacfAligner::addImuSample(const ImuSample& s) { impl_->addImuSample(s); }

void HacfAligner::addGnssBearing(float bearing_deg, int64_t t_ns) {
  impl_->addGnssBearing(bearing_deg, t_ns);
}

AlignState HacfAligner::state() const { return impl_->state(); }

}  // namespace navcore
