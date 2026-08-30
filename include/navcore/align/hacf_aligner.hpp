#pragma once

#include <memory>

#include "navcore/types.hpp"

namespace navcore {

// Solves the rotation from the phone's frame into the vehicle's frame: pitch and roll
// from gravity (Stage 1), yaw from PCA on dynamic-driving acceleration (Stage 2). Runs
// standalone on nothing but IMU samples (an optional GNSS bearing is a sign-fix hint
// only, never a fusion input) — no Kalman filter, no GNSS fusion, no model integration
// belongs here. See hacf_aligner.cpp for the full design notes.
//
// Pimpl'd deliberately: the implementation buffers Eigen types internally, and this
// header — the actual public contract other components include — has no reason to drag
// Eigen along with it.
class HacfAligner {
 public:
  HacfAligner();
  ~HacfAligner();

  HacfAligner(const HacfAligner&) = delete;
  HacfAligner& operator=(const HacfAligner&) = delete;
  HacfAligner(HacfAligner&&) noexcept;
  HacfAligner& operator=(HacfAligner&&) noexcept;

  void addImuSample(const ImuSample& s);

  // Optional. Used only to disambiguate PCA's sign (an axis, not a direction) when the
  // vehicle is genuinely moving — never a fusion input.
  void addGnssBearing(float bearing_deg, int64_t t_ns);

  AlignState state() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace navcore
