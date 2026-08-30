// Reads a CSV of ImuSample rows (t_ns,ax,ay,az,gx,gy,gz — one line per sample, no
// header), feeds them through HacfAligner in timestamp order, and prints
// pitch/roll/yaw/yaw_sigma/converged/progress once per simulated second. This is the
// manual "does it actually look right" check for the aligner, and doubles as the tool
// whoever ends up with real recorded sessions can point at those once they exist —
// nothing here is specific to synthetic data.
#include "navcore/align/hacf_aligner.hpp"

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>

namespace {

bool ParseImuLine(const std::string& line, navcore::ImuSample* out) {
  std::istringstream ss(line);
  std::string field;
  float values[7];
  for (int i = 0; i < 7; ++i) {
    if (!std::getline(ss, field, ',')) return false;
    values[i] = std::strtof(field.c_str(), nullptr);
  }
  out->t_ns = static_cast<int64_t>(values[0]);
  out->ax = values[1];
  out->ay = values[2];
  out->az = values[3];
  out->gx = values[4];
  out->gy = values[5];
  out->gz = values[6];
  out->temp_c = std::nanf("");  // not carried in the CSV format
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::fprintf(stderr, "usage: %s <imu_samples.csv>\n", argv[0]);
    return 1;
  }

  std::ifstream file(argv[1]);
  if (!file.is_open()) {
    std::fprintf(stderr, "error: could not open '%s'\n", argv[1]);
    return 1;
  }

  navcore::HacfAligner aligner;
  bool have_t0 = false;
  int64_t t0_ns = 0;
  int64_t next_print_ns = 0;

  std::string line;
  while (std::getline(file, line)) {
    if (line.empty()) continue;
    navcore::ImuSample sample{};
    if (!ParseImuLine(line, &sample)) {
      std::fprintf(stderr, "warning: skipping unparsable line: %s\n", line.c_str());
      continue;
    }

    if (!have_t0) {
      t0_ns = sample.t_ns;
      next_print_ns = t0_ns;
      have_t0 = true;
    }

    aligner.addImuSample(sample);

    while (sample.t_ns >= next_print_ns) {
      const navcore::AlignState s = aligner.state();
      const double elapsed_s = static_cast<double>(next_print_ns - t0_ns) * 1e-9;
      std::printf(
          "t=%6.1fs  pitch=%7.2f  roll=%7.2f  yaw=%7.2f  yaw_sigma=%6.2f  converged=%-5s  progress=%.2f\n",
          elapsed_s, s.pitch_deg, s.roll_deg, s.yaw_deg, s.yaw_sigma_deg, s.converged ? "true" : "false",
          s.progress);
      next_print_ns += 1'000'000'000LL;
    }
  }

  return 0;
}
