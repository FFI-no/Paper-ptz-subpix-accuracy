// Copyright (c) 2025 Norwegian Defence Research Establishment (FFI)

#pragma once

#include "Eigen/Core"
#include "gtsam/geometry/Unit3.h"
#include "ptcee/pan_tilt.h"
#include "ptcee/timestamp.h"

#include <functional>
#include <ostream>
#include <random>

namespace mc
{
struct Config
{
  double img_t_begin;
  double img_t_end;
  double img_rate;

  double pt_rate;
  double pt_t_begin;
  double pt_t_end;

  double sigma_angle_rad;
  double sigma_px;

  double sigma_t_img;
  double sigma_t_pt;
  double sigma_dt_img;
  double sigma_dt_pt;

  double actual_hfov;
  double actual_k;
  double actual_dt;
  gtsam::Unit3 actual_pan_axis;
  gtsam::Unit3 actual_tilt_axis;
  Eigen::Array2d pt_scale;
  Eigen::Array2d pt_scale_sigmas;
  double actual_line_duration;

  double initial_guess_sigma_k;
  double initial_guess_var_ell;

  size_t num_octaves;
  bool fixate_k;
  bool generate_multi_level_observations;
};

using RNG = std::mt19937;
using ConfigProvider = std::function<Config(RNG& rng)>;
using Timestamp = ptc::Timestamp;

struct TruePanTilt
{
  ptc::PanTilt pt;
  bool image_taken;
};

using PanTiltProvider = std::function<TruePanTilt(Timestamp timestamp, const Config& config)>;

void performSimulations(
  size_t seed,
  size_t num_threads,
  size_t num_iter,
  const Config& config,
  const PanTiltProvider& pt_provider,
  std::ostream& res_out
);

void performSimulations(
  size_t seed,
  size_t num_threads,
  size_t num_iter,
  const ConfigProvider& config_provider,
  const PanTiltProvider& pt_provider,
  std::ostream& res_out
);
}
