// Copyright (c) 2025 Norwegian Defence Research Establishment (FFI)

#include "mc-analysis/generate_config.h"

#include "mc-analysis/utils.h"

#include "gtsam/geometry/Rot3.h"

namespace mc
{
Config generateConfig(
  const double sigma_pt_scale,
  RNG& rng
)
{
  Config config;

  config.img_t_begin = 0;
  config.img_t_end = 10;

  {
    std::uniform_real_distribution<double> rate_dist(10, 30);

    config.img_rate = rate_dist(rng);
  }

  {
    std::uniform_real_distribution<double> rate_dist(3*config.img_rate, 100);

    config.pt_rate = rate_dist(rng);
  }

  config.pt_t_begin = config.img_t_begin - 1;
  config.pt_t_end = config.img_t_end + 1;

  {
    std::uniform_real_distribution<double> angle_dist(std::log(1e-5), std::log(1e-4));

    config.sigma_angle_rad = std::exp(angle_dist(rng));
  }

  {
    std::uniform_real_distribution<double> px_dist(std::log(0.2), std::log(0.5));

    config.sigma_px = std::exp(px_dist(rng));
  }

  {
    std::uniform_real_distribution<double> t_dist(std::log(1e-4), std::log(5e-3));

    config.sigma_t_img = std::exp(t_dist(rng));
  }

  {
    std::uniform_real_distribution<double> t_dist(std::log(1e-4), std::log(5e-3));

    config.sigma_t_pt = std::exp(t_dist(rng));
  }

  {
    std::uniform_real_distribution<double> t_dist(std::log(1e-5), std::log(std::min(1e-4, config.sigma_t_img)));

    config.sigma_dt_img = std::exp(t_dist(rng));
  }

  {
    std::uniform_real_distribution<double> t_dist(std::log(1e-5), std::log(std::min(1e-4, config.sigma_t_pt)));

    config.sigma_dt_pt = std::exp(t_dist(rng));
  }

  {
    constexpr double w = 1920;
    std::uniform_real_distribution<double> f_dist(
      fFromHfov(w, 1/180.*M_PI),
      fFromHfov(w, 60/180.*M_PI)
    );
    config.actual_hfov = hfovFromF(w, f_dist(rng));
  }

  {
    std::uniform_real_distribution<double> k_dist(-0.3, 0.3);
    config.actual_k = k_dist(rng);
    config.initial_guess_sigma_k = 1;
  }

  {
    std::uniform_real_distribution<double> dt_dist(-100e-3, 100e-3);
    config.actual_dt = dt_dist(rng);
  }

  {
    std::uniform_real_distribution<double> axis_dist(-50e-3, 50e-3);

    config.actual_pan_axis = gtsam::Unit3(Eigen::Vector3d::UnitZ()).retract({axis_dist(rng), axis_dist(rng)});
    config.actual_tilt_axis = gtsam::Unit3(Eigen::Vector3d::UnitY()).retract({axis_dist(rng), axis_dist(rng)});
  }

  {
    std::uniform_real_distribution<double> scale_dist(1 - 2*sigma_pt_scale, 1 + 2*sigma_pt_scale);

    config.pt_scale = {scale_dist(rng), scale_dist(rng)};
    config.pt_scale_sigmas = {sigma_pt_scale, sigma_pt_scale};
  }

  {
    std::uniform_real_distribution<double> ld_dist(0., 2e-3/1080);
    config.actual_line_duration = ld_dist(rng);
    config.initial_guess_var_ell = 1e-6;
  }

  config.num_octaves = 1;
  config.fixate_k = false;
  config.generate_multi_level_observations = false;

  return config;
}
}
