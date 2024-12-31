// Copyright (c) 2025 Norwegian Defence Research Establishment (FFI)

#include "mc-analysis/argparse.h"
#include "mc-analysis/triple_cosine_path.h"
#include "mc-analysis/generate_config.h"
#include "mc-analysis/simulation.h"

#include <fstream>
#include <iomanip>

int main(const int argc, const char** argv)
{
  constexpr size_t seed = 47;
  const auto args = mc::readStandardArgs(argc, argv);

  constexpr bool add_octaves = true;
  constexpr double min_hfov_deg = 1;
  constexpr size_t num_zoom_steps = 6;

  auto hfov_deg = min_hfov_deg;

  for (size_t i = 0; i < num_zoom_steps; ++i)
  {
    const auto num_octaves = add_octaves
      ? num_zoom_steps - i
      : 1;

    std::ostringstream ss;
    ss <<
       args.out_path
       << "-" << std::setw(2) << std::setfill('0') << static_cast<int>(hfov_deg) << "deg";

    if (add_octaves)
    {
      ss << "+" << static_cast<int>(hfov_deg*(1 << (num_octaves-1)));
    }

    ss << ".dat";

    std::ofstream res_out{ss.str()};

    mc::performSimulations(
      seed + i,
      args.num_threads,
      args.num_iter,
      [hfov_deg, num_octaves](mc::RNG& rng)
      {
        auto config = mc::generateConfig(1e-8, rng);

        config.actual_hfov = hfov_deg/180.*M_PI;
        config.num_octaves = num_octaves;
        config.fixate_k = true;
        config.generate_multi_level_observations = true;
        config.actual_line_duration = 0;
        config.actual_k = 0;
        config.img_rate = 12.5;
        config.pt_rate = 30;
        config.sigma_px = 0.5;
        config.sigma_angle_rad = 1e-3;
        config.initial_guess_sigma_k = 1e-8;
        config.initial_guess_var_ell = 1e-9;

        return config;
      },
      mc::tripleCosinePath,
      res_out
    );
    hfov_deg *= 2;
  }

  return 0;
}
