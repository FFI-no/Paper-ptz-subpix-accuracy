// Copyright (c) 2025 Norwegian Defence Research Establishment (FFI)

#pragma once

#include "mc-analysis/simulation.h"

namespace mc
{
template<int sigma_pt_scale_exponent>
Config generateConfig(
  RNG& rng
);

Config generateConfig(
  double sigma_pt_scale,
  RNG& rng
);

// ----- Implementation -----
template<int sigma_pt_scale_exponent>
Config generateConfig(
  RNG& rng
)
{
  constexpr double sigma_pt_scale = std::pow(10., sigma_pt_scale_exponent);

  return generateConfig(sigma_pt_scale, rng);
}
}
