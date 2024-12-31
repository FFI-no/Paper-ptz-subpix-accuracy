// Copyright (c) 2025 Norwegian Defence Research Establishment (FFI)

#include "mc-analysis/utils.h"

#include <cmath>

namespace mc
{
double hfovFromF(
  const double w,
  const double f
)
{
  return 2*std::atan(w/(2*f));
}

double fFromHfov(
  const double w,
  const double hfov
)
{
  return w/(2*std::tan(hfov/2));
}
}
