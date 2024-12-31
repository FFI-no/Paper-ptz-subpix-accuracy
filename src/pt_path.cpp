// Copyright (c) 2025 Norwegian Defence Research Establishment (FFI)

#include "mc-analysis/pt_path.h"

namespace mc
{
double totalDuration(const std::vector<std::pair<Segment::Ptr, bool>>& segments)
{
  double duration = 0;

  for (const auto& segment : segments)
  {
    duration += segment.first->duration;
  }

  return duration;
}

std::pair<ptc::PanTilt, size_t> eval(
  const std::vector<std::pair<Segment::Ptr, bool>>& segments,
  const double t
)
{
  double duration = 0;

  for (;;)
  {
    size_t i = 0;
    for (const auto& p : segments)
    {
      const auto& segment = p.first;

      if (duration + segment->duration < t)
      {
        duration += segment->duration;
        ++i;
        continue;
      }

      const auto l = (t - duration)/segment->duration;

      return {(*segment)(l), i};
    }
  }
}
}
