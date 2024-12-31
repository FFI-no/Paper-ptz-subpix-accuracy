// Copyright (c) 2025 Norwegian Defence Research Establishment (FFI)

#include "mc-analysis/triple_cosine_path.h"

#include "mc-analysis/pt_path.h"

namespace mc
{
TruePanTilt tripleCosinePath(
  const Timestamp timestamp,
  const Config& config
)
{
  const auto swath_rad = 3*config.actual_hfov;
  const auto s = swath_rad/3;

  static thread_local const std::vector<std::pair<Segment::Ptr, bool>> segments = {
    {
      (makeSegment(
        9,
        [s](const double l)
        {
          const auto pan = 9*s*std::sin(2*M_PI*l)/(2*M_PI);
          const auto tilt = -3*s*std::cos(2*M_PI*l*3)/(2*M_PI);

          return ptc::PanTilt{pan, tilt};
        }
      )),
      true
    },
  };

  static thread_local const auto duration = totalDuration(segments);

  constexpr Timestamp t0 = 0;
  const auto dt_period = (timestamp - t0)/10;

  const auto t = duration*(dt_period - std::floor(dt_period));

  const auto [pt, i] = eval(segments, t);

  return {pt, segments[i].second};
}
}
