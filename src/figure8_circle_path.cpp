// Copyright (c) 2025 Norwegian Defence Research Establishment (FFI)

#include "mc-analysis/figure8_circle_path.h"

#include "mc-analysis/pt_path.h"

namespace mc
{
TruePanTilt figure8CirclePath(
  const Timestamp timestamp,
  const Config& config
)
{
  const auto swath_rad = config.actual_hfov;

  const auto r = 2*swath_rad/(3 + 2*M_SQRT2);
  const auto r_inner = swath_rad/8;

  static thread_local const std::vector<std::pair<Segment::Ptr, bool>> segments = {
    {
      (makeSegment(
        2*M_PI*r_inner/r,
        [r_inner](const double l)
        {
          const auto pan = r_inner*(std::cos(2*M_PI*(l - 1/8.)) - M_SQRT1_2);
          const auto tilt = r_inner*(std::sin(2*M_PI*(l - 1/8.)) + M_SQRT1_2);

          return ptc::PanTilt{pan, tilt};
        }
      )),
      true
    },
    {
      (makeSegment(
        (2 + M_SQRT2)/2,
        [r](const double l)
        {
          const auto pan = r*(1/2. + M_SQRT1_2)*l;
          const auto tilt = r*(1/2. + M_SQRT1_2)*l;

          return ptc::PanTilt{pan, tilt};
        }
      )),
      true
    },
    {
      (makeSegment(
        3/4.*M_PI,
        [r](const double l)
        {
          // fra 3/4 til 0
          const auto w = M_PI*1/4.*(3 - 3*l);

          const auto pan = r*(1/2. + M_SQRT2 + std::cos(w));
          const auto tilt = r*(1/2. + std::sin(w));

          return ptc::PanTilt{pan, tilt};
        }
      )),
      true
    },
    {
      (makeSegment(
        1,
        [r](const double l)
        {
          const auto pan = r*(1 + 1/2. + M_SQRT2);
          const auto tilt = r/2 - r*l;

          return ptc::PanTilt{pan, tilt};
        }
      )),
      true
    },
    {
      (makeSegment(
        3/4.*M_PI,
        [r](const double l)
        {
          // fra 0 til -3/4
          const auto w = M_PI*1/4.*(0 - 3*l);

          const auto pan = r*(1/2. + M_SQRT2 + std::cos(w));
          const auto tilt = r*(-1/2. + std::sin(w));

          return ptc::PanTilt{pan, tilt};
        }
      )),
      true
    },
    {
      (makeSegment(
        2 + M_SQRT2,
        [r](const double l)
        {
          const auto pan = r*(1/2. + M_SQRT1_2)*(1 - 2*l);
          const auto tilt = r*(1/2. + M_SQRT1_2)*(-1 + 2*l);

          return ptc::PanTilt{pan, tilt};
        }
      )),
      true
    },
    {
      (makeSegment(
        3/4.*M_PI,
        [r](const double l)
        {
          // fra 1/4 til 4/4
          const auto w = M_PI*1/4.*(1 + 3*l);

          const auto pan = r*(-1/2. - M_SQRT2 + std::cos(w));
          const auto tilt = r*(1/2. + std::sin(w));

          return ptc::PanTilt{pan, tilt};
        }
      )),
      true
    },
    {
      (makeSegment(
        1,
        [r](const double l)
        {
          const auto pan = r*(-1 - 1/2. - M_SQRT2);
          const auto tilt = r/2 - r*l;

          return ptc::PanTilt{pan, tilt};
        }
      )),
      true
    },
    {
      (makeSegment(
        3/4.*M_PI,
        [r](const double l)
        {
          // fra -4/4 til -1/4
          const auto w = M_PI*1/4.*(-4 + 3*l);

          const auto pan = r*(-1/2. - M_SQRT2 + std::cos(w));
          const auto tilt = r*(-1/2. + std::sin(w));

          return ptc::PanTilt{pan, tilt};
        }
      )),
      true
    },
    {
      (makeSegment(
        (2 + M_SQRT2)/2,
        [r](const double l)
        {
          const auto pan = r*(1/2. + M_SQRT1_2)*(-1 + l);
          const auto tilt = r*(1/2. + M_SQRT1_2)*(-1 + l);

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
