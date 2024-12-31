// Copyright (c) 2025 Norwegian Defence Research Establishment (FFI)

#pragma once

#include "ptcee/pan_tilt.h"

#include <memory>

namespace mc
{
struct Segment
{
  using Ptr = std::shared_ptr<Segment>;

  virtual ~Segment() = default;

  double duration;

  explicit Segment(const double duration_in)
    : duration{duration_in}
  {}

  virtual ptc::PanTilt operator()(double l) const = 0;
};

template<typename F>
struct FSegment : public Segment
{
  F f;

  FSegment(const double duration_in, F f_in)
    : Segment{duration_in}
    , f{f_in}
  {}

  ptc::PanTilt operator()(const double l) const override
  {
    return f(l);
  }
};

template<typename F>
std::shared_ptr<FSegment<F>> makeSegment(const double duration, F f)
{
  return std::make_shared<FSegment<F>>(duration, f);
}

double totalDuration(const std::vector<std::pair<Segment::Ptr, bool>>& segments);

std::pair<ptc::PanTilt, size_t> eval(
  const std::vector<std::pair<Segment::Ptr, bool>>& segments,
  double t
);
}
