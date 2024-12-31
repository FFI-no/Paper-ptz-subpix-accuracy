// Copyright (c) 2025 Norwegian Defence Research Establishment (FFI)

#pragma once

#include "ptcee/vector_traits.h"
#include "gtsam/base/VectorSpace.h"

namespace li
{
struct CalF
{
  using Vector = gtsam::Vector;

  double f;
  double w;
  double h;

  CalF(const double f_in, const double w_in, const double h_in)
    : f{f_in}
    , w{w_in}
    , h{h_in}
  {}

  enum {dimension = 1};

  CalF retract(const Vector& v) const
  {
    return CalF{f + v(0), w, h};
  }

  Vector localCoordinates(const CalF& cal) const
  {
    return Eigen::Vector<double, 1>{cal.f - f};
  }

  Eigen::Vector2d calibrate(
    const Eigen::Vector2d& uv,
    gtsam::OptionalJacobian<2, 1> J_xy_wrt_f = boost::none,
    gtsam::OptionalJacobian<2, 2> J_xy_wrt_uv = boost::none
  ) const
  {
    const Eigen::Vector2d xy = (uv - Eigen::Vector2d{w/2, h/2})/f;

    if (J_xy_wrt_f || J_xy_wrt_uv)
    {
      Eigen::Vector2d J_uv_wrt_f;
      Eigen::Matrix2d J_uv_wrt_xy;
      uncalibrate(xy, J_uv_wrt_f, J_uv_wrt_xy);

      const auto inv_J_uv_wrt_xy = J_uv_wrt_xy.inverse().eval();

      if (J_xy_wrt_f)
      {
        *J_xy_wrt_f = -inv_J_uv_wrt_xy*J_uv_wrt_f;
      }

      if (J_xy_wrt_uv)
      {
        *J_xy_wrt_uv = inv_J_uv_wrt_xy;
      }
    }

    return xy;
  }

  /// \brief Convert to normalized coordinates xy to pixel coordinates uv
  Eigen::Vector2d uncalibrate(
    const Eigen::Vector2d& xy,
    gtsam::OptionalJacobian<2, 1> J_uv_wrt_f = boost::none,
    gtsam::OptionalJacobian<2, 2> J_uv_wrt_xy = boost::none
  ) const
  {
    if (J_uv_wrt_f)
    {
      J_uv_wrt_f->col(0) = xy;
    }

    if (J_uv_wrt_xy)
    {
      J_uv_wrt_xy->col(0) = Eigen::Vector2d{f, 0};
      J_uv_wrt_xy->col(1) = Eigen::Vector2d{0, f};
    }

    return {
      xy.x()*f + w/2,
      xy.y()*f + h/2
    };
  }

  Eigen::Vector<double, 1> vector() const
  {
    return Eigen::Vector<double, 1>{f};
  }

  void print(const std::string&) const {}

  bool equals(const CalF& other, const double tol) const
  {
    return std::abs(f - other.f) < tol;
  }
};
}

template<>
struct gtsam::traits<li::CalF> : public gtsam::internal::Manifold<li::CalF>
{};