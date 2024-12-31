#pragma once

#include "mc-analysis/cal_f.h"

#include "ptcee/gaussian.h"
#include "ptcee/rotation_from_pan_tilt.h"
#include "ptcee/scalar.h"

#include "gtsam/geometry/PinholeCamera.h"
#include "gtsam/geometry/Point2.h"
#include "gtsam/nonlinear/NonlinearFactor.h"

namespace li
{
struct UnitProjectionFactor : public gtsam::NonlinearFactor
{
  using Cal = CalF;

  struct Error
  {
    Eigen::Vector2d err;
    Eigen::Matrix2d P_err;
    Eigen::Matrix2d J_err_wrt_target_bearing;
    Eigen::Matrix<double, 2, 3> J_err_wrt_rot;
    Eigen::Matrix<double, 2, Cal::dimension> J_err_wrt_cal;
  };

  UnitProjectionFactor(
    const gtsam::Key& target_key,
    const gtsam::Key& rot_key,
    const gtsam::Key& cal_key,
    const ptc::Gaussian<Eigen::Vector2d>& measured_uv
  );

  [[nodiscard]] Error error(
    const gtsam::Unit3& target_bearing_0,
    const gtsam::Rot3& R_i_from_0,
    const Cal& cal
  ) const;

  [[nodiscard]] double error(
    const gtsam::Values& x
  ) const override;

  [[nodiscard]] boost::shared_ptr<gtsam::GaussianFactor> linearize(
    const gtsam::Values& x
  ) const override;

  [[nodiscard]] size_t dim() const override
  {
    return 2;
  }

private:
  gtsam::Key target_key_;
  gtsam::Key rot_key_;
  gtsam::Key cal_key_;
  ptc::Gaussian<Eigen::Vector2d> measured_uv_;
};
}
