#include "mc-analysis/unit_projection_factor.h"

#include "gtsam/nonlinear/Symbol.h"

namespace li
{
UnitProjectionFactor::UnitProjectionFactor(
  const gtsam::Key& target_key,
  const gtsam::Key& rot_key,
  const gtsam::Key& cal_key,
  const ptc::Gaussian<Eigen::Vector2d>& measured_uv
)
  : gtsam::NonlinearFactor{
  std::vector{
    target_key,
    rot_key,
    cal_key,
  }}
  , target_key_{target_key}
  , rot_key_{rot_key}
  , cal_key_{cal_key}
  , measured_uv_{measured_uv}
{}

UnitProjectionFactor::Error UnitProjectionFactor::error(
  const gtsam::Unit3& target_bearing_0,
  const gtsam::Rot3& R_i_from_0,
  const li::UnitProjectionFactor::Cal& cal
) const
{
  const gtsam::Pose3 pose = {
    R_i_from_0,
    Eigen::Vector3d::Zero()
  };
  const gtsam::PinholeCamera camera{
    pose,
    cal
  };

  Eigen::Matrix<double, 2, gtsam::Pose3::dimension> J_predicted_uv_wrt_pose;
  Eigen::Matrix2d J_predicted_uv_wrt_target_bearing;
  Eigen::Matrix<double, 2, Cal::dimension> J_predicted_uv_wrt_cal;

  const auto predicted_uv = camera.project(target_bearing_0, J_predicted_uv_wrt_pose, J_predicted_uv_wrt_target_bearing, J_predicted_uv_wrt_cal);
  const auto J_predicted_uv_wrt_R_i_from_0 = J_predicted_uv_wrt_pose.topLeftCorner<2, 3>().eval();

  const auto err = predicted_uv - measured_uv_.x;

  return {
    err,
    measured_uv_.P,
    J_predicted_uv_wrt_target_bearing,
    J_predicted_uv_wrt_R_i_from_0,
    J_predicted_uv_wrt_cal
  };
}

double UnitProjectionFactor::error(
  const gtsam::Values& x
) const
{
  const auto target_bearing_0 = x.at<gtsam::Unit3>(target_key_);
  const auto R_i_from_0 = x.at<gtsam::Rot3>(rot_key_);
  const auto cal = x.at<Cal>(cal_key_);

  const auto full_error = error(target_bearing_0, R_i_from_0, cal);

  return gtsam::noiseModel::Gaussian::Covariance(full_error.P_err)->squaredMahalanobisDistance(full_error.err);
}

boost::shared_ptr<gtsam::GaussianFactor> UnitProjectionFactor::linearize(
  const gtsam::Values& x
) const
{
  const auto target_bearing_0 = x.at<gtsam::Unit3>(target_key_);
  const auto R_i_from_0 = x.at<gtsam::Rot3>(rot_key_);
  const auto cal = x.at<Cal>(cal_key_);

  const auto res = error(target_bearing_0, R_i_from_0, cal);

  std::vector<gtsam::Matrix> A{
    res.J_err_wrt_target_bearing,
    res.J_err_wrt_rot,
    res.J_err_wrt_cal
  };
  gtsam::Vector b = -res.err;

  return boost::make_shared<gtsam::JacobianFactor>(
    std::vector{
      std::pair{target_key_, std::move(A[0])},
      std::pair{rot_key_, std::move(A[1])},
      std::pair{cal_key_, std::move(A[2])}
    },
    b
  );
}
}