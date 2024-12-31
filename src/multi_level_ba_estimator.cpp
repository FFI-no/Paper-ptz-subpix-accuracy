#include "mc-analysis/multi_level_ba_estimator.h"

#include "mc-analysis/cal_f.h"
#include "mc-analysis/unit_projection_factor.h"

#include "gtsam/nonlinear/LevenbergMarquardtOptimizer.h"
#include "gtsam/nonlinear/NonlinearFactorGraph.h"
#include "gtsam/inference/Symbol.h"

namespace li
{
namespace key
{
using gtsam::symbol_shorthand::K;
using gtsam::symbol_shorthand::L;
using gtsam::symbol_shorthand::R;
}

Eigen::Matrix3d estimate(
  const std::vector<OctaveSets>& octaves
)
{
  gtsam::Values initial_guess;
  gtsam::NonlinearFactorGraph graph;

  size_t rot_counter = 0;

  for (size_t i = 0; i < octaves.size(); ++i)
  {
    const auto cal_key = key::K(i);
    const auto& [initial_guess_f, dlt_f, all_observations] = octaves.at(i);

    const CalF initial_cal{initial_guess_f, 1920, 1080};
    initial_guess.insert(cal_key, initial_cal);

    for (const auto& [frame_id, R_i_from_0, observations] : all_observations)
    {
      const auto rotation_key = key::R(rot_counter++);
      initial_guess.insert(rotation_key, R_i_from_0);

      const auto R_0_from_i = R_i_from_0.inverse();

      for (const auto& [landmark_id, observation] : observations)
      {
        const auto landmark_key = key::L(landmark_id);

        if (initial_guess.find(landmark_key) == initial_guess.end())
        {
          const auto initial_guess_dir = R_0_from_i*gtsam::Unit3(initial_cal.calibrate(observation.x).homogeneous());
          initial_guess.insert(landmark_key, initial_guess_dir);
        }

        graph.push_back(
          UnitProjectionFactor{
            landmark_key,
            rotation_key,
            cal_key,
            observation
          }
        );
      }
    }
  }

  gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial_guess);
  const auto estimate = optimizer.optimize();

  const auto f = estimate.at<CalF>(key::K(0)).f;

  return Eigen::Vector2d::Constant(f).homogeneous().asDiagonal();
}
}