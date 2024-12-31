#pragma once

#include "Eigen/Core"
#include "gtsam/geometry/Rot3.h"
#include "ptcee/gaussian.h"

#include <map>

namespace li
{
using FrameId = size_t;
using LandmarkId = size_t;

struct ObservationSet
{
  FrameId frame_id;
  gtsam::Rot3 initial_guess_R_i_from_0;
  std::map<LandmarkId, ptc::Gaussian<Eigen::Vector2d>> observations;
};

struct OctaveSets
{
  double initial_guess_f;
  double dlt_f;
  std::vector<ObservationSet> all_observations;
};

[[nodiscard]] Eigen::Matrix3d estimate(
  const std::vector<OctaveSets>& octaves
);
}