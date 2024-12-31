// Copyright (c) 2025 Norwegian Defence Research Establishment (FFI)

#include "mc-analysis/simulation.h"

#include "mc-analysis/multi_level_ba_estimator.h"
#include "mc-analysis/utils.h"

#include "Eigen/SVD"
#include "opencv2/calib3d.hpp"
#include "opencv2/stitching.hpp"
#include "opencv2/stitching/detail/autocalib.hpp"
#include "ptcee/cal_fk.h"
#include "ptcee/gaussian.h"
#include "ptcee/noisy_unit.h"
#include "ptcee/pt_buffer_factor.h"
#include "ptcee/ptz_estimator.h"
#include "ptcee/ptz_graph.h"
#include "gtsam/nonlinear/Marginals.h"

#include <condition_variable>
#include <future>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <random>
#include <vector>

namespace mc
{
namespace
{
using Cal = ptc::CalFK;
using Camera = gtsam::PinholeCamera<Cal>;
using PTZEstimator = ptc::PTZEstimator<Cal>;

struct Result
{
  Eigen::Vector3d true_cal;
  ptc::Gaussian<Cal> est_cal;
  double true_dt;
  ptc::Gaussian<double> est_dt;
  gtsam::Unit3 true_pan_axis;
  ptc::Gaussian<gtsam::Unit3> est_pan_axis;
  gtsam::Unit3 true_tilt_axis;
  ptc::Gaussian<gtsam::Unit3> est_tilt_axis;
  Eigen::Vector2d true_pt_scale;
  ptc::Gaussian<Eigen::Vector2d> est_pt_scale;
  double true_line_duration;
  ptc::Gaussian<double> est_ld;
  double mean_pixel_error;
  double max_pixel_error;
  double pan_std;
  double tilt_std;
  gtsam::JointMarginal marginal;
  double dlt_f;
  double single_level_ba_f;
  double multi_level_ba_f;
};

struct RunData
{
  Config config;
  Result result;
};

Cal createCalibration(
  double hfov,
  double k
);

ptc::Gaussian<Cal> createInitialGuessCalibration(
  RNG& rng,
  double mean_hfov,
  double sigma_k
);

Eigen::Vector2d project(
  const Eigen::Vector3d& p_frame,
  const Cal& cal,
  double t_img_begin,
  const PanTiltProvider& pt_provider,
  const Config& config
);

Eigen::Vector2d projectGS(
  const Eigen::Vector3d& p_frame,
  const Cal& cal,
  double t,
  const PanTiltProvider& pt_provider,
  const Config& config
);

Eigen::Vector2d measureObservation(
  const Eigen::Vector3d& p_frame,
  const Cal& cal,
  double t_img_begin,
  const PanTiltProvider& pt_provider,
  const Config& config,
  RNG& rng
);

ptc::PanTilt measurePanTilt(
  RNG& rng,
  const ptc::PanTilt& pan_tilt,
  const Eigen::Array2d& pan_tilt_scale,
  double sigma_rad
);

ptc::NoisyTimestamp measureTimestamp(
  RNG& rng,
  Timestamp true_timestamp,
  double sigma_t
);

gtsam::Rot3 getOrientation(
  double t,
  const gtsam::Unit3& pan_axis,
  const gtsam::Unit3& tilt_axis,
  const PanTiltProvider& pt_provider,
  const Config& config
);

std::pair<double, double> getMeanAndMaxPixelError(
  const PTZEstimator& estimator
);

std::pair<double, double> getPanTiltStdDev(
  const PTZEstimator& estimator
);

gtsam::Rot3 getOrientation(
  const ptc::PanTilt& pan_tilt,
  const gtsam::Unit3& pan_axis,
  const gtsam::Unit3& tilt_axis
);

Camera getCamera(
  const Cal& cal,
  const gtsam::Rot3& R_base_from_cam_frd
);

std::pair<PTZEstimator, std::vector<li::OctaveSets>> setupEstimator(
  const PanTiltProvider& pt_provider,
  const Config& config,
  RNG& rng
);

Result simulate(
  const PanTiltProvider& pt_provider,
  const Config& config,
  RNG& rng
);

Eigen::Matrix3d cvToEigen(
  const cv::Mat& M
);

cv::Mat EigenToCV(
  const Eigen::MatrixXd& M
);

Eigen::Matrix3d calibrateRotatingCamera(
  std::vector<Eigen::Matrix3d> Hs
);

Eigen::Matrix3d calibrateRotatingCamera2(
  std::vector<Eigen::Matrix3d> Hs
);

Eigen::Matrix3d calibrateRotatingCamera(
  const std::vector<cv::Mat>& Hs
);

struct ObservationSet
{
  double timestamp;
  ptc::NoisyTimestamp measured_timestamp;
  gtsam::Rot3 R_base_from_cam_frd;
  std::map<size_t, ptc::Gaussian<Eigen::Vector2d>> observations;
};

std::vector<ObservationSet> generateObservations(
  const PanTiltProvider& pt_provider,
  const Config& config,
  RNG& rng,
  double img_t_offset,
  size_t octave,
  bool measure_timestamps
);
}

void performSimulations(
  const size_t seed,
  const size_t num_threads,
  const size_t num_iter,
  const Config& config,
  const PanTiltProvider& pt_provider,
  std::ostream& res_out
)
{
  performSimulations(
    seed,
    num_threads,
    num_iter,
    [&config](const RNG&)
    {
      return config;
    },
    pt_provider,
    res_out
  );
}

void performSimulations(
  const size_t seed,
  const size_t num_threads,
  const size_t num_iter,
  const ConfigProvider& config_provider,
  const PanTiltProvider& pt_provider,
  std::ostream& res_out
)
{
  std::mutex mutex;
  std::condition_variable cv;
  size_t num_active_workers = 0;
  size_t num_finished = 0;
  size_t num_crashed = 0;
  std::vector<std::future<RunData>> runs;

  std::vector<size_t> seeds;

  {
    RNG rng{seed};
    seeds.reserve(num_iter);

    for (size_t l = 0; l < num_iter; ++l)
    {
      seeds.push_back(rng());
    }
  }

  const auto begin = std::chrono::high_resolution_clock::now();

  for (size_t l = 0; l < num_iter; ++l)
  {
    {
      std::unique_lock lock{mutex};

      cv.wait(
        lock,
        [&num_active_workers, num_threads]()
        {
          return num_active_workers < num_threads;
        }
      );
    }

    ++num_active_workers;

    runs.push_back(
      std::async(
        std::launch::async,
        [&cv, &mutex, &num_active_workers, &num_finished, &num_crashed, l, begin, &seeds, num_iter, &config_provider, &pt_provider]()
        {
          try
          {
            RNG rng{seeds[l]};
            auto config = config_provider(rng);

            const auto result = simulate(pt_provider, config, rng);

            {
              const auto now = std::chrono::high_resolution_clock::now();
              const auto duration = std::chrono::duration<double>{now - begin}.count();

              const auto elapsed_min = static_cast<int>(std::floor(duration/60));
              const auto elapsed_sec = duration - 60*elapsed_min;

              std::scoped_lock lock{mutex};
              --num_active_workers;

              ++num_finished;

              const auto progress = static_cast<double>(num_finished)/num_iter;
              const auto eta = duration/progress;
              const auto eta_min = static_cast<int>(std::floor(eta/60));
              const auto eta_sec = eta - 60*eta_min;

              std::cerr
                << "\r"
                << std::setw(5) << std::setfill(' ')
                << num_finished << "/" << (num_iter - num_crashed)
                << " ("
                << std::setw(2) << std::setfill('0') << elapsed_min
                << ":" << std::setw(5) << std::setfill('0') << std::fixed << std::setprecision(2) << elapsed_sec
                << " / "
                << std::setw(2) << std::setfill('0') << eta_min
                << ":" << std::setw(5) << std::setfill('0') << std::fixed << std::setprecision(2) << eta_sec
                << ")"
                << std::flush;
            }

            cv.notify_one();

            return RunData{
              config,
              result
            };
          }
          catch (...)
          {
            std::scoped_lock lock{mutex};
            --num_active_workers;
            ++num_crashed;

            cv.notify_one();

            std::rethrow_exception(std::current_exception());
          }
        }
      )
    );
  }

  {
    std::unique_lock lock{mutex};

    cv.wait(
      lock,
      [&num_finished, &num_crashed, num_iter]()
      {
        return (num_finished + num_crashed) == num_iter;
      }
    );
  }

  std::cerr << std::endl;

  bool first_crash = true;

  for (auto& run_future : runs)
  try
  {
    const auto [config, result] = run_future.get();

    const auto est_sigma_f = std::sqrt(result.est_cal.P(0, 0));
    const auto est_sigma_ps = std::sqrt(result.est_pt_scale.P(0, 0));
    const double est_c_f_ps = result.marginal.at(ptc::PTZGraph::cal_key, ptc::PTZGraph::pt_scale_key)(0, 0);
    const auto est_corr_f_ps = est_c_f_ps/(est_sigma_f*est_sigma_ps);

    const auto pan_axis_err = result.est_pan_axis.x.localCoordinates(result.true_pan_axis);
    const double pan_sq_mah_dist = pan_axis_err.transpose()*result.est_pan_axis.P.inverse()*pan_axis_err;
    const auto tilt_axis_err = result.est_tilt_axis.x.localCoordinates(result.true_tilt_axis);
    const double tilt_sq_mah_dist = tilt_axis_err.transpose()*result.est_tilt_axis.P.inverse()*tilt_axis_err;

    res_out
      << std::setprecision(16) << std::scientific
      << " " << result.true_cal.transpose()
      << "\n"
      << " " << result.est_cal.x.vector().transpose()
      << "\n"
      << " " << result.est_cal.P.diagonal().array().sqrt().transpose()
      << "\n"
      << " " << result.true_dt
      << " " << result.est_dt.x
      << " " << std::sqrt(result.est_dt.P(0))
      << "\n"
      << " " << result.true_pan_axis.localCoordinates(result.est_pan_axis.x).transpose()
      << " " << result.est_pan_axis.P.diagonal().array().sqrt().transpose()
      << " " << pan_sq_mah_dist
      << "\n"
      << " " << result.true_tilt_axis.localCoordinates(result.est_tilt_axis.x).transpose()
      << " " << result.est_tilt_axis.P.diagonal().array().sqrt().transpose()
      << " " << tilt_sq_mah_dist
      << "\n"
      << " " << result.true_pt_scale.transpose()
      << "\n"
      << " " << result.est_pt_scale.x.transpose()
      << "\n"
      << " " << result.est_pt_scale.P.diagonal().array().sqrt().transpose()
      << "\n"
      << " " << result.true_line_duration
      << " " << result.est_ld.x
      << " " << std::sqrt(result.est_ld.P(0))
      << "\n"
      << " " << result.mean_pixel_error
      << " " << result.max_pixel_error
      << "\n"
      << " " << result.pan_std
      << " " << result.tilt_std
      << "\n"
      << " " << config.img_rate
      << " " << config.pt_rate
      << "\n"
      << " " << config.sigma_angle_rad
      << " " << config.sigma_px
      << " " << config.sigma_t_img
      << " " << config.sigma_t_pt
      << " " << config.sigma_dt_img
      << " " << config.sigma_dt_pt
      << "\n"
      << config.pt_scale_sigmas.transpose()
      << "\n"
      << est_corr_f_ps
      << "\n"
      << result.dlt_f
      << " " << result.single_level_ba_f
      << " " << result.multi_level_ba_f
      << std::endl;
  }
  catch (const std::exception& e)
  {
    if (first_crash)
    {
      std::cerr
        << "Summary of errors:"
        << "\n------------------"
        << std::endl;
      first_crash = false;
    }

    std::cerr
      << e.what()
      << "\n------------------"
      << std::endl;
  }
}

namespace
{
Eigen::Vector2d project(
  const Eigen::Vector3d& p_frame,
  const Cal& cal,
  const double t_img_begin,
  const PanTiltProvider& pt_provider,
  const Config& config
)
{
  const auto line_duration = config.actual_line_duration;
  auto t = t_img_begin + cal.h*line_duration/2;

  for (size_t i = 0; i < 10; ++i)
  {
    const auto uv = projectGS(p_frame, cal, t, pt_provider, config);
    const auto new_t = t_img_begin + uv.y()*line_duration;

    const auto diff_t = std::abs((t - t_img_begin) - uv.y()*line_duration);
    t = new_t;

    if (diff_t < 0.2*line_duration + 1e-8)
    {
      return projectGS(p_frame, cal, t, pt_provider, config);
    }
  }

  throw std::runtime_error("project failed to converge");
}

Eigen::Vector2d projectGS(
  const Eigen::Vector3d& p_frame,
  const Cal& cal,
  const double t,
  const PanTiltProvider& pt_provider,
  const Config& config
)
{
  const auto R_base_from_cam_frd = getOrientation(
    t,
    config.actual_pan_axis,
    config.actual_tilt_axis,
    pt_provider,
    config
  );

  return getCamera(cal, R_base_from_cam_frd).project(p_frame);
}

Eigen::Vector2d measureObservation(
  const Eigen::Vector3d& p_frame,
  const Cal& cal,
  const double t_img_begin,
  const PanTiltProvider& pt_provider,
  const Config& config,
  RNG& rng
)
{
  std::normal_distribution<double> px_dist{0., config.sigma_px};
  const auto uv = project(p_frame, cal, t_img_begin, pt_provider, config);

  return {
    uv.x() + px_dist(rng),
    uv.y() + px_dist(rng)
  };
}

ptc::PanTilt measurePanTilt(
  RNG& rng,
  const ptc::PanTilt& pan_tilt,
  const Eigen::Array2d& pan_tilt_scale,
  const double sigma_rad
)
{
  std::normal_distribution<double> angle_dist{0., sigma_rad};

  return ptc::oplus(pan_tilt_scale*pan_tilt, Eigen::Array2d{angle_dist(rng), angle_dist(rng)});
}

ptc::NoisyTimestamp measureTimestamp(
  RNG& rng,
  const Timestamp true_timestamp,
  const double sigma_t
)
{
  std::normal_distribution<double> t_dist{0., sigma_t};

  return {true_timestamp + t_dist(rng), sigma_t};
}

Cal createCalibration(
  const double hfov,
  const double k
)
{
  constexpr double w = 1920;
  constexpr double h = 1080;

  const auto f = fFromHfov(w, hfov);

  return Cal{f, k, w, h};
}

ptc::Gaussian<Cal> createInitialGuessCalibration(
  RNG& rng,
  const double mean_hfov,
  const double sigma_k
)
{
  constexpr double w = 1920;
  constexpr double h = 1080;

  const auto mean_f = w/(2*std::tan(mean_hfov/2));
  std::uniform_real_distribution<double> f_dist{2/3.*mean_f, 3/2.*mean_f};
  const auto f = f_dist(rng);
  constexpr auto k = 0;

  return {
    Cal{f, k, w, h},
    Eigen::Array<double, 2, 1>{0.5*mean_f, sigma_k}.square().matrix().asDiagonal()
  };
}

gtsam::Rot3 getOrientation(
  const double t,
  const gtsam::Unit3& pan_axis,
  const gtsam::Unit3& tilt_axis,
  const PanTiltProvider& pt_provider,
  const Config& config
)
{
  return getOrientation(pt_provider(t, config).pt, pan_axis, tilt_axis);
}

gtsam::Rot3 getOrientation(
  const ptc::PanTilt& pan_tilt,
  const gtsam::Unit3& pan_axis,
  const gtsam::Unit3& tilt_axis
)
{
  return ptc::getRotation(pan_tilt, pan_axis, tilt_axis).rot;
}

std::pair<double, double> getMeanAndMaxPixelError(
  const PTZEstimator& estimator
)
{
  double max_err = 0;
  double err_sum = 0;
  size_t n = 0;

  const auto& raw_estimate = estimator.getRawEstimate();

  for (const auto& factor : estimator.getGraph())
  {
    const auto* projection_factor = dynamic_cast<ptc::UnitProjectionFactor<Cal>*>(factor.get());

    if (projection_factor)
    {
      const auto err = projection_factor->pixelError(raw_estimate);
      err_sum += err;
      max_err = std::max(max_err, err);
      ++n;
    }
  }

  return {err_sum/n, max_err};
}

std::pair<double, double> getPanTiltStdDev(
  const PTZEstimator& estimator
)
{
  std::vector<double> pan_diffs;
  std::vector<double> tilt_diffs;

  const auto& raw_estimate = estimator.getRawEstimate();

  for (const auto& factor : estimator.getGraph())
  {
    const auto* pt_buffer_factor = dynamic_cast<ptc::PTBufferFactor*>(factor.get());

    if (pt_buffer_factor)
    {
      const auto err = pt_buffer_factor->panTiltError(raw_estimate);
      pan_diffs.push_back(err.x());
      tilt_diffs.push_back(err.y());
    }
  }

  const auto pans = Eigen::VectorXd::Map(pan_diffs.data(), pan_diffs.size());
  const auto tilts = Eigen::VectorXd::Map(tilt_diffs.data(), tilt_diffs.size());

  return {
    std::sqrt((pans.array() - pans.mean()).square().sum()/(pans.size() - 1)),
    std::sqrt((tilts.array() - tilts.mean()).square().sum()/(tilts.size() - 1))
  };
}

Camera getCamera(
  const Cal& cal,
  const gtsam::Rot3& R_base_from_cam_frd
)
{
  static thread_local const gtsam::Rot3 R_FRD_from_RDF{
    (Eigen::Matrix3d{}
      << Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitX()
    ).finished()
  };

  const gtsam::Pose3 pose = {
    R_base_from_cam_frd*R_FRD_from_RDF,
    Eigen::Vector3d::Zero()
  };

  return Camera{pose, cal};
}

std::pair<PTZEstimator, std::vector<li::OctaveSets>> setupEstimator(
  const PanTiltProvider& pt_provider,
  const Config& config,
  RNG& rng
)
{
  std::uniform_real_distribution<double> img_t_offset_dist(0, 1/config.img_rate);
  const auto img_t_offset = img_t_offset_dist(rng);

  const auto initial_guess_cal = createInitialGuessCalibration(rng, config.actual_hfov, config.initial_guess_sigma_k);
  const ptc::Gaussian<double> initial_guess_dt{0., Eigen::Matrix<double, 1, 1>{1.}};
  const ptc::Gaussian<double> initial_guess_line_duration{0., Eigen::Matrix<double, 1, 1>{config.initial_guess_var_ell}};
  const auto initial_guess_pt_scale = Eigen::Vector2d::Ones();

  std::pair<PTZEstimator, std::vector<li::OctaveSets>> out{
    PTZEstimator{
      initial_guess_cal,
      initial_guess_dt,
      initial_guess_line_duration,
      initial_guess_pt_scale,
      config.pt_scale_sigmas,
      config.fixate_k
    },
    {}
  };
  auto& estimator = out.first;
  auto& octaves = out.second;

  const Eigen::Matrix2d P_pan_tilt = config.sigma_angle_rad*config.sigma_angle_rad*Eigen::Matrix2d::Identity();

  const auto measured_dt_img = measureTimestamp(rng, 1/config.img_rate, config.sigma_dt_img);
  const auto measured_dt_pt = measureTimestamp(rng, 1/config.pt_rate, config.sigma_dt_pt);

  for (size_t k = 0;; ++k)
  {
    const auto t = config.pt_t_begin + k / config.pt_rate;

    if (t > config.pt_t_end)
    {
      break;
    }

    const auto pan_tilt = pt_provider(t, config).pt;

    const auto measured_pan_tilt = measurePanTilt(rng, pan_tilt, config.pt_scale, config.sigma_angle_rad);
    const auto measured_timestamp = measureTimestamp(rng, t + config.actual_dt, config.sigma_t_pt);
    estimator.insertPanTilt({measured_timestamp, measured_dt_pt}, {measured_pan_tilt, P_pan_tilt});
  }

  if (config.generate_multi_level_observations)
  {
    for (size_t octave = 0; octave < config.num_octaves; ++octave)
    {
      const auto all_observations = generateObservations(pt_provider, config, rng, img_t_offset, octave, false);

      if (octave == 0)
      {
        for (const auto& [t, unmeasured_timestamp, R_base_from_cam_frd, observations] : all_observations)
        {
          const auto measured_timestamp = measureTimestamp(rng, t, config.sigma_t_img);
          estimator.insertObservations({measured_timestamp, measured_dt_img}, observations);
        }
      }

      const auto num_frames = all_observations.size();
      std::vector<cv::detail::MatchesInfo> pairwise_matches;
      pairwise_matches.reserve(num_frames * num_frames);
      std::vector<cv::Mat> all_H;

      std::map<size_t, Eigen::Matrix3d> Hi0s;
      std::map<size_t, size_t> Hi0s_num_matches;
      Hi0s[0] = Eigen::Matrix3d::Identity();
      Hi0s_num_matches[0] = 0;

      for (size_t src_idx = 0; src_idx < all_observations.size(); ++src_idx)
      {
        for (size_t dst_idx = 0; dst_idx < all_observations.size(); ++dst_idx)
        {
          if (dst_idx == src_idx)
          {
            pairwise_matches.emplace_back();
            continue;
          }

          std::vector<cv::Point2f> src_points;
          std::vector<cv::Point2f> dst_points;

          {
            const auto src_begin = all_observations[src_idx].observations.begin();
            const auto src_end = all_observations[src_idx].observations.end();
            const auto dst_begin = all_observations[dst_idx].observations.begin();
            const auto dst_end = all_observations[dst_idx].observations.end();

            auto src_it = src_begin;
            auto dst_it = dst_begin;

            while (src_it != src_end && dst_it != dst_end)
            {
              if (src_it->first == dst_it->first)
              {
                {
                  const auto& src_pt = src_it->second;

                  src_points.emplace_back(
                    src_pt.x.x() - 1920 / 2,
                    src_pt.x.y() - 1080 / 2
                  );
                }

                {
                  const auto& dst_pt = dst_it->second;

                  dst_points.emplace_back(
                    dst_pt.x.x() - 1920 / 2,
                    dst_pt.x.y() - 1080 / 2
                  );
                }

                ++src_it;
                ++dst_it;
              } else if (src_it->first < dst_it->first)
              {
                ++src_it;
              } else
              {
                ++dst_it;
              }
            }
          }

          auto& match = pairwise_matches.emplace_back();
          match.src_img_idx = src_idx;
          match.dst_img_idx = dst_idx;

          if (src_points.size() >= 20)
          {
            match.H = cv::findHomography(
              src_points,
              dst_points
            );
            all_H.push_back(match.H);

            const auto H_dst_from_src = cvToEigen(match.H);

            if (src_idx == 0)
            {
              Hi0s[dst_idx] = H_dst_from_src;
              Hi0s_num_matches[dst_idx] = src_points.size();
            } else
            {
              const auto src_it = Hi0s.find(src_idx);

              if (src_it != Hi0s.end())
              {
                const auto dst_it = Hi0s.find(dst_idx);

                if (dst_it == Hi0s.end())
                {
                  Hi0s[dst_idx] = H_dst_from_src * src_it->second;
                  Hi0s_num_matches[dst_idx] = src_points.size();
                } else if (Hi0s_num_matches.at(dst_idx) < src_points.size())
                {
                  dst_it->second = H_dst_from_src * src_it->second;
                  Hi0s_num_matches[dst_idx] = src_points.size();
                }
              }
            }
          }
        }
      }

      std::vector<Eigen::Matrix3d> eig_Hs;
      eig_Hs.reserve(all_H.size());

      for (const auto& H: all_H)
      {
        eig_Hs.push_back(cvToEigen(H));
      }

      const auto strict_dlt_f = calibrateRotatingCamera2(eig_Hs).diagonal().head<2>().array().sqrt().prod();

      std::vector<li::ObservationSet> observation_sets;
      observation_sets.reserve(all_observations.size());

      for (size_t k = 0; k < all_observations.size(); ++k)
      {
        const auto R_i_from_0 = all_observations.at(k).R_base_from_cam_frd.inverse(); // TODO: don't use GT for initial guess

        observation_sets.push_back(
          {
            k,
            R_i_from_0,
            all_observations.at(k).observations
          }
        );
      }

      constexpr auto w = 1920;
      const auto rand_guess_f = fFromHfov(w, hfovFromF(w, initial_guess_cal.x.f) * (1 << octave));
      const auto actual_f = fFromHfov(w, config.actual_hfov * (1 << octave));

      const auto initial_guess_f = (std::isnan(strict_dlt_f) || std::abs(actual_f - rand_guess_f) < std::abs(actual_f - strict_dlt_f))
                                     ? rand_guess_f
                                     : strict_dlt_f;

      octaves.push_back(
        {
          .initial_guess_f=initial_guess_f,
          .dlt_f=strict_dlt_f,
          .all_observations=std::move(observation_sets)
        }
      );
    }
  }
  else
  {
    const auto all_observations = generateObservations(pt_provider, config, rng, img_t_offset, 0, true);

    for (const auto& [t, measured_timestamp, R_base_from_cam_frd, observations]: all_observations)
    {
      estimator.insertObservations({measured_timestamp, measured_dt_img}, observations);
    }

    const auto initial_guess_f = initial_guess_cal.x.f;

    octaves.push_back(
      {
        .initial_guess_f = initial_guess_f,
        .dlt_f = initial_guess_f,
        .all_observations = {}
      }
    );
  }

  return out;
}

Result simulate(
  const PanTiltProvider& pt_provider,
  const Config& config,
  RNG& rng
)
{
  auto [estimator, octaves] = setupEstimator(pt_provider, config, rng);
  estimator.estimate();

  const auto estimate = estimator.getEstimate();

  const auto cal = createCalibration(config.actual_hfov, config.actual_k);
  Eigen::Vector3d cal_vals = Eigen::Vector3d::Zero();
  cal_vals.head<std::min(3, static_cast<int>(Cal::dimension))>() = cal.vector();

  const auto [mean_pixel_error, max_pixel_err] = getMeanAndMaxPixelError(estimator);
  const auto [pan_std, tilt_std] = getPanTiltStdDev(estimator);

  const auto dlt_f = octaves.front().dlt_f;
  const auto single_level_ba_f = li::estimate(std::vector{octaves.front()})(0, 0);
  const auto multi_level_ba_f = li::estimate(octaves)(0, 0);

  return Result{
    .true_cal=cal_vals,
    .est_cal=estimate.cal,
    .true_dt=config.actual_dt,
    .est_dt=estimate.dt,
    .true_pan_axis=config.actual_pan_axis,
    .est_pan_axis=estimate.pan_axis,
    .true_tilt_axis=config.actual_tilt_axis,
    .est_tilt_axis=estimate.tilt_axis,
    .true_pt_scale=config.pt_scale,
    .est_pt_scale=estimate.pt_scale,
    .true_line_duration=config.actual_line_duration,
    .est_ld=estimate.ld,
    .mean_pixel_error=mean_pixel_error,
    .max_pixel_error=max_pixel_err,
    .pan_std=pan_std,
    .tilt_std=tilt_std,
    .marginal=estimate.marginal,
    .dlt_f=dlt_f,
    .single_level_ba_f=single_level_ba_f,
    .multi_level_ba_f=multi_level_ba_f,
  };
}

Eigen::Matrix3d cvToEigen(
  const cv::Mat& M
)
{
  Eigen::Matrix3d out;

  for (int i = 0; i < out.rows(); ++i)
  {
    for (int j = 0; j < out.cols(); ++j)
    {
      out(i, j) = M.at<double>(i, j);
    }
  }

  return out;
}

cv::Mat EigenToCV(
  const Eigen::MatrixXd& eig_M
)
{
  cv::Mat_<double> M(eig_M.rows(), eig_M.cols());

  for (int i = 0; i < eig_M.rows(); ++i)
  {
    for (int j = 0; j < eig_M.cols(); ++j)
    {
      M(i, j) = eig_M(i, j);
    }
  }

  return M;
}

Eigen::Matrix3d calibrateRotatingCamera(
  std::vector<Eigen::Matrix3d> Hs
)
{
  assert(!Hs.empty());

  for (auto& H : Hs)
  {
    const double det = H.determinant();
    H /= std::pow(det, 1./3.); // dette normaliserer determinanten til en 3x3-matrise
  }

  const int idx_map[3][3] = {{0, 1, 2}, {1, 3, 4}, {2, 4, 5}};
  Eigen::MatrixXd A(6*Hs.size(), 6);
  A.setZero();

  int eq_idx = 0;
  for (const auto& H : Hs)
  {
    for (int i = 0; i < 3; ++i)
    {
      for (int j = i; j < 3; ++j, ++eq_idx)
      {
        for (int l = 0; l < 3; ++l)
        {
          for (int s = 0; s < 3; ++s)
          {
            int idx = idx_map[l][s];
            A(eq_idx, idx) += H(i,l) * H(j,s);
          }
        }
        A(eq_idx, idx_map[i][j]) -= 1;
      }
    }
  }

  const Eigen::VectorXd wcoef = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).matrixV().col(A.cols() - 1); // Dette svarer til cv::SVD::solveZ, som er å finne unit-length vektoren som minimierer Ax = 0

  Eigen::Matrix3d W;
  for (int i = 0; i < 3; ++i)
  {
    for (int j = i; j < 3; ++j)
    {
      W(i,j) = W(j,i) = wcoef(idx_map[i][j], 0) / wcoef(5,0);
    }
  }

  const Eigen::Matrix3d K = W.llt().matrixL().transpose();

  return K/K(2, 2);
}

Eigen::Matrix3d calibrateRotatingCamera2(
  std::vector<Eigen::Matrix3d> Hs
)
{
  assert(!Hs.empty());

  for (auto& H : Hs)
  {
    const double det = H.determinant();
    H /= std::pow(det, 1./3.); // dette normaliserer determinanten til en 3x3-matrise
  }

  Eigen::MatrixXd A(7*Hs.size(), 3);

  for (size_t i = 0; i < Hs.size(); ++i)
  {
    const auto& H = Hs[i];

    constexpr auto sq = [](const double v)
    {
      return v*v;
    };

    A.row(7*i + 0) = Eigen::Array3d{sq(H(0, 0)) - 1, sq(H(0, 1)), sq(H(0, 2))};
    A.row(7*i + 1) = Eigen::Array3d{sq(H(1, 0)), sq(H(1, 1)) - 1, sq(H(1, 2))};
    A.row(7*i + 2) = Eigen::Array3d{sq(H(2, 0)), sq(H(2, 1)), 0};
    A.row(7*i + 3) = Eigen::Array3d{sq(H(0, 0)) - sq(H(1, 0)), sq(H(0, 1)) - sq(H(1, 1)), sq(H(0, 2)) - sq(H(1, 2))};
    A.row(7*i + 4) = Eigen::Array3d{H(0, 0)*H(1, 0), H(0, 1)*H(1, 1), H(0, 2)*H(1, 2)};
    A.row(7*i + 5) = Eigen::Array3d{H(0, 0)*H(2, 0), H(0, 1)*H(2, 1), H(0, 2)};
    A.row(7*i + 6) = Eigen::Array3d{H(1, 0)*H(2, 0), H(1, 1)*H(2, 1), H(1, 2)};
  }

  const Eigen::VectorXd wcoef = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).matrixV().col(A.cols() - 1); // Dette svarer til cv::SVD::solveZ, som er å finne unit-length vektoren som minimierer Ax = 0
  const Eigen::Array3d w = wcoef.array().sqrt();

  return (w/w(2)).matrix().asDiagonal();
}

Eigen::Matrix3d calibrateRotatingCamera(
  const std::vector<cv::Mat>& cv_Hs
)
{
  std::vector<Eigen::Matrix3d> Hs;
  Hs.reserve(cv_Hs.size());

  for (const auto& H : cv_Hs)
  {
    Hs.push_back(cvToEigen(H));
  }

  return calibrateRotatingCamera(std::move(Hs));
}

std::vector<ObservationSet> generateObservations(
  const PanTiltProvider& pt_provider,
  const Config& base_config,
  RNG& rng,
  const double img_t_offset,
  const size_t octave,
  const bool measure_timestamps
)
{
  auto config = base_config;
  config.actual_hfov = base_config.actual_hfov*(1 << octave);

  const auto cal = createCalibration(config.actual_hfov, config.actual_k);
  const Eigen::Matrix2d P_px = Eigen::Array2d::Constant(config.sigma_px).square().matrix().asDiagonal();

  std::vector<ObservationSet> all_observations;

  for (size_t k = 0;; ++k)
  {
    const auto t = img_t_offset + config.img_t_begin + k / config.img_rate;

    if (t > img_t_offset + config.img_t_end)
    {
      break;
    }

    const auto [pan_tilt, image_taken] = pt_provider(t, config);

    if (!image_taken)
    {
      continue;
    }

    const auto R_base_from_cam_frd = getOrientation(pan_tilt, config.actual_pan_axis, config.actual_tilt_axis);
    const auto cam = getCamera(cal, R_base_from_cam_frd);

    auto azi_lo = std::numeric_limits<int>::max();
    auto azi_hi = -std::numeric_limits<int>::max();
    auto elev_lo = std::numeric_limits<int>::max();
    auto elev_hi = -std::numeric_limits<int>::max();
    const auto grid_size_rad = config.actual_hfov / 10;
    const auto base_grid_size_rad = base_config.actual_hfov / 10;
    const auto N = static_cast<int>(std::floor(2 * M_PI / base_grid_size_rad));

    constexpr int w = 1920;
    constexpr int h = 1080;

    {
      const std::vector<Eigen::Vector2d> uvs{
        {0, 0},
        {0, h/2},
        {0, h},
        {w/2, 0},
        {w, 0},
        {w, h/2},
        {w, h},
        {w/2, h},
        {w, h},
      };

      for (const auto& uv : uvs)
      {
        const auto p = cam.backprojectPointAtInfinity(uv).point3();

        const auto azi = std::atan2(p.y(), p.x()) + M_PI;
        const auto elev = std::asin(-p.z() / p.norm()) + M_PI / 2;

        azi_lo = std::min(azi_lo, static_cast<int>(std::floor(azi / grid_size_rad))*(1 << octave));
        azi_hi = std::max(azi_hi, static_cast<int>(std::ceil(azi / grid_size_rad))*(1 << octave));
        elev_lo = std::min(elev_lo, static_cast<int>(std::floor(elev / grid_size_rad))*(1 << octave));
        elev_hi = std::max(elev_hi, static_cast<int>(std::ceil(elev / grid_size_rad))*(1 << octave));
      }
    }

    const Eigen::AlignedBox<double, 2> cam_rect(Eigen::Vector2d{0, 0}, Eigen::Vector2d{w, h});

    std::map<size_t, ptc::Gaussian<Eigen::Vector2d>> observations;

    for (auto i = elev_lo; i < elev_hi; i += (1 << octave))
    {
      for (auto j = azi_lo; j < azi_hi; j += (1 << octave))
      {
        constexpr auto r = 1e4;

        const auto azi = base_grid_size_rad * j - M_PI;
        const auto elev = base_grid_size_rad * i - M_PI / 2;

        const Eigen::Vector3d p_frame = {
          r * std::cos(azi) * std::cos(elev),
          r * std::sin(azi) * std::cos(elev),
          -r * std::sin(elev)
        };

        const auto observation = measureObservation(p_frame, cal, t, pt_provider, config, rng);

        if (!cam_rect.contains(observation))
        {
          continue;
        }

        observations[i * N + j] = {
          observation,
          P_px
        };
      }
    }

    const auto measured_timestamp = measure_timestamps
        ? measureTimestamp(rng, t, config.sigma_t_img)
        : ptc::NoisyTimestamp{0, 0};

    all_observations.push_back(
      {
        t,
        measured_timestamp,
        R_base_from_cam_frd,
        std::move(observations)
      }
    );
  }

  return all_observations;
}
}
}
