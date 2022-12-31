// Copyright (c) 2022 Norwegian Defence Research Establishment (FFI)

#include "mc-analysis/simulation.h"

#include "mc-analysis/utils.h"

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
  double mean_hfov
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

PTZEstimator setupEstimator(
  const PanTiltProvider& pt_provider,
  const Config& config,
  RNG& rng
);

Result simulate(
  const PanTiltProvider& pt_provider,
  const Config& config,
  RNG& rng
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
    std::unique_lock lock{mutex};

    cv.wait(
      lock,
      [&num_active_workers, num_threads]()
      {
        return num_active_workers < num_threads;
      }
    );

    ++num_active_workers;

    runs.push_back(
      std::async(
        std::launch::async,
        [&cv, &mutex, &num_active_workers, &num_finished, &num_crashed, l, begin, &seeds, num_iter, &config_provider, &pt_provider]()
        {
          try
          {
            RNG rng{seeds[l]};
            const auto config = config_provider(rng);
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
  const double mean_hfov
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
    Eigen::Array<double, 2, 1>{0.5*mean_f, 1}.square().matrix().asDiagonal()
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

PTZEstimator setupEstimator(
  const PanTiltProvider& pt_provider,
  const Config& config,
  RNG& rng
)
{
  std::uniform_real_distribution<double> img_t_offset_dist(0, 1/config.img_rate);
  const auto img_t_offset = img_t_offset_dist(rng);

  const auto initial_guess_cal = createInitialGuessCalibration(rng, config.actual_hfov);
  const ptc::Gaussian<double> initial_guess_dt{0., Eigen::Matrix<double, 1, 1>{1.}};
  const ptc::Gaussian<double> initial_guess_line_duration{0., Eigen::Matrix<double, 1, 1>{1e-6}};
  const auto initial_guess_pt_scale = Eigen::Vector2d::Ones();

  PTZEstimator estimator{
    initial_guess_cal,
    initial_guess_dt,
    initial_guess_line_duration,
    initial_guess_pt_scale,
    config.pt_scale_sigmas
  };

  const Eigen::Matrix2d P_pan_tilt = config.sigma_angle_rad*config.sigma_angle_rad*Eigen::Matrix2d::Identity();
  const Eigen::Matrix2d P_px = config.sigma_px*config.sigma_px*Eigen::Matrix2d::Identity();

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

  const auto cal = createCalibration(config.actual_hfov, config.actual_k);

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
    const auto N = static_cast<int>(std::floor(2 * M_PI / grid_size_rad));

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

        azi_lo = std::min(azi_lo, static_cast<int>(std::floor(azi / grid_size_rad)));
        azi_hi = std::max(azi_hi, static_cast<int>(std::ceil(azi / grid_size_rad)));
        elev_lo = std::min(elev_lo, static_cast<int>(std::floor(elev / grid_size_rad)));
        elev_hi = std::max(elev_hi, static_cast<int>(std::ceil(elev / grid_size_rad)));
      }
    }

    const Eigen::AlignedBox<double, 2> cam_rect(Eigen::Vector2d{0, 0}, Eigen::Vector2d{w, h});

    std::map<size_t, ptc::Gaussian<Eigen::Vector2d>> observations;

    for (auto i = elev_lo; i < elev_hi; ++i)
    {
      for (auto j = azi_lo; j < azi_hi; ++j)
      {
        constexpr auto r = 1e4;

        const auto azi = grid_size_rad * j - M_PI;
        const auto elev = grid_size_rad * i - M_PI / 2;

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

    const auto measured_timestamp = measureTimestamp(rng, t, config.sigma_t_img);
    estimator.insertObservations({measured_timestamp, measured_dt_img}, observations);
  }

  return estimator;
}

Result simulate(
  const PanTiltProvider& pt_provider,
  const Config& config,
  RNG& rng
)
{
  auto estimator = setupEstimator(pt_provider, config, rng);
  estimator.estimate();

  const auto estimate = estimator.getEstimate();

  const auto cal = createCalibration(config.actual_hfov, config.actual_k);
  Eigen::Vector3d cal_vals = Eigen::Vector3d::Zero();
  cal_vals.head<std::min(3, static_cast<int>(Cal::dimension))>() = cal.vector();

  const auto [mean_pixel_error, max_pixel_err] = getMeanAndMaxPixelError(estimator);
  const auto [pan_std, tilt_std] = getPanTiltStdDev(estimator);

  return Result{
    cal_vals,
    estimate.cal,
    config.actual_dt,
    estimate.dt,
    config.actual_pan_axis,
    estimate.pan_axis,
    config.actual_tilt_axis,
    estimate.tilt_axis,
    config.pt_scale,
    estimate.pt_scale,
    config.actual_line_duration,
    estimate.ld,
    mean_pixel_error,
    max_pixel_err,
    pan_std,
    tilt_std,
    estimate.marginal
  };
}
}
}
