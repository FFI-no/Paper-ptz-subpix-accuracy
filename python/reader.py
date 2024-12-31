# Copyright (c) 2025 Norwegian Defence Research Establishment (FFI)

import numpy as np


class Reader:
    def read(self, path):
        with open(path, 'r') as f:
            true_cals = []
            est_cals = []
            est_cals_sigmas = []

            true_dts = []
            est_dts = []
            est_dts_sigma = []

            est_pan_axis_errors = []
            est_pan_axis_sigmas = []
            est_pan_axis_sq_mah_dists = []

            est_tilt_axis_errors = []
            est_tilt_axis_sigmas = []
            est_tilt_axis_sq_mah_dists = []

            true_pt_scales = []
            est_pt_scales = []
            est_pt_scales_sigmas = []

            true_lds = []
            est_lds = []
            est_lds_sigma = []

            mean_pixel_errors = []
            max_pixel_errors = []

            pan_stds = []
            tilt_stds = []

            img_rates = []
            pt_rates = []

            sigmas_angle_rad = []
            sigmas_px = []
            sigmas_t_img = []
            sigmas_t_pt = []
            sigmas_dt_img = []
            sigmas_dt_pt = []
            sigmas_pt_scale = []

            est_corr_f_ps = []

            dlt_fs = []
            single_level_ba_fs = []
            multi_level_ba_fs = []

            for l in f:
                true_cals.append(list(map(float, l.strip().split())))

                l = next(f)
                est_cals.append(list(map(float, l.strip().split())))

                l = next(f)
                est_cals_sigmas.append(list(map(float, l.strip().split())))

                l = next(f)
                true_dt, est_dt, est_dt_sigma = map(float, l.strip().split())
                true_dts.append(true_dt)
                est_dts.append(est_dt)
                est_dts_sigma.append(est_dt_sigma)

                l = next(f)
                pan_ax_data = np.array(list(map(float, l.strip().split())))
                est_pan_axis_errors.append(pan_ax_data[:2])
                est_pan_axis_sigmas.append(pan_ax_data[2:4])
                est_pan_axis_sq_mah_dists.append(pan_ax_data[4])

                l = next(f)
                tilt_ax_data = np.array(list(map(float, l.strip().split())))
                est_tilt_axis_errors.append(tilt_ax_data[:2])
                est_tilt_axis_sigmas.append(tilt_ax_data[2:4])
                est_tilt_axis_sq_mah_dists.append(tilt_ax_data[4])

                l = next(f)
                true_pt_scales.append(list(map(float, l.strip().split())))
                l = next(f)
                est_pt_scales.append(list(map(float, l.strip().split())))
                l = next(f)
                est_pt_scales_sigmas.append(list(map(float, l.strip().split())))

                l = next(f)
                true_ld, est_ld, est_ld_sigma = map(float, l.strip().split())
                true_lds.append(true_ld)
                est_lds.append(est_ld)
                est_lds_sigma.append(est_ld_sigma)

                mean_pixel_error, max_pixel_error = map(float, next(f).strip().split())
                mean_pixel_errors.append(mean_pixel_error)
                max_pixel_errors.append(max_pixel_error)

                pan_std, tilt_std = map(float, next(f).strip().split())
                pan_stds.append(pan_std)
                tilt_stds.append(tilt_std)

                img_rate, pt_rate = map(float, next(f).strip().split())
                img_rates.append(img_rate)
                pt_rates.append(pt_rate)

                sigma_angle_rad, sigma_px, sigma_t_img, sigma_t_pt, sigma_dt_img, sigma_dt_pt = (list(map(float, next(f).strip().split())) + [0, 0])[:6]
                sigmas_angle_rad.append(sigma_angle_rad)
                sigmas_px.append(sigma_px)
                sigmas_t_img.append(sigma_t_img)
                sigmas_t_pt.append(sigma_t_pt)
                sigmas_dt_img.append(sigma_dt_img)
                sigmas_dt_pt.append(sigma_dt_pt)

                sigmas_pt_scale.append(list(map(float, next(f).strip().split())))

                est_corr_f_ps.append(float(next(f)))

                dlt_f, single_level_ba_f, multi_level_ba_f = map(float, next(f).strip().split())
                dlt_fs.append(dlt_f)
                single_level_ba_fs.append(single_level_ba_f)
                multi_level_ba_fs.append(multi_level_ba_f)

            valid = (np.array(max_pixel_errors) < 50) & (np.array(mean_pixel_errors) / np.array(sigmas_px) < 1.5)

            self.true_cals = np.array(true_cals)[valid, :]
            self.est_cals = np.array(est_cals)[valid, :]
            self.est_cals_sigmas = np.array(est_cals_sigmas)[valid]
            self.true_dts = np.array(true_dts)[valid]
            self.est_dts = np.array(est_dts)[valid]
            self.est_dts_sigma = np.array(est_dts_sigma)[valid]
            self.est_pan_axis_errors = np.array(est_pan_axis_errors)[valid, :]
            self.est_pan_axis_sigmas = np.array(est_pan_axis_sigmas)[valid, :]
            self.est_pan_axis_sq_mah_dists = np.array(est_pan_axis_sq_mah_dists)[valid]
            self.est_tilt_axis_errors = np.array(est_tilt_axis_errors)[valid, :]
            self.est_tilt_axis_sigmas = np.array(est_tilt_axis_sigmas)[valid, :]
            self.est_tilt_axis_sq_mah_dists = np.array(est_tilt_axis_sq_mah_dists)[valid]
            self.true_pt_scales = np.array(true_pt_scales)[valid, :]
            self.est_pt_scales = np.array(est_pt_scales)[valid, :]
            self.est_pt_scales_sigmas = np.array(est_pt_scales_sigmas)[valid]
            self.true_lds = np.array(true_lds)[valid]
            self.est_lds = np.array(est_lds)[valid]
            self.est_lds_sigma = np.array(est_lds_sigma)[valid]
            self.mean_pixel_errors = np.array(mean_pixel_errors)[valid]
            self.max_pixel_errors = np.array(max_pixel_errors)[valid]
            self.pan_stds = np.array(pan_stds)[valid]
            self.tilt_stds = np.array(tilt_stds)[valid]
            self.img_rates = np.array(img_rates)[valid]
            self.pt_rates = np.array(pt_rates)[valid]
            self.sigmas_angle_rad = np.array(sigmas_angle_rad)[valid]
            self.sigmas_px = np.array(sigmas_px)[valid]
            self.sigmas_t_img = np.array(sigmas_t_img)[valid]
            self.sigmas_t_pt = np.array(sigmas_t_pt)[valid]
            self.sigmas_dt_img = np.array(sigmas_dt_img)[valid]
            self.sigmas_dt_pt = np.array(sigmas_dt_pt)[valid]
            self.sigmas_pt_scale = np.array(sigmas_pt_scale)[valid, :]
            self.est_corr_f_ps = np.array(est_corr_f_ps)[valid]
            self.dlt_fs = np.array(dlt_fs)[valid]
            self.single_level_ba_fs = np.array(single_level_ba_fs)[valid]
            self.multi_level_ba_fs = np.array(multi_level_ba_fs)[valid]
