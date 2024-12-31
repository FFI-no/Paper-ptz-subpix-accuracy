# Copyright (c) 2025 Norwegian Defence Research Establishment (FFI)

import numpy as np
from reader import Reader
import sys

if len(sys.argv) < 2:
    print('USAGE: %s <result prefix>' % sys.argv[0])
    sys.exit(1)

path_prefix = sys.argv[1]
paths = (
    path_prefix + '-01deg+32.dat',
    path_prefix + '-02deg+32.dat',
    path_prefix + '-04deg+32.dat',
    path_prefix + '-08deg+32.dat',
    path_prefix + '-16deg+32.dat',
    path_prefix + '-32deg+32.dat',
)


def hfov_from_f(f):
    return 2*np.arctan(1920/(2*f))


def f_from_hfov(hfov):
    return 1920/(2*np.tan(hfov/2))


results_per_f = []

for path in paths:
    reader = Reader()
    reader.read(path)

    true_f = reader.true_cals[0, 0]
    true_fov_deg = hfov_from_f(reader.true_cals[0, 0])/np.pi*180

    ptcee_f_errs = np.abs(reader.true_cals[:, 0] - reader.est_cals[:, 0])
    dlt_f_errs = np.abs(reader.true_cals[:, 0] - reader.dlt_fs)
    single_level_ba_f_errs = np.abs(reader.true_cals[:, 0] - reader.single_level_ba_fs)
    multi_level_ba_f_errs = np.abs(reader.true_cals[:, 0] - reader.multi_level_ba_fs)

    true_fovs = hfov_from_f(reader.true_cals[:, 0])
    ptcee_fov_deg_errs = np.abs(true_fovs - hfov_from_f(reader.est_cals[:, 0]))/np.pi*180
    dlt_fov_deg_errs = np.abs(true_fovs - hfov_from_f(reader.dlt_fs))/np.pi*180
    single_level_ba_fov_deg_errs = np.abs(true_fovs - hfov_from_f(reader.single_level_ba_fs))/np.pi*180
    multi_level_ba_fov_deg_errs = np.abs(true_fovs - hfov_from_f(reader.multi_level_ba_fs))/np.pi*180

    results_per_f.append(
        (
            true_f,
            true_fov_deg,
            ptcee_f_errs,
            dlt_f_errs,
            single_level_ba_f_errs,
            multi_level_ba_f_errs,
            ptcee_fov_deg_errs,
            dlt_fov_deg_errs,
            single_level_ba_fov_deg_errs,
            multi_level_ba_fov_deg_errs,
        )
    )
    
print(r'''
%########################################
%            Table I begin             
%########################################
\begin{tabular}{r|rrrr} 
  \textbf{true}
&
&
&
&
\\
\textbf{\ac{HFOV}}
& \multicolumn{4}{c}{\textbf{Mean absolute error in estimated \ac{HFOV}}}
\\
\hline''')

fov_rows = []

for i, (true_f, true_fov_deg, ptcee_f_errs, dlt_f_errs, single_level_ba_f_errs, multi_level_ba_f_errs, ptcee_fov_deg_errs, dlt_fov_deg_errs, single_level_ba_fov_deg_errs, multi_level_ba_fov_deg_errs) in enumerate(results_per_f):
    ptcee_mean_err = np.mean(ptcee_fov_deg_errs)
    dlt_mean_err = np.mean(dlt_fov_deg_errs[~np.isnan(dlt_f_errs)])
    single_level_ba_mean_err = np.mean(single_level_ba_fov_deg_errs[~np.isnan(single_level_ba_f_errs)])
    multi_level_ba_mean_err = np.mean(multi_level_ba_fov_deg_errs[~np.isnan(multi_level_ba_f_errs)])

    fov_rows.append(' & '.join(
        [
            r'$\mathbf{' + f'{true_fov_deg:.0f}\\degree' + r'}$',
            f'${dlt_mean_err:.3f}\\degree$',
            f'${single_level_ba_mean_err:.3f}\\degree$',
            f'${multi_level_ba_mean_err:.3f}\\degree$',
            f'${ptcee_mean_err:.3f}\\degree$'
        ]
    ))

print(' \\\\\n'.join(fov_rows[::-1]))

print(r'''\\
\hline
& \multicolumn{1}{c}{DLT only}
& \multicolumn{1}{c}{DLT + \ac{BA}}
& \multicolumn{1}{c}{DLT + \ac{BA}}
& \multicolumn{1}{c}{PTCEE}
\\
&
& \multicolumn{1}{c}{single-level}
& \multicolumn{1}{c}{multi-level}
& \multicolumn{1}{c}{(ours)}
\end{tabular}
%########################################
%             Table I end             
%########################################''')
