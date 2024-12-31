# Copyright (c) 2025 Norwegian Defence Research Establishment (FFI)

import matplotlib.pyplot as plt
import numpy as np
from plotting import plothist
import re
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

import matplotlib.pyplot as plt
import matplotlib.gridspec as grid_spec
from sklearn.neighbors import KernelDensity

plt.rcParams['text.usetex'] = True

gs = grid_spec.GridSpec(len(results_per_f), 1)
fig = plt.figure(figsize=(9, 3.6))
fig.set_dpi(300)
colors = ['#0000ff', '#3300cc', '#660099', '#990066', '#cc0033', '#ff0000']

ax_objs = []

for i, (true_f, true_fov_deg, ptcee_f_errs, dlt_f_errs, single_level_ba_f_errs, multi_level_ba_f_errs, ptcee_fov_deg_errs, dlt_fov_deg_errs, single_level_ba_fov_deg_errs, multi_level_ba_fov_deg_errs) in enumerate(results_per_f):
    x_d = np.linspace(-5, np.log10(2), 1000)

    dlt_fov_deg_errs[np.isnan(dlt_fov_deg_errs)] = 100
    single_level_ba_fov_deg_errs[np.isnan(single_level_ba_f_errs)] = 100
    multi_level_ba_fov_deg_errs[np.isnan(multi_level_ba_f_errs)] = 100

    bw = 8e-2
    ptcee_kde = KernelDensity(bandwidth=bw, kernel='gaussian')
    ptcee_kde.fit(np.log10(ptcee_fov_deg_errs[:, None]))
    ptcee_prob = np.exp(ptcee_kde.score_samples(x_d[:, None]))
    print(
        'num nan',
        np.isnan(ptcee_f_errs).sum(),
        np.isnan(dlt_f_errs).sum(),
        np.isnan(single_level_ba_f_errs).sum(),
        np.isnan(multi_level_ba_f_errs).sum(),
    )

    dlt_kde = KernelDensity(bandwidth=bw, kernel='gaussian')
    dlt_kde.fit(np.log10(dlt_fov_deg_errs[:, None]))
    dlt_prob = np.exp(dlt_kde.score_samples(x_d[:, None]))

    single_level_ba_kde = KernelDensity(bandwidth=bw, kernel='gaussian')
    single_level_ba_kde.fit(np.log10(single_level_ba_fov_deg_errs[:, None]))
    single_level_ba_prob = np.exp(single_level_ba_kde.score_samples(x_d[:, None]))

    multi_level_ba_kde = KernelDensity(bandwidth=bw, kernel='gaussian')
    multi_level_ba_kde.fit(np.log10(multi_level_ba_fov_deg_errs[:, None]))
    multi_level_ba_prob = np.exp(multi_level_ba_kde.score_samples(x_d[:, None]))

    ax_objs.append(fig.add_subplot(gs[i:i+1, 0:]))

    lw = 0.2

    ax_objs[-1].fill_between(10**x_d, ptcee_prob, alpha=1, lw=0, color='#1e73ae', zorder=100, label='PTCEE (ours)')
    ax_objs[-1].plot(10**x_d, ptcee_prob, color="#f0f0f0", lw=lw, zorder=100)
    #ax_objs[-1].fill_between(10**x_d, dlt_prob, alpha=1, color='C4', zorder=100, label='DLT')
    #ax_objs[-1].plot(10**x_d, dlt_prob, color="#f0f0f0", lw=lw, zorder=100)
    ax_objs[-1].fill_between(10**x_d, single_level_ba_prob, alpha=1, lw=0, color='#828282', zorder=100, label='DLT + single-level BA')
    ax_objs[-1].plot(10**x_d, single_level_ba_prob, color="#f0f0f0", lw=lw, zorder=100)
    ax_objs[-1].fill_between(10**x_d, multi_level_ba_prob, alpha=1, lw=0, color='#ff881f', zorder=100, label='DLT + multi-level BA')
    ax_objs[-1].plot(10**x_d, multi_level_ba_prob, color="#f0f0f0", lw=lw, zorder=100)
    ax_objs[-1].set_xscale('log')

    ax_objs[-1].set_xlim(3e-5, 2e0)
    ax_objs[-1].set_ylim(0, max(ptcee_prob.max(), single_level_ba_prob.max(), multi_level_ba_prob.max())*1.1)

    rect = ax_objs[-1].patch
    rect.set_alpha(0)
    ax_objs[-1].set_yticks([])

    if i == len(results_per_f)//2 + 1:
        ax_objs[-1].set_ylabel("true HFOV [deg]", labelpad=24)

    if i == len(results_per_f) - 1:
        ax_objs[-1].set_xlabel("estimate HFOV error [deg] (lower is better)")
    else:
        ax_objs[-1].set_xticks([])
        ax_objs[-1].minorticks_off()

    for s in ["top", "right", "left", "bottom"]:
        ax_objs[-1].spines[s].set_visible(False)

    ax_objs[-1].text(0.9*3e-5, 0, f'${true_fov_deg:.0f}' + r'^\circ$', ha="right")

res = re.search(r'.*deg(\+32)?-(.+).dat', paths[0])
ax_objs[-1].legend(loc=4)
gs.update(hspace=-0.4)
plt.gcf().subplots_adjust(bottom=0.13)

plt.tight_layout()
plt.savefig('multi-level-ba-compare.pdf')
plt.show()
