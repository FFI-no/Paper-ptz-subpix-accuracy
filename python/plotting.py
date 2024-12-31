# Copyright (c) 2025 Norwegian Defence Research Establishment (FFI)

import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
import matplotlib.transforms as transforms

import numpy as np
from scipy.stats import norm

def plothist(true_vals, est_vals, est_sigmas, name, show=True, filename=None):
    mah_dists = (true_vals - est_vals)/est_sigmas
    valid = np.abs(mah_dists) < 4

    if valid.sum()/len(mah_dists) < 0.99:
        raise Exception('Clipped more than 1% of hist')

    mah_dists = mah_dists[valid]

    v = np.linspace(-4, 4)
    n, bins, patches = plt.hist(mah_dists, bins=30, density=True, histtype='stepfilled', color='#1e73ae', label='estimate')
    plt.axvline(x=0, linestyle='dotted', color='#e05253', linewidth=4, label='ground truth')
    plt.axvline(x=mah_dists.mean(), linestyle='-', color='C2', linewidth=4, label='mean')
    plt.plot(v, norm.pdf(v, mah_dists.mean(), 1), color='#ff881f', linewidth=4, label='predicted')
    plt.xlim(-4, 4)
    plt.gca().axes.get_yaxis().set_ticks([])
    plt.ylabel('density')

    plt.legend()

    if filename is not None:
        plt.gcf().set_size_inches(6, 2)
        plt.savefig(filename, dpi=200, bbox_inches='tight')

    if show:
        plt.title(name)
        plt.show()
    plt.clf()

def ploterrhist(est_vals, true_val, pred_sigma, show=True, filename=None):
    plt.rc('font', size=12)
    plt.hist(est_vals, bins=30, density=True, histtype='stepfilled', color='C0', alpha=0.7)

    mean = est_vals.mean()
    num_sig = 3.5
    v = np.linspace(mean - num_sig*pred_sigma, mean + num_sig*pred_sigma)

    plt.fill_between(v, norm.pdf(v, est_vals.mean(), pred_sigma), alpha=0.5, color='C1', linewidth=4)
    plt.axvline(x=true_val, linestyle='dotted', color='C3', linewidth=4, label='ground truth')
    plt.axvline(x=mean, linestyle='-', color='C2', linewidth=4, label='mean')
    plt.plot(v, norm.pdf(v, est_vals.mean(), pred_sigma), color='C1', linewidth=4, label='predicted')

    plt.xlim(v[0], v[-1])
    plt.gca().axes.get_yaxis().set_ticks([])
    plt.ylabel('density')

    plt.legend()

    if filename is not None:
        plt.gcf().set_size_inches(8, 2.5)
        plt.savefig(filename, dpi=200, bbox_inches='tight')

    if show:
        plt.show()

    plt.clf()


def plotcovellipse(mean, cov, ax, n_std=3.0, facecolor='none', **kwargs):
    pearson = cov[0, 1]/np.sqrt(cov[0, 0] * cov[1, 1])

    ell_radius_x = np.sqrt(1 + pearson)
    ell_radius_y = np.sqrt(1 - pearson)
    ellipse = Ellipse((0, 0), width=ell_radius_x * 2, height=ell_radius_y * 2,
                      facecolor=facecolor, **kwargs)

    scale_x = np.sqrt(cov[0, 0]) * n_std
    scale_y = np.sqrt(cov[1, 1]) * n_std

    transf = transforms.Affine2D() \
        .rotate_deg(45) \
        .scale(scale_x, scale_y) \
        .translate(mean[0], mean[1])

    ellipse.set_transform(transf + ax.transData)
    return ax.add_patch(ellipse)
