# Copyright (c) 2022 Norwegian Defence Research Establishment (FFI)

import sys
import matplotlib.pyplot as plt
import numpy as np
from plotting import plothist
from reader import Reader

def plotmepehist(reader, path, show=False):
    plt.clf()
    normerr = reader.mean_pixel_errors/reader.sigmas_px

    # ignore 0.1% outliers
    p = 0.0005
    normerr = normerr[(normerr > np.percentile(normerr, 100*p)) & (normerr < np.percentile(normerr, 100*(1 - p)))]

    n, bins, patches = plt.hist(normerr, bins=30, density=True, histtype='stepfilled', color='C0', alpha=0.7)
    plt.axvline(x=(reader.mean_pixel_errors/reader.sigmas_px).mean(), linestyle='-', color='C2', linewidth=4, label='mean')
    plt.legend()
    plt.gcf().set_size_inches(8, 2.5)
    plt.gca().axes.get_yaxis().set_ticks([])
    plt.ylabel('density')
    plt.savefig(path, dpi=200, bbox_inches='tight')

    if show:
        plt.show()


def plotrelferr(reader, path, show=False):
    plt.clf()
    f_rel_err = np.abs(reader.true_cals[:, 0] - reader.est_cals[:, 0])/reader.true_cals[:, 0]
    plt.semilogy(reader.true_cals[:, 0], f_rel_err, 'C0x', label=r'$\frac{\|f - \widehat{f}\|}{f}$')
    plt.semilogy(reader.true_cals[:, 0], reader.est_cals_sigmas[:, 0]/reader.true_cals[:, 0], 'C1x', label=r'$\frac{\widehat{\sigma}_f}{f}$')
    f7 = 1920/(2*np.tan(7/180*np.pi/2))
    plt.semilogy([f7, f7], [f_rel_err.min(), f_rel_err.max()], 'C2', label=r'hfov = $7\degree$')
    plt.xlabel(r'$f\, \mathrm{[px]}$')
    plt.legend()
    plt.gcf().set_size_inches(8, 3)
    plt.savefig(path, dpi=200, bbox_inches='tight')

    if show:
        plt.show()


if len(sys.argv) < 3:
    print('USAGE: %s <path to hard beta results> <path to soft beta results>' % sys.argv[0])
    sys.exit(1)


show = len(sys.argv) >= 4 and sys.argv[3] == '--show'

hard_beta_path = sys.argv[1]
soft_beta_path = sys.argv[2]

hard_beta_reader = Reader()
hard_beta_reader.read(hard_beta_path)
soft_beta_reader = Reader()
soft_beta_reader.read(soft_beta_path)

plothist(hard_beta_reader.true_cals[:, 0], hard_beta_reader.est_cals[:, 0], hard_beta_reader.est_cals_sigmas[:, 0], 'f', filename='hard-beta-f-hist.pdf', show=show)
plothist(hard_beta_reader.true_cals[:, 1], hard_beta_reader.est_cals[:, 1], hard_beta_reader.est_cals_sigmas[:, 1], 'k', filename='hard-beta-k-hist.pdf', show=show)
plothist(hard_beta_reader.true_dts, hard_beta_reader.est_dts, hard_beta_reader.est_dts_sigma, 'd', filename='hard-beta-d-hist.pdf', show=show)
plothist(hard_beta_reader.true_lds, hard_beta_reader.est_lds, hard_beta_reader.est_lds_sigma, '$\ell$', filename='hard-beta-ell-hist.pdf', show=show)

plotmepehist(hard_beta_reader, 'hard-beta-mepe-hist.pdf', show=show)

plotrelferr(soft_beta_reader, 'soft-beta-rel-f-err.pdf', show=show)
