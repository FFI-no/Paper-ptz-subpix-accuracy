# Copyright (c) 2025 Norwegian Defence Research Establishment (FFI)

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

    n, bins, patches = plt.hist(normerr, bins=30, density=True, histtype='stepfilled', color='#1e73ae', label='estimate')
    plt.axvline(x=(reader.mean_pixel_errors/reader.sigmas_px).mean(), linestyle='-', color='C2', linewidth=4, label='mean')
    plt.legend()
    plt.gcf().set_size_inches(6, 2)
    plt.gca().axes.get_yaxis().set_ticks([])
    plt.xlabel(r'$\widehat{e}^{\mathrm{(proj)}} / \sigma_{\mathrm{px}}$')
    plt.ylabel('density')
    plt.savefig(path, dpi=200, bbox_inches='tight')

    if show:
        plt.show()


def plotrelferr(reader, path, show=False):
    plt.clf()
    f_rel_err = np.abs(reader.true_cals[:, 0] - reader.est_cals[:, 0])/reader.true_cals[:, 0]
    plt.semilogy(reader.true_cals[:, 0], f_rel_err, 'C0x', label=r'$\|f - \widehat{f}\|\, /\, f$')
    plt.semilogy(reader.true_cals[:, 0], reader.est_cals_sigmas[:, 0]/reader.true_cals[:, 0], 'C1x', label=r'$\widehat{\sigma}_f\, /\, f$')
    f7 = 1920/(2*np.tan(7/180*np.pi/2))
    plt.semilogy([f7, f7], [f_rel_err.min(), f_rel_err.max()], 'C2', linewidth=3.34, label=r'hfov = $7\degree$')
    plt.xlim(0, reader.true_cals[:, 0].max())
    plt.ylim(f_rel_err.min(), f_rel_err.max())
    plt.xlabel(r'$f\, \mathrm{[px]}$')
    plt.legend(loc='lower right')
    plt.gcf().set_size_inches(5, 2.5)
    plt.savefig(path, dpi=350, bbox_inches='tight')

    if show:
        plt.show()


if len(sys.argv) < 3:
    print('USAGE: %s <path to base ptcee results> <path to soft ptcee results>' % sys.argv[0])
    sys.exit(1)


show = len(sys.argv) >= 4 and sys.argv[3] == '--show'

base_ptcee_path = sys.argv[1]
soft_ptcee_path = sys.argv[2]

base_ptcee_reader = Reader()
base_ptcee_reader.read(base_ptcee_path)
soft_ptcee_reader = Reader()
soft_ptcee_reader.read(soft_ptcee_path)

plothist(base_ptcee_reader.true_cals[:, 0], base_ptcee_reader.est_cals[:, 0], base_ptcee_reader.est_cals_sigmas[:, 0], 'f', filename='base-ptcee-f-hist.pdf', show=show)
plothist(base_ptcee_reader.true_cals[:, 1], base_ptcee_reader.est_cals[:, 1], base_ptcee_reader.est_cals_sigmas[:, 1], 'k', filename='base-ptcee-k-hist.pdf', show=show)
plothist(base_ptcee_reader.true_dts, base_ptcee_reader.est_dts, base_ptcee_reader.est_dts_sigma, 'd', filename='base-ptcee-d-hist.pdf', show=show)
plothist(base_ptcee_reader.true_lds, base_ptcee_reader.est_lds, base_ptcee_reader.est_lds_sigma, r'$\ell$', filename='base-ptcee-ell-hist.pdf', show=show)

plotmepehist(base_ptcee_reader, 'base-ptcee-mepe-hist.pdf', show=show)

plotrelferr(soft_ptcee_reader, 'soft-ptcee-rel-f-err.png', show=show)
