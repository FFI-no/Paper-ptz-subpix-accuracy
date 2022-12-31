# Copyright (c) 2022 Norwegian Defence Research Establishment (FFI)

import sys
import numpy as np

from reader import Reader
from stats import get_r_number


def printTable2RawSection(name, hard_gt, hard_vals, hard_sigmas, soft_gt, soft_vals, soft_sigmas, print_mae=True, decimals=4, scientific=False):

    print(('{:10s}  {:^20s}').format('', name))
    print(('{:10s}  {:20s}').format('', '-'*20))

    value_str = '{:9s} | {:8.%df}  {:5.2f}±{:4.2f}' if not scientific else '{:9s} | {:8.%de}  {:5.2f}±{:4.2f}'

    hard_pred_err = (hard_gt - hard_vals)/hard_sigmas
    soft_pred_err = (soft_gt - soft_vals)/soft_sigmas

    hard_mean_pred_err = hard_pred_err.mean()
    hard_std_pred_err = np.std(hard_pred_err)
    soft_mean_pred_err = soft_pred_err.mean()
    soft_std_pred_err = np.std(soft_pred_err)

    if print_mae:
        hard_mae = np.mean(np.abs(hard_gt - hard_vals))
        soft_mae = np.mean(np.abs(soft_gt - soft_vals))

        print(('{:10s}  {:^8s}  {:^10s}').format('', 'MAE', 'e_sig'))
        print('-'*10 + '+' + '-'*21)
        print((value_str % (decimals,)*1).format('hard beta', hard_mae, hard_mean_pred_err, hard_std_pred_err))
        print((value_str % (decimals,)*1).format('soft beta', soft_mae, soft_mean_pred_err, soft_std_pred_err))
    else:
        hard_mre = np.mean(np.abs(hard_gt - hard_vals)/np.abs(hard_gt))
        soft_mre = np.mean(np.abs(soft_gt - soft_vals)/np.abs(soft_gt))

        print(('{:10s}  {:^8s}  {:^10s}').format('', 'MRE', 'e_sig'))
        print('-'*10 + '+' + '-'*21)
        print((value_str % (decimals,)*1).format('hard beta', hard_mre, hard_mean_pred_err, hard_std_pred_err))
        print((value_str % (decimals,)*1).format('soft beta', soft_mre, soft_mean_pred_err, soft_std_pred_err))


def printTable2RawAxisSection(name, hard_err, hard_sigmas, soft_err, soft_sigmas):

    print(('{:10s}  {:^16s}').format('', name))
    print(('{:10s}  {:16s}').format('', '-'*16))

    value_str = '{:9s} | {:5.2f}  {:5.2f}'

    hard_mae = np.mean(np.abs(hard_err))
    soft_mae = np.mean(np.abs(soft_err))

    hard_mean_sigmas = np.mean(np.abs(hard_sigmas))
    soft_mean_sigmas = np.mean(np.abs(soft_sigmas))

    print(('{:10s}  {:^8s}  {:^6s}').format('', 'MAE', 'sig'))
    print('-'*10 + '+' + '-'*17)
    print((value_str).format('hard beta', hard_mae, hard_mean_sigmas))
    print((value_str).format('soft beta', soft_mae, soft_mean_sigmas))


def printTable2Raw(hard_reader, soft_reader):
    print('='*32)
    print('{:^32s}'.format('Table II'))
    print('='*32)
    printTable2RawSection(
        'f [px]',
        hard_reader.true_cals[:, 0], hard_reader.est_cals[:, 0], hard_reader.est_cals_sigmas[:, 0],
        soft_reader.true_cals[:, 0], soft_reader.est_cals[:, 0], soft_reader.est_cals_sigmas[:, 0],
        print_mae=False,
        decimals=2,
        scientific=True
    )
    print()
    printTable2RawSection(
        'k',
        hard_reader.true_cals[:, 1], hard_reader.est_cals[:, 1], hard_reader.est_cals_sigmas[:, 1],
        soft_reader.true_cals[:, 1], soft_reader.est_cals[:, 1], soft_reader.est_cals_sigmas[:, 1],
        decimals=2,
        scientific=True
    )
    print()
    printTable2RawSection(
        'd [ms]',
        1e3*hard_reader.true_dts, 1e3*hard_reader.est_dts, 1e3*hard_reader.est_dts_sigma,
        1e3*soft_reader.true_dts, 1e3*soft_reader.est_dts, 1e3*soft_reader.est_dts_sigma,
        decimals=3
    )
    print()
    printTable2RawSection(
        'ell [ns]',
        1e9*hard_reader.true_lds, 1e9*hard_reader.est_lds, 1e9*hard_reader.est_lds_sigma,
        1e9*soft_reader.true_lds, 1e9*soft_reader.est_lds, 1e9*soft_reader.est_lds_sigma,
        decimals=3
    )
    print()
    printTable2RawSection(
        'beta_phi',
        hard_reader.true_pt_scales[:, 0], hard_reader.est_pt_scales[:, 0], hard_reader.est_pt_scales_sigmas[:, 0],
        soft_reader.true_pt_scales[:, 0], soft_reader.est_pt_scales[:, 0], soft_reader.est_pt_scales_sigmas[:, 0],
        decimals=1,
        scientific=True
    )
    print()
    printTable2RawSection(
        'beta_psi',
        hard_reader.true_pt_scales[:, 1], hard_reader.est_pt_scales[:, 1], hard_reader.est_pt_scales_sigmas[:, 1],
        soft_reader.true_pt_scales[:, 1], soft_reader.est_pt_scales[:, 1], soft_reader.est_pt_scales_sigmas[:, 1],
        decimals=1,
        scientific=True
    )
    print()
    printTable2RawAxisSection(
        'a_phi [mrad]',
        1e3*hard_reader.est_pan_axis_errors, 1e3*hard_reader.est_pan_axis_sigmas,
        1e3*soft_reader.est_pan_axis_errors, 1e3*soft_reader.est_pan_axis_sigmas
    )
    print()
    printTable2RawAxisSection(
        'a_psi [mrad]',
        1e3*hard_reader.est_tilt_axis_errors, 1e3*hard_reader.est_tilt_axis_sigmas,
        1e3*soft_reader.est_tilt_axis_errors, 1e3*soft_reader.est_tilt_axis_sigmas
    )

def printTexMRE(gt, est, decimals=2, scientific=False):
    mre = np.mean(np.abs(gt - est)/np.abs(gt))

    if scientific:
        exponent = int(np.floor(np.log10(mre)))
        digits = mre/10**exponent
        print(r'& $\expnumber{'
              + ('{:.%df}' % decimals).format(digits)
              + '}{'
              + '{:d}'.format(exponent)
              + '}$')
    else:
        print((r'& ${:.%df}$' % decimals).format(mre))

def printTexMAE(gt, est, decimals=2, scientific=False):
    mae = np.mean(np.abs(gt - est))

    if scientific:
        exponent = int(np.floor(np.log10(mae)))
        digits = mae/10**exponent
        print(r'& $\expnumber{'
              + ('{:.%df}' % decimals).format(digits)
              + '}{'
              + '{:d}'.format(exponent)
              + '}$')
    else:
        print((r'& ${:.%df}$' % decimals).format(mae))

def printTexAxisMAE(err, decimals=2):
    mae = np.mean(np.abs(err))
    print((r'& ${:.%df}$' % decimals).format(mae))

def printTexAxisSigma(sigmas, decimals=2):
    mae = np.mean(np.abs(sigmas))
    print((r'& ${:.%df}$' % decimals).format(mae))

def printTexPred(gt, est, sigmas, decimals=2):
    pred_err = (gt - est)/sigmas
    mu = pred_err.mean()
    sigma = np.std(pred_err)

    print((r'& ${:.%df} \pm {:.%df}$' % (decimals, decimals)).format(mu, sigma))

def printTable2Tex(hard_reader, soft_reader):
    print('%' + '#'*40)
    print('%' +'{:^40s}'.format('Table II begin'))
    print('%' +'#'*40)
    print(r'\begin{tabular}{@{\extracolsep{4pt}}rcccccccc}')
    print(r'''& \multicolumn{2}{c}{$f$}
& \multicolumn{2}{c}{$k$}
& \multicolumn{2}{c}{$d$}
& \multicolumn{2}{c}{$\ell$}
\\ 
\cline{2-3}
\cline{4-5}
\cline{6-7}
\cline{8-9}
\rule{0pt}{12pt}
\rule[-6pt]{0pt}{0pt}
& \textbf{MRE}
& \textbf{Mean} $e^{\widehat{\sigma}}$
& \textbf{MAE}
& \textbf{Mean} $e^{\widehat{\sigma}}$
& \textbf{MAE}$\, \mathrm{[ms]}$
& \textbf{Mean} $e^{\widehat{\sigma}}$
& \textbf{MAE}$\, \mathrm{[ns]}$
& \textbf{Mean} $e^{\widehat{\sigma}}$
\\
\hline''')

    print(r'Hard $\beta$')
    # f
    printTexMRE(hard_reader.true_cals[:, 0], hard_reader.est_cals[:, 0], scientific=True)
    printTexPred(hard_reader.true_cals[:, 0], hard_reader.est_cals[:, 0], hard_reader.est_cals_sigmas[:, 0])

    # k
    printTexMAE(hard_reader.true_cals[:, 1], hard_reader.est_cals[:, 1], scientific=True)
    printTexPred(hard_reader.true_cals[:, 1], hard_reader.est_cals[:, 1], hard_reader.est_cals_sigmas[:, 1])

    # d [ms]
    printTexMAE(1e3*hard_reader.true_dts, 1e3*hard_reader.est_dts, decimals=3)
    printTexPred(hard_reader.true_dts, hard_reader.est_dts, hard_reader.est_dts_sigma)

    # ell [ns]
    printTexMAE(1e9*hard_reader.true_lds, 1e9*hard_reader.est_lds)
    printTexPred(hard_reader.true_lds, hard_reader.est_lds, hard_reader.est_lds_sigma)
    print(r'\\')

    print(r'Soft $\beta$')
    # f
    printTexMRE(soft_reader.true_cals[:, 0], soft_reader.est_cals[:, 0], scientific=True)
    printTexPred(soft_reader.true_cals[:, 0], soft_reader.est_cals[:, 0], soft_reader.est_cals_sigmas[:, 0])

    # k
    printTexMAE(soft_reader.true_cals[:, 1], soft_reader.est_cals[:, 1], scientific=True)
    printTexPred(soft_reader.true_cals[:, 1], soft_reader.est_cals[:, 1], soft_reader.est_cals_sigmas[:, 1])

    # d [ms]
    printTexMAE(1e3*soft_reader.true_dts, 1e3*soft_reader.est_dts, decimals=3)
    printTexPred(soft_reader.true_dts, soft_reader.est_dts, soft_reader.est_dts_sigma)

    # ell [ns]
    printTexMAE(1e9*soft_reader.true_lds, 1e9*soft_reader.est_lds)
    printTexPred(soft_reader.true_lds, soft_reader.est_lds, soft_reader.est_lds_sigma)
    print(r'\\')

    print(r'''& \multicolumn{2}{c}{$\beta_\phi$}
& \multicolumn{2}{c}{$\beta_\psi$}
& \multicolumn{2}{c}{$\veca_\phi*\, \mathrm{[mrad]}$}
& \multicolumn{2}{c}{$\veca_\psi*\, \mathrm{[mrad]}$}
\\ 
\cline{2-3}
\cline{4-5}
\cline{6-7}
\cline{8-9}
\rule{0pt}{12pt}
\rule[-6pt]{0pt}{0pt}
& \textbf{MAE}
& \textbf{Mean} $e^{\widehat{\sigma}}$
& \textbf{MAE}
& \textbf{Mean} $e^{\widehat{\sigma}}$
& \textbf{MAE}
& \textbf{Mean} $\widehat{\sigma}$
& \textbf{MAE}
& \textbf{Mean} $\widehat{\sigma}$
\\
\hline''')

    print(r'Hard $\beta$')
    print(r'& $-$')
    print(r'& $-$')
    print(r'& $-$')
    print(r'& $-$')

    # a_phi [mrad]
    printTexAxisMAE(1e3*hard_reader.est_pan_axis_errors)
    printTexAxisSigma(1e3*hard_reader.est_pan_axis_sigmas)

    # a_psi [mrad]
    printTexAxisMAE(1e3*hard_reader.est_tilt_axis_errors)
    printTexAxisSigma(1e3*hard_reader.est_tilt_axis_sigmas)
    print(r'\\')

    print(r'Soft $\beta$')

    # beta_phi
    printTexMAE(soft_reader.true_pt_scales[:, 0], soft_reader.est_pt_scales[:, 0], scientific=True)
    printTexPred(soft_reader.true_pt_scales[:, 0], soft_reader.est_pt_scales[:, 0], soft_reader.est_pt_scales_sigmas[:, 0])

    # beta_psi
    printTexMAE(soft_reader.true_pt_scales[:, 1], soft_reader.est_pt_scales[:, 1], scientific=True)
    printTexPred(soft_reader.true_pt_scales[:, 1], soft_reader.est_pt_scales[:, 1], soft_reader.est_pt_scales_sigmas[:, 1])

    # a_phi [mrad]
    printTexAxisMAE(1e3*soft_reader.est_pan_axis_errors)
    printTexAxisSigma(1e3*soft_reader.est_pan_axis_sigmas)

    # a_psi [mrad]
    printTexAxisMAE(1e3*soft_reader.est_tilt_axis_errors)
    printTexAxisSigma(1e3*soft_reader.est_tilt_axis_sigmas)

    print(r'\end{tabular}')
    print('%' + '#'*40)
    print('%' +'{:^40s}'.format('Table II end'))
    print('%' +'#'*40)


def printTable3Raw(reader):
    print('='*43)
    print('{:^43s}'.format('Table III'))
    print('='*43)

    frelerr = np.abs(reader.true_cals[:, 0] - reader.est_cals[:, 0])/reader.true_cals[:, 0]
    kabserr = np.abs(reader.true_cals[:, 1] - reader.est_cals[:, 1])
    dabserr = np.abs(reader.true_dts - reader.est_dts)
    labserr = np.abs(reader.true_lds - reader.est_lds)

    lhs_names = ('e_proj', 'f_RE', 'k_AE', 'd_AE', 'ell_AE')
    lhs = np.column_stack((
        reader.mean_pixel_errors,
        frelerr,
        kabserr,
        dabserr,
        labserr
    ))
    rhs_names = ('f', 's_pt', 's_px', 's_tim', 's_tpt')
    rhs = np.column_stack((
        reader.true_cals[:, 0],
        reader.sigmas_angle_rad,
        reader.sigmas_px,
        reader.sigmas_t_img,
        reader.sigmas_t_pt
    ))

    lhs_width = max(map(len, lhs_names))

    print(' '*(lhs_width + 2) + ''.join('{:^7s}'.format(name) for name in rhs_names))
    print('-'*(lhs_width + 1) + '+' + '-'*7*len(rhs_names))

    for i, name in zip(range(len(lhs_names)), lhs_names):
        print(('{:>%is} |' % lhs_width).format(name), end='')

        for j in range(len(rhs_names)):
            r = get_r_number(lhs[:, i], rhs[:, j])
            print(' % 2.3f' % r, end='')
        print('')

def printTable3Tex(reader):
    print('%' + '#'*40)
    print('%' +'{:^40s}'.format('Table III begin'))
    print('%' + '#'*40)

    print(r'''\begin{tabular}{r|rrrrr} 
& \multicolumn{1}{c}{$f$}
& \multicolumn{1}{c}{$\sigma_{\mathrm{pt}}$}
& \multicolumn{1}{c}{$\sigma_{\mathrm{px}}$}
& \multicolumn{1}{c}{$\sigma_{\mathrm{t}}^{\mathrm{(img)}}$}
& \multicolumn{1}{c}{$\sigma_{\mathrm{t}}^{\mathrm{(pt)}}$}
\\
\hline''')

    frelerr = np.abs(reader.true_cals[:, 0] - reader.est_cals[:, 0])/reader.true_cals[:, 0]
    kabserr = np.abs(reader.true_cals[:, 1] - reader.est_cals[:, 1])
    dabserr = np.abs(reader.true_dts - reader.est_dts)
    labserr = np.abs(reader.true_lds - reader.est_lds)

    lhs_names = (
        r'$\widehat{e}^{\mathrm{(proj)}}$',
        r'$\frac{|f - \widehat{f}|}{f}$',
        r'$|k - \widehat{k}|$',
        r'$|d - \widehat{d}|$',
        r'$|\ell - \widehat{\ell}|$'
    )
    lhs = np.column_stack((
        reader.mean_pixel_errors,
        frelerr,
        kabserr,
        dabserr,
        labserr
    ))
    rhs = np.column_stack((
        reader.true_cals[:, 0],
        reader.sigmas_angle_rad,
        reader.sigmas_px,
        reader.sigmas_t_img,
        reader.sigmas_t_pt
    ))

    lhs_width = max(map(len, lhs_names))

    for i, name in zip(range(len(lhs_names)), lhs_names):
        if i > 0:
            print(r'\\')

        print(name)

        for j in range(rhs.shape[1]):
            r = get_r_number(lhs[:, i], rhs[:, j])

            if r > 0.1:
                print(r'& $\textbf{' + '{:.3f}'.format(r) + '}$')
            else:
                print('& ${:.3f}$'.format(r))

    print(r'\end{tabular}')
    print('%' +'#'*40)
    print('%' +'{:^40s}'.format('Table III end'))
    print('%' +'#'*40)

def getNMEPE(reader):
    return np.mean(reader.mean_pixel_errors/reader.sigmas_px)

if len(sys.argv) < 3:
    print('USAGE: %s <path to hard beta results> <path to soft beta results>' % sys.argv[0])
    sys.exit(1)

hard_beta_path = sys.argv[1]
soft_beta_path = sys.argv[2]

print_tex = len(sys.argv) >= 4 and sys.argv[3] == '--latex'

hard_beta_reader = Reader()
hard_beta_reader.read(hard_beta_path)
soft_beta_reader = Reader()
soft_beta_reader.read(soft_beta_path)

if print_tex:
    printTable2Tex(hard_beta_reader, soft_beta_reader)
    print()
    printTable3Tex(hard_beta_reader)
    print()
    print('% Hard NMEPE: {:.2f}'.format(getNMEPE(hard_beta_reader)))
    print('% Soft NMEPE: {:.2f}'.format(getNMEPE(soft_beta_reader)))
else:
    printTable2Raw(hard_beta_reader, soft_beta_reader)
    print()
    printTable3Raw(hard_beta_reader)

    print('Hard NMEPE: {:.2f}'.format(getNMEPE(hard_beta_reader)))
    print('Soft NMEPE: {:.2f}'.format(getNMEPE(soft_beta_reader)))
