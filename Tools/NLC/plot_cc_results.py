#!/usr/bin/env python3
"""
Figuras de comparación: controlador por defecto de ArduCopter (P+PID) contra un
custom controller (AC_CustomControl), a partir de los CSV que genera
Tools/NLC/sitl_rms_test.py.

No depende del algoritmo: sirve para cualquier backend de AC_CustomControl
mientras la corrida tenga los mismos CSV. Los prefijos y etiquetas se pasan
por línea de comandos.

Uso:
    python3 Tools/NLC/plot_cc_results.py /tmp/nlc_sitl_20261022_202821
    python3 Tools/NLC/plot_cc_results.py RUN_DIR --custom-label "Mi algoritmo" --format pdf

Entradas (en RUN_DIR):
    <B>_rate.csv, <C>_rate.csv           tasa medida, deseada y salida de control
    <B>_attitude.csv, <C>_attitude.csv   actitud medida y deseada
    <B>_ccmo.csv, <C>_ccmo.csv           cadena por motor: Thr, Out, PWM
    segments.csv                         inicio de cada maniobra (opcional)
    params.csv                           parámetros del vuelo (opcional)
    (B = --baseline-prefix, C = --custom-prefix)

Salidas (en RUN_DIR/figs o --out):
    01_seguimiento_tasa       tasa deseada vs medida, por eje y controlador
    02_error_tasa             error de tasa en el tiempo, ambos controladores
    03_resumen_rms            RMS del error de tasa y de actitud, % de mejora
    04_respuesta_maniobra     error alineado al inicio de cada maniobra
    05_asentamiento           tiempo de asentamiento y pico de error por maniobra
    06_esfuerzo_control       salida de control y su rugosidad (chattering)
    07_linealidad_ccmo        Out vs Thr por motor contra la recta teórica
    08_reversibilidad_ccmo    residuos de Out = f(Thr), de su inversa y del PWM
    09_pwm_motores            PWM de cada motor en el tiempo
    metricas.csv              todas las métricas numéricas de las figuras
"""

import argparse
import csv
import math
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402

AXES = (('roll', 'Roll'), ('pitch', 'Pitch'), ('yaw', 'Yaw'))

# paleta categórica validada (orden fijo): 1 = controlador base, 2 = custom
C_BASE = '#2a78d6'
C_CUSTOM = '#eb6834'
C_MOTORS = ('#2a78d6', '#eb6834', '#1baf7a', '#eda100')
C_REF = '#52514e'      # referencia / teórico (tinta secundaria)
C_GRID = '#e4e3dd'

plt.rcParams.update({
    'figure.dpi': 110,
    'savefig.dpi': 200,
    'font.size': 10,
    'axes.titlesize': 11,
    'axes.labelsize': 10,
    'axes.spines.top': False,
    'axes.spines.right': False,
    'axes.edgecolor': '#8a8984',
    'axes.grid': True,
    'grid.color': C_GRID,
    'grid.linewidth': 0.6,
    'legend.frameon': False,
    'lines.linewidth': 1.2,
})


# ----------------------------------------------------------------- datos
def load(run_dir, prefix, name):
    path = os.path.join(run_dir, '%s_%s.csv' % (prefix, name))
    if not os.path.exists(path):
        return None
    df = pd.read_csv(path)
    return df if len(df) else None


def load_params(run_dir):
    path = os.path.join(run_dir, 'params.csv')
    if not os.path.exists(path):
        return {}
    return dict(pd.read_csv(path).values)


def load_segments(run_dir, prefix):
    path = os.path.join(run_dir, 'segments.csv')
    if not os.path.exists(path):
        return None
    seg = pd.read_csv(path)
    seg = seg[seg.phase == prefix].sort_values('index').reset_index(drop=True)
    return seg if len(seg) else None


def rms(x):
    x = np.asarray(x, dtype=float)
    x = x[np.isfinite(x)]
    return float(np.sqrt(np.mean(x * x))) if len(x) else float('nan')


def rate_error(df, ax):
    return df['des_%s_deg_s' % ax] - df['%s_deg_s' % ax]


def improvement(base, custom):
    """% de mejora del custom sobre la base (positivo = el custom es mejor)"""
    return 100.0 * (base - custom) / base if base else float('nan')


# --------------------------------------------------------------- maniobras
MANEUVER_NAMES = {('roll', +1): 'roll +', ('roll', -1): 'roll −',
                  ('pitch', +1): 'pitch +', ('pitch', -1): 'pitch −',
                  ('yaw', +1): 'yaw +', ('yaw', -1): 'yaw −'}


def maneuvers(rate, seg, ref_window=(0.25, 0.05), search=(0.2, 0.6), thr=2.0):
    """Detecta el inicio real de cada maniobra.

    segments.csv da el instante en que cambió la palanca con ~0.1 s de
    resolución. Se refina buscando el primer instante, en una ventana alrededor
    de esa marca, en que la tasa deseada se aparta más de `thr` °/s de su valor
    previo. Devuelve una lista de dicts: axis, name, t0, t_end, step (cambio de
    PWM de palanca, con signo).
    """
    out = []
    if seg is None:
        return out
    t = rate.time_s.values
    cols = {'roll': 'roll_pwm', 'pitch': 'pitch_pwm', 'yaw': 'yaw_pwm'}
    for i in range(1, len(seg)):
        prev, cur = seg.iloc[i - 1], seg.iloc[i]
        t_mark = cur.t_start_s
        t_end = seg.iloc[i + 1].t_start_s if i + 1 < len(seg) else t[-1]
        for ax, col in cols.items():
            step = int(cur[col] - prev[col])
            if step == 0:
                continue
            des = rate['des_%s_deg_s' % ax].values
            ref = (t >= t_mark - ref_window[0]) & (t <= t_mark - ref_window[1])
            base = np.median(des[ref]) if ref.any() else des[np.searchsorted(t, t_mark)]
            win = np.where((t >= t_mark - search[0]) & (t <= t_mark + search[1])
                           & (np.abs(des - base) > thr))[0]
            t0 = t[win[0]] if len(win) else t_mark
            if cur[col] == 1500:
                name = '%s → 0' % ax
            else:
                name = MANEUVER_NAMES[(ax, 1 if cur[col] > 1500 else -1)]
            out.append(dict(axis=ax, name=name, t0=t0, t_end=t_end, step=step, index=i))
    return out


def settle(rate, man, band):
    """pico de |error|, IAE y tiempo de asentamiento de una maniobra.

    Tiempo de asentamiento = desde el inicio de la maniobra hasta la última vez
    que |error| supera `band` antes de la siguiente maniobra. NaN si al final
    del tramo el error sigue fuera de la banda (no asentó).
    """
    t = rate.time_s.values
    e = np.abs(rate_error(rate, man['axis']).values)
    sel = (t >= man['t0']) & (t < man['t_end'])
    ts, es = t[sel], e[sel]
    if len(ts) < 2:
        return float('nan'), float('nan'), float('nan')
    peak = float(es.max())
    iae = float(np.trapezoid(es, ts)) if hasattr(np, 'trapezoid') else float(np.trapz(es, ts))
    outside = np.where(es > band)[0]
    if len(outside) == 0:
        t_set = 0.0
    elif outside[-1] == len(es) - 1:
        t_set = float('nan')
    else:
        t_set = float(ts[outside[-1] + 1] - man['t0'])
    return peak, iae, t_set


# --------------------------------------------------------------- utilidades
def save(fig, out_dir, name, fmt):
    paths = []
    for f in fmt:
        p = os.path.join(out_dir, '%s.%s' % (name, f))
        fig.savefig(p, bbox_inches='tight')
        paths.append(p)
    plt.close(fig)
    return paths


def mark_maneuvers(ax, mans):
    for t0 in sorted(set(m['t0'] for m in mans)):
        ax.axvline(t0, color=C_GRID, linewidth=0.8, zorder=0)


# ----------------------------------------------------------------- figuras
def fig_tracking(rb, rc, lb, lc):
    fig, axs = plt.subplots(3, 2, figsize=(11, 7.5), sharex='col', sharey='row')
    for row, (ax_key, ax_name) in enumerate(AXES):
        for col, (df, lab, c) in enumerate(((rb, lb, C_BASE), (rc, lc, C_CUSTOM))):
            a = axs[row, col]
            a.plot(df.time_s, df['des_%s_deg_s' % ax_key], color=C_REF, linestyle='--',
                   linewidth=1.0, label='deseada')
            a.plot(df.time_s, df['%s_deg_s' % ax_key], color=c, linewidth=1.0, label='medida')
            if row == 0:
                a.set_title(lab)
            if col == 0:
                a.set_ylabel('%s (°/s)' % ax_name)
            if row == 2:
                a.set_xlabel('tiempo (s)')
    handles = [plt.Line2D([], [], color=C_REF, linestyle='--', linewidth=1.0),
               plt.Line2D([], [], color=C_BASE), plt.Line2D([], [], color=C_CUSTOM)]
    fig.legend(handles, ['deseada', 'medida %s' % lb, 'medida %s' % lc], loc='upper right',
               ncol=3, bbox_to_anchor=(1.0, 1.0))
    fig.suptitle('Seguimiento de la tasa angular', x=0.01, ha='left')
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    return fig


def fig_error(rb, rc, lb, lc, mans):
    fig, axs = plt.subplots(3, 1, figsize=(11, 7.5), sharex=True)
    for row, (ax_key, ax_name) in enumerate(AXES):
        a = axs[row]
        eb, ec = rate_error(rb, ax_key), rate_error(rc, ax_key)
        mark_maneuvers(a, mans)
        # el custom se dibuja primero: si oscila de forma continua no tapa a la base
        a.plot(rc.time_s, ec, color=C_CUSTOM, linewidth=0.8,
               label='%s  (RMS %.2f °/s)' % (lc, rms(ec)))
        a.plot(rb.time_s, eb, color=C_BASE, linewidth=1.1,
               label='%s  (RMS %.2f °/s)' % (lb, rms(eb)))
        a.set_ylabel('error %s (°/s)' % ax_name)
        h, l = a.get_legend_handles_labels()
        a.legend(h[::-1], l[::-1], loc='upper right')
    axs[-1].set_xlabel('tiempo desde el inicio de la fase (s)')
    fig.suptitle('Error de tasa angular (deseada − medida)', x=0.01, ha='left')
    fig.tight_layout()
    return fig


def bar_pair(a, labels, vb, vc, lb, lc, unit):
    x = np.arange(len(labels))
    w = 0.36
    bb = a.bar(x - w / 2 - 0.01, vb, w, color=C_BASE, label=lb)
    bc = a.bar(x + w / 2 + 0.01, vc, w, color=C_CUSTOM, label=lc)
    a.set_xticks(x, labels)
    a.set_ylabel(unit)
    a.grid(axis='x', visible=False)
    for rect_b, rect_c, b, c in zip(bb, bc, vb, vc):
        if not (math.isfinite(b) and math.isfinite(c)):
            continue
        a.annotate('%+.1f %%' % improvement(b, c), (rect_c.get_x() + rect_c.get_width() / 2, c),
                   xytext=(0, 3), textcoords='offset points', ha='center', fontsize=9,
                   color='#0b0b0b')


def fig_rms(rb, rc, ab, ac, lb, lc, metrics):
    fig, axs = plt.subplots(1, 2, figsize=(11, 4), gridspec_kw={'width_ratios': [3, 2]})
    names = [n for _, n in AXES]
    vb = [rms(rate_error(rb, k)) for k, _ in AXES]
    vc = [rms(rate_error(rc, k)) for k, _ in AXES]
    bar_pair(axs[0], names, vb, vc, lb, lc, 'RMS error de tasa (°/s)')
    axs[0].set_title('Error de tasa  (%% = mejora del %s)' % lc)
    for (k, n), b, c in zip(AXES, vb, vc):
        metrics.append(('rms_error_tasa_deg_s', n, b, c, improvement(b, c)))
    if ab is not None and ac is not None:
        att = (('roll', 'Roll'), ('pitch', 'Pitch'))
        vb2 = [rms(ab['des_%s_deg' % k] - ab['%s_deg' % k]) for k, _ in att]
        vc2 = [rms(ac['des_%s_deg' % k] - ac['%s_deg' % k]) for k, _ in att]
        bar_pair(axs[1], [n for _, n in att], vb2, vc2, lb, lc, 'RMS error de actitud (°)')
        axs[1].set_title('Error de actitud')
        for (k, n), b, c in zip(att, vb2, vc2):
            metrics.append(('rms_error_actitud_deg', n, b, c, improvement(b, c)))
    else:
        axs[1].set_visible(False)
    h, l = axs[0].get_legend_handles_labels()
    fig.legend(h, l, loc='upper right', ncol=2, bbox_to_anchor=(1.0, 1.06))
    fig.tight_layout()
    return fig


def fig_response(rb, rc, mb, mc, lb, lc, window):
    """error alineado al inicio de cada maniobra: muestra qué tan rápido se corrige"""
    names = [m['name'] for m in mb if '→' not in m['name']]
    names = list(dict.fromkeys(names))
    if not names:
        return None
    ncol = 3
    nrow = int(math.ceil(len(names) / float(ncol)))
    fig, axs = plt.subplots(nrow, ncol, figsize=(11, 3.1 * nrow), squeeze=False)
    for i, name in enumerate(names):
        a = axs[i // ncol, i % ncol]
        for df, mans, lab, c in ((rb, mb, lb, C_BASE), (rc, mc, lc, C_CUSTOM)):
            ms = [m for m in mans if m['name'] == name]
            if not ms:
                continue
            m = ms[0]
            sel = (df.time_s >= m['t0'] - 0.1) & (df.time_s <= m['t0'] + window)
            a.plot(df.time_s[sel] - m['t0'], rate_error(df, m['axis'])[sel], color=c,
                   linewidth=1.1, label=lab)
        a.axhline(0, color=C_REF, linewidth=0.6)
        a.set_title(name)
        a.set_xlabel('tiempo desde el inicio de la maniobra (s)')
        if i % ncol == 0:
            a.set_ylabel('error de tasa (°/s)')
        if i == 0:
            a.legend(loc='upper right')
    for j in range(len(names), nrow * ncol):
        axs[j // ncol, j % ncol].set_visible(False)
    fig.suptitle('Respuesta del error al inicio de cada maniobra', x=0.01, ha='left')
    fig.tight_layout()
    return fig


def fig_settling(rb, rc, mb, mc, lb, lc, band, metrics):
    rows = []
    for m in mb:
        mcs = [x for x in mc if x['index'] == m['index'] and x['axis'] == m['axis']]
        if not mcs:
            continue
        pb, ib, tb = settle(rb, m, band)
        pc, ic, tc = settle(rc, mcs[0], band)
        label = '%d: %s' % (m['index'], m['name'])
        rows.append((label, pb, pc, ib, ic, tb, tc))
        metrics.append(('asentamiento_s', label, tb, tc, improvement(tb, tc)))
        metrics.append(('pico_error_tasa_deg_s', label, pb, pc, improvement(pb, pc)))
        metrics.append(('iae_tasa_deg', label, ib, ic, improvement(ib, ic)))
    if not rows:
        return None
    labels = [r[0] for r in rows]
    fig, axs = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    x = np.arange(len(labels))
    w = 0.38
    for a, (ib_, ic_), unit, title in (
            (axs[0], (5, 6), 's', 'Tiempo de asentamiento (banda ±%.1f °/s)' % band),
            (axs[1], (1, 2), '°/s', 'Pico de |error| de tasa')):
        vb = np.array([r[ib_] for r in rows], dtype=float)
        vc = np.array([r[ic_] for r in rows], dtype=float)
        a.bar(x - w / 2 - 0.01, np.nan_to_num(vb), w, color=C_BASE, label=lb)
        a.bar(x + w / 2 + 0.01, np.nan_to_num(vc), w, color=C_CUSTOM, label=lc)
        for xi, b, c in zip(x, vb, vc):
            for dx, v in ((-w / 2, b), (w / 2, c)):
                if not math.isfinite(v):
                    a.annotate('no asentó', (xi + dx, 0), xytext=(0, 3), textcoords='offset points',
                               ha='center', fontsize=8, rotation=90, color='#52514e')
        a.set_ylabel(unit)
        a.set_title(title, loc='left')
        a.grid(axis='x', visible=False)
    axs[0].legend(loc='upper right')
    axs[1].set_xticks(x, labels, rotation=35, ha='right')
    fig.tight_layout()
    return fig


def fig_effort(rb, rc, lb, lc, metrics):
    fig = plt.figure(figsize=(11, 8))
    gs = fig.add_gridspec(3, 4)
    vb, vc = [], []
    for row, (k, n) in enumerate(AXES):
        a = fig.add_subplot(gs[row, :3])
        ob, oc = rb['out_%s' % k], rc['out_%s' % k]
        a.plot(rc.time_s, oc, color=C_CUSTOM, linewidth=0.8, label=lc)
        a.plot(rb.time_s, ob, color=C_BASE, linewidth=1.1, label=lb)
        a.set_ylabel('u %s (−1..1)' % n)
        if row == 0:
            h, l = a.get_legend_handles_labels()
            a.legend(h[::-1], l[::-1], loc='upper right')
            a.set_title('Salida de control hacia el mixer (RATE.%sOut)' % 'R/P/Y', loc='left')
        if row == 2:
            a.set_xlabel('tiempo desde el inicio de la fase (s)')
        vb.append(rms(np.diff(ob.values)))
        vc.append(rms(np.diff(oc.values)))
        metrics.append(('rugosidad_du_rms', n, vb[-1], vc[-1], improvement(vb[-1], vc[-1])))
        metrics.append(('esfuerzo_u_rms', n, rms(ob), rms(oc), improvement(rms(ob), rms(oc))))
    a = fig.add_subplot(gs[:, 3])
    x = np.arange(3)
    w = 0.38
    a.bar(x - w / 2 - 0.01, vb, w, color=C_BASE, label=lb)
    a.bar(x + w / 2 + 0.01, vc, w, color=C_CUSTOM, label=lc)
    a.set_xticks(x, [n for _, n in AXES])
    a.set_title('Rugosidad\nRMS(Δu por muestra)', loc='left')
    a.grid(axis='x', visible=False)
    a.legend(loc='upper left')
    fig.tight_layout()
    return fig


def ccmo_arrays(dfs, n_motors):
    """apila Thr, Out, PWM de todos los motores; solo filas en vuelo (Thr > 0)"""
    thr, out, pwm, mot = [], [], [], []
    for df in dfs:
        if df is None:
            continue
        for i in range(1, n_motors + 1):
            t = df['Thr_m%d' % i].values
            ok = t > 0
            thr.append(t[ok])
            out.append(df['Out_m%d' % i].values[ok])
            pwm.append(df['PWM_m%d' % i].values[ok])
            mot.append(np.full(ok.sum(), i))
    return (np.concatenate(thr), np.concatenate(out), np.concatenate(pwm), np.concatenate(mot))


def chain_params(params):
    smin = float(params.get('MOT_SPIN_MIN', 0.15))
    smax = float(params.get('MOT_SPIN_MAX', 0.95))
    pmin = float(params.get('MOT_PWM_MIN', 1000) or 1000)
    pmax = float(params.get('MOT_PWM_MAX', 2000) or 2000)
    return smin, smax, pmin, pmax


def fig_linearity(cb, cc, lb, lc, n_motors, params, metrics):
    smin, smax, _, _ = chain_params(params)
    fig, axs = plt.subplots(2, 2, figsize=(10, 8), sharex=True, sharey=True)
    xs = np.linspace(0, 1, 2)
    for i in range(1, n_motors + 1):
        a = axs[(i - 1) // 2, (i - 1) % 2]
        for df, lab, c in ((cb, lb, C_BASE), (cc, lc, C_CUSTOM)):
            if df is None:
                continue
            t, o = df['Thr_m%d' % i], df['Out_m%d' % i]
            ok = t > 0
            a.plot(t[ok], o[ok], linestyle='none', marker='o', markersize=2.5, alpha=0.35,
                   color=c, label=lab)
        a.plot(xs, smin + (smax - smin) * xs, color=C_REF, linestyle='--', linewidth=1.0,
               label='Out = %.2f + %.2f·Thr' % (smin, smax - smin))
        a.set_title('Motor %d' % i)
        if i in (3, 4):
            a.set_xlabel('Thr  (_thrust_rpyt_out)')
        if i in (1, 3):
            a.set_ylabel('Out  (_actuator)')
        if i == 1:
            a.legend(loc='upper left', markerscale=3)
    # zoom al rango usado en vuelo
    thr, out, _, _ = ccmo_arrays([cb, cc], n_motors)
    pad = 0.03
    axs[0, 0].set_xlim(max(0, thr.min() - pad), min(1, thr.max() + pad))
    axs[0, 0].set_ylim(max(0, out.min() - pad), min(1, out.max() + pad))
    res = out - (smin + (smax - smin) * thr)
    metrics.append(('ccmo_max_abs_residuo_out', 'todos', float(np.max(np.abs(res))), '', ''))
    fig.suptitle('Linealidad Thr → Out (CCMO), puntos en vuelo de ambas fases', x=0.01, ha='left')
    fig.tight_layout()
    return fig


def fig_reversibility(cb, cc, n_motors, params, metrics):
    smin, smax, pmin, pmax = chain_params(params)
    thr, out, pwm, _ = ccmo_arrays([cb, cc], n_motors)
    r_fwd = out - (smin + (smax - smin) * thr)                 # Out predicho desde Thr
    r_inv = (out - smin) / (smax - smin) - thr                 # Thr reconstruido desde Out
    r_pwm = pwm - (pmin + (pmax - pmin) * out)                 # PWM predicho desde Out
    metrics.append(('ccmo_max_abs_residuo_inversa_thr', 'todos', float(np.max(np.abs(r_inv))), '', ''))
    metrics.append(('ccmo_max_abs_residuo_pwm_us', 'todos', float(np.max(np.abs(r_pwm))), '', ''))
    fig, axs = plt.subplots(1, 3, figsize=(12, 3.8))
    for a, r, title, unit, note in (
            (axs[0], r_fwd, 'Directa: Out − (%.2f + %.2f·Thr)' % (smin, smax - smin), 'residuo Out',
             'precisión float32'),
            (axs[1], r_inv, 'Inversa: (Out − %.2f)/%.2f − Thr' % (smin, smax - smin), 'residuo Thr',
             'precisión float32'),
            (axs[2], r_pwm, 'PWM − (%.0f + %.0f·Out)' % (pmin, pmax - pmin), 'residuo PWM (µs)',
             'truncado a entero (int16)')):
        a.hist(r, bins=60, color=C_REF)
        a.set_title('%s\nmáx |residuo| = %.2g  (%s)' % (title, np.max(np.abs(r)), note), fontsize=10)
        a.set_xlabel(unit)
        a.set_ylabel('muestras')
        a.grid(axis='x', visible=False)
    fig.suptitle('Reversibilidad de la cadena de motores (n = %d muestras)' % len(thr),
                 x=0.01, ha='left')
    fig.tight_layout()
    return fig


def fig_pwm(cb, cc, lb, lc, n_motors):
    fig, axs = plt.subplots(2, 1, figsize=(11, 6), sharex=True, sharey=True)
    for a, df, lab in ((axs[0], cb, lb), (axs[1], cc, lc)):
        if df is None:
            a.set_visible(False)
            continue
        for i in range(1, n_motors + 1):
            a.plot(df.time_s, df['PWM_m%d' % i], color=C_MOTORS[(i - 1) % len(C_MOTORS)],
                   linewidth=0.8, label='motor %d' % i)
        a.set_title(lab, loc='left')
        a.set_ylabel('PWM (µs)')
    axs[0].legend(loc='upper right', ncol=n_motors)
    axs[1].set_xlabel('tiempo desde el inicio de la fase (s)')
    fig.suptitle('PWM por motor (CCMO.PWM)', x=0.01, ha='left')
    fig.tight_layout()
    return fig


# -------------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('run_dir', help='carpeta de una corrida de sitl_rms_test.py')
    ap.add_argument('--baseline-prefix', default='PID')
    ap.add_argument('--custom-prefix', default='STA')
    ap.add_argument('--baseline-label', default='P+PID')
    ap.add_argument('--custom-label', default='STA')
    ap.add_argument('--out', help='carpeta de salida (por defecto RUN_DIR/figs)')
    ap.add_argument('--format', default='png', help='png, pdf, svg o lista separada por comas')
    ap.add_argument('--band', type=float, default=5.0,
                    help='banda de asentamiento del error de tasa, °/s (por defecto 5)')
    ap.add_argument('--window', type=float, default=1.0,
                    help='segundos mostrados después del inicio de cada maniobra (fig. 04)')
    ap.add_argument('--motors', type=int, default=4)
    args = ap.parse_args()

    fmt = [f.strip() for f in args.format.split(',') if f.strip()]
    out_dir = args.out or os.path.join(args.run_dir, 'figs')
    os.makedirs(out_dir, exist_ok=True)
    B, C, lb, lc = args.baseline_prefix, args.custom_prefix, args.baseline_label, args.custom_label

    rb, rc = load(args.run_dir, B, 'rate'), load(args.run_dir, C, 'rate')
    if rb is None or rc is None:
        sys.exit('faltan %s_rate.csv o %s_rate.csv en %s' % (B, C, args.run_dir))
    ab, ac = load(args.run_dir, B, 'attitude'), load(args.run_dir, C, 'attitude')
    cb, cc = load(args.run_dir, B, 'ccmo'), load(args.run_dir, C, 'ccmo')
    params = load_params(args.run_dir)
    mb = maneuvers(rb, load_segments(args.run_dir, B))
    mc = maneuvers(rc, load_segments(args.run_dir, C))

    metrics = []
    written = []
    written += save(fig_tracking(rb, rc, lb, lc), out_dir, '01_seguimiento_tasa', fmt)
    written += save(fig_error(rb, rc, lb, lc, mb), out_dir, '02_error_tasa', fmt)
    written += save(fig_rms(rb, rc, ab, ac, lb, lc, metrics), out_dir, '03_resumen_rms', fmt)
    f = fig_response(rb, rc, mb, mc, lb, lc, args.window)
    if f is not None:
        written += save(f, out_dir, '04_respuesta_maniobra', fmt)
    f = fig_settling(rb, rc, mb, mc, lb, lc, args.band, metrics)
    if f is not None:
        written += save(f, out_dir, '05_asentamiento', fmt)
    if 'out_roll' in rb.columns and 'out_roll' in rc.columns:
        written += save(fig_effort(rb, rc, lb, lc, metrics), out_dir, '06_esfuerzo_control', fmt)
    else:
        print('aviso: los CSV de tasa no tienen out_roll/out_pitch/out_yaw; se omite la fig. 06')
    if cb is not None or cc is not None:
        written += save(fig_linearity(cb, cc, lb, lc, args.motors, params, metrics),
                        out_dir, '07_linealidad_ccmo', fmt)
        written += save(fig_reversibility(cb, cc, args.motors, params, metrics),
                        out_dir, '08_reversibilidad_ccmo', fmt)
        written += save(fig_pwm(cb, cc, lb, lc, args.motors), out_dir, '09_pwm_motores', fmt)
    else:
        print('aviso: no hay CSV de CCMO; se omiten las figs. 07-09')

    mpath = os.path.join(out_dir, 'metricas.csv')
    with open(mpath, 'w', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(('metrica', 'caso', lb, lc, 'mejora_%'))
        for row in metrics:
            w.writerow(row)
    written.append(mpath)

    print('Figuras y métricas:')
    for p in written:
        print(' ', p)


if __name__ == '__main__':
    main()
