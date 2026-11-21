#!/usr/bin/env python3
"""Genera las graficas y la tabla de errores a partir de los CSV de results/.

Uso:
  python3 scripts/plot_results.py                       # todos los results/*.csv
  python3 scripts/plot_results.py results/fig8_run1.csv results/fig8_run2.csv

Salidas (en results/plots/):
  xy_<run>.png      trayectoria deseada vs odometria vs pose real (plano XY)
  error_<run>.png   error de seguimiento en el tiempo
  drift_<run>.png   deriva de la odometria (odometria - pose real)
  error_all_runs.png  error real de todas las corridas superpuesto
  metrics.csv / metrics.md  error maximo y RMS por corrida

Definiciones:
  e_real(t)  = distancia de la pose REAL (ground truth) a la trayectoria deseada
  e_odom(t)  = distancia de la pose ESTIMADA (odometria) a la trayectoria deseada
               (es el error que "ve" el controlador)
  deriva(t)  = || p_odom(t) - p_real(t) ||
"""
import glob
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

# Paleta (slots categoricos 1-3 validados + neutros para la referencia)
C_REF = '#52514e'      # trayectoria deseada (gris oscuro, discontinua)
C_ODOM = '#2a78d6'     # odometria (azul)
C_GT = '#eb6834'       # pose real (naranja)
C_THIRD = '#1baf7a'    # tercera serie (aqua)
RUN_COLORS = ['#2a78d6', '#eb6834', '#1baf7a']
TEXT2 = '#52514e'
GRID = '#e4e3df'

plt.rcParams.update({
    'figure.dpi': 110, 'savefig.dpi': 220, 'font.size': 10,
    'axes.edgecolor': '#9a9993', 'axes.labelcolor': '#0b0b0b',
    'axes.spines.top': False, 'axes.spines.right': False,
    'axes.grid': True, 'grid.color': GRID, 'grid.linewidth': 0.8,
    'xtick.color': TEXT2, 'ytick.color': TEXT2, 'legend.frameon': False,
    'lines.linewidth': 2.0,
})


def load(csv_path):
    d = np.genfromtxt(csv_path, delimiter=',', names=True)
    path_csv = csv_path[:-4] + '_path.csv'
    p = np.genfromtxt(path_csv, delimiter=',', names=True)
    return d, np.column_stack((p['x'], p['y']))


def dist_to_path(x, y, path):
    """Distancia minima punto-poligonal (proyeccion sobre cada segmento)."""
    a, b = path[:-1], path[1:]
    ab = b - a
    L2 = np.maximum(np.sum(ab * ab, axis=1), 1e-12)
    out = np.empty(len(x))
    for i, (px, py) in enumerate(zip(x, y)):
        ap = np.array([px, py]) - a
        t = np.clip(np.sum(ap * ab, axis=1) / L2, 0.0, 1.0)
        proj = a + ab * t[:, None]
        out[i] = np.min(np.hypot(proj[:, 0] - px, proj[:, 1] - py))
    return out


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def metrics(d, path):
    e_real = dist_to_path(d['x_gt'], d['y_gt'], path)
    e_odom = dist_to_path(d['x_odom'], d['y_odom'], path)
    drift = np.hypot(d['x_odom'] - d['x_gt'], d['y_odom'] - d['y_gt'])
    dyaw = np.degrees(wrap(d['yaw_odom'] - d['yaw_gt']))
    length = float(np.sum(np.hypot(np.diff(d['x_gt']), np.diff(d['y_gt']))))
    rms = lambda v: float(np.sqrt(np.mean(v ** 2)))  # noqa: E731
    return dict(e_real=e_real, e_odom=e_odom, drift=drift, dyaw=dyaw, summary={
        'duration_s': float(d['t'][-1]),
        'distance_m': length,
        'e_real_max_m': float(e_real.max()), 'e_real_rms_m': rms(e_real),
        'e_odom_max_m': float(e_odom.max()), 'e_odom_rms_m': rms(e_odom),
        'drift_final_m': float(drift[-1]), 'drift_max_m': float(drift.max()),
        'drift_pct_dist': 100.0 * float(drift[-1]) / max(length, 1e-6),
        'yaw_drift_max_deg': float(np.max(np.abs(dyaw))),
    })


def plot_xy(name, d, path, out):
    fig, ax = plt.subplots(figsize=(7.0, 4.2))
    ax.plot(path[:, 0], path[:, 1], '--', color=C_REF, lw=1.6, label='Desired path')
    ax.plot(d['x_odom'], d['y_odom'], color=C_ODOM, label='Wheel odometry (estimate)')
    ax.plot(d['x_gt'], d['y_gt'], color=C_GT, label='Ground truth (Gazebo)')
    ax.plot(d['x_gt'][0], d['y_gt'][0], 'o', ms=8, color='#0b0b0b', mec='white', mew=2,
            label='Start')
    ax.set_aspect('equal', adjustable='datalim')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_title(f'Trajectory in the XY plane — {name}', loc='left', fontsize=11)
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.14), ncol=4, fontsize=9)
    fig.tight_layout()
    fig.savefig(out)
    plt.close(fig)


def plot_error(name, d, m, out):
    fig, ax = plt.subplots(figsize=(7.0, 3.2))
    ax.plot(d['t'], m['e_real'] * 100, color=C_GT, label='Real error (ground truth vs. path)')
    ax.plot(d['t'], m['e_odom'] * 100, color=C_ODOM, label='Estimated error (odometry vs. path)')
    s = m['summary']
    ax.axhline(s['e_real_rms_m'] * 100, color=C_GT, lw=1, ls=':')
    ax.annotate(f"RMS {s['e_real_rms_m']*100:.1f} cm", (d['t'][-1], s['e_real_rms_m'] * 100),
                xytext=(-4, 4), textcoords='offset points', ha='right', color=TEXT2, fontsize=9)
    ax.set_xlabel('time [s]')
    ax.set_ylabel('tracking error [cm]')
    ax.set_ylim(bottom=0)
    ax.set_title(f'Tracking error over time — {name}', loc='left', fontsize=11)
    ax.legend(loc='upper left', fontsize=9)
    fig.tight_layout()
    fig.savefig(out)
    plt.close(fig)


def plot_drift(name, d, m, out):
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(7.0, 4.6), sharex=True)
    a1.plot(d['t'], (d['x_odom'] - d['x_gt']) * 100, color=C_ODOM, label='x odom − x real')
    a1.plot(d['t'], (d['y_odom'] - d['y_gt']) * 100, color=C_THIRD, label='y odom − y real')
    a1.plot(d['t'], m['drift'] * 100, color=C_GT, label='‖position drift‖')
    a1.set_ylabel('position drift [cm]')
    a1.legend(loc='upper left', fontsize=9, ncol=3)
    a1.set_title(f'Odometry drift (odometry − ground truth) — {name}', loc='left', fontsize=11)
    a2.plot(d['t'], m['dyaw'], color=C_ODOM)
    a2.axhline(0, color='#9a9993', lw=0.8)
    a2.set_ylabel('heading drift [deg]')
    a2.set_xlabel('time [s]')
    fig.tight_layout()
    fig.savefig(out)
    plt.close(fig)


def plot_all_runs(runs, out):
    fig, ax = plt.subplots(figsize=(7.0, 3.2))
    for i, (name, d, m) in enumerate(runs[:3]):
        ax.plot(d['t'], m['e_real'] * 100, color=RUN_COLORS[i], label=name)
    ax.set_xlabel('time [s]')
    ax.set_ylabel('real tracking error [cm]')
    ax.set_ylim(bottom=0)
    ax.set_title('Real tracking error — repeated runs', loc='left', fontsize=11)
    ax.legend(loc='upper left', fontsize=9)
    fig.tight_layout()
    fig.savefig(out)
    plt.close(fig)


def main():
    root = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'results')
    files = sys.argv[1:] or sorted(f for f in glob.glob(os.path.join(root, '*.csv'))
                                   if not f.endswith('_path.csv'))
    if not files:
        sys.exit('No hay CSV en results/. Corre primero la simulacion.')
    out_dir = os.path.join(os.path.dirname(os.path.abspath(files[0])), 'plots')
    os.makedirs(out_dir, exist_ok=True)

    runs, rows = [], []
    for f in files:
        name = os.path.basename(f)[:-4]
        d, path = load(f)
        m = metrics(d, path)
        runs.append((name, d, m))
        rows.append((name, m['summary']))
        plot_xy(name, d, path, os.path.join(out_dir, f'xy_{name}.png'))
        plot_error(name, d, m, os.path.join(out_dir, f'error_{name}.png'))
        plot_drift(name, d, m, os.path.join(out_dir, f'drift_{name}.png'))
        print(f'[ok] {name}')

    multi = [r for r in runs if not r[0].endswith('_gt')]
    if len(multi) > 1:
        plot_all_runs(multi, os.path.join(out_dir, 'error_all_runs.png'))

    keys = list(rows[0][1].keys())
    with open(os.path.join(out_dir, 'metrics.csv'), 'w') as f:
        f.write('run,' + ','.join(keys) + '\n')
        for name, s in rows:
            f.write(name + ',' + ','.join(f'{s[k]:.4f}' for k in keys) + '\n')

    md = ['| Run | Duration [s] | Distance [m] | Real error max [cm] | Real error RMS [cm] '
          '| Odom-estimate error max [cm] | Odom-estimate error RMS [cm] | Odom drift final [cm] '
          '| Odom drift max [cm] | Heading drift max [deg] |',
          '|---|---|---|---|---|---|---|---|---|---|']
    for name, s in rows:
        md.append(f"| {name} | {s['duration_s']:.1f} | {s['distance_m']:.2f} "
                  f"| {s['e_real_max_m']*100:.1f} | {s['e_real_rms_m']*100:.1f} "
                  f"| {s['e_odom_max_m']*100:.1f} | {s['e_odom_rms_m']*100:.1f} "
                  f"| {s['drift_final_m']*100:.1f} | {s['drift_max_m']*100:.1f} "
                  f"| {s['yaw_drift_max_deg']:.1f} |")
    with open(os.path.join(out_dir, 'metrics.md'), 'w') as f:
        f.write('\n'.join(md) + '\n')
    print('\n'.join(md))
    print(f'\nGraficas y tablas en: {out_dir}')


if __name__ == '__main__':
    main()
