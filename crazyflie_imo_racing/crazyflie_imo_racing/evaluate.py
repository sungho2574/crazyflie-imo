"""Compare simulator truth with host logs, without aligning away drift."""
import argparse
import json
from pathlib import Path
import numpy as np


def evaluate(run, truth):
    run = Path(run)
    a = np.genfromtxt(run/'flight.csv', delimiter=',', names=True, dtype=None, encoding='utf8')
    b = np.genfromtxt(truth, delimiter=',', names=True)
    if a.size < 2 or b.size < 2:
        raise ValueError('Need at least two estimated and ground-truth samples')
    if a['t_us'][0] < b['t_us'][0] or a['t_us'][-1] > b['t_us'][-1]:
        raise ValueError('Ground truth does not cover the flight timestamp range')
    est = np.column_stack([a[k] for k in ('px', 'py', 'pz')])
    ref = np.column_stack([a[k] for k in ('ref_x', 'ref_y', 'ref_z')])
    gt = np.column_stack([np.interp(a['t_us'], b['t_us'], b[k]) for k in ('px', 'py', 'pz')])
    result = json.loads((run/'result.json').read_text())
    for label, error in [('estimation', est-gt), ('tracking', ref-gt), ('estimated_tracking', ref-est)]:
        norm = np.linalg.norm(error, axis=1)
        result[label+'_rmse_m'] = float(np.sqrt(np.mean(norm**2)))
        result[label+'_max_m'] = float(norm.max())
    result['actual_last_position'] = gt[-1].tolist()
    result['actual_min_position'] = gt.min(axis=0).tolist()
    result['actual_max_position'] = gt.max(axis=0).tolist()
    result['ground_truth_file'] = str(Path(truth).resolve())
    result['samples'] = len(a)
    # Completing the command sequence is not proof of tracking the trajectory.
    result['tracking_pass'] = bool(result['phase'] == 'complete'
                                   and result['tracking_rmse_m'] < .5
                                   and result['tracking_max_m'] < 1.)
    return result, a['elapsed'], est, ref, gt


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run', required=True)
    p.add_argument('--truth', required=True)
    p.add_argument('--plot', action='store_true')
    args = p.parse_args()
    result, ts, est, ref, gt = evaluate(args.run, args.truth)
    run = Path(args.run)
    (run/'evaluation.json').write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))
    if args.plot:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(1, 3, figsize=(14, 4))
        for values, label, style in [(ref, 'Reference', '--'), (gt, 'Actual', '-'), (est, 'IMO + EKF', '-')]:
            axes[0].plot(values[:, 0], values[:, 1], style, label=label)
            axes[1].plot(ts, values[:, 2], style, label=label)
        axes[0].set(xlabel='x [m]', ylabel='y [m]', title='World XY (no alignment)')
        axes[0].axis('equal')
        axes[1].set(xlabel='Time after IMO handoff [s]', ylabel='z [m]', title='Altitude')
        axes[2].plot(ts, np.linalg.norm(est-gt, axis=1), label='Estimation')
        axes[2].plot(ts, np.linalg.norm(ref-gt, axis=1), label='Tracking')
        axes[2].set(xlabel='Time after IMO handoff [s]', ylabel='Error [m]', title='Position error')
        for ax in axes:
            ax.grid(alpha=.25); ax.legend()
        fig.tight_layout(); fig.savefig(run/'evaluation.png', dpi=160)


if __name__ == '__main__':
    main()
