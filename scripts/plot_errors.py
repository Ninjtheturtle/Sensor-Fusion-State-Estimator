"""
plot_errors.py
Computes and plots position error between ESKF estimate and EuRoC ground truth.
Prints RMSE per axis and overall. Validates < 5 cm target.

Usage:
    python scripts/plot_errors.py \
        --est  output/trajectory.csv \
        --gt   data/MH_01_easy/state_groundtruth_estimate0/data.csv
"""

import argparse
import numpy as np
import matplotlib.pyplot as plt


def load_estimate(path: str) -> np.ndarray:
    return np.loadtxt(path, delimiter=',', skiprows=1)


def load_ground_truth(path: str) -> np.ndarray:
    data = np.loadtxt(path, delimiter=',', comments='#')
    data[:, 0] *= 1e-9  # ns -> s
    return data


def interpolate_gt_positions(gt: np.ndarray, timestamps: np.ndarray) -> np.ndarray:
    gt_t = gt[:, 0]
    gt_p = gt[:, 1:4]
    interp = np.zeros((len(timestamps), 3))
    for i, t in enumerate(timestamps):
        if t < gt_t[0] or t > gt_t[-1]:
            interp[i] = np.nan
            continue
        idx = np.searchsorted(gt_t, t)
        idx = min(max(idx, 1), len(gt_t) - 1)
        alpha = (t - gt_t[idx - 1]) / (gt_t[idx] - gt_t[idx - 1])
        interp[i] = gt_p[idx - 1] + alpha * (gt_p[idx] - gt_p[idx - 1])
    return interp


def compute_rmse(errors: np.ndarray) -> tuple:
    """Returns (rmse_x, rmse_y, rmse_z, rmse_3d) in meters."""
    rmse_x   = np.sqrt(np.nanmean(errors[:, 0] ** 2))
    rmse_y   = np.sqrt(np.nanmean(errors[:, 1] ** 2))
    rmse_z   = np.sqrt(np.nanmean(errors[:, 2] ** 2))
    rmse_3d  = np.sqrt(np.nanmean(np.sum(errors ** 2, axis=1)))
    return rmse_x, rmse_y, rmse_z, rmse_3d


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--est',  default='output/trajectory.csv')
    parser.add_argument('--gt',   default='data/MH_01_easy/state_groundtruth_estimate0/data.csv')
    args = parser.parse_args()

    est = load_estimate(args.est)
    gt  = load_ground_truth(args.gt)

    est_t = est[:, 0]
    est_p = est[:, 1:4]

    # Trim to overlapping time range.
    mask = (est_t >= gt[0, 0]) & (est_t <= gt[-1, 0])
    est_t = est_t[mask]
    est_p = est_p[mask]

    gt_interp = interpolate_gt_positions(gt, est_t)
    errors = est_p - gt_interp
    valid = ~np.isnan(errors[:, 0])

    rmse_x, rmse_y, rmse_z, rmse_3d = compute_rmse(errors[valid])

    # ── Print RMSE table ──────────────────────────────────────────────────────
    print('\n' + '=' * 42)
    print('  Position RMSE vs EuRoC Ground Truth')
    print('=' * 42)
    print(f'  X:    {rmse_x * 100:.3f} cm')
    print(f'  Y:    {rmse_y * 100:.3f} cm')
    print(f'  Z:    {rmse_z * 100:.3f} cm')
    print(f'  3D:   {rmse_3d * 100:.3f} cm')
    print('=' * 42)
    if rmse_3d < 0.05:
        print('  [PASS] RMSE < 5 cm target achieved!')
    else:
        print(f'  [NOTE] RMSE = {rmse_3d * 100:.1f} cm. Tuning may be needed.')
    print('=' * 42 + '\n')

    # ── Figure: per-axis error + 3-sigma bounds ───────────────────────────────
    t_rel = est_t[valid] - est_t[0]
    err   = errors[valid]

    # Extract position variance diagonal from CSV (columns 11-13 are P_pos).
    if est.shape[1] >= 14:
        P_pos_diag = est[mask][valid, 11:14]
        sigma3_pos = 3.0 * np.sqrt(np.abs(P_pos_diag))
    else:
        sigma3_pos = None

    fig, axes = plt.subplots(4, 1, figsize=(14, 12), sharex=True)
    labels = ['Error X [m]', 'Error Y [m]', 'Error Z [m]', '|Error| [m]']
    colors = ['#e41a1c', '#377eb8', '#4daf4a', '#984ea3']

    for i in range(3):
        axes[i].plot(t_rel, err[:, i], color=colors[i], linewidth=0.8, label='Error')
        if sigma3_pos is not None:
            axes[i].fill_between(t_rel,
                                 -sigma3_pos[:, i], sigma3_pos[:, i],
                                 alpha=0.2, color=colors[i], label='3σ bound')
        axes[i].axhline(0, color='k', linewidth=0.5, linestyle='--')
        axes[i].set_ylabel(labels[i])
        axes[i].legend(loc='upper right', fontsize=8)
        axes[i].grid(True, alpha=0.3)

    # 3D error magnitude.
    err_3d = np.linalg.norm(err, axis=1)
    axes[3].plot(t_rel, err_3d * 100, color=colors[3], linewidth=0.8, label='|Error| [cm]')
    axes[3].axhline(5.0, color='r', linewidth=1.0, linestyle='--', label='5 cm target')
    axes[3].set_ylabel('|Error| [cm]')
    axes[3].set_xlabel('Time [s]')
    axes[3].legend(loc='upper right', fontsize=8)
    axes[3].grid(True, alpha=0.3)

    fig.suptitle(f'ESKF Position Error (RMSE: {rmse_3d * 100:.2f} cm)\nEuRoC MH_01_easy')
    plt.tight_layout()
    plt.savefig('output/position_errors.png', dpi=150, bbox_inches='tight')
    print('  Plot saved to output/position_errors.png')
    plt.show()


if __name__ == '__main__':
    main()
