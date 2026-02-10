"""
visualize_trajectory.py
3D trajectory visualization comparing ESKF estimate vs EuRoC ground truth.

Usage:
    python scripts/visualize_trajectory.py \
        --est  output/trajectory.csv \
        --gt   data/MH_01_easy/state_groundtruth_estimate0/data.csv
"""

import argparse
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401


def load_estimate(path: str) -> np.ndarray:
    """Load ESKF output CSV. Columns: timestamp, px, py, pz, qw..."""
    return np.loadtxt(path, delimiter=',', skiprows=1)


def load_ground_truth(path: str) -> np.ndarray:
    """Load EuRoC ground truth CSV. Timestamp in nanoseconds."""
    data = np.loadtxt(path, delimiter=',', comments='#')
    data[:, 0] *= 1e-9  # ns -> s
    return data


def interpolate_gt(gt: np.ndarray, timestamps: np.ndarray) -> np.ndarray:
    """Linearly interpolate ground truth positions to estimated timestamps."""
    gt_t = gt[:, 0]
    gt_p = gt[:, 1:4]

    interp = np.zeros((len(timestamps), 3))
    for i, t in enumerate(timestamps):
        if t < gt_t[0] or t > gt_t[-1]:
            interp[i] = np.nan
            continue
        idx = np.searchsorted(gt_t, t)
        idx = min(idx, len(gt_t) - 1)
        if idx == 0:
            interp[i] = gt_p[0]
        else:
            alpha = (t - gt_t[idx - 1]) / (gt_t[idx] - gt_t[idx - 1])
            interp[i] = gt_p[idx - 1] + alpha * (gt_p[idx] - gt_p[idx - 1])
    return interp


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--est',  default='output/trajectory.csv')
    parser.add_argument('--gt',   default='data/MH_01_easy/state_groundtruth_estimate0/data.csv')
    args = parser.parse_args()

    est = load_estimate(args.est)
    gt  = load_ground_truth(args.gt)

    est_t = est[:, 0]
    est_p = est[:, 1:4]

    # Subsample GT for 3D plot (plotting 16k points is slow).
    gt_sub = gt[::5]

    # ── Figure 1: 3D Trajectory ────────────────────────────────────────────────
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')

    ax.plot(gt_sub[:, 1], gt_sub[:, 2], gt_sub[:, 3],
            'g-', linewidth=1.5, label='Ground Truth', alpha=0.7)
    ax.plot(est_p[:, 0], est_p[:, 1], est_p[:, 2],
            'b--', linewidth=1.0, label='ESKF Estimate', alpha=0.9)

    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_zlabel('Z [m]')
    ax.set_title('EuRoC MH_01_easy - 3D Trajectory')
    ax.legend()
    plt.tight_layout()

    # ── Figure 2: Position components over time ───────────────────────────────
    gt_interp = interpolate_gt(gt, est_t)
    valid = ~np.isnan(gt_interp[:, 0])

    fig2, axes = plt.subplots(3, 1, figsize=(14, 8), sharex=True)
    labels = ['X [m]', 'Y [m]', 'Z [m]']

    for i in range(3):
        axes[i].plot(gt[::5, 0] - gt[0, 0], gt[::5, i+1],
                     'g-', linewidth=1.2, label='GT', alpha=0.7)
        axes[i].plot(est_t[valid] - est_t[0], est_p[valid, i],
                     'b--', linewidth=1.0, label='ESKF')
        axes[i].set_ylabel(labels[i])
        axes[i].legend(loc='upper right')
        axes[i].grid(True, alpha=0.3)

    axes[2].set_xlabel('Time [s]')
    fig2.suptitle('Position Components vs Time')
    plt.tight_layout()

    # ── Figure 3: Velocity estimate ───────────────────────────────────────────
    if est.shape[1] >= 11:
        est_v = est[:, 8:11]
        gt_v  = gt[::5, 8:11] if gt.shape[1] >= 11 else None

        fig3, axes = plt.subplots(3, 1, figsize=(14, 8), sharex=True)
        vlabels = ['Vx [m/s]', 'Vy [m/s]', 'Vz [m/s]']

        for i in range(3):
            if gt_v is not None:
                axes[i].plot(gt[::5, 0] - gt[0, 0], gt_v[:, i],
                             'g-', linewidth=1.2, label='GT', alpha=0.7)
            axes[i].plot(est_t - est_t[0], est_v[:, i],
                         'b--', linewidth=1.0, label='ESKF')
            axes[i].set_ylabel(vlabels[i])
            axes[i].legend(loc='upper right')
            axes[i].grid(True, alpha=0.3)

        axes[2].set_xlabel('Time [s]')
        fig3.suptitle('Velocity Components vs Time')
        plt.tight_layout()

    plt.show()


if __name__ == '__main__':
    main()
