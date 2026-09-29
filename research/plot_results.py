"""Plot start-anchored trajectories using original image capture timestamps."""

import argparse
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import rosbag
from scipy.spatial.transform import Rotation

from run_odom import aligned_trajectories, bag_calibration, reference_poses

plt.rcParams['svg.hashsalt'] = 'levio-research'


def save_svg(fig, output):
    fig.savefig(output, format='svg')
    output.write_text('\n'.join(line.rstrip() for line in output.read_text().splitlines()) + '\n')


def draw_uninitialized(run_dir, output, summary, preview_png=None):
    """Show estimator diagnostics when its translation has no metric scale."""
    trace_file = run_dir / 'runtime_match_trace.csv'
    if not trace_file.exists():
        raise ValueError(f'Missing runtime trace for uninitialized run: {trace_file}')
    with trace_file.open(newline='') as handle:
        rows = list(csv.DictReader(handle))
    first_time = float(rows[0]['capture_time_s'])
    rows = [row for row in rows if row['keyframe_id']]
    times = np.array([float(row['capture_time_s']) - first_time for row in rows])
    matches = np.array([int(row['matches_hamming_threshold']) for row in rows])
    ages = np.array([float(row['keyframe_age_s']) for row in rows])
    skipped = np.array([row['essential_skipped'] == '1' for row in rows])
    plt.rcParams.update({'font.size': 10, 'svg.fonttype': 'none'})
    fig, axes = plt.subplots(2, 1, figsize=(10.2, 5.2), sharex=True,
                             layout='constrained')
    axes[0].plot(times, matches, color='#245a9b', lw=1.1)
    axes[0].axhline(8, color='#555555', ls='--', lw=1,
                    label='protective skip threshold (8 matches)')
    axes[0].set(ylabel='matches to selected keyframe',
                title='Runtime LEVIO feature matches')
    axes[0].legend(loc='upper right', fontsize=8)
    axes[1].plot(times, ages, color='#a55d17', lw=1.1)
    if skipped.any():
        axes[1].scatter(times[skipped], ages[skipped], color='#d94841', s=5,
                        label='essential update skipped')
        axes[1].legend(loc='upper left', fontsize=8)
    axes[1].set(xlabel='time from first RGB frame (s)',
                ylabel='selected keyframe age (s)')
    for ax in axes:
        ax.grid(alpha=0.3)
    fig.suptitle(f'{run_dir.name} | no valid metric trajectory score',
                 fontsize=10)
    output.parent.mkdir(parents=True, exist_ok=True)
    save_svg(fig, output)
    if preview_png is not None:
        fig.savefig(preview_png, dpi=150)
    plt.close(fig)
    print(output, f'essential-skipped={skipped.sum()}',
          f'keyframe-age-max={ages.max():.3f} s')


def aligned_samples(run_dir):
    summary = json.loads((run_dir / 'summary.json').read_text())
    poses = np.loadtxt(run_dir / 'online_frames.tum', comments='#')
    if poses.ndim == 1:
        poses = poses[None, :]
    capture_file = run_dir / 'capture_times.txt'
    if capture_file.exists():
        image_t = np.atleast_1d(np.loadtxt(capture_file))
        poses = poses[:len(image_t)]
    else:
        image_t = poses[:, 0]
    first_skip = summary.get('first_essential_skip_frame')
    if first_skip is not None:
        poses = poses[:first_skip]
        image_t = image_t[:first_skip]
    init = summary.get('initialization_frame')
    if init is not None:
        poses = poses[init:]
        image_t = image_t[init:]
    if summary.get('dataset') == 'euroc':
        from run_euroc import ground_truth_poses
        ref_t, ref_xyz, ref_rotations = ground_truth_poses(
            summary['groundtruth_csv'], np.array(summary['T_imu_camera']),
            with_rotations=True)
    else:
        with rosbag.Bag(summary['bag']) as bag:
            _, _, _, _, T_base_camera = bag_calibration(bag, summary['camera'])
            ref_t, ref_xyz, ref_rotations = reference_poses(
                bag, T_base_camera, with_rotations=True)
    estimate_rotations = Rotation.from_quat(poses[:, 4:8]).as_matrix()
    samples, metric = aligned_trajectories(image_t, poses[:, 1:4], ref_t, ref_xyz,
                                            estimate_rotations, ref_rotations)
    if samples is None:
        raise ValueError('Fewer than three timestamp-matched trajectory poses')
    times = samples['times'] - samples['times'][0]
    expected = (summary['comparison_after_initialization'] if init is not None else
                summary['comparison_to_euroc_groundtruth'] if summary.get('dataset') == 'euroc'
                else summary['comparison_to_recorded_odometry'])
    if not np.isclose(metric['se3_aligned_ate_rmse_m'],
                      expected['se3_aligned_ate_rmse_m'], atol=1e-6):
        raise ValueError(f'Plot/summary metric mismatch: {metric} vs {expected}')
    if 'initial_pose_aligned_rmse_m' in expected and not np.isclose(
            metric['initial_pose_aligned_rmse_m'],
            expected['initial_pose_aligned_rmse_m'], atol=1e-6):
        raise ValueError(f'First-pose metric mismatch: {metric} vs {expected}')
    if not np.allclose(samples['reference'][0], samples['initial_pose_fit'][0],
                       atol=1e-9):
        raise ValueError('First-pose alignment did not make the starts coincide')
    return summary, times, samples['reference'], samples['initial_pose_fit'], metric


def draw(run_dir, output, preview_png=None):
    summary = json.loads((run_dir / 'summary.json').read_text())
    if not summary['comparison_after_initialization'].get(
            'metric_scale_valid', summary['vio_initialized']):
        return draw_uninitialized(run_dir, output, summary, preview_png)
    summary, times, reference, estimate, metric = aligned_samples(run_dir)
    euroc = summary.get('dataset') == 'euroc'
    plt.rcParams.update({'font.size': 10, 'svg.fonttype': 'none'})
    fig = plt.figure(figsize=(10.2, 6.5), layout='constrained')
    grid = fig.add_gridspec(2, 2, height_ratios=[2.1, 1])
    axes = [fig.add_subplot(grid[0, 0]), fig.add_subplot(grid[0, 1])]
    error_ax = fig.add_subplot(grid[1, :])
    for ax, x_ref, x_est, y_ref, y_est, xlabel, ylabel, title in [
        (axes[0], reference[:, 0], estimate[:, 0], reference[:, 1], estimate[:, 1],
         'x (m)', 'y (m)', 'X–Y trajectory'),
        (axes[1], times, times, reference[:, 2], estimate[:, 2],
         'time from first matched frame (s)', 'z (m)', 'Height over time'),
    ]:
        ax.plot(x_ref, y_ref, color='#238b45', lw=1.8,
                label='EuRoC ground truth' if euroc else 'Recorded fusion odometry')
        ax.plot(x_est, y_est, color='#d94841', lw=1.35,
                label='LEVIO, first-pose aligned')
        ax.plot(x_ref[0], y_ref[0], 'o', color='#238b45', ms=8,
                label='shared start')
        ax.plot(x_est[0], y_est[0], 'o', mfc='#d94841', mec='white',
                mew=0.8, ms=4)
        ax.plot(x_ref[-1], y_ref[-1], 'x', color='#238b45', ms=7)
        ax.plot(x_est[-1], y_est[-1], 'x', color='#d94841', ms=7)
        ax.set(xlabel=xlabel, ylabel=ylabel, title=title)
        ax.grid(alpha=0.3)
    axes[0].set_aspect('equal', adjustable='datalim')
    axes[0].legend(loc='best', fontsize=8)
    drift = np.linalg.norm(estimate - reference, axis=1)
    np.savetxt(run_dir / 'initial_pose_error.csv',
               np.column_stack((times, reference, estimate, drift)),
               delimiter=',', comments='',
               header='elapsed_s,ref_x_m,ref_y_m,ref_z_m,pred_x_m,pred_y_m,pred_z_m,error_m')
    error_ax.plot(times, drift, color='#674ea7', lw=1.35)
    error_ax.set(xlabel='time from first matched frame (s)',
                 ylabel='position difference (m)',
                 title=('Position difference (scale-free diagnostic)'
                        if not summary['vio_initialized'] else
                        'Position error from shared initial pose'))
    error_ax.grid(alpha=0.3)
    status = ('initialized' if summary['vio_initialized'] else 'not initialized; scale-free')
    suffix = 'EuRoC ground truth' if euroc else 'reference is not ground truth'
    score = (f"first-pose RMSE {metric['initial_pose_aligned_rmse_m']:.3f} m; "
             f"end error {metric['initial_pose_aligned_end_error_m']:.3f} m"
             if summary['vio_initialized'] else 'no metric VIO score')
    first_skip = summary.get('first_essential_skip_frame')
    limit = (f' | scored through frame {first_skip - 1}'
             if first_skip is not None else '')
    fig.suptitle(f'{run_dir.name} | {status} | {score}{limit}\n{suffix}', fontsize=10)
    output.parent.mkdir(parents=True, exist_ok=True)
    save_svg(fig, output)
    if preview_png is not None:
        fig.savefig(preview_png, dpi=150)
    plt.close(fig)
    print(output, f"global={metric['se3_aligned_ate_rmse_m']:.3f} m",
          f"first-pose={metric['initial_pose_aligned_rmse_m']:.3f} m")


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--run-dir', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--preview-png', type=Path)
    args = parser.parse_args()
    draw(args.run_dir, args.output, args.preview_png)
