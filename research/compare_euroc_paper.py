"""Offline EuRoC alignment diagnostics for an existing LEVIO online trajectory.

These are explicit least-squares SE(3) and Sim(3) fits over all matched poses in
each reported window. They do not run RPG trajectory evaluation, and the LEVIO
paper does not publish the alignment configuration behind its Table V values.
"""

import argparse
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


def euroc_camera_groundtruth(csv_path, T_imu_camera):
    data = np.loadtxt(csv_path, delimiter=',', comments='#')
    times = data[:, 0] / 1e9
    if np.any(np.diff(times) <= 0):
        raise ValueError('Ground-truth timestamps are not strictly increasing')
    # EuRoC CSV columns: timestamp, p_WI (3), q_WI (w, x, y, z), ...
    q_xyzw = data[:, [5, 6, 7, 4]]
    R_world_imu = Rotation.from_quat(q_xyzw).as_matrix()
    positions = data[:, 1:4] + np.einsum(
        'nij,j->ni', R_world_imu, T_imu_camera[:3, 3])
    rotations = R_world_imu @ T_imu_camera[:3, :3]
    return times, positions, rotations


def match_nearest(estimate_times, reference_times, tolerance_seconds):
    indexes = np.searchsorted(reference_times, estimate_times)
    indexes = np.clip(indexes, 1, len(reference_times) - 1)
    indexes -= (np.abs(reference_times[indexes - 1] - estimate_times)
                < np.abs(reference_times[indexes] - estimate_times))
    errors = np.abs(reference_times[indexes] - estimate_times)
    valid = errors <= tolerance_seconds
    return valid, indexes[valid], errors[valid]


def fit_positions(estimate, reference, allow_scale):
    """Fit row-vector estimate to reference, with proper rotation only."""
    X = estimate - estimate.mean(axis=0)
    Y = reference - reference.mean(axis=0)
    U, singular, Vt = np.linalg.svd(X.T @ Y / len(X))
    correction = np.diag([1.0, 1.0, np.linalg.det(U @ Vt)])
    rotation = U @ correction @ Vt
    variance = np.mean(np.sum(X * X, axis=1))
    if variance <= 0:
        raise ValueError('Estimated trajectory has zero position variance')
    scale = (float(np.sum(singular * np.diag(correction)) / variance)
             if allow_scale else 1.0)
    aligned = scale * X @ rotation + reference.mean(axis=0)
    residual = np.linalg.norm(aligned - reference, axis=1)
    return {
        'scale': scale,
        'ate_rmse_m': float(np.sqrt(np.mean(residual ** 2))),
        'start_offset_m': float(residual[0]),
        'end_error_m': float(residual[-1]),
    }


def evaluate(poses, reference_times, reference_positions, reference_rotations,
             start_index, tolerance_seconds):
    rows = poses[start_index:]
    valid, indexes, errors = match_nearest(
        rows[:, 0], reference_times, tolerance_seconds)
    if np.count_nonzero(valid) < 3:
        raise ValueError('Fewer than three matched poses in evaluation window')
    estimate = rows[valid, 1:4]
    reference = reference_positions[indexes]
    estimate_rotations = Rotation.from_quat(rows[valid, 4:8]).as_matrix()
    R_first = reference_rotations[indexes[0]] @ estimate_rotations[0].T
    first_pose_fit = (R_first @ (estimate - estimate[0]).T).T + reference[0]
    first_pose_error = np.linalg.norm(first_pose_fit - reference, axis=1)
    return {
        'poses_in_window': int(len(rows)),
        'matched_poses': int(len(estimate)),
        'max_time_error_ms': float(1000 * errors.max()),
        'alignment_uses': 'all matched positions in this window',
        'se3_all_positions': fit_positions(estimate, reference, False),
        'sim3_all_positions': fit_positions(estimate, reference, True),
        'first_pose_se3': {
            'scale': 1.0,
            'ate_rmse_m': float(np.sqrt(np.mean(first_pose_error ** 2))),
            'start_offset_m': float(first_pose_error[0]),
            'end_error_m': float(first_pose_error[-1]),
        },
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir', type=Path,
                        default=Path('research_results/MH01_euroc_full'))
    parser.add_argument('--groundtruth-csv', type=Path,
                        default=Path('research_data/MH_01_easy_gt.csv'))
    parser.add_argument('--max-time-error-ms', type=float, default=20.0)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if args.max_time_error_ms <= 0:
        parser.error('--max-time-error-ms must be positive')

    summary = json.loads((args.run_dir / 'summary.json').read_text())
    poses = np.loadtxt(args.run_dir / 'online_frames.tum', comments='#')
    if poses.ndim != 2 or poses.shape[1] != 8:
        raise ValueError('Expected 8-column online_frames.tum')
    if np.any(np.diff(poses[:, 0]) <= 0):
        raise ValueError('Estimated timestamps are not strictly increasing')
    T_imu_camera = np.asarray(summary['T_imu_camera'], dtype=float)
    if T_imu_camera.shape != (4, 4):
        raise ValueError('Expected a 4x4 T_imu_camera in summary.json')
    reference_times, reference_positions, reference_rotations = (
        euroc_camera_groundtruth(args.groundtruth_csv, T_imu_camera))
    init = summary.get('initialization_frame')
    if init is not None and not 0 <= init < len(poses):
        raise ValueError('Initialization index is outside the online trajectory')
    tolerance = args.max_time_error_ms / 1000
    results = {
        'protocol': ('Nearest ground truth within stated tolerance; reference IMU '
                     'poses converted to camera centers; least-squares position '
                     'fits over all matched poses in each window. This is a '
                     'diagnostic, not a reproduction of the unspecified paper '
                     'RPG alignment configuration.'),
        'run_dir': str(args.run_dir),
        'groundtruth_csv': str(args.groundtruth_csv),
        'max_time_error_ms': args.max_time_error_ms,
        'initialization_frame_zero_based': init,
        'whole_recording': evaluate(poses, reference_times, reference_positions,
                                    reference_rotations, 0, tolerance),
        'post_initialization': (evaluate(poses, reference_times,
                                         reference_positions, reference_rotations,
                                         init, tolerance)
                                if init is not None else None),
    }
    expected = summary.get('comparison_after_initialization', {})
    if results['post_initialization'] and 'se3_aligned_ate_rmse_m' in expected:
        observed = results['post_initialization']['se3_all_positions']['ate_rmse_m']
        if not np.isclose(observed, expected['se3_aligned_ate_rmse_m'], atol=1e-8):
            raise ValueError('SE(3) audit disagrees with run summary')
    payload = json.dumps(results, indent=2) + '\n'
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(payload)
    print(payload, end='')


if __name__ == '__main__':
    main()
