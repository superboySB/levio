"""Synthetic check for fixed first-pose alignment and drift metrics."""

import numpy as np
from scipy.spatial.transform import Rotation

from run_odom import aligned_trajectories


def main():
    times = np.arange(4, dtype=float)
    reference = np.array([[0, 0, 0], [1, 0, 0], [1, 1, 0], [2, 1, 0]], float)
    arbitrary_rotation = Rotation.from_euler('xyz', [23, -11, 72], degrees=True).as_matrix()
    arbitrary_translation = np.array([4.5, -3.2, 7.1])
    estimate = (arbitrary_rotation @ reference.T).T + arbitrary_translation
    estimate_rotations = np.repeat(arbitrary_rotation[None], len(times), axis=0)
    reference_rotations = np.repeat(np.eye(3)[None], len(times), axis=0)
    samples, metric = aligned_trajectories(times, estimate, times, reference,
                                            estimate_rotations, reference_rotations)
    np.testing.assert_allclose(samples['initial_pose_fit'], reference, atol=1e-12)
    assert metric['initial_pose_aligned_start_error_m'] < 1e-12
    assert metric['initial_pose_aligned_rmse_m'] < 1e-12

    estimate[-1] += arbitrary_rotation @ np.array([0.25, 0, 0])
    samples, metric = aligned_trajectories(times, estimate, times, reference,
                                            estimate_rotations, reference_rotations)
    assert metric['initial_pose_aligned_start_error_m'] < 1e-12
    np.testing.assert_allclose(metric['initial_pose_aligned_end_error_m'], 0.25,
                               atol=1e-12)
    np.testing.assert_allclose(metric['initial_pose_aligned_rmse_m'], 0.125,
                               atol=1e-12)
    print('first-pose alignment and 0.25 m endpoint drift verified')


if __name__ == '__main__':
    main()
