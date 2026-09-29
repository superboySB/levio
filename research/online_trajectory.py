"""Immutable per-frame outputs for a causal VIO trajectory evaluation."""

from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


class OnlineTrajectory:
    def __init__(self):
        self.times = []
        self.positions = []
        self.rotations = []

    def record(self, frame, capture_time):
        # LEVIO stores T_camera_world. Copy its inverse immediately after this
        # frame is processed; later graph optimization may mutate frame.pose.
        T_world_camera = np.linalg.inv(frame.pose.copy())
        self.times.append(float(capture_time))
        self.positions.append(T_world_camera[:3, 3].copy())
        self.rotations.append(T_world_camera[:3, :3].copy())

    def save(self, path):
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open('w') as handle:
            handle.write('# time x y z qx qy qz qw\n')
            for stamp, xyz, matrix in zip(self.times, self.positions, self.rotations):
                quat = Rotation.from_matrix(matrix).as_quat()
                handle.write(' '.join(str(value) for value in
                                      (stamp, *xyz, *quat)) + '\n')
