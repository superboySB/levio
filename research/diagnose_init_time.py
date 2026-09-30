"""Audit the original first-keyframe timestamp and motion at the second keyframe."""

import argparse
import json
from pathlib import Path

import numpy as np
import rosbag

from run_odom import bag_calibration, reference_poses


def run(args):
    with rosbag.Bag(args.bag) as bag:
        topic, _, _, _, T_base_camera = bag_calibration(bag, args.camera)
        raw_first_image_t = next(bag.read_messages(topics=[topic]))[1].header.stamp.to_sec()
        ref_t, ref_xyz = reference_poses(bag, T_base_camera)
    capture_times = np.atleast_1d(np.loadtxt(Path(args.run_dir) / 'capture_times.txt'))
    if len(capture_times) < 2:
        raise ValueError('At least two processed RGB capture times are required')
    first_image_t = float(capture_times[0])
    keyframes = np.loadtxt(Path(args.run_dir) / 'keyframes.tum', comments='#')
    if keyframes.ndim == 1 or len(keyframes) < 2:
        raise ValueError('At least two keyframes are required')
    rewritten_first_t, second_t = keyframes[:2, 0]
    if second_t <= first_image_t:
        raise ValueError('Second keyframe predates first image')
    first_ref = int(np.argmin(abs(ref_t - first_image_t)))
    last_ref = int(np.argmin(abs(ref_t - second_t)))
    segment = ref_xyz[first_ref:last_ref + 1]
    report = {
        'bag': args.bag, 'run_dir': args.run_dir,
        'raw_first_image_timestamp_s': raw_first_image_t,
        'first_processed_image_timestamp_s': first_image_t,
        'raw_to_processed_first_image_s': first_image_t - raw_first_image_t,
        'stored_first_keyframe_timestamp_s': rewritten_first_t,
        'second_keyframe_timestamp_s': second_t,
        'actual_first_to_second_keyframe_s': second_t - first_image_t,
        'stored_first_to_second_keyframe_s': second_t - rewritten_first_t,
        'first_timestamp_shift_s': rewritten_first_t - first_image_t,
        'recorded_odometry_net_displacement_m': float(np.linalg.norm(segment[-1] - segment[0])),
        'recorded_odometry_path_length_m': float(np.linalg.norm(np.diff(segment, axis=0),
                                                                axis=1).sum()),
        'reference_is_ground_truth': False,
    }
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--bag', required=True)
    parser.add_argument('--run-dir', required=True)
    parser.add_argument('--camera', choices=['color', 'infra1'], default='color')
    parser.add_argument('--output', required=True)
    run(parser.parse_args())
