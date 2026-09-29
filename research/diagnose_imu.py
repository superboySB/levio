"""Summarize timestamp and motion statistics without changing LEVIO inputs."""

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import rosbag

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'levio_python_model'))
from main_levio import VIOSystem  # noqa: E402
from run_odom import bag_calibration, reference_poses  # noqa: E402
from run_euroc import ground_truth_poses  # noqa: E402


def percentiles(data):
    return {name: float(np.percentile(data, p)) for name, p in
            [('median', 50), ('p90', 90), ('p99', 99)]}


def run(args):
    with rosbag.Bag(args.bag) as bag:
        if args.dataset == 'euroc':
            image_topic = '/cam0/image_raw'
            T_imu_camera = VIOSystem().graph.optimizer.cam_to_imu_tf
            gt_t, gt_xyz = ground_truth_poses(args.groundtruth_csv, T_imu_camera)
        else:
            image_topic, _, _, _, T_imu_camera = bag_calibration(bag, args.camera)
            gt_t, gt_xyz = reference_poses(bag, T_imu_camera)
        first_image = next(bag.read_messages(topics=[image_topic]))[1].header.stamp.to_sec()
        end_time = first_image + args.max_seconds
        image_t = np.array([m.header.stamp.to_sec() for _, m, _ in
                            bag.read_messages(topics=[image_topic])
                            if first_image <= m.header.stamp.to_sec() <= end_time])
        imu_t, accelerations, gyros = [], [], []
        frame_ids = set()
        for _, m, _ in bag.read_messages(topics=[args.imu_topic]):
            stamp = m.header.stamp.to_sec()
            if stamp < first_image or stamp > end_time:
                continue
            imu_t.append(stamp)
            accelerations.append([m.linear_acceleration.x, m.linear_acceleration.y,
                                  m.linear_acceleration.z])
            gyros.append([m.angular_velocity.x, m.angular_velocity.y,
                          m.angular_velocity.z])
            frame_ids.add(m.header.frame_id)
    imu_t = np.array(imu_t)
    imu_dt = np.diff(imu_t)
    acc = np.array(accelerations)
    gyro = np.array(gyros)
    ix = np.searchsorted(imu_t, image_t)
    ix = np.clip(ix, 1, len(imu_t) - 1)
    nearest = np.minimum(abs(imu_t[ix] - image_t), abs(imu_t[ix - 1] - image_t))
    camera_acc = (np.linalg.inv(T_imu_camera)[:3, :3] @ acc.T).T
    in_window = (gt_t >= first_image) & (gt_t <= end_time)
    reference = gt_xyz[in_window]
    report = {
        'dataset': args.dataset, 'bag': args.bag, 'imu_topic': args.imu_topic,
        'image_topic': image_topic, 'window_s': args.max_seconds,
        'images': len(image_t), 'imu_messages': len(imu_t),
        'imu_frame_ids': sorted(frame_ids),
        'imu_dt_ms': percentiles(imu_dt * 1000),
        'imu_min_dt_ms': float(np.min(imu_dt) * 1000),
        'imu_max_dt_ms': float(np.max(imu_dt) * 1000),
        'imu_nonpositive_dt_count': int(np.count_nonzero(imu_dt <= 0)),
        'imu_first_nonpositive_pair_s': (
            imu_t[np.flatnonzero(imu_dt <= 0)[0]:np.flatnonzero(imu_dt <= 0)[0] + 2].tolist()
            if np.any(imu_dt <= 0) else None),
        'nearest_image_imu_ms': percentiles(nearest * 1000),
        'acceleration_norm_mps2': percentiles(np.linalg.norm(acc, axis=1)),
        'gyro_norm_radps': percentiles(np.linalg.norm(gyro, axis=1)),
        'mean_camera_frame_accel_first_2s': camera_acc[imu_t < first_image + 2].mean(axis=0).tolist(),
        'reference_samples': len(reference),
        'reference_path_length_m': float(np.linalg.norm(np.diff(reference, axis=0),
                                                       axis=1).sum()),
        'reference_net_displacement_m': float(np.linalg.norm(reference[-1] - reference[0])),
    }
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--bag', required=True)
    parser.add_argument('--dataset', choices=['euroc', 'odom'], required=True)
    parser.add_argument('--camera', choices=['color', 'infra1'], default='color')
    parser.add_argument('--imu-topic', required=True)
    parser.add_argument('--groundtruth-csv', default='research_data/MH_01_easy_gt.csv')
    parser.add_argument('--max-seconds', type=float, default=35)
    parser.add_argument('--output', required=True)
    run(parser.parse_args())
