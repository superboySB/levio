"""Run the unmodified EuRoC LEVIO configuration on MH_01_easy."""

import argparse
import json
import os
import sys
import traceback
from pathlib import Path

import numpy as np
import rosbag

ROOT = Path(__file__).resolve().parents[1]
MODEL_DIR = Path(os.environ.get('LEVIO_MODEL_DIR', ROOT / 'levio_python_model'))
sys.path.insert(0, str(MODEL_DIR))
from main_levio import VIOSystem  # noqa: E402
from utilities.draw_trajectory import TrajectoryVisualizer  # noqa: E402
from utilities.rosbag_extractor import RosbagExtractor  # noqa: E402
from run_odom import aligned_trajectories, matrix  # noqa: E402
from runtime_trace import RuntimeMatchTrace  # noqa: E402
from online_trajectory import OnlineTrajectory  # noqa: E402


def ground_truth_poses(csv_path, T_imu_camera, with_rotations=False):
    # OpenVINS mirrors EuRoC's state_groundtruth_estimate0/data.csv. Its pose is
    # T_world_imu; convert to the camera center before trajectory comparison.
    data = np.loadtxt(csv_path, delimiter=',', comments='#')
    timestamps = data[:, 0] / 1e9
    positions, rotations = [], []
    for row in data:
        T_world_imu = matrix(row[1:4], [row[5], row[6], row[7], row[4]])
        T_world_camera = T_world_imu @ T_imu_camera
        positions.append(T_world_camera[:3, 3])
        rotations.append(T_world_camera[:3, :3])
    if with_rotations:
        return timestamps, np.array(positions), np.array(rotations)
    return timestamps, np.array(positions)


def run(args):
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    (output / 'failure.txt').unlink(missing_ok=True)
    system = VIOSystem()  # EuRoC K, D, and T_cam_imu from the original model.
    trace = RuntimeMatchTrace(system) if args.trace_output else None
    online = OnlineTrajectory()
    system.draw_each_frame = False
    system.visualization = TrajectoryVisualizer(800, 800, 400, 400, name='MH01 cam0')
    # Original upstream code always calls these two display methods. Disabling
    # their file output does not change feature detection or pose estimation.
    system.visualization.update_frame = lambda *a, **k: None
    system.visualization.draw = lambda *a, **k: None
    T_imu_camera = system.graph.optimizer.cam_to_imu_tf.copy()
    gt_t, gt_xyz, gt_rotations = ground_truth_poses(
        args.groundtruth_csv, T_imu_camera, with_rotations=True)

    with rosbag.Bag(args.bag) as bag:
        if '/cam0/image_raw' not in bag.get_type_and_topic_info().topics:
            raise ValueError('Missing EuRoC /cam0/image_raw')
    extractor = RosbagExtractor(args.bag, '/cam0/image_raw', '/imu0')
    system.graph.optimizer.set_imu_data_loader(extractor.imu_generator())
    failure = None
    initialization_frame = None
    processed = 0
    capture_times = []
    for image, stamp in extractor.img_generator():
        if args.max_frames and processed >= args.max_frames:
            break
        try:
            initialized_before = system.graph.is_initialized
            system.process_frame(image, stamp)
            if not initialized_before and system.graph.is_initialized:
                initialization_frame = processed
            processed += 1
            capture_times.append(stamp)
            online.record(system.graph.frames[-1], stamp)
            if trace is not None:
                trace.record(stamp)
        except Exception:
            failure = traceback.format_exc()
            (output / 'failure.txt').write_text(failure)
            break
    extractor.bag.close()
    if trace is not None:
        trace.save(args.trace_output)

    system.visualization.save_stamped_poses_to_file(system.graph.frames,
                                                      str(output / 'frames.tum'))
    system.visualization.save_stamped_poses_to_file(system.graph.keyframes,
                                                      str(output / 'keyframes.tum'))
    online.save(output / 'online_frames.tum')
    np.savetxt(output / 'capture_times.txt', capture_times, fmt='%.9f')
    before_timing = aligned_trajectories(online.times, online.positions, gt_t, gt_xyz,
                                         online.rotations, gt_rotations)[1]
    before = {key: before_timing[key] for key in
              ('matched_frames', 'max_sync_error_ms') if key in before_timing}
    before.update(metric_scale_valid=False, diagnostic_only=True,
                  reason='includes pre-initialization frames')
    after = aligned_trajectories(
        online.times[initialization_frame:] if initialization_frame is not None else [],
        online.positions[initialization_frame:] if initialization_frame is not None else [],
        gt_t, gt_xyz,
        online.rotations[initialization_frame:] if initialization_frame is not None else [],
        gt_rotations)[1]
    after['metric_scale_valid'] = initialization_frame is not None
    if 'reference_is_ground_truth' in after:
        after['reference_is_ground_truth'] = True
    summary = {
        'dataset': 'euroc', 'bag': args.bag, 'groundtruth_csv': args.groundtruth_csv,
        'image_topic': '/cam0/image_raw', 'imu_topic': '/imu0',
        'reference_topic': 'state_groundtruth_estimate0/data.csv',
        'capture_time_source': 'original ROS image header.stamp before VIO mutation',
        'evaluated_trajectory': 'online_frames.tum: pose copied after each processed frame',
        'image_size': [system.W, system.H], 'K': system.K.tolist(),
        'D': system.D.tolist(), 'T_imu_camera': T_imu_camera.tolist(),
        'frames_processed': processed, 'keyframes': len(system.graph.keyframes),
        'skipped_essential_frames': getattr(system, 'skipped_essential_frames', None),
        'model_dir': str(MODEL_DIR),
        'vio_initialized': bool(system.graph.is_initialized),
        'initialization_frame': initialization_frame,
        'comparison_to_euroc_groundtruth': before,
        'comparison_after_initialization': after,
        'failure': failure.strip().splitlines()[-1] if failure else None,
        'evaluation_note': 'SE(3) camera-center ATE with 20 ms nearest GT match; '
                           'not the paper RPG evaluation protocol',
    }
    (output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps(summary, indent=2))
    return 1 if failure else 0


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--bag', default='research_data/MH_01_easy.bag')
    parser.add_argument('--groundtruth-csv', default='research_data/MH_01_easy_gt.csv')
    parser.add_argument('--output', required=True)
    parser.add_argument('--trace-output', help='Optional CSV of actual runtime match pairs')
    parser.add_argument('--max-frames', type=int, default=0, help='0 = full bag')
    sys.exit(run(parser.parse_args()))
