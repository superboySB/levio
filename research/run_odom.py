"""Run the LEVIO Python model on one odom_dataset ROS1 bag.

All sensor calibration is taken from the checked-in dataset calibration snapshot
and the selected bag. The resulting trajectory and diagnostics go to --output.
"""

import argparse
import json
import sys
import traceback
from pathlib import Path

import numpy as np
import rosbag
import yaml
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'levio_python_model'))
from main_levio import VIOSystem  # noqa: E402
from utilities.draw_trajectory import TrajectoryVisualizer  # noqa: E402
from utilities.rosbag_extractor import RosbagExtractor  # noqa: E402
from runtime_trace import RuntimeMatchTrace  # noqa: E402
from online_trajectory import OnlineTrajectory  # noqa: E402

CAMERAS = {
    'color': ('/camera/color/image_raw', '/camera/color/camera_info', 'camera_color_optical_frame'),
    'infra1': ('/camera/infra1/image_rect_raw', '/camera/infra1/camera_info', 'camera_infra1_optical_frame'),
}
IMU_TOPIC = '/mavros/imu/data_raw'
ODOM_TOPIC = '/fusion_odometry/lazy_point_odom'


def matrix(translation, quaternion):
    result = np.eye(4)
    result[:3, :3] = Rotation.from_quat(quaternion).as_matrix()
    result[:3, 3] = translation
    return result


def bag_calibration(bag, camera):
    image_topic, info_topic, optical_frame = CAMERAS[camera]
    infos = list(bag.read_messages(topics=[info_topic]))
    if not infos:
        raise ValueError(f'Missing camera_info: {info_topic}')
    info = infos[0][1]
    if info.header.frame_id != optical_frame or info.distortion_model != 'plumb_bob':
        raise ValueError(f'Unexpected camera calibration: {info.header.frame_id}, {info.distortion_model}')
    K = np.array(info.K).reshape(3, 3)
    D = np.array(info.D)
    for _, other, _ in infos[1:]:
        if not np.allclose(K, np.array(other.K).reshape(3, 3)) or not np.allclose(D, other.D):
            raise ValueError('Camera calibration changed within the bag')

    # The bag's RealSense TF tree is rooted at camera_link, not base_link.
    # Anchor it using the published base_link -> infra1 optical calibration.
    source = yaml.safe_load((ROOT / 'research/calibration/camera_infra1.yaml').read_text())
    T_base_infra = np.array(source['transform_matrix_row_major']['data'])
    edges = {}
    for _, message, _ in bag.read_messages(topics=['/tf_static']):
        for stamped in message.transforms:
            t = stamped.transform.translation
            q = stamped.transform.rotation
            edges[(stamped.header.frame_id, stamped.child_frame_id)] = matrix(
                [t.x, t.y, t.z], [q.x, q.y, q.z, q.w])

    def camera_link_to(target):
        from collections import deque
        queue = deque([('camera_link', np.eye(4))])
        seen = set()
        while queue:
            node, transform = queue.popleft()
            if node == target:
                return transform
            if node in seen:
                continue
            seen.add(node)
            for (parent, child), edge in edges.items():
                if node == parent:
                    queue.append((child, transform @ edge))
                if node == child:
                    queue.append((parent, transform @ np.linalg.inv(edge)))
        raise ValueError(f'Cannot resolve camera_link to {target} in bag TF')

    T_link_infra = camera_link_to('camera_infra1_optical_frame')
    T_link_selected = camera_link_to(optical_frame)
    T_base_camera = T_base_infra @ np.linalg.inv(T_link_infra) @ T_link_selected
    # The recorded MAVROS raw IMU messages have frame_id=base_link. Thus
    # R_imu_camera equals R_base_camera. LEVIO uses only this rotation.
    return image_topic, info, K, D, T_base_camera


def reference_poses(bag, T_base_camera, with_rotations=False):
    stamps, positions, rotations = [], [], []
    for _, msg, _ in bag.read_messages(topics=[ODOM_TOPIC]):
        if msg.child_frame_id != 'base_link':
            raise ValueError(f'Unexpected odometry child frame: {msg.child_frame_id}')
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        T_world_body = matrix([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])
        T_world_camera = T_world_body @ T_base_camera
        stamps.append(msg.header.stamp.to_sec())
        positions.append(T_world_camera[:3, 3])
        rotations.append(T_world_camera[:3, :3])
    if with_rotations:
        return np.array(stamps), np.array(positions), np.array(rotations)
    return np.array(stamps), np.array(positions)


def aligned_trajectories(image_t, estimates, reference_t, reference_xyz,
                         estimate_rotations=None, reference_rotations=None):
    """Match capture times and compute both global and start-anchored SE(3) fits."""
    if len(image_t) < 2 or len(reference_t) < 2:
        return None, {'matched_frames': 0}
    image_t = np.asarray(image_t)
    estimates = np.asarray(estimates)
    if len(image_t) != len(estimates):
        raise ValueError('Capture timestamp count differs from pose count')
    indexes = np.searchsorted(reference_t, image_t)
    indexes = np.clip(indexes, 1, len(reference_t) - 1)
    indexes -= np.abs(reference_t[indexes - 1] - image_t) < np.abs(reference_t[indexes] - image_t)
    valid = np.abs(reference_t[indexes] - image_t) <= 0.02
    est, ref = estimates[valid], reference_xyz[indexes[valid]]
    if len(est) < 3:
        return None, {'matched_frames': int(len(est))}
    # Row-vector Kabsch: global translation minimizes total squared error.
    X, Y = est - est.mean(axis=0), ref - ref.mean(axis=0)
    U, _, Vt = np.linalg.svd(X.T @ Y)
    reflect = np.diag([1, 1, np.linalg.det(U @ Vt)])
    R = U @ reflect @ Vt
    global_fit = X @ R + ref.mean(axis=0)
    start_fit = (est - est[0]) @ R + ref[0]
    estimate_length = float(np.linalg.norm(np.diff(est, axis=0), axis=1).sum())
    reference_length = float(np.linalg.norm(np.diff(ref, axis=0), axis=1).sum())
    metrics = {
        'matched_frames': int(len(est)),
        'max_sync_error_ms': float(1000 * np.max(np.abs(reference_t[indexes[valid]] - image_t[valid]))),
        'se3_aligned_ate_rmse_m': float(np.sqrt(np.mean(np.sum((global_fit - ref) ** 2, axis=1)))),
        'start_anchored_rmse_m': float(np.sqrt(np.mean(np.sum((start_fit - ref) ** 2, axis=1)))),
        'global_fit_start_offset_m': float(np.linalg.norm(global_fit[0] - ref[0])),
        'start_anchored_end_error_m': float(np.linalg.norm(start_fit[-1] - ref[-1])),
        'estimated_path_length_m': estimate_length,
        'reference_path_length_m': reference_length,
        'path_length_ratio': estimate_length / reference_length if reference_length else None,
        'reference_is_ground_truth': False,
    }
    initial_pose_fit = None
    if estimate_rotations is not None and reference_rotations is not None:
        R_est0 = np.asarray(estimate_rotations)[valid][0]
        R_ref0 = np.asarray(reference_rotations)[indexes[valid]][0]
        R_initial = R_ref0 @ R_est0.T
        initial_pose_fit = (R_initial @ (est - est[0]).T).T + ref[0]
        initial_error = np.linalg.norm(initial_pose_fit - ref, axis=1)
        end_error = float(initial_error[-1])
        metrics.update({
            'initial_pose_aligned_rmse_m': float(np.sqrt(np.mean(initial_error ** 2))),
            'initial_pose_aligned_start_error_m': float(initial_error[0]),
            'initial_pose_aligned_end_error_m': end_error,
            'initial_pose_alignment_rotation_deg': float(np.degrees(
                np.arccos(np.clip((np.trace(R_initial) - 1) / 2, -1, 1)))),
            'endpoint_drift_rate_percent': (end_error / reference_length * 100
                                             if reference_length else None),
        })
    samples = {'times': image_t[valid], 'reference': ref,
               'global_fit': global_fit, 'start_fit': start_fit,
               'initial_pose_fit': initial_pose_fit}
    return samples, metrics


def comparison(frames, reference_t, reference_xyz, capture_times=None,
               reference_rotations=None):
    if len(frames) < 2 or len(reference_t) == 0:
        return {'matched_frames': 0}
    image_t = np.array(capture_times if capture_times is not None else
                       [frame.t for frame in frames])
    estimates = np.array([np.linalg.inv(frame.pose)[:3, 3] for frame in frames])
    estimate_rotations = np.array([np.linalg.inv(frame.pose)[:3, :3]
                                   for frame in frames])
    return aligned_trajectories(image_t, estimates, reference_t, reference_xyz,
                                estimate_rotations, reference_rotations)[1]


def run(args):
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    (output / 'failure.txt').unlink(missing_ok=True)
    with rosbag.Bag(args.bag) as bag:
        image_topic, info, K, D, T_base_camera = bag_calibration(bag, args.camera)
        first_imu = next(bag.read_messages(topics=[args.imu_topic]))[1]
        first_reference = next(bag.read_messages(topics=[ODOM_TOPIC]))[1]
        topic_info = bag.get_type_and_topic_info().topics
        reference_callers = sorted({
            connection.header.get('callerid', b'').decode()
            for connection in bag._get_connections()
            if connection.topic == ODOM_TOPIC
        })
        if first_imu.header.frame_id != 'base_link':
            raise ValueError(f'Expected base_link IMU, got {first_imu.header.frame_id}')
        reference_t, reference_xyz, reference_rotations = reference_poses(
            bag, T_base_camera, with_rotations=True)
        extractor = RosbagExtractor(args.bag, image_topic, args.imu_topic)
        system = VIOSystem()
        trace = RuntimeMatchTrace(system) if args.trace_output else None
        online = OnlineTrajectory()
        system.use_optimization = not args.disable_optimization
        system.rewrite_first_timestamp = not args.preserve_first_timestamp
        if args.general_initialization:
            system.standing_start_time_shortcut = False
            system.rewrite_first_timestamp = False
            system.vio_initializer.allow_standing_start = False
        system.K, system.D = K, D
        system.W, system.H = info.width, info.height
        system.draw_each_frame = False
        system.graph.optimizer.cam_to_imu_tf = T_base_camera
        system.graph.optimizer.imu_to_cam_tf = np.linalg.inv(T_base_camera)
        system.graph.optimizer.set_imu_data_loader(extractor.imu_generator())
        system.visualization = TrajectoryVisualizer(800, 800, 400, 400, name=args.camera)

        failure = None
        processed = 0
        capture_times = []
        initialization_frame = None
        first_essential_skip_frame = None
        next_camera_time = None
        for source_index, (image, stamp) in enumerate(extractor.img_generator()):
            if source_index % args.frame_step:
                continue
            if args.target_fps:
                if next_camera_time is None:
                    next_camera_time = stamp
                if stamp + 1e-6 < next_camera_time:
                    continue
                while next_camera_time <= stamp + 1e-6:
                    next_camera_time += 1.0 / args.target_fps
            if args.max_frames and processed >= args.max_frames:
                break
            try:
                was_initialized = system.graph.is_initialized
                skipped_before = system.skipped_essential_frames
                system.process_frame(image, stamp)
                if not was_initialized and system.graph.is_initialized:
                    initialization_frame = processed
                if (first_essential_skip_frame is None and
                        system.skipped_essential_frames > skipped_before):
                    first_essential_skip_frame = processed
                processed += 1
                capture_times.append(stamp)
                online.record(system.graph.frames[-1], stamp)
                if trace is not None:
                    trace.record(stamp)
            except Exception:
                failure = traceback.format_exc()
                (output / 'failure.txt').write_text(failure)
                break

        visualizer = system.visualization
        if trace is not None:
            trace.save(args.trace_output)
        visualizer.save_stamped_poses_to_file(system.graph.frames, str(output / 'frames.tum'))
        visualizer.save_stamped_poses_to_file(system.graph.keyframes, str(output / 'keyframes.tum'))
        visualizer.save_trajectory_visualization_to_file(str(output / 'trajectory.png'))
        online.save(output / 'online_frames.tum')
        np.savetxt(output / 'capture_times.txt', capture_times, fmt='%.9f')
        whole_timing = aligned_trajectories(
            online.times, online.positions, reference_t, reference_xyz,
            online.rotations, reference_rotations)[1]
        whole_comparison = {
            key: whole_timing[key] for key in ('matched_frames', 'max_sync_error_ms')
            if key in whole_timing
        }
        whole_comparison['metric_scale_valid'] = False
        whole_comparison['diagnostic_only'] = True
        whole_comparison['reason'] = ('visual-inertial scale not initialized'
                                      if initialization_frame is None else
                                      'includes pre-initialization frames')
        metric_end = first_essential_skip_frame if first_essential_skip_frame is not None else processed
        valid_metric_window = (initialization_frame is not None and
                               initialization_frame < metric_end)
        metric_start = initialization_frame if valid_metric_window else metric_end
        initialized_comparison = aligned_trajectories(
            online.times[metric_start:metric_end],
            online.positions[metric_start:metric_end],
            reference_t, reference_xyz,
            online.rotations[metric_start:metric_end],
            reference_rotations)[1]
        initialized_comparison['metric_scale_valid'] = valid_metric_window
        initialized_comparison['stops_before_first_essential_failure'] = (
            first_essential_skip_frame is not None and valid_metric_window)
        report = {
            'bag': str(args.bag), 'camera': args.camera, 'image_topic': image_topic,
            'frame_step': args.frame_step,
            'target_fps': args.target_fps,
            'imu_topic': args.imu_topic, 'reference_topic': ODOM_TOPIC,
            'optimization_enabled': not args.disable_optimization,
            'preserve_first_timestamp': not system.rewrite_first_timestamp,
            'general_initialization': args.general_initialization,
            'reference_frame_id': first_reference.header.frame_id,
            'reference_child_frame_id': first_reference.child_frame_id,
            'reference_publishers': reference_callers,
            'reference_algorithm': 'unverified from released metadata and bag',
            'lidar_messages': topic_info['/livox/lidar'].message_count
                if '/livox/lidar' in topic_info else 0,
            'reference_messages': len(reference_t),
            'reference_max_gap_s': float(np.max(np.diff(reference_t)))
                if len(reference_t) > 1 else None,
            'image_encoding': next(bag.read_messages(topics=[image_topic]))[1].encoding,
            'image_size': [info.width, info.height], 'K': K.tolist(), 'D': D.tolist(),
            'T_base_camera': T_base_camera.tolist(),
            'T_imu_camera_used_by_levio': T_base_camera.tolist(),
            'frames_processed': processed, 'keyframes': len(system.graph.keyframes),
            'skipped_essential_frames': system.skipped_essential_frames,
            'first_essential_skip_frame': first_essential_skip_frame,
            'landmarks': len(system.graph.points), 'vio_initialized': bool(system.graph.is_initialized),
            'initialization_frame': initialization_frame,
            'failure': failure.strip().splitlines()[-1] if failure else None,
            'capture_time_source': 'original ROS image header.stamp before VIO mutation',
            'evaluated_trajectory': 'online_frames.tum: pose copied after each processed frame',
            'comparison_to_recorded_odometry': whole_comparison,
            'comparison_after_initialization': initialized_comparison,
        }
        (output / 'summary.json').write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps(report, indent=2))
        return 1 if failure else 0


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--bag', required=True)
    parser.add_argument('--camera', choices=CAMERAS, default='color')
    parser.add_argument('--imu-topic', choices=[IMU_TOPIC, '/mavros/imu/data'],
                        default=IMU_TOPIC)
    parser.add_argument('--disable-optimization', action='store_true',
                        help='Ablate post-initialization GTSAM updates')
    parser.add_argument('--preserve-first-timestamp', action='store_true',
                        help='Ablate the original standing-start timestamp rewrite')
    parser.add_argument('--general-initialization', action='store_true',
                        help='Ablate both standing-start shortcuts and solve initial velocity')
    parser.add_argument('--output', required=True)
    parser.add_argument('--trace-output', help='Optional CSV of actual runtime match pairs')
    parser.add_argument('--max-frames', type=int, default=0, help='0 means all frames')
    parser.add_argument('--frame-step', type=int, default=1, help='Use every Nth image (default: all)')
    parser.add_argument('--target-fps', type=float, default=None,
                help='Select images on a time grid; IMU stays at its recorded rate')
    args = parser.parse_args()
    if args.frame_step < 1:
        parser.error('--frame-step must be positive')
    if args.target_fps is not None and args.target_fps <= 0:
        parser.error('--target-fps must be positive')
    if args.target_fps is not None and args.frame_step != 1:
        parser.error('--target-fps and --frame-step cannot be combined')
    raise SystemExit(run(args))
