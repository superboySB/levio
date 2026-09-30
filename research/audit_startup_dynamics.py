"""Audit startup motion and recorded timing without claiming flight ground truth.

Run inside the research Docker image after run_all.sh. This script reads only
the already downloaded, default-cohort bags. Heights are differences in the
recorded fusion-odometry ``world`` frame, never height above the floor.
"""

import argparse
import json
from pathlib import Path

import numpy as np
import rosbag


ROOT = Path(__file__).resolve().parents[1]
RGB = '/camera/color/image_raw'
IMU = '/mavros/imu/data_raw'
REFERENCE = '/fusion_odometry/lazy_point_odom'


def default_labels(manifest):
    return sorted(line.split('\t')[0] for line in manifest.read_text().splitlines()
                  if line and not line.startswith('#') and
                  line.split('\t')[-1] == 'default')


def nearest_index(times, query):
    return int(np.argmin(np.abs(times - query)))


def reference_interval(times, xyz, origin, seconds):
    start = nearest_index(times, origin)
    finish = nearest_index(times, origin + seconds)
    segment = xyz[start:finish + 1]
    dt = np.diff(times[start:finish + 1])
    steps = np.linalg.norm(np.diff(segment, axis=0), axis=1)
    valid = dt > 1e-6
    speeds = steps[valid] / dt[valid]
    return {
        'requested_duration_s': seconds,
        'observed_duration_s': float(times[finish] - times[start]),
        'horizontal_net_m': float(np.linalg.norm(segment[-1, :2] - segment[0, :2])),
        'vertical_world_delta_m': float(segment[-1, 2] - segment[0, 2]),
        'net_displacement_m': float(np.linalg.norm(segment[-1] - segment[0])),
        'path_length_m': float(steps.sum()),
        'max_reference_step_m': float(steps.max()) if len(steps) else None,
        'median_reference_speed_m_s': float(np.median(speeds)) if len(speeds) else None,
    }


def imu_interval(times, gyro, accel, start, duration):
    use = (times >= start) & (times < start + duration)
    g, a = gyro[use], accel[use]
    if not len(g):
        return None
    gn = np.linalg.norm(g, axis=1)
    an = np.linalg.norm(a, axis=1)
    return {
        'messages': int(len(g)),
        'gyro_norm_median_rad_s': float(np.median(gn)),
        'gyro_norm_p95_rad_s': float(np.percentile(gn, 95)),
        'gyro_over_0_1_rad_s_fraction': float(np.mean(gn > 0.1)),
        'accel_norm_median_m_s2': float(np.median(an)),
        'accel_norm_std_m_s2': float(np.std(an)),
    }


def audit_one(label, bag_path, results_root):
    run_dir = results_root / f'{label}_color_20hz'
    summary = json.loads((run_dir / 'summary.json').read_text())
    existing = json.loads((results_root / 'rgb_audit' / f'{label}.json').read_text())
    capture = np.atleast_1d(np.loadtxt(run_dir / 'capture_times.txt'))
    keyframes = np.atleast_2d(np.loadtxt(run_dir / 'keyframes.tum', comments='#'))
    if len(capture) != summary['frames_processed'] or len(keyframes) < 2:
        raise ValueError(f'{label}: saved frame or keyframe count is inconsistent')
    with rosbag.Bag(str(bag_path)) as bag:
        topics = bag.get_type_and_topic_info().topics
        reference_t, reference_xyz, imu_t, imu_gyro, imu_accel = [], [], [], [], []
        for topic, msg, _ in bag.read_messages(topics=[REFERENCE, IMU]):
            if topic == REFERENCE:
                p = msg.pose.pose.position
                reference_t.append(msg.header.stamp.to_sec())
                reference_xyz.append((p.x, p.y, p.z))
            else:
                g, a = msg.angular_velocity, msg.linear_acceleration
                imu_t.append(msg.header.stamp.to_sec())
                imu_gyro.append((g.x, g.y, g.z))
                imu_accel.append((a.x, a.y, a.z))
    reference_t = np.asarray(reference_t)
    reference_xyz = np.asarray(reference_xyz)
    imu_t = np.asarray(imu_t)
    imu_gyro = np.asarray(imu_gyro)
    imu_accel = np.asarray(imu_accel)
    first = float(capture[0])
    init_frame = summary['initialization_frame']
    init_s = float(capture[init_frame] - first) if init_frame is not None else None
    actual_second_s = float(keyframes[1, 0] - first)
    stored_first_shift_s = float(keyframes[0, 0] - first)
    if actual_second_s <= 0:
        raise ValueError(f'{label}: second keyframe predates the first RGB image')
    ref0 = nearest_index(reference_t, first)
    valid_ref = reference_xyz[(reference_t >= first) & (reference_t <= capture[-1])]
    time_offsets = {topic: existing['topics'][topic]['header_minus_bag_ms']
                    for topic in (RGB, IMU, REFERENCE)}
    intervals = {str(seconds): reference_interval(reference_t, reference_xyz,
                                                  first, seconds)
                 for seconds in (1, 2, 5)}
    intervals['first_to_second_keyframe'] = reference_interval(
        reference_t, reference_xyz, first, actual_second_s)
    if init_s is not None:
        intervals['first_to_initialization'] = reference_interval(
            reference_t, reference_xyz, first, init_s)
    return {
        'label': label,
        'bag': f'research_data/{label}.bag',
        'first_rgb_header_s': first,
        'recorded_duration_s': float(capture[-1] - first),
        'initialization_frame': init_frame,
        'initialization_from_first_rgb_s': init_s,
        'first_essential_skip_frame': summary['first_essential_skip_frame'],
        'first_reference_world_z_m': float(reference_xyz[ref0, 2]),
        'recorded_world_z_delta_min_max_m': [
            float(np.min(valid_ref[:, 2]) - reference_xyz[ref0, 2]),
            float(np.max(valid_ref[:, 2]) - reference_xyz[ref0, 2])],
        'actual_first_to_second_keyframe_s': actual_second_s,
        'stored_first_to_second_keyframe_s': float(keyframes[1, 0] - keyframes[0, 0]),
        'stored_first_keyframe_timestamp_shift_s': stored_first_shift_s,
        'standing_start_rewrite_detected': stored_first_shift_s > 1e-4,
        'reference_intervals': intervals,
        'imu_first_2_s': imu_interval(imu_t, imu_gyro, imu_accel, first, 2),
        'imu_first_5_s': imu_interval(imu_t, imu_gyro, imu_accel, first, 5),
        'rgb_to_nearest_raw_imu_ms_all_recorded': existing['rgb_to_nearest_imu'],
        'rgb_to_nearest_raw_imu_ms_common_interval':
            existing['strict_common_interval']['rgb_to_nearest_imu'],
        'header_minus_bag_ms': time_offsets,
        'available_direct_altitude_topics': sorted(t for t in topics
            if any(token in t.lower() for token in
                   ('altitude', 'rangefinder', 'distance_sensor', 'height', 'ground_truth'))),
        'tof_topic_present': '/nlink_tofsensem_cascade' in topics,
        'depth_image_topic_present': '/camera/depth/image_rect_raw' in topics,
        'tof_fixed_extrinsic_documented': False,
        'reference_is_independent_ground_truth': False,
        'height_above_ground_measured': False,
    }


def fmt(value, decimals=2):
    return '—' if value is None else f'{value:.{decimals}f}'


def audit_euroc(results_root):
    run_dir = results_root / 'MH01_euroc_full'
    summary = json.loads((run_dir / 'summary.json').read_text())
    capture = np.atleast_1d(np.loadtxt(run_dir / 'capture_times.txt'))
    keyframes = np.atleast_2d(np.loadtxt(run_dir / 'keyframes.tum', comments='#'))
    reference = np.loadtxt(ROOT / summary['groundtruth_csv'], delimiter=',', comments='#')
    ref_t, ref_xyz = reference[:, 0] / 1e9, reference[:, 1:4]
    first = float(capture[0])
    imu_t, gyro, accel = [], [], []
    with rosbag.Bag(str(ROOT / 'research_data/MH_01_easy.bag')) as bag:
        for _, msg, _ in bag.read_messages(topics=['/imu0']):
            t = msg.header.stamp.to_sec()
            if t >= first + 2:
                break
            g, a = msg.angular_velocity, msg.linear_acceleration
            imu_t.append(t)
            gyro.append((g.x, g.y, g.z))
            accel.append((a.x, a.y, a.z))
    return {
        'label': 'EuRoC MH01',
        'source': 'independent EuRoC ground-truth CSV and original /imu0',
        'initialization_from_first_rgb_s': float(
            capture[summary['initialization_frame']] - first),
        'first_ground_truth_minus_first_rgb_s': float(ref_t[0] - first),
        'first_reference_world_z_m': float(ref_xyz[0, 2]),
        'reference_first_5_s': reference_interval(ref_t, ref_xyz, ref_t[0], 5),
        'imu_first_2_s': imu_interval(np.asarray(imu_t), np.asarray(gyro),
                                     np.asarray(accel), first, 2),
        'actual_first_to_second_keyframe_s': float(keyframes[1, 0] - first),
        'stored_first_to_second_keyframe_s': float(keyframes[1, 0] - keyframes[0, 0]),
        'height_above_ground_measured': False,
    }


def markdown(rows, euroc):
    lines = [
        '# Startup dynamics and recorded timestamp audit', '',
        'Computed from the existing complete bags with '
        '`python research/audit_startup_dynamics.py`. All height changes use '
        'the recorded fusion odometry `world` z axis relative to the first '
        'processed RGB frame. Most bags include a ToF cascade and all include depth images, '
        'but the released dataset lacks a fixed ToF extrinsic and a labeled '
        'floor plane, so these streams have not been converted into a calibrated '
        'height above ground. Neither positive world z nor its magnitude alone '
        'proves takeoff. Fusion odometry is not independently verified ground truth.', '',
        '| Bag | Duration s | Init after RGB s | z change first 5 s m | '
        'XYZ path first 5 s m | gyro p95 first 2 s rad/s | '
        'actual first→second KF s | stored first→second KF s |',
        '|---|---:|---:|---:|---:|---:|---:|---:|',
    ]
    for r in rows:
        first5 = r['reference_intervals']['5']
        lines.append('| {label} | {duration} | {init} | {z} | {path} | {gyro} | '
                     '{actual} | {stored} |'.format(
                         label=r['label'], duration=fmt(r['recorded_duration_s'], 1),
                         init=fmt(r['initialization_from_first_rgb_s'], 1),
                         z=fmt(first5['vertical_world_delta_m'], 3),
                         path=fmt(first5['path_length_m'], 3),
                         gyro=fmt(r['imu_first_2_s']['gyro_norm_p95_rad_s'], 3),
                         actual=fmt(r['actual_first_to_second_keyframe_s'], 3),
                         stored=fmt(r['stored_first_to_second_keyframe_s'], 3)))
    lines += ['', 'The first-keyframe timestamp rewrite is triggered when the '
              'second keyframe has frame ID > 10. This audit detects the rewrite '
              'from the saved keyframe timestamp; it does not infer physical '
              'stationarity. RGB-to-IMU header proximity only tests recorded '
              'clock coverage. `header_minus_bag_ms` measures writing latency, '
              'not sensor hardware time offset. JSON contains all per-run '
              'intervals, world-z ranges and timestamp statistics.', '',
              'EuRoC MH01 is a control using its independent ground-truth CSV. '
              f'The ground truth starts '
              f'{euroc["first_ground_truth_minus_first_rgb_s"]:.3f} s after '
              f'the first RGB image; its first 5 s path is '
              f'{euroc["reference_first_5_s"]["path_length_m"]:.3f} m, '
              f'the first 2 s gyro p95 is '
              f'{euroc["imu_first_2_s"]["gyro_norm_p95_rad_s"]:.3f} rad/s, '
              f'initialization occurs at {euroc["initialization_from_first_rgb_s"]:.2f} s, '
              f'and the actual/stored first-to-second keyframe intervals are both '
              f'{euroc["actual_first_to_second_keyframe_s"]:.3f} s. '
              'It does not trigger the 0.5 s startup timestamp rewrite. '
              'EuRoC world z also does not establish floor height.', '']
    return '\n'.join(lines)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--manifest', type=Path,
                        default=ROOT / 'research/metadata/bags.tsv')
    parser.add_argument('--results-root', type=Path,
                        default=ROOT / 'research_results')
    parser.add_argument('--json-output', type=Path,
                        default=ROOT / 'research_results/startup_dynamics.json')
    parser.add_argument('--markdown-output', type=Path,
                        default=ROOT / 'research_results/startup_dynamics.md')
    parser.add_argument('--euroc-json-output', type=Path,
                        default=ROOT / 'research_results/startup_euroc.json')
    args = parser.parse_args()
    rows = [audit_one(label, ROOT / 'research_data' / f'{label}.bag',
                      args.results_root) for label in default_labels(args.manifest)]
    euroc = audit_euroc(args.results_root)
    args.json_output.parent.mkdir(parents=True, exist_ok=True)
    args.json_output.write_text(json.dumps(rows, indent=2) + '\n')
    args.euroc_json_output.write_text(json.dumps(euroc, indent=2) + '\n')
    args.markdown_output.write_text(markdown(rows, euroc))
    print(f'Audited {len(rows)} default-cohort bags: {args.markdown_output}')


if __name__ == '__main__':
    main()
