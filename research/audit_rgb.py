"""Audit ROS1 RGB recording continuity before using a bag for VIO.

Reads timestamps and image metadata without decoding all pixels. Header time is
the time LEVIO uses; bag time and ROS sequence numbers provide independent checks.
"""

import argparse
import json
from pathlib import Path

import numpy as np
import rosbag

RGB = '/camera/color/image_raw'
IMU = '/mavros/imu/data_raw'
REFERENCE = '/fusion_odometry/lazy_point_odom'


def topic_stats(records):
    if not records:
        return None
    stamps = np.array([r['header_s'] for r in records])
    recorded = np.array([r['bag_s'] for r in records])
    dt = np.diff(stamps)
    result = {
        'count': len(records),
        'first_header_s': float(stamps[0]), 'last_header_s': float(stamps[-1]),
        'header_duration_s': float(stamps[-1] - stamps[0]),
        'first_bag_s': float(recorded[0]), 'last_bag_s': float(recorded[-1]),
        'min_dt_ms': float(np.min(dt) * 1000) if len(dt) else None,
        'median_dt_ms': float(np.median(dt) * 1000) if len(dt) else None,
        'p99_dt_ms': float(np.percentile(dt, 99) * 1000) if len(dt) else None,
        'max_dt_ms': float(np.max(dt) * 1000) if len(dt) else None,
        'nonpositive_dt_count': int(np.count_nonzero(dt <= 0)),
        'negative_dt_count': int(np.count_nonzero(dt < 0)),
        'duplicate_dt_count': int(np.count_nonzero(dt == 0)),
        'bag_time_nonpositive_count': int(np.count_nonzero(np.diff(recorded) <= 0)),
        'header_minus_bag_ms': {
            'median': float(np.median((stamps - recorded) * 1000)),
            'p01': float(np.percentile((stamps - recorded) * 1000, 1)),
            'p99': float(np.percentile((stamps - recorded) * 1000, 99)),
        },
    }
    if 'seq' in records[0]:
        seq = np.array([r['seq'] for r in records], dtype=np.int64)
        jumps = np.diff(seq)
        result['nonunit_sequence_increments'] = int(np.count_nonzero(jumps != 1))
        result['positive_sequence_skips'] = int(np.maximum(jumps - 1, 0).sum())
    return result


def nearest_timestamp_stats(query_stamps, sensor_stamps):
    """Summarize recorded timestamp proximity, without assuming clock calibration."""
    if not len(query_stamps) or not len(sensor_stamps):
        return None
    sorted_stamps = np.sort(sensor_stamps)
    right = np.searchsorted(sorted_stamps, query_stamps)
    left = np.clip(right - 1, 0, len(sorted_stamps) - 1)
    right = np.clip(right, 0, len(sorted_stamps) - 1)
    distance_ms = np.minimum(np.abs(query_stamps - sorted_stamps[left]),
                             np.abs(query_stamps - sorted_stamps[right])) * 1000
    return {
        'median_ms': float(np.median(distance_ms)),
        'p99_ms': float(np.percentile(distance_ms, 99)),
        'max_ms': float(np.max(distance_ms)),
    }


def run(args):
    records = {topic: [] for topic in (RGB, IMU, REFERENCE)}
    malformed = []
    with rosbag.Bag(args.bag) as bag:
        bag_span = [bag.get_start_time(), bag.get_end_time()]
        for topic, msg, bag_t in bag.read_messages(topics=list(records)):
            row = {'header_s': msg.header.stamp.to_sec(), 'bag_s': bag_t.to_sec(),
                   'seq': msg.header.seq}
            if topic == IMU:
                row['frame_id'] = msg.header.frame_id
            if topic == RGB:
                row['width'], row['height'], row['step'] = msg.width, msg.height, msg.step
                row['encoding'], row['bytes'] = msg.encoding, len(msg.data)
                if msg.step * msg.height != len(msg.data):
                    malformed.append({'index': len(records[topic]), 'issue': 'byte_length', **row})
            if topic == REFERENCE:
                p = msg.pose.pose.position
                row['xyz'] = [p.x, p.y, p.z]
            records[topic].append(row)

    rgb = records[RGB]
    if not rgb:
        raise ValueError(f'{args.bag}: missing {RGB}')
    stamps = np.array([r['header_s'] for r in rgb])
    dt = np.diff(stamps)
    imu_stamps = np.array([r['header_s'] for r in records[IMU]])
    reference = records[REFERENCE]
    reference_stamps = np.array([r['header_s'] for r in reference])
    common = None
    if len(imu_stamps) and len(reference_stamps):
        common_start = max(stamps[0], imu_stamps[0], reference_stamps[0])
        common_end = min(stamps[-1], imu_stamps[-1], reference_stamps[-1])
        common_stamps = stamps[(stamps >= common_start) & (stamps <= common_end)]
        common = {
            'start_header_s': float(common_start),
            'end_header_s': float(common_end),
            'rgb_frames': int(len(common_stamps)),
            'rgb_trimmed_start_frames': int(np.count_nonzero(stamps < common_start)),
            'rgb_trimmed_end_frames': int(np.count_nonzero(stamps > common_end)),
            'rgb_to_nearest_imu': nearest_timestamp_stats(common_stamps, imu_stamps),
        }
    repeated_reference = np.flatnonzero(np.diff(reference_stamps) == 0)
    reference_xyz = np.array([r['xyz'] for r in reference], dtype=float).reshape(-1, 3)
    reference_dt = np.diff(reference_stamps)
    reference_steps = np.linalg.norm(np.diff(reference_xyz, axis=0), axis=1)
    large_reference_steps = np.flatnonzero(reference_steps > 1.0)
    max_reference_step_index = (int(np.argmax(reference_steps))
                                if len(reference_steps) else None)
    duplicate_differences = np.linalg.norm(
        reference_xyz[repeated_reference + 1] - reference_xyz[repeated_reference], axis=1)
    nominal = 1 / args.nominal_fps
    gaps = []
    for index in np.flatnonzero(dt > args.gap_factor * nominal):
        gaps.append({
            'left_index': int(index), 'right_index': int(index + 1),
            'left_header_s': float(stamps[index]),
            'right_header_s': float(stamps[index + 1]),
            'gap_s': float(dt[index]),
            'estimated_missing_frames': max(0, int(round(dt[index] / nominal)) - 1),
            'sequence_increment': int(rgb[index + 1]['seq'] - rgb[index]['seq']),
            'bag_dt_s': rgb[index + 1]['bag_s'] - rgb[index]['bag_s'],
        })
    result = {
        'bag': args.bag, 'rgb_topic': RGB, 'imu_topic': IMU,
        'nominal_fps': args.nominal_fps, 'gap_factor': args.gap_factor,
        'bag_span_s': bag_span, 'bag_duration_s': bag_span[1] - bag_span[0],
        'topics': {topic: topic_stats(rows) for topic, rows in records.items()},
        'rgb_encodings': sorted({r['encoding'] for r in rgb}),
        'imu_frame_ids': sorted({r['frame_id'] for r in records[IMU]}),
        'rgb_sizes': sorted({(r['width'], r['height'], r['step'], r['bytes'])
                             for r in rgb}),
        'malformed_images': malformed,
        'rgb_gap_count': len(gaps),
        'estimated_missing_rgb_frames': sum(g['estimated_missing_frames'] for g in gaps),
        'rgb_gaps': gaps,
        'duplicate_reference_position_difference_m': {
            'max': float(np.max(duplicate_differences)) if len(duplicate_differences) else 0.0,
            'p99': float(np.percentile(duplicate_differences, 99))
            if len(duplicate_differences) else 0.0,
            'pairs_over_1cm': int(np.count_nonzero(duplicate_differences > 0.01)),
        },
        'reference_position_continuity': {
            'max_step_m': (float(reference_steps[max_reference_step_index])
                           if max_reference_step_index is not None else None),
            'max_step_dt_ms': (float(reference_dt[max_reference_step_index] * 1000)
                               if max_reference_step_index is not None else None),
            'steps_over_1m': int(len(large_reference_steps)),
            'events_over_1m': [
                {'left_header_s': float(reference_stamps[i]),
                 'right_header_s': float(reference_stamps[i + 1]),
                 'dt_ms': float(reference_dt[i] * 1000),
                 'position_step_m': float(reference_steps[i])}
                for i in large_reference_steps
            ],
        },
        'rgb_first_minus_imu_first_s': (rgb[0]['header_s'] - records[IMU][0]['header_s']
                                       if records[IMU] else None),
        'imu_last_minus_rgb_last_s': (records[IMU][-1]['header_s'] - rgb[-1]['header_s']
                                     if records[IMU] else None),
        'rgb_before_first_imu_count': (int(np.count_nonzero(stamps < imu_stamps[0]))
                                       if len(imu_stamps) else None),
        'rgb_after_last_imu_count': (int(np.count_nonzero(stamps > imu_stamps[-1]))
                                    if len(imu_stamps) else None),
        'rgb_to_nearest_imu': nearest_timestamp_stats(stamps, imu_stamps),
        'rgb_first_minus_reference_first_s': (
            rgb[0]['header_s'] - reference[0]['header_s'] if reference else None),
        'reference_last_minus_rgb_last_s': (
            reference[-1]['header_s'] - rgb[-1]['header_s'] if reference else None),
        'strict_common_interval': common,
        'selection_thresholds': {
            'max_rgb_gap_ms': args.gap_factor * nominal * 1000,
            'max_imu_gap_ms': args.max_imu_gap_ms,
            'max_rgb_to_nearest_imu_ms': args.max_rgb_to_nearest_imu_ms,
            'max_imu_edge_shortfall_ms': args.max_imu_edge_shortfall_ms,
            'max_reference_gap_ms': args.max_reference_gap_ms,
            'max_reference_edge_shortfall_ms': args.max_reference_edge_shortfall_ms,
        },
    }
    reasons = []
    rgb_stats = result['topics'][RGB]
    imu_stats = result['topics'][IMU]
    ref_stats = result['topics'][REFERENCE]
    if gaps or malformed or rgb_stats['nonpositive_dt_count'] or \
            rgb_stats['nonunit_sequence_increments']:
        reasons.append('RGB gap, malformed image, nonmonotonic timestamp, or sequence jump')
    if (imu_stats is None or imu_stats['count'] < 2 or
            imu_stats['nonpositive_dt_count'] or
            imu_stats['max_dt_ms'] > args.max_imu_gap_ms or
            result['rgb_first_minus_imu_first_s'] < -args.max_imu_edge_shortfall_ms / 1000 or
            result['imu_last_minus_rgb_last_s'] < -args.max_imu_edge_shortfall_ms / 1000 or
            common is None or common['rgb_frames'] < 2 or
            common['rgb_to_nearest_imu']['max_ms'] > args.max_rgb_to_nearest_imu_ms):
        reasons.append('raw IMU timestamp proximity or coverage insufficient')
    if result['imu_frame_ids'] != ['base_link']:
        reasons.append('raw IMU frame_id is not consistently base_link')
    if (ref_stats is None or ref_stats['count'] < 2 or
            ref_stats['negative_dt_count'] or
            ref_stats['max_dt_ms'] > args.max_reference_gap_ms or
            result['rgb_first_minus_reference_first_s'] <
            -args.max_reference_edge_shortfall_ms / 1000 or
            result['reference_last_minus_rgb_last_s'] <
            -args.max_reference_edge_shortfall_ms / 1000):
        reasons.append('reference odometry timestamp or coverage insufficient')
    result['vio_ready'] = not reasons
    result['exclusion_reasons'] = reasons
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({k: v for k, v in result.items() if k != 'rgb_gaps'}, indent=2))
    return 2 if reasons and args.require_vio_ready else 0


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--bag', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--nominal-fps', type=float, default=30.0)
    parser.add_argument('--gap-factor', type=float, default=1.5)
    parser.add_argument('--max-imu-gap-ms', type=float, default=100.0)
    parser.add_argument('--max-rgb-to-nearest-imu-ms', type=float, default=50.0)
    parser.add_argument('--max-imu-edge-shortfall-ms', type=float, default=100.0)
    parser.add_argument('--max-reference-gap-ms', type=float, default=200.0)
    parser.add_argument('--max-reference-edge-shortfall-ms', type=float, default=250.0)
    parser.add_argument('--require-vio-ready', action='store_true')
    raise SystemExit(run(parser.parse_args()))
