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


def run(args):
    records = {topic: [] for topic in (RGB, IMU, REFERENCE)}
    malformed = []
    with rosbag.Bag(args.bag) as bag:
        bag_span = [bag.get_start_time(), bag.get_end_time()]
        for topic, msg, bag_t in bag.read_messages(topics=list(records)):
            row = {'header_s': msg.header.stamp.to_sec(), 'bag_s': bag_t.to_sec(),
                   'seq': msg.header.seq}
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
    reference = records[REFERENCE]
    reference_stamps = np.array([r['header_s'] for r in reference])
    repeated_reference = np.flatnonzero(np.diff(reference_stamps) == 0)
    reference_xyz = np.array([r['xyz'] for r in reference])
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
        'rgb_first_minus_imu_first_s': (rgb[0]['header_s'] - records[IMU][0]['header_s']
                                       if records[IMU] else None),
        'imu_last_minus_rgb_last_s': (records[IMU][-1]['header_s'] - rgb[-1]['header_s']
                                     if records[IMU] else None),
    }
    reasons = []
    rgb_stats = result['topics'][RGB]
    imu_stats = result['topics'][IMU]
    ref_stats = result['topics'][REFERENCE]
    if gaps or malformed or rgb_stats['nonpositive_dt_count'] or \
            rgb_stats['nonunit_sequence_increments']:
        reasons.append('RGB gap, malformed image, nonmonotonic timestamp, or sequence jump')
    if imu_stats is None or imu_stats['nonpositive_dt_count'] or \
            imu_stats['max_dt_ms'] > 100 or \
            result['rgb_first_minus_imu_first_s'] < -0.1 or \
            result['imu_last_minus_rgb_last_s'] < -0.1:
        reasons.append('raw IMU timestamp or coverage insufficient')
    if ref_stats is None or ref_stats['negative_dt_count'] or \
            ref_stats['max_dt_ms'] > 200 or \
            ref_stats['last_header_s'] - rgb[-1]['header_s'] < -0.25:
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
    parser.add_argument('--require-vio-ready', action='store_true')
    raise SystemExit(run(parser.parse_args()))
