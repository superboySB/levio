"""Validate the pinned 18-bag odom cohort and summarize its online results.

The report is generated from the checked-in manifest plus the bag audit and
run outputs. It never interprets uninitialized monocular translation in metres.
Run after ``research/run_all.sh``; use ``--allow-incomplete`` while jobs run.
"""

import argparse
import csv
import json
import math
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
RGB = '/camera/color/image_raw'
IMU = '/mavros/imu/data_raw'
REFERENCE = '/fusion_odometry/lazy_point_odom'


def finite(value):
    return isinstance(value, (float, int)) and not isinstance(value, bool) and math.isfinite(value)


def require(condition, issues, message):
    if not condition:
        issues.append(message)


def close(a, b, tolerance=1e-6):
    return finite(a) and finite(b) and abs(a - b) <= tolerance


def load_manifest(path):
    lines = path.read_text().splitlines()
    prefix = '# HF dataset revision: '
    if not lines or not lines[0].startswith(prefix):
        raise ValueError(f'{path}: missing pinned HF revision')
    revision = lines[0][len(prefix):]
    rows = []
    seen = set()
    for line in lines:
        if not line or line.startswith('#'):
            continue
        parts = line.split('\t')
        if len(parts) != 5:
            raise ValueError(f'{path}: expected five tab-separated fields: {line}')
        label, session, size, sha, selection = parts
        if label in seen or not label.startswith('run') or not label[3:].isdigit():
            raise ValueError(f'{path}: invalid or duplicate label: {label}')
        if selection not in ('default', 'optional') or len(sha) != 64:
            raise ValueError(f'{path}: invalid selection or SHA-256: {label}')
        seen.add(label)
        rows.append({'label': label, 'session_id': session,
                     'bag_size_bytes': int(size), 'sha256': sha,
                     'selection': selection})
    selected = [row for row in rows if row['selection'] == 'default']
    selected.sort(key=lambda row: int(row['label'][3:]))
    return revision, selected


def load_json(path, issues):
    if not path.is_file():
        issues.append(f'missing {path.relative_to(ROOT)}')
        return None
    try:
        value = json.loads(path.read_text())
    except (OSError, json.JSONDecodeError) as error:
        issues.append(f'invalid JSON {path.relative_to(ROOT)}: {error}')
        return None
    if not isinstance(value, dict):
        issues.append(f'{path.relative_to(ROOT)} is not a JSON object')
        return None
    return value


def audit_one(audit, label, issues):
    require(audit.get('bag') == f'research_data/{label}.bag', issues, 'audit bag does not match manifest')
    require(audit.get('rgb_topic') == RGB and audit.get('imu_topic') == IMU,
            issues, 'audit uses unexpected sensor topics')
    require(audit.get('vio_ready') is True, issues, 'audit marks bag not VIO-ready')
    require(audit.get('exclusion_reasons') == [], issues, 'audit lists exclusion reasons')
    require(audit.get('rgb_gap_count') == 0, issues, 'RGB has gaps over 50 ms')
    require(audit.get('estimated_missing_rgb_frames') == 0, issues, 'RGB has estimated missing frames')
    require(audit.get('malformed_images') == [], issues, 'RGB has malformed images')
    require(audit.get('rgb_encodings') == ['rgb8'], issues, 'RGB encoding is not rgb8')
    require(audit.get('imu_frame_ids') == ['base_link'], issues,
            'raw IMU frame_id is not consistently base_link')
    require(audit.get('rgb_sizes') == [[640, 480, 1920, 921600]], issues,
            'RGB geometry or byte length differs from 640x480 rgb8')
    topics = audit.get('topics') or {}
    for topic in (RGB, IMU, REFERENCE):
        stats = topics.get(topic)
        if not isinstance(stats, dict):
            issues.append(f'audit missing topic {topic}')
            continue
        require(isinstance(stats.get('count'), int) and stats['count'] > 2,
                issues, f'{topic}: insufficient messages')
        require(finite(stats.get('max_dt_ms')) and stats['max_dt_ms'] >= 0,
                issues, f'{topic}: invalid maximum timestamp interval')
        require(finite(stats.get('first_header_s')) and finite(stats.get('last_header_s')) and
                stats['first_header_s'] < stats['last_header_s'],
                issues, f'{topic}: invalid timestamp span')
    rgb, imu, ref = (topics.get(topic) or {} for topic in (RGB, IMU, REFERENCE))
    for topic, stats, maximum in ((RGB, rgb, 50), (IMU, imu, 100), (REFERENCE, ref, 200)):
        if finite(stats.get('max_dt_ms')):
            require(stats['max_dt_ms'] <= maximum + 0.01, issues,
                    f'{topic}: maximum gap exceeds {maximum} ms')
    for topic, stats in ((RGB, rgb), (IMU, imu)):
        require(stats.get('nonpositive_dt_count') == 0, issues,
                f'{topic}: non-increasing header timestamps')
    require(ref.get('negative_dt_count') == 0, issues, 'reference timestamps go backward')
    require(rgb.get('positive_sequence_skips') == 0 and
            rgb.get('nonunit_sequence_increments') == 0,
            issues, 'RGB sequence numbers are discontinuous')
    common = audit.get('strict_common_interval') or {}
    start, end = common.get('start_header_s'), common.get('end_header_s')
    require(finite(start) and finite(end) and start < end,
            issues, 'missing positive RGB/IMU/reference common interval')
    require(isinstance(common.get('rgb_frames'), int) and common['rgb_frames'] > 2,
            issues, 'common interval has too few RGB frames')
    if isinstance(rgb.get('count'), int) and isinstance(common.get('rgb_frames'), int):
        trimmed = common.get('rgb_trimmed_start_frames', 0) + common.get('rgb_trimmed_end_frames', 0)
        require(rgb['count'] == common['rgb_frames'] + trimmed,
                issues, 'common-interval RGB trim counts do not add up')
    nearest = (common.get('rgb_to_nearest_imu') or {}).get('max_ms')
    require(finite(nearest) and nearest <= 50.01,
            issues, 'RGB-to-nearest-IMU gap exceeds 50 ms')
    require(finite(audit.get('rgb_first_minus_imu_first_s')) and
            audit['rgb_first_minus_imu_first_s'] >= -0.10001,
            issues, 'raw IMU begins over 100 ms after RGB')
    require(finite(audit.get('imu_last_minus_rgb_last_s')) and
            audit['imu_last_minus_rgb_last_s'] >= -0.10001,
            issues, 'raw IMU ends over 100 ms before RGB')
    require(finite(audit.get('reference_last_minus_rgb_last_s')) and
            audit['reference_last_minus_rgb_last_s'] >= -0.25001,
            issues, 'reference ends over 250 ms before RGB')
    reference_continuity = audit.get('reference_position_continuity') or {}
    require(finite(reference_continuity.get('max_step_m')) and
            reference_continuity['max_step_m'] >= 0 and
            isinstance(reference_continuity.get('steps_over_1m'), int),
            issues, 'reference position continuity statistics are missing')
    return {
        'rgb_frames': rgb.get('count'), 'imu_messages': imu.get('count'),
        'reference_messages': ref.get('count'),
        'rgb_max_gap_ms': rgb.get('max_dt_ms'),
        'imu_max_gap_ms': imu.get('max_dt_ms'),
        'reference_max_gap_ms': ref.get('max_dt_ms'),
        'rgb_to_nearest_imu_max_ms': nearest,
        'reference_max_position_step_m': reference_continuity.get('max_step_m'),
        'reference_steps_over_1m': reference_continuity.get('steps_over_1m'),
        'common_rgb_frames': common.get('rgb_frames'),
        'common_interval_s': [start, end],
        'trimmed_rgb_start': common.get('rgb_trimmed_start_frames'),
        'trimmed_rgb_end': common.get('rgb_trimmed_end_frames'),
    }


def tum_rows(path, issues):
    if not path.is_file():
        issues.append(f'missing {path.relative_to(ROOT)}')
        return None
    rows = []
    try:
        with path.open() as handle:
            for line in handle:
                if not line.strip() or line.startswith('#'):
                    continue
                fields = [float(x) for x in line.split()]
                if len(fields) != 8 or not all(finite(x) for x in fields):
                    raise ValueError('expected eight finite TUM columns')
                rows.append(fields)
    except (OSError, ValueError) as error:
        issues.append(f'invalid {path.relative_to(ROOT)}: {error}')
        return None
    return rows


def read_times(path, issues):
    if not path.is_file():
        issues.append(f'missing {path.relative_to(ROOT)}')
        return None
    try:
        values = [float(line) for line in path.read_text().splitlines() if line.strip()]
        if not all(finite(x) for x in values):
            raise ValueError('non-finite capture time')
        return values
    except (OSError, ValueError) as error:
        issues.append(f'invalid {path.relative_to(ROOT)}: {error}')
        return None


def step_diagnostics(path, metric, issues):
    if not path.is_file():
        issues.append(f'missing {path.relative_to(ROOT)} for valid metric interval')
        return None
    required = ('elapsed_s', 'ref_x_m', 'ref_y_m', 'ref_z_m',
                'pred_x_m', 'pred_y_m', 'pred_z_m', 'error_m')
    try:
        with path.open(newline='') as handle:
            reader = csv.DictReader(handle)
            if tuple(reader.fieldnames or ()) != required:
                raise ValueError('unexpected metric CSV columns')
            data = [{key: float(row[key]) for key in required} for row in reader]
        if not data or not all(all(finite(v) for v in row.values()) for row in data):
            raise ValueError('empty or non-finite metric CSV')
    except (OSError, ValueError, TypeError) as error:
        issues.append(f'invalid {path.relative_to(ROOT)}: {error}')
        return None
    require(len(data) == metric.get('matched_frames'), issues,
            'metric CSV row count does not match matched_frames')
    require(abs(data[0]['elapsed_s']) <= 1e-6 and abs(data[0]['error_m']) <= 1e-6,
            issues, 'metric CSV does not begin at the shared pose')
    def xyz(row, prefix):
        return (row[f'{prefix}_x_m'], row[f'{prefix}_y_m'], row[f'{prefix}_z_m'])
    def distance(a, b):
        return math.dist(a, b)
    pred_steps = [(distance(xyz(data[i-1], 'pred'), xyz(data[i], 'pred')), i)
                  for i in range(1, len(data))]
    ref_steps = [(distance(xyz(data[i-1], 'ref'), xyz(data[i], 'ref')), i)
                 for i in range(1, len(data))]
    pred_length = sum(step for step, _ in pred_steps)
    ref_length = sum(step for step, _ in ref_steps)
    if finite(metric.get('estimated_path_length_m')):
        path_tolerance = max(1e-4, 1e-8 * metric['estimated_path_length_m'])
        require(close(pred_length, metric['estimated_path_length_m'], path_tolerance),
                issues, 'metric CSV predicted path length differs from summary')
    if finite(metric.get('reference_path_length_m')):
        require(close(ref_length, metric['reference_path_length_m'], 1e-4),
                issues, 'metric CSV reference path length differs from summary')
    errors = [distance(xyz(row, 'pred'), xyz(row, 'ref')) for row in data]
    require(all(abs(error - row['error_m']) <= 1e-5 for error, row in zip(errors, data)),
            issues, 'metric CSV error column differs from positions')
    if finite(metric.get('initial_pose_aligned_rmse_m')):
        rmse = math.sqrt(sum(error*error for error in errors) / len(errors))
        require(close(rmse, metric['initial_pose_aligned_rmse_m'], 1e-4),
                issues, 'metric CSV RMSE differs from summary')
    if finite(metric.get('matched_time_span_s')):
        require(close(data[-1]['elapsed_s'], metric['matched_time_span_s'], 1e-4),
                issues, 'metric CSV time span differs from summary')
    max_pred, pred_index = max(pred_steps, default=(0.0, 0))
    max_ref, ref_index = max(ref_steps, default=(0.0, 0))
    return {
        'scored_time_span_s': data[-1]['elapsed_s'],
        'max_online_step_m': max_pred,
        'max_online_step_elapsed_s': data[pred_index]['elapsed_s'],
        'max_online_step_dt_s': (data[pred_index]['elapsed_s'] -
                                 data[pred_index - 1]['elapsed_s']) if pred_index else None,
        'max_reference_step_m': max_ref,
        'max_reference_step_elapsed_s': data[ref_index]['elapsed_s'],
        'max_reference_step_dt_s': (data[ref_index]['elapsed_s'] -
                                    data[ref_index - 1]['elapsed_s']) if ref_index else None,
        'step_threshold_m': 1.0,
        'online_steps_over_1m': sum(step > 1.0 for step, _ in pred_steps),
        'reference_steps_over_1m': sum(step > 1.0 for step, _ in ref_steps),
        'max_position_error_m': max(errors),
        'max_position_error_elapsed_s': data[max(range(len(errors)), key=errors.__getitem__)]['elapsed_s'],
    }


def result_one(summary, label, audit_stats, run_dir, figure, issues):
    expected = {
        'bag': f'research_data/{label}.bag', 'camera': 'color', 'image_topic': RGB,
        'imu_topic': IMU, 'reference_topic': REFERENCE, 'frame_step': 1,
        'optimization_enabled': True, 'general_initialization': False,
        'adjacent_recovery': False, 'bootstrap_scale_until_initialized': False,
        'input_common_interval_filter_applied': True,
    }
    for key, value in expected.items():
        require(summary.get(key) == value, issues, f'summary {key} does not match baseline run')
    require(close(summary.get('target_fps'), 20.0), issues, 'summary is not the 20 Hz run')
    require(summary.get('failure') is None, issues, 'run reports an exception')
    require(summary.get('evaluated_trajectory', '').startswith('online_frames.tum:'),
            issues, 'metric does not use online pose snapshots')
    require(summary.get('capture_time_source', '').startswith('original ROS image header.stamp'),
            issues, 'metric does not use original RGB capture timestamps')
    require(summary.get('reference_algorithm') == 'unverified from released metadata and bag',
            issues, 'recorded odometry reference is mislabeled')
    require(summary.get('image_encoding') == 'rgb8' and summary.get('image_size') == [640, 480],
            issues, 'summary camera image format differs from audited RGB')
    K, D, T = summary.get('K'), summary.get('D'), summary.get('T_base_camera')
    require(isinstance(K, list) and len(K) == 3 and all(isinstance(r, list) and len(r) == 3 for r in K) and
            all(finite(x) for r in K for x in r) and K[0][0] > 100 and K[1][1] > 100,
            issues, 'missing or implausible D435i camera matrix')
    require(isinstance(D, list) and len(D) >= 4 and all(finite(x) for x in D),
            issues, 'missing or invalid camera distortion')
    require(isinstance(T, list) and len(T) == 4 and all(isinstance(r, list) and len(r) == 4 for r in T) and
            all(finite(x) for r in T for x in r) and T[3] == [0.0, 0.0, 0.0, 1.0],
            issues, 'missing or invalid camera-to-base transform')
    require(T == summary.get('T_imu_camera_used_by_levio'), issues,
            'LEVIO does not use the recorded color camera transform')
    n = summary.get('frames_processed')
    require(isinstance(n, int) and n > 2, issues, 'missing or too few processed frames')
    require(isinstance(summary.get('keyframes'), int) and summary['keyframes'] > 0,
            issues, 'missing keyframe count')
    skips = summary.get('skipped_essential_frames')
    require(isinstance(skips, int) and skips >= 0, issues, 'invalid essential-skip count')
    require(summary.get('reference_messages') == audit_stats.get('reference_messages'),
            issues, 'summary and audit reference message counts differ')
    if finite(summary.get('reference_max_gap_s')) and finite(audit_stats.get('reference_max_gap_ms')):
        require(close(summary['reference_max_gap_s'], audit_stats['reference_max_gap_ms']/1000, 1e-5),
                issues, 'summary and audit reference maximum gaps differ')
    common = summary.get('input_common_interval_header_s') or []
    audited_common = audit_stats.get('common_interval_s') or []
    require(len(common) == 2 and len(audited_common) == 2 and
            close(common[0], audited_common[0], 1e-5) and
            close(common[1], audited_common[1], 1e-5),
            issues, 'summary and audit common sensor intervals differ')
    require(summary.get('input_rgb_trimmed_start_images') == audit_stats.get('trimmed_rgb_start') and
            summary.get('input_rgb_trimmed_end_images') == audit_stats.get('trimmed_rgb_end'),
            issues, 'summary and audit RGB edge trimming differ')

    online = tum_rows(run_dir / 'online_frames.tum', issues)
    final = tum_rows(run_dir / 'frames.tum', issues)
    keyframes = tum_rows(run_dir / 'keyframes.tum', issues)
    times = read_times(run_dir / 'capture_times.txt', issues)
    for path, rows, expected_count in ((run_dir / 'online_frames.tum', online, n),
                                       (run_dir / 'frames.tum', final, n),
                                       (run_dir / 'keyframes.tum', keyframes,
                                        summary.get('keyframes')),
                                       (run_dir / 'capture_times.txt', times, n)):
        if rows is not None:
            require(len(rows) == expected_count, issues,
                    f'{path.relative_to(ROOT)} row count does not match summary')
    if online is not None and times is not None and len(online) == len(times):
        require(all(close(row[0], stamp, 1e-5) for row, stamp in zip(online, times)),
                issues, 'online pose timestamps differ from original capture times')
        require(all(times[i] > times[i-1] for i in range(1, len(times))),
                issues, 'processed RGB timestamps are not increasing')
        if len(common) == 2 and times:
            require(times[0] >= common[0] - 1e-5 and times[0] - common[0] < 0.11 and
                    times[-1] <= common[1] + 1e-5 and common[1] - times[-1] < 0.11,
                    issues, 'run does not cover the full common sensor interval')
    trace_path = run_dir / 'runtime_match_trace.csv'
    if trace_path.is_file():
        try:
            with trace_path.open(newline='') as handle:
                trace = list(csv.DictReader(handle))
            require(len(trace) == n, issues, 'runtime trace row count differs from processed frames')
            require(all(int(row['frame_id']) == i for i, row in enumerate(trace)),
                    issues, 'runtime trace frame IDs are not contiguous')
            if times is not None and len(trace) == len(times):
                require(all(close(float(row['capture_time_s']), t, 1e-5)
                            for row, t in zip(trace, times)),
                        issues, 'runtime trace timestamps differ from capture times')
            if isinstance(skips, int):
                skipped_rows = [i for i, row in enumerate(trace)
                                if row.get('essential_skipped') == '1']
                require(len(skipped_rows) == skips, issues,
                        'runtime trace essential skips differ from summary')
                require((skipped_rows[0] if skipped_rows else None) ==
                        summary.get('first_essential_skip_frame'), issues,
                        'runtime trace first essential skip differs from summary')
        except (OSError, ValueError, KeyError) as error:
            issues.append(f'invalid {trace_path.relative_to(ROOT)}: {error}')
    else:
        issues.append(f'missing {trace_path.relative_to(ROOT)}')
    require(figure.is_file() and figure.stat().st_size > 100,
            issues, f'missing or empty {figure.relative_to(ROOT)}')

    init = summary.get('initialization_frame')
    initialized = summary.get('vio_initialized')
    require(isinstance(initialized, bool), issues, 'missing VIO initialization flag')
    require((init is None and initialized is False) or
            (isinstance(init, int) and 0 <= init < n and initialized is True),
            issues, 'initialization frame and initialized flag disagree')
    whole = summary.get('comparison_to_recorded_odometry') or {}
    require(whole.get('diagnostic_only') is True and whole.get('metric_scale_valid') is False,
            issues, 'pre-initialization comparison is not marked diagnostic-only')
    metric = summary.get('comparison_after_initialization') or {}
    metric_valid = metric.get('metric_scale_valid')
    diagnostics = None
    first_skip = summary.get('first_essential_skip_frame')
    has_post_init_interval = (initialized is True and
                              (first_skip is None or init < first_skip))
    if has_post_init_interval:
        require(metric_valid is True, issues,
                'initialized run lacks a valid post-initialization metric interval')
    else:
        require(metric_valid is False and metric.get('matched_frames') == 0,
                issues, 'run without a valid post-initialization window reports a metric interval')
    if metric_valid is True:
        matched = metric.get('matched_frames')
        require(isinstance(matched, int) and 3 <= matched <= n - init,
                issues, 'invalid number of matched metric frames')
        require(metric.get('reference_is_ground_truth') is False,
                issues, 'recorded odometry is incorrectly labeled ground truth')
        for field in ('initial_pose_aligned_rmse_m', 'initial_pose_aligned_start_error_m',
                      'initial_pose_aligned_end_error_m', 'endpoint_drift_rate_percent',
                      'estimated_path_length_m', 'reference_path_length_m',
                      'path_length_ratio', 'max_sync_error_ms'):
            require(finite(metric.get(field)) and metric[field] >= 0,
                    issues, f'invalid metric {field}')
        require(finite(metric.get('initial_pose_aligned_start_error_m')) and
                metric['initial_pose_aligned_start_error_m'] <= 1e-6,
                issues, 'post-initialization trajectories do not share the first pose')
        require(finite(metric.get('max_sync_error_ms')) and
                metric['max_sync_error_ms'] <= 20.01,
                issues, 'reference matching exceeds 20 ms')
        if finite(metric.get('reference_path_length_m')) and metric['reference_path_length_m'] > 0:
            require(close(metric.get('endpoint_drift_rate_percent'),
                          100 * metric['initial_pose_aligned_end_error_m'] /
                          metric['reference_path_length_m'], 1e-4),
                    issues, 'EDR does not match endpoint error and reference length')
            require(close(metric.get('path_length_ratio'),
                          metric['estimated_path_length_m'] /
                          metric['reference_path_length_m'], 1e-5),
                    issues, 'path-length ratio does not match path lengths')
        diagnostics = step_diagnostics(run_dir / 'initial_pose_error.csv', metric, issues)
    return {
        'processed_frames': n, 'keyframes': summary.get('keyframes'),
        'initialized': initialized, 'initialization_frame': init,
        'essential_skips': skips,
        'first_essential_skip_frame': summary.get('first_essential_skip_frame'),
        'matched_metric_frames': metric.get('matched_frames') if metric_valid is True else None,
        'scored_time_span_s': diagnostics.get('scored_time_span_s')
            if diagnostics is not None else None,
        'rmse_m': metric.get('initial_pose_aligned_rmse_m') if metric_valid is True else None,
        'end_error_m': metric.get('initial_pose_aligned_end_error_m') if metric_valid is True else None,
        'edr_percent': metric.get('endpoint_drift_rate_percent') if metric_valid is True else None,
        'path_length_ratio': metric.get('path_length_ratio') if metric_valid is True else None,
        'estimated_path_length_m': metric.get('estimated_path_length_m') if metric_valid is True else None,
        'reference_path_length_m': metric.get('reference_path_length_m') if metric_valid is True else None,
        'metric_diagnostics': diagnostics,
    }


def fmt(value, digits=2):
    return f'{value:.{digits}f}' if finite(value) else '—'


def markdown(report):
    lines = [f'已列入默认 cohort：{report["selected_runs"]}/{report["expected_runs"]} 段；'
             f'时间轴审计通过 {report["audit_passed_runs"]} 段；'
             f'完整结果 {report["complete_result_runs"]} 段；'
             f'两项均满足 {report["ready_runs"]} 段。', '',
             '| Bag | RGB 帧 | 最大 RGB 间隔 (ms) | raw IMU 条数 | 最大 IMU 间隔 (ms) | RGB–最近 IMU 最大差 (ms) | 参考 >1 m 单步 | 时间轴审计 |',
             '|---|---:|---:|---:|---:|---:|---:|---|']
    for row in report['runs']:
        a = row['audit'] or {}
        lines.append(f'| {row["label"]} | {a.get("rgb_frames") or "—"} | '
                     f'{fmt(a.get("rgb_max_gap_ms"), 1)} | '
                     f'{a.get("imu_messages") or "—"} | '
                     f'{fmt(a.get("imu_max_gap_ms"), 1)} | '
                     f'{fmt(a.get("rgb_to_nearest_imu_max_ms"), 1)} | '
                     f'{a.get("reference_steps_over_1m") if a.get("reference_steps_over_1m") is not None else "—"} | '
                     f'{row["audit_status"]} |')
    lines.extend(['',
                  '| Bag | 处理帧 | 初始化帧 | 评分帧 | 评分跨度 (s) | 首位姿 RMSE (m) | 端点差 (m) | EDR (%) | 路径比 | 最大预测单步 (m) | 最大参考单步 (m) | >1 m 步数（预测/参考） | 结果 |',
                  '|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|'])
    for row in report['runs']:
        result = row['result'] or {}
        diag = result.get('metric_diagnostics') or {}
        init = result.get('initialization_frame')
        large_steps = (f'{diag["online_steps_over_1m"]}/'
                       f'{diag["reference_steps_over_1m"]}') if diag else '—'
        lines.append(f'| {row["label"]} | {result.get("processed_frames") or "—"} | '
                     f'{init if init is not None else "—"} | '
                     f'{result.get("matched_metric_frames") or "—"} | '
                     f'{fmt(result.get("scored_time_span_s"), 1)} | '
                     f'{fmt(result.get("rmse_m"), 3)} | '
                     f'{fmt(result.get("end_error_m"), 3)} | '
                     f'{fmt(result.get("edr_percent"), 1)} | '
                     f'{fmt(result.get("path_length_ratio"), 2)} | '
                     f'{fmt(diag.get("max_online_step_m"), 3)} | '
                     f'{fmt(diag.get("max_reference_step_m"), 3)} | '
                     f'{large_steps} | '
                     f'{row["result_status"]} |')
    failures = [row for row in report['runs'] if not row['ready']]
    if failures:
        lines.extend(['', '未通过项：', ''])
        for row in failures:
            lines.append(f'- {row["label"]}: ' + '; '.join(row['issues']))
    return '\n'.join(lines) + '\n'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, default=ROOT / 'research/metadata/bags.tsv')
    parser.add_argument('--results-root', type=Path, default=ROOT / 'research_results')
    parser.add_argument('--figures-root', type=Path, default=ROOT / 'research/figures')
    parser.add_argument('--output-json', type=Path,
                        default=ROOT / 'research_results/cohort_report.json')
    parser.add_argument('--output-markdown', type=Path,
                        default=ROOT / 'research_results/cohort_report.md')
    parser.add_argument('--allow-incomplete', action='store_true',
                        help='write the partial report and return success during an ongoing run')
    args = parser.parse_args()
    revision, cohort = load_manifest(args.manifest)
    rows = []
    for entry in cohort:
        label = entry['label']
        audit_path = args.results_root / 'rgb_audit' / f'{label}.json'
        run_dir = args.results_root / f'{label}_color_20hz'
        summary_path = run_dir / 'summary.json'
        figure = args.figures_root / f'{label}_color_20hz.svg'
        audit_issues, result_issues = [], []
        audit = load_json(audit_path, audit_issues)
        audit_stats = audit_one(audit, label, audit_issues) if audit is not None else None
        bag = ROOT / 'research_data' / f'{label}.bag'
        if not bag.is_file():
            result_issues.append(f'missing research_data/{label}.bag')
        elif bag.stat().st_size != entry['bag_size_bytes']:
            result_issues.append(f'bag byte size differs from pinned manifest: {label}')
        summary = load_json(summary_path, result_issues)
        result_stats = (result_one(summary, label, audit_stats or {}, run_dir, figure,
                                   result_issues) if summary is not None else None)
        audit_status = ('missing' if audit is None else
                        'pass' if not audit_issues else 'fail')
        result_status = ('missing' if summary is None else
                         'complete' if not result_issues else 'invalid')
        rows.append({**entry, 'audit_path': str(audit_path.relative_to(ROOT)),
                     'summary_path': str(summary_path.relative_to(ROOT)),
                     'figure_path': str(figure.relative_to(ROOT)),
                     'audit_status': audit_status, 'result_status': result_status,
                     'ready': audit_status == 'pass' and result_status == 'complete',
                     'audit': audit_stats, 'result': result_stats,
                     'issues': audit_issues + result_issues})
    report = {
        'hf_revision': revision, 'expected_runs': 18, 'selected_runs': len(rows),
        'audit_passed_runs': sum(row['audit_status'] == 'pass' for row in rows),
        'complete_result_runs': sum(row['result_status'] == 'complete' for row in rows),
        'ready_runs': sum(row['ready'] for row in rows),
        'all_ready': len(rows) == 18 and all(row['ready'] for row in rows), 'runs': rows,
    }
    for path, content in ((args.output_json, json.dumps(report, indent=2, allow_nan=False) + '\n'),
                          (args.output_markdown, markdown(report))):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content)
    print(f'{report["ready_runs"]}/{report["expected_runs"]} default runs verified; '
          f'JSON: {args.output_json}; Markdown: {args.output_markdown}')
    return 0 if report['all_ready'] or args.allow_incomplete else 2


if __name__ == '__main__':
    raise SystemExit(main())
