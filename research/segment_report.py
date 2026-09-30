"""Evaluate causal, nonoverlapping fixed-duration intervals from saved trajectories.

Each interval uses exactly one transform from its first matched *pose*; no
positions later in the interval, scale fit, or future estimator state are used.
The input is the same initialized, pre-first-E-skip prefix as the main report.
"""

import argparse
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import rosbag
from scipy.spatial.transform import Rotation

from compare_euroc_paper import euroc_camera_groundtruth
from cohort_report import load_manifest
from run_odom import bag_calibration, reference_poses


ROOT = Path(__file__).resolve().parents[1]
plt.rcParams.update({'font.size': 9, 'svg.fonttype': 'none',
                     'svg.hashsalt': 'levio-research-segments'})


def score_inputs(label, results_root):
    run_dir = results_root / (label if label == 'MH01_euroc_full'
                             else f'{label}_color_20hz')
    summary = json.loads((run_dir / 'summary.json').read_text())
    poses = np.atleast_2d(np.loadtxt(run_dir / 'online_frames.tum', comments='#'))
    capture = np.atleast_1d(np.loadtxt(run_dir / 'capture_times.txt'))
    if poses.shape[1] != 8 or len(poses) != len(capture) or not np.allclose(
            poses[:, 0], capture, rtol=0, atol=1e-6):
        raise ValueError(f'{label}: online poses and original capture times disagree')
    init = summary.get('initialization_frame')
    stop = summary.get('first_essential_skip_frame')
    if init is None or (stop is not None and stop <= init):
        return summary, None
    end = len(poses) if stop is None else stop
    poses = poses[init:end]
    if label == 'MH01_euroc_full':
        ref_t, ref_xyz, ref_rot = euroc_camera_groundtruth(
            ROOT / summary['groundtruth_csv'],
            np.asarray(summary['T_imu_camera']))
    else:
        with rosbag.Bag(str(ROOT / summary['bag'])) as bag:
            _, _, _, _, T_base_camera = bag_calibration(bag, summary['camera'])
            ref_t, ref_xyz, ref_rot = reference_poses(
                bag, T_base_camera, with_rotations=True)
    index = np.searchsorted(ref_t, poses[:, 0])
    index = np.clip(index, 1, len(ref_t) - 1)
    index -= (np.abs(ref_t[index - 1] - poses[:, 0])
              < np.abs(ref_t[index] - poses[:, 0]))
    valid = np.abs(ref_t[index] - poses[:, 0]) <= 0.02
    if np.count_nonzero(valid) < 3:
        raise ValueError(f'{label}: fewer than three matched poses')
    matched = poses[valid]
    ref_index = index[valid]
    reported = summary['comparison_after_initialization']
    if reported.get('metric_scale_valid') is not True or (
            len(matched) != reported['matched_frames']):
        raise ValueError(f'{label}: saved score window and matched poses disagree')
    return summary, {
        'times': matched[:, 0], 'estimate': matched[:, 1:4],
        'estimate_rotations': Rotation.from_quat(matched[:, 4:8]).as_matrix(),
        'reference': ref_xyz[ref_index], 'reference_rotations': ref_rot[ref_index],
    }


def evaluate_selected(source, select, ordinal, duration=None, target_path=None):
    time = source['times']
    first, last = time[select[0]], time[select[-1]]
    est = source['estimate'][select]
    ref = source['reference'][select]
    R0 = (source['reference_rotations'][select[0]] @
          source['estimate_rotations'][select[0]].T)
    aligned = (R0 @ (est - est[0]).T).T + ref[0]
    if np.linalg.norm(aligned[0] - ref[0]) > 1e-9:
        raise ValueError('Window first poses do not coincide')
    error = np.linalg.norm(aligned - ref, axis=1)
    ref_steps = np.linalg.norm(np.diff(ref, axis=0), axis=1)
    est_steps = np.linalg.norm(np.diff(est, axis=0), axis=1)
    ref_length = float(ref_steps.sum())
    max_ref_step = float(ref_steps.max())
    reasons = []
    if max_ref_step > 1.0:
        reasons.append('reference step > 1 m')
    if ref_length < 0.25:
        reasons.append('reference path < 0.25 m')
    result = {
        'ordinal': ordinal, 'start_from_first_score_s': float(first - time[0]),
        'duration_requested_s': duration, 'target_reference_path_m': target_path,
        'duration_observed_s': float(last - first),
        'matched_frames': len(select), 'first_pose_error_m': float(error[0]),
        'rmse_m': float(np.sqrt(np.mean(error ** 2))),
        'end_error_m': float(error[-1]),
        'reference_path_m': ref_length,
        'estimate_path_m': float(est_steps.sum()),
        'edr_percent': float(100 * error[-1] / ref_length)
            if ref_length else None,
        'max_reference_step_m': max_ref_step,
        'max_estimate_step_m': float(est_steps.max()),
        'clean_reference': not reasons,
        'reference_flags': reasons,
    }
    return result, (ref, aligned, time[select] - first)


def evaluate_window(source, start, duration, ordinal):
    time = source['times']
    finish = start + duration
    select = np.flatnonzero((time >= start - 1e-6) &
                           (time <= finish + 1e-6))
    if len(select) < 3:
        return None
    first, last = time[select[0]], time[select[-1]]
    # 20 Hz processing leaves at most one nominal frame period at each edge.
    if first - start > 0.08 or finish - last > 0.08:
        return None
    return evaluate_selected(source, select, ordinal, duration=duration)


def windows(source, duration):
    if source is None:
        return [], {}
    origin = source['times'][0]
    count = max(0, int(math.floor((source['times'][-1] - origin + 0.08)
                                  / duration)))
    results, plots = [], {}
    for ordinal in range(count):
        outcome = evaluate_window(source, origin + ordinal * duration,
                                  duration, ordinal)
        if outcome is not None:
            row, samples = outcome
            results.append(row)
            plots[ordinal] = samples
    return results, plots


def path_windows(source, target_path=5.0):
    """Use nonoverlapping reference-distance windows, sharing only endpoints."""
    if source is None:
        return []
    positions = source['reference']
    cumulative = np.r_[0, np.cumsum(np.linalg.norm(
        np.diff(positions, axis=0), axis=1))]
    results = []
    first = 0
    while first < len(positions) - 2:
        end = int(np.searchsorted(cumulative, cumulative[first] + target_path))
        if end >= len(positions):
            break
        selected = np.arange(first, end + 1)
        row, _ = evaluate_selected(source, selected, len(results),
                                   target_path=target_path)
        results.append(row)
        first = end
    return results


def representative_plot(label, duration, row, samples, out_dir, ground_truth):
    ref, est, elapsed = samples
    figure, axes = plt.subplots(1, 2, figsize=(9.8, 3.9), layout='constrained')
    axes[0].plot(ref[:, 0], ref[:, 1], color='#238b45', lw=1.8,
                 label='EuRoC ground truth' if ground_truth else 'Recorded odometry')
    axes[0].plot(est[:, 0], est[:, 1], color='#d94841', lw=1.4,
                 label='LEVIO')
    axes[0].scatter([ref[0, 0]], [ref[0, 1]], marker='o', s=34,
                    color='#238b45', label='shared start', zorder=5)
    axes[0].scatter([est[0, 0]], [est[0, 1]], marker='o', s=9,
                    color='#d94841', zorder=6)
    axes[0].set(xlabel='x (m)', ylabel='y (m)', title='X–Y trajectory')
    axes[0].set_aspect('equal', adjustable='datalim')
    axes[0].legend(fontsize=7)
    axes[1].plot(elapsed, ref[:, 2], color='#238b45', lw=1.8)
    axes[1].plot(elapsed, est[:, 2], color='#d94841', lw=1.4)
    axes[1].scatter([elapsed[0]], [ref[0, 2]], s=34, color='#238b45', zorder=5)
    axes[1].scatter([elapsed[0]], [est[0, 2]], s=9, color='#d94841', zorder=6)
    axes[1].set(xlabel='time from window start (s)', ylabel='z (m)',
                title='Height')
    for ax in axes:
        ax.grid(alpha=0.3)
    figure.suptitle(f'{label} | {duration:g} s window {row["ordinal"]} | '
                   f'RMSE {row["rmse_m"]:.3f} m | '
                   f'{"clean reference" if row["clean_reference"] else "reference flagged"}')
    out_dir.mkdir(parents=True, exist_ok=True)
    path = out_dir / f'{label}_{int(duration)}s_window{row["ordinal"]:02d}.svg'
    figure.savefig(path, format='svg')
    plt.close(figure)
    path.write_text('\n'.join(line.rstrip() for line in
                              path.read_text().splitlines()) + '\n')
    return str(path.relative_to(ROOT))


def fmt(value, digits=2):
    return f'{value:.{digits}f}' if value is not None else '—'


def summarize(rows):
    clean = [row for row in rows if row['clean_reference']]
    return {'complete_windows': len(rows), 'clean_windows': len(clean),
            'reference_flagged_windows': len(rows) - len(clean),
            'median_rmse_m': float(np.median([r['rmse_m'] for r in clean]))
                if clean else None,
            'max_rmse_m': max((r['rmse_m'] for r in clean), default=None),
            'median_edr_percent': float(np.median([r['edr_percent'] for r in clean]))
                if clean else None}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--results-root', type=Path, default=ROOT / 'research_results')
    parser.add_argument('--figures-root', type=Path, default=ROOT / 'research/figures')
    parser.add_argument('--output-json', type=Path,
                        default=ROOT / 'research_results/segment_report.json')
    parser.add_argument('--output-markdown', type=Path,
                        default=ROOT / 'research_results/segment_report.md')
    args = parser.parse_args()
    _, manifest = load_manifest(ROOT / 'research/metadata/bags.tsv')
    labels = ['MH01_euroc_full'] + [row['label'] for row in manifest]
    reports = []
    for label in labels:
        summary, source = score_inputs(label, args.results_root)
        audit = (json.loads((args.results_root / 'rgb_audit' / f'{label}.json').read_text())
                 if label != 'MH01_euroc_full' else None)
        common = audit.get('strict_common_interval', {}) if audit else {}
        raw_span = (common['end_header_s'] - common['start_header_s']) if audit else None
        scored = float(source['times'][-1] - source['times'][0]) if source is not None else None
        report = {'label': label, 'raw_common_span_s': raw_span,
                  'processed_frames': summary['frames_processed'],
                  'initialized': summary['vio_initialized'],
                  'score_span_s': scored, 'durations': {}}
        for duration in (10, 30):
            rows, plots = windows(source, duration)
            info = {**summarize(rows), 'windows': rows, 'figures': []}
            if duration == 30 and label in (
                    'MH01_euroc_full', 'run007', 'run032', 'run034') and rows:
                for row in [rows[0], rows[-1]] if len(rows) > 1 else [rows[0]]:
                    path = representative_plot(
                        label, duration, row, plots[row['ordinal']],
                        args.figures_root, label == 'MH01_euroc_full')
                    info['figures'].append(path)
            report['durations'][str(duration)] = info
        path_rows = path_windows(source)
        report['reference_5m'] = {**summarize(path_rows), 'windows': path_rows,
                                  'median_duration_s': float(np.median([
                                      r['duration_observed_s'] for r in path_rows
                                      if r['clean_reference']]))
                                  if any(r['clean_reference'] for r in path_rows)
                                  else None}
        reports.append(report)
        print(label, f'raw={fmt(raw_span, 1)} s',
              f'score={fmt(scored, 1)} s',
              '10s=', report['durations']['10']['clean_windows'],
              '30s=', report['durations']['30']['clean_windows'], flush=True)
    result = {'protocol': ('Nonoverlapping 10 s and 30 s intervals applied '
                           'uniformly to the cohort. Each starts at the first matched pose of '
                           'that interval and is aligned by one SE(3) from the '
                           'first pair only. Post-initialization prefix ends '
                           'before the first skipped essential update. Complete '
                           'intervals require <=80 ms endpoint slack. Separate '
                           '5 m reference-distance intervals follow the 4–7 m '
                           'test-sequence scale reported by TIO-Former, without '
                           'claiming protocol or truth equivalence. Reference '
                           'jumps >1 m and path length <0.25 m are flagged '
                           'and excluded only from clean-reference summaries.'),
              'reports': reports}
    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    args.output_json.write_text(json.dumps(result, indent=2) + '\n')
    lines = [result['protocol'], '',
             '| Bag | 原始共同时间 (s) | 处理帧 | 可评分时间 (s) | '
             '10 s 完整/参考合格窗 | 10 s 合格窗 RMSE 中位/最大 (m) | '
             '30 s 完整/参考合格窗 | 30 s 合格窗 RMSE 中位/最大 (m) |',
             '|---|---:|---:|---:|---:|---:|---:|---:|']
    for row in reports:
        a, b = row['durations']['10'], row['durations']['30']
        lines.append(f'| {row["label"]} | {fmt(row["raw_common_span_s"], 1)} | '
                     f'{row["processed_frames"]} | {fmt(row["score_span_s"], 1)} | '
                     f'{a["complete_windows"]}/{a["clean_windows"]} | '
                     f'{fmt(a["median_rmse_m"], 3)}/{fmt(a["max_rmse_m"], 3)} | '
                     f'{b["complete_windows"]}/{b["clean_windows"]} | '
                     f'{fmt(b["median_rmse_m"], 3)}/{fmt(b["max_rmse_m"], 3)} |')
    lines.extend(['', '| Bag | 5 m 完整/参考合格窗 | 合格窗时长中位数 (s) | '
                  '合格窗 RMSE 中位/最大 (m) |',
                  '|---|---:|---:|---:|'])
    for row in reports:
        value = row['reference_5m']
        lines.append(f'| {row["label"]} | '
                     f'{value["complete_windows"]}/{value["clean_windows"]} | '
                     f'{fmt(value["median_duration_s"], 1)} | '
                     f'{fmt(value["median_rmse_m"], 3)}/'
                     f'{fmt(value["max_rmse_m"], 3)} |')
    lines.extend(['', '参考异常窗口（包括从合格窗汇总中排除的窗口）：', ''])
    flagged = [(report['label'], duration, row) for report in reports
               for duration in ('10', '30')
               for row in report['durations'][duration]['windows']
               if not row['clean_reference']]
    flagged.extend((report['label'], '5 m', row) for report in reports
                   for row in report['reference_5m']['windows']
                   if not row['clean_reference'])
    if flagged:
        for label, duration, row in flagged:
            lines.append(f'- {label}, {duration}, 窗口 {row["ordinal"]}: '
                         + ', '.join(row['reference_flags']) + '.')
    else:
        lines.append('- 无。')
    args.output_markdown.write_text('\n'.join(lines) + '\n')
    print(args.output_markdown)


if __name__ == '__main__':
    main()
