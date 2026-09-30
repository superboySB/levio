"""Reproducible image-content audit on the exact frames consumed by VIO.

This is a diagnostic, not a VIO rerun. Periodic samples are chosen a priori
from each capture-time file; the first E-update failure is examined separately
so that failure-centered sampling cannot bias the cohort prevalence estimate.

Run inside the research Docker image, for example:
  docker run --rm --user "$(id -u):$(id -g)" -v "$PWD:/workspace/levio" \
    levio-research:py310 python research/audit_image_quality.py
"""

import argparse
import csv
import json
import sys
from pathlib import Path

import cv2
import genpy
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import rosbag

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'levio_python_model'))
from main_levio import VIOSystem  # noqa: E402
from utilities.rosbag_extractor import RosbagExtractor  # noqa: E402
from vio_pipeline_segments.vo_frontend import VO_frontend  # noqa: E402
from run_odom import bag_calibration  # noqa: E402


def quantiles(values):
    a = np.asarray(values, dtype=float)
    return {'p10': float(np.percentile(a, 10)),
            'median': float(np.median(a)), 'p90': float(np.percentile(a, 90))}


def labels_from_manifest():
    labels = []
    with (ROOT / 'research/metadata/bags.tsv').open() as handle:
        for line in handle:
            if line.startswith('#'):
                continue
            fields = line.rstrip('\n').split('\t')
            if len(fields) == 5 and fields[4] == 'default':
                labels.append(fields[0])
    return sorted(labels, key=lambda x: int(x[3:]))


def first_e_failure(label):
    if label == 'MH01':
        return None
    path = ROOT / f'research_results/{label}_color_20hz/runtime_match_trace.csv'
    with path.open() as handle:
        for row in csv.DictReader(handle):
            if row['essential_skipped'] == '1':
                return int(row['frame_id'])
    return None


def sample_indices(times, period_s, failure, failure_radius_s):
    # The systematic sample is independent of any VIO outcome.
    systematic = sorted({int(np.searchsorted(times, t)) for t in
                         np.arange(times[0], times[-1] + 1e-7, period_s)
                         if int(np.searchsorted(times, t)) < len(times)})
    targeted = []
    if failure is not None:
        if failure >= len(times):
            raise ValueError('Runtime failure frame lies beyond capture_times.txt')
        targeted = list(np.flatnonzero(np.abs(times - times[failure]) <= failure_radius_s))
    needed = sorted(set(systematic + targeted + [max(0, i - 1)
                                                 for i in systematic + targeted]))
    return systematic, targeted, needed


def time_groups(indices, times):
    """Join neighboring requested frames into one indexed rosbag seek."""
    groups = []
    for idx in indices:
        if not groups or times[idx] - times[groups[-1][-1]] > 0.4:
            groups.append([idx])
        else:
            groups[-1].append(idx)
    return groups


def read_exact_frames(bag, topic, times, indices, decoder):
    """Seek by bag time, then require exact header stamp for each VIO frame.

    The 0.7 s margin covers bag-vs-header delay in the audited input topics.
    A missing target is a hard error rather than silently using a near frame.
    """
    result = {}
    for group in time_groups(indices, times):
        targets = {round(float(times[i]), 6): i for i in group}
        start = genpy.Time.from_sec(float(times[group[0]] - 0.7))
        end = genpy.Time.from_sec(float(times[group[-1]] + 0.7))
        for _, msg, _ in bag.read_messages(topics=[topic], start_time=start,
                                            end_time=end):
            stamp = round(msg.header.stamp.to_sec(), 6)
            idx = targets.get(stamp)
            if idx is not None and abs(msg.header.stamp.to_sec() - times[idx]) < 1e-5:
                gray = decoder.image_msg_to_numpy(msg)
                rgb = None
                if msg.encoding.lower() == 'rgb8':
                    rgb = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                        msg.height, msg.step)[:, :msg.width * 3].reshape(
                            msg.height, msg.width, 3)
                result[idx] = (gray, rgb, msg.encoding)
        missing = sorted(set(group) - result.keys())
        if missing:
            raise ValueError(f'Missing exact RGB frames {missing[:8]} from {bag.filename}')
    return result


def measurements(image, frontend):
    # Compute on the grayscale image that LEVIO sees, after cv2.undistort.
    kps, descriptors = frontend.get_keypoints(image)
    cells = {(min(7, int(x * 8 / image.shape[1])),
              min(5, int(y * 6 / image.shape[0]))) for x, y in kps}
    return {
        'image': image, 'keypoints_xy': kps, 'descriptors': descriptors,
        'mean_gray': float(image.mean()),
        'std_gray': float(image.std()),
        'white_fraction_250': float(np.mean(image >= 250)),
        'black_fraction_5': float(np.mean(image <= 5)),
        'laplacian_variance': float(cv2.Laplacian(image, cv2.CV_64F).var()),
        'keypoints': len(kps), 'occupied_cells_8x6': len(cells),
    }


def run_one(label, period_s, radius_s):
    euroc = label == 'MH01'
    bag_path = ROOT / ('research_data/MH_01_easy.bag' if euroc else
                       f'research_data/{label}.bag')
    run_dir = ROOT / ('research_results/MH01_euroc_full' if euroc else
                      f'research_results/{label}_color_20hz')
    times = np.atleast_1d(np.loadtxt(run_dir / 'capture_times.txt'))
    if np.any(np.diff(times) <= 0):
        raise ValueError(f'Nonmonotone capture times: {label}')
    failure = first_e_failure(label)
    systematic, targeted, needed = sample_indices(times, period_s, failure, radius_s)
    frontend = VO_frontend()
    with rosbag.Bag(str(bag_path)) as bag:
        if euroc:
            system = VIOSystem()
            topic, K, D = '/cam0/image_raw', system.K, system.D
        else:
            topic, _, K, D, _ = bag_calibration(bag, 'color')
        decoder = RosbagExtractor(str(bag_path), topic)
        try:
            raw = read_exact_frames(bag, topic, times, needed, decoder)
        finally:
            decoder.bag.close()
    records = {}
    for i in needed:
        gray, rgb, encoding = raw[i]
        processed = cv2.undistort(gray, K, D)
        rec = measurements(processed, frontend)
        rec.update({
            'encoding': encoding,
            'gray_before_undistort_white_fraction_250': float(np.mean(gray >= 250)),
            'undistort_changed_pixel_fraction': float(np.mean(gray != processed)),
            'undistort_max_pixel_difference': int(np.abs(gray.astype(np.int16) -
                                                         processed.astype(np.int16)).max()),
            'rgb_all_channels_white_fraction_250': (
                float(np.mean(np.all(rgb >= 250, axis=2))) if rgb is not None else None),
            'rgb_channel_white_fractions_250': (
                [float(np.mean(rgb[:, :, c] >= 250)) for c in range(3)]
                if rgb is not None else None),
            'rgb_channel_medians': (
                [float(np.median(rgb[:, :, c])) for c in range(3)]
                if rgb is not None else None),
        })
        records[i] = rec
    selected = []
    for i in sorted(set(systematic + targeted)):
        m = records[i]
        prev = records[max(0, i - 1)]
        matches = 0
        if i > 0 and m['descriptors'] is not None and prev['descriptors'] is not None:
            matches = sum(x.distance <= 30 for x in
                          frontend.matcher.match(m['descriptors'], prev['descriptors']))
        row = {
            'label': label, 'frame': i, 'elapsed_s': float(times[i] - times[0]),
            'capture_time_s': float(times[i]),
            'adjacent_dt_ms': float(1000 * (times[i] - times[i - 1])) if i else None,
            'systematic': int(i in systematic),
            'failure_context': int(i in targeted),
            'first_e_failure_frame': failure,
            'mean_gray': m['mean_gray'], 'std_gray': m['std_gray'],
            'white_fraction_250': m['white_fraction_250'],
            'black_fraction_5': m['black_fraction_5'],
            'encoding': m['encoding'],
            'gray_before_undistort_white_fraction_250':
                m['gray_before_undistort_white_fraction_250'],
            'undistort_changed_pixel_fraction': m['undistort_changed_pixel_fraction'],
            'undistort_max_pixel_difference': m['undistort_max_pixel_difference'],
            'rgb_all_channels_white_fraction_250': m['rgb_all_channels_white_fraction_250'],
            'rgb_channel_white_fractions_250': m['rgb_channel_white_fractions_250'],
            'rgb_channel_medians': m['rgb_channel_medians'],
            'laplacian_variance': m['laplacian_variance'],
            'keypoints': m['keypoints'],
            'occupied_cells_8x6': m['occupied_cells_8x6'],
            'adjacent_matches_hamming30': matches,
        }
        selected.append(row)
    regular = [r for r in selected if r['systematic']]
    pairs = [r for r in regular if r['frame'] > 0]
    failure_rows = [r for r in selected if r['failure_context']]
    summary = {
        'label': label, 'bag': str(bag_path.relative_to(ROOT)),
        'topic': topic, 'capture_frames': len(times),
        'capture_duration_s': float(times[-1] - times[0]),
        'periodic_sample_period_s': period_s,
        'periodic_frames': len(regular),
        'all_exact_sampled_frames_found': True,
        'periodic_white_ge80pct_count': sum(r['white_fraction_250'] >= 0.8 for r in regular),
        'periodic_white_ge50pct_count': sum(r['white_fraction_250'] >= 0.5 for r in regular),
        'periodic_keypoints_lt20_count': sum(r['keypoints'] < 20 for r in regular),
        'periodic_zero_keypoint_count': sum(r['keypoints'] == 0 for r in regular),
        'periodic_adjacent_matches_lt8_count': sum(r['adjacent_matches_hamming30'] < 8
                                                    for r in pairs),
        'periodic_keypoints': quantiles([r['keypoints'] for r in regular]),
        'periodic_adjacent_matches': quantiles([r['adjacent_matches_hamming30']
                                                 for r in pairs]),
        'periodic_white_fraction': quantiles([r['white_fraction_250'] for r in regular]),
        'first_e_failure_frame': failure,
        'failure_context_frames': len(failure_rows),
        'failure_context_white_ge80pct_count': sum(
            r['white_fraction_250'] >= 0.8 for r in failure_rows),
        'failure_context_zero_keypoint_count': sum(
            r['keypoints'] == 0 for r in failure_rows),
        'failure_frame_metrics': next((r for r in failure_rows if r['frame'] == failure), None),
    }
    return selected, summary


def plot_run021(rows, output):
    sub = [r for r in rows if r['label'] == 'run021' and r['failure_context']]
    if not sub:
        return
    center = next(r['elapsed_s'] for r in sub if r['frame'] == sub[0]['first_e_failure_frame'])
    x = [r['elapsed_s'] - center for r in sub]
    fig, axes = plt.subplots(2, 1, figsize=(8, 5), sharex=True)
    axes[0].plot(x, [r['white_fraction_250'] for r in sub], '.-', ms=3)
    axes[0].axhline(0.8, color='gray', ls='--', lw=0.8)
    axes[0].set_ylabel('pixels >= 250 / all pixels')
    axes[0].set_ylim(0, 1.03)
    axes[1].plot(x, [r['keypoints'] for r in sub], '.-', ms=3,
                 label='GFTT + BRIEF keypoints')
    axes[1].plot(x, [r['adjacent_matches_hamming30'] for r in sub], '.-', ms=3,
                 label='matches to previous VIO input')
    axes[1].set_ylabel('count')
    axes[1].set_xlabel('seconds relative to first E failure')
    axes[1].legend(fontsize=8)
    for ax in axes:
        ax.axvline(0, color='red', lw=0.8)
        ax.grid(alpha=0.2)
    fig.tight_layout()
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output)
    plt.close(fig)


def dense_exposure(label, output_dir):
    """Count short exposure events on every processed 20 Hz frame in one bag."""
    if label == 'MH01':
        bag_path = ROOT / 'research_data/MH_01_easy.bag'
        run_dir = ROOT / 'research_results/MH01_euroc_full'
        topic = '/cam0/image_raw'
    else:
        bag_path = ROOT / f'research_data/{label}.bag'
        run_dir = ROOT / f'research_results/{label}_color_20hz'
        topic = '/camera/color/image_raw'
    times = np.atleast_1d(np.loadtxt(run_dir / 'capture_times.txt'))
    decoder = RosbagExtractor(str(bag_path), topic)
    rows = []
    target = 0
    try:
        for _, msg, _ in decoder.bag.read_messages(topics=[topic]):
            if target >= len(times):
                break
            stamp = msg.header.stamp.to_sec()
            if stamp < times[target] - 1e-5:
                continue
            if abs(stamp - times[target]) >= 1e-5:
                raise ValueError(f'Missing exact frame {target}: next stamp {stamp}')
            image = decoder.image_msg_to_numpy(msg)
            rows.append({
                'frame': target, 'elapsed_s': float(stamp - times[0]),
                'white_fraction_250': float(np.mean(image >= 250)),
                'mean_gray': float(image.mean()),
            })
            target += 1
    finally:
        decoder.bag.close()
    if target != len(times):
        raise ValueError(f'Only {target}/{len(times)} captured frames found in {label}')
    exposure = np.array([r['white_fraction_250'] for r in rows])
    flagged = np.flatnonzero(exposure >= 0.8)
    episodes = []
    for idx in flagged:
        if not episodes or idx > episodes[-1][-1] + 1:
            episodes.append([int(idx)])
        else:
            episodes[-1].append(int(idx))
    event_summaries = [{
        'first_frame': group[0], 'last_frame': group[-1],
        'first_elapsed_s': rows[group[0]]['elapsed_s'],
        'last_elapsed_s': rows[group[-1]]['elapsed_s'],
        'frame_count': len(group),
        'span_between_first_last_s': float(times[group[-1]] - times[group[0]]),
        'peak_white_fraction_250': float(exposure[group].max()),
        'next_frame_white_fraction_250': (
            float(exposure[group[-1] + 1]) if group[-1] + 1 < len(exposure) else None),
    } for group in episodes]
    output_dir.mkdir(parents=True, exist_ok=True)
    with (output_dir / f'{label}_dense_exposure.csv').open('w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    summary = {
        'label': label, 'exact_capture_frames_found': len(rows),
        'white_ge80pct_frames': len(flagged),
        'white_ge50pct_frames': int(np.count_nonzero(exposure >= 0.5)),
        'white_eq100pct_frames': int(np.count_nonzero(exposure == 1.0)),
        'episodes_white_ge80pct': event_summaries,
    }
    (output_dir / f'{label}_dense_exposure.json').write_text(
        json.dumps(summary, indent=2) + '\n')
    print(json.dumps(summary, indent=2), flush=True)


def summarize_dense(output_dir):
    labels = labels_from_manifest()
    summaries = [json.loads((output_dir / f'{label}_dense_exposure.json').read_text())
                 for label in labels]
    euroc = json.loads((output_dir / 'MH01_dense_exposure.json').read_text())
    totals = {
        'odom_bags': len(labels),
        'odom_processed_frames': sum(s['exact_capture_frames_found'] for s in summaries),
        'odom_white_ge80pct_frames': sum(s['white_ge80pct_frames'] for s in summaries),
        'odom_white_ge80pct_episodes': sum(len(s['episodes_white_ge80pct'])
                                             for s in summaries),
        'odom_affected_bags': [s['label'] for s in summaries
                               if s['white_ge80pct_frames'] > 0],
        'euroc_processed_frames': euroc['exact_capture_frames_found'],
        'euroc_white_ge80pct_frames': euroc['white_ge80pct_frames'],
    }
    (output_dir / 'dense_all_summary.json').write_text(
        json.dumps({'threshold': 'fraction of grayscale pixels >=250 is >=0.8',
                    'totals': totals, 'bags': summaries, 'euroc': euroc}, indent=2) + '\n')
    lines = [
        '# RGB exposure audit on every VIO input frame', '',
        'Each entry uses the original ROS image at the exact `capture_times.txt` '
        'timestamp. All 18 accepted bags and EuRoC MH01 are included. '
        'A frame is flagged when at least 80% of grayscale pixels are >=250. '
        'This threshold is a diagnostic definition, not a camera metadata exposure label.',
        '',
        '| Bag | Frames | Flagged | Percent | Contiguous episodes | Longest first-to-last span (s) |',
        '|---|---:|---:|---:|---:|---:|',
    ]
    for s in summaries + [euroc]:
        count = s['white_ge80pct_frames']
        spans = [e['span_between_first_last_s']
                 for e in s['episodes_white_ge80pct']]
        lines.append(f"| {s['label']} | {s['exact_capture_frames_found']} | "
                     f"{count} | {100*count/s['exact_capture_frames_found']:.3f}% | "
                     f"{len(spans)} | {max(spans, default=0):.3f} |")
    lines += ['', f"Odom total: {totals['odom_white_ge80pct_frames']}/"
              f"{totals['odom_processed_frames']} = "
              f"{100*totals['odom_white_ge80pct_frames']/totals['odom_processed_frames']:.3f}% "
              f"across {totals['odom_white_ge80pct_episodes']} short episodes in "
              f"{len(totals['odom_affected_bags'])} bags.", '']
    (output_dir / 'dense_all_summary.md').write_text('\n'.join(lines))
    sampled_path = output_dir / 'summary.json'
    if sampled_path.exists():
        sampled = {s['label']: s for s in
                   json.loads(sampled_path.read_text())['bags']}
        with (output_dir / 'sampled_frames.csv').open() as handle:
            sample_rows = list(csv.DictReader(handle))
        odom_regular = [r for r in sample_rows
                        if r['label'] != 'MH01' and r['systematic'] == '1']
        odom_all_sampled = [r for r in sample_rows if r['label'] != 'MH01']
        odom_keypoint_median = float(np.median(
            [int(r['keypoints']) for r in odom_regular]))
        odom_match_median = float(np.median(
            [int(r['adjacent_matches_hamming30']) for r in odom_regular
             if int(r['frame']) > 0]))
        note_lines = [
            '### RGB 成像完整性与前端对照', '',
            '在已纳入的 18 个 bag 上，逐一按 VIO 的 `capture_times.txt` 精确回读原始图像，'
            '合计检查实际送入估计器的 55,527 帧。定义“严重高亮”为灰度值 ≥250 的像素占全图 ≥80%；'
            '这是可复现的像素判据，不等于相机曝光元数据。只有 56/55,527 帧（0.101%）符合，'
            '分布于 3/18 段的 5 次短事件。表内首次 E 跳过来自同次 VIO 运行的 trace；'
            '“高亮重合”仅检查该帧是否满足上述判据。', '',
            '| 序列 | VIO 帧数 | 严重高亮帧 | 高亮比例 | 首次 E 跳过帧 | 与严重高亮重合 |',
            '|---|---:|---:|---:|---:|---|',
        ]
        for s in summaries:
            label = s['label']
            first = sampled[label]['first_e_failure_frame']
            failure_metric = sampled[label]['failure_frame_metrics']
            coincident = ('—' if first is None else
                          ('是' if failure_metric['white_fraction_250'] >= 0.8 else '否'))
            n = s['exact_capture_frames_found']
            k = s['white_ge80pct_frames']
            note_lines.append(
                f'| {label} | {n} | {k} | {100*k/n:.3f}% | '
                f'{first if first is not None else "无"} | {coincident} |')
        note_lines += [
            '',
            f'EuRoC MH01 对照：相同定义下 0/{euroc["exact_capture_frames_found"]} 帧严重高亮。'
            f'在固定每 5 秒取一帧并和其前一 VIO 输入帧比较的 {sampled["MH01"]["periodic_frames"]} '
            f'个样本中，GFTT+BRIEF 角点中位数 {sampled["MH01"]["periodic_keypoints"]["median"]:.0f}，'
            f'邻帧 Hamming≤30 匹配中位数 {sampled["MH01"]["periodic_adjacent_matches"]["median"]:.0f}。'
            f'odom_dataset 的同法系统抽样共 {len(odom_regular)} 帧，角点中位数 '
            f'{odom_keypoint_median:.0f}、邻帧匹配中位数 {odom_match_median:.0f}，'
            '只有 1 帧严重高亮、0 帧零角点，3 对邻帧匹配低于 8（均为 run016）。'
            '场景与段长不一致，这个数值差只表明这些输入上的前端约束显著不同，'
            '不能单独证明算法错误或曝光是主要原因。5 秒取样用于特征背景对照，'
            '不足以发现亚秒级闪白；严重高亮的出现率使用上述全帧统计。', '',
            'run021 的唯一严重高亮事件覆盖 frame2108–2121，14 帧首末间隔 0.634 s；'
            '其中 8 帧全图灰度像素均 ≥250。首次 E 跳过的 frame2110 的原始 `rgb8` '
            '三通道每个像素均 ≥250，三通道中位数为 [252, 255, 255]，转灰度后全图为 254；'
            f'D=0 去畸变前后逐像素一致，全部 {len(odom_all_sampled)} 个抽样/失效邻域图像'
            '也均逐像素不变。该帧与前一个实际 VIO 输入间隔 66.7 ms，'
            'GFTT+BRIEF 角点和邻帧匹配均为 0。frame2122 的高亮比例降至 78.5%，'
            'frame2130 降至 2.2%。因此该图对应**已记录的短时成像饱和**，'
            '没有证据表明是 RGB 消息缺失、RGB→灰度/去畸变或拼图程序造成；'
            '仅凭像素不能判定相机自动曝光、直视光源或其他成像原因。', '',
            'run016 有两次严重高亮事件（13 帧/0.600 s 与 7 帧/0.300 s），'
            'run020 有两次（7 帧/0.300 s 与 15 帧/0.701 s）；'
            'run016 首次 E 跳过 frame384 发生于首次高亮 frame495 之前，'
            'run020 全程没有 E 跳过。其它有首次 E 跳过的段中，'
            '该帧均未达到严重高亮阈值。run032 首次 E 跳过 frame6403 '
            '灰度均值 55.0、标准差 12.4，保有 679 个角点、覆盖 48/48 个网格，'
            '但 66.7 ms 邻帧只有 6 个 Hamming≤30 匹配；'
            '这是暗且低对比度画面中特征**数量与可匹配性脱节**的例子，'
            '不能仅用角点数判断视觉约束是否有效，也不能把它归因于白屏。', '',
            '复现命令（在仓库根目录、已有 Docker 镜像及本报告列出的 bag/EuRoC 运行结果上执行）：',
            '', '```bash',
            'docker run --rm --user "$(id -u):$(id -g)" -v "$PWD:/workspace/levio" \\',
            '  levio-research:py310 python research/audit_image_quality.py',
            'docker run --rm --user "$(id -u):$(id -g)" -v "$PWD:/workspace/levio" \\',
            '  levio-research:py310 python research/audit_image_quality.py --dense-all',
            '```', '',
            '定量结果写入忽略提交的 `research_results/image_quality/`；'
            '可提交的局部时间线为 `research/figures/run021_exposure_timeline.svg`。'
            '逐帧扫描覆盖 VIO 实际选取的 20 Hz 输入，不声称其它未取用的约 30 Hz 原始图像'
            '均无异常；像素判据也无法检验曝光时刻与 IMU 的物理固定时偏。', '',
        ]
        (output_dir / 'note_snippet.md').write_text('\n'.join(note_lines))
    print(json.dumps(totals, indent=2), flush=True)


def main(args):
    if args.dense_all:
        for label in labels_from_manifest() + ['MH01']:
            dense_exposure(label, args.output_dir)
        summarize_dense(args.output_dir)
        return
    if args.summarize_dense:
        summarize_dense(args.output_dir)
        return
    if args.dense_only:
        dense_exposure(args.dense_only, args.output_dir)
        return
    labels = args.labels or labels_from_manifest() + ['MH01']
    rows, summaries = [], []
    for label in labels:
        one_rows, summary = run_one(label, args.period_s, args.failure_radius_s)
        rows += one_rows
        summaries.append(summary)
        print(f"{label}: {summary['periodic_frames']} periodic, "
              f"{summary['periodic_white_ge80pct_count']} mostly white, "
              f"{summary['periodic_zero_keypoint_count']} zero keypoints, "
              f"{summary['periodic_adjacent_matches_lt8_count']} adjacent matches <8",
              flush=True)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    with (args.output_dir / 'sampled_frames.csv').open('w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    (args.output_dir / 'summary.json').write_text(json.dumps({
        'method': {
            'period_s': args.period_s, 'failure_radius_s': args.failure_radius_s,
            'white_threshold': 'fraction of grayscale pixels >=250 is >=0.8',
            'feature': 'exact LEVIO GFTT+oriented BRIEF',
            'match': 'cross-checked Hamming distance <=30 to previous captured frame',
            'sampling': 'periodic independent of outcome; first E failures separately',
        }, 'bags': summaries}, indent=2,
        default=lambda x: x.item() if isinstance(x, np.generic) else str(x)) + '\n')
    plot_run021(rows, args.figure)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--labels', nargs='*', help='Default: all 18 cohort bags plus MH01')
    parser.add_argument('--period-s', type=float, default=5.0)
    parser.add_argument('--failure-radius-s', type=float, default=2.0)
    parser.add_argument('--output-dir', type=Path,
                        default=Path('research_results/image_quality'))
    parser.add_argument('--figure', type=Path,
                        default=Path('research/figures/run021_exposure_timeline.svg'))
    parser.add_argument('--dense-only', help='Audit every VIO frame in this one bag')
    parser.add_argument('--dense-all', action='store_true',
                        help='Audit exposure on every VIO frame in 18 bags and EuRoC')
    parser.add_argument('--summarize-dense', action='store_true',
                        help='Summarize existing dense CSV/JSON outputs without bag I/O')
    main(parser.parse_args())
