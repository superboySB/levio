"""Compare input variants on identical captures and a common initialized prefix.

Each estimate is aligned to the same recorded reference pose at the first
shared frame using its own initial orientation. No trajectory/scale fitting.
"""

import argparse
import json
from pathlib import Path

import numpy as np
import rosbag
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation

from run_odom import aligned_trajectories, bag_calibration, reference_poses


def load_run(path):
    summary = json.loads((path / 'summary.json').read_text())
    pose = np.atleast_2d(np.loadtxt(path / 'online_frames.tum', comments='#'))
    capture = np.atleast_1d(np.loadtxt(path / 'capture_times.txt'))
    if pose.shape != (len(capture), 8) or not np.allclose(
            pose[:, 0], capture, rtol=0, atol=1e-6):
        raise ValueError(f'{path}: exported poses and capture times disagree')
    return summary, pose, capture


def main(args):
    base_summary, base_pose, base_capture = load_run(Path(args.baseline))
    variant_summary, variant_pose, variant_capture = load_run(Path(args.variant))
    if (base_summary['bag'] != variant_summary['bag'] or
            base_summary['camera'] != variant_summary['camera'] or
            len(base_capture) != len(variant_capture) or
            not np.allclose(base_capture, variant_capture, rtol=0, atol=1e-6)):
        raise ValueError('Comparisons require the same bag, camera, and exact captured images')
    init = [base_summary['initialization_frame'],
            variant_summary['initialization_frame']]
    report = {
        'baseline': str(args.baseline), 'variant': str(args.variant),
        'bag': base_summary['bag'], 'same_captures': True,
        'processed_frames': len(base_capture),
        'baseline_initialization_frame': init[0],
        'variant_initialization_frame': init[1],
        'baseline_first_E_skip_frame': base_summary['first_essential_skip_frame'],
        'variant_first_E_skip_frame': variant_summary['first_essential_skip_frame'],
        'reference_is_ground_truth': False,
    }
    if any(x is None for x in init):
        report['status'] = 'no common metric window: one or both variants never initialized'
    else:
        start = max(init)
        stops = [len(base_capture) if summary['first_essential_skip_frame'] is None
                 else summary['first_essential_skip_frame']
                 for summary in (base_summary, variant_summary)]
        end = min(stops)
        report.update(start_frame=start, end_frame_exclusive=end)
        if end - start < 3:
            report['status'] = 'no common metric window before first E skip'
        else:
            with rosbag.Bag(base_summary['bag']) as bag:
                _, _, _, _, T = bag_calibration(bag, base_summary['camera'])
                reference_t, reference_xyz, reference_rot = reference_poses(
                    bag, T, with_rotations=True)
            samples_by_name = {}
            for name, poses in (('baseline_common_window', base_pose),
                                ('variant_common_window', variant_pose)):
                subset = poses[start:end]
                samples, metric = aligned_trajectories(
                    base_capture[start:end], subset[:, 1:4], reference_t,
                    reference_xyz,
                    Rotation.from_quat(subset[:, 4:8]).as_matrix(),
                    reference_rot)
                if metric['matched_frames'] != end - start or metric[
                        'initial_pose_aligned_start_error_m'] > 1e-9:
                    raise ValueError(f'{name}: unmatched input or shared start failed')
                report[name] = {
                    key: metric[key] for key in (
                        'matched_frames', 'matched_time_span_s',
                        'initial_pose_aligned_start_error_m',
                        'initial_pose_aligned_rmse_m',
                        'initial_pose_aligned_end_error_m',
                        'estimated_path_length_m', 'reference_path_length_m',
                        'path_length_ratio')
                }
                samples_by_name[name] = samples
            report['status'] = 'matched shared initialized and pre-failure interval'
            if args.figure:
                base = samples_by_name['baseline_common_window']
                variant = samples_by_name['variant_common_window']
                if not np.allclose(base['reference'], variant['reference'],
                                   rtol=0, atol=1e-9):
                    raise ValueError('Comparison paths do not share reference samples')
                ref = base['reference']
                pred0 = base['initial_pose_fit']
                pred1 = variant['initial_pose_fit']
                if (np.linalg.norm(ref[0] - pred0[0]) > 1e-9 or
                        np.linalg.norm(ref[0] - pred1[0]) > 1e-9):
                    raise ValueError('Figure paths do not share the first pose')
                fig, axes = plt.subplots(1, 2, figsize=(10, 3.8),
                                         layout='constrained')
                elapsed = base['times'] - base['times'][0]
                for xyz, label, color in ((ref, 'Recorded odometry', '#238b45'),
                                          (pred0, 'Baseline', '#d94841'),
                                          (pred1, 'Variant', '#3268a8')):
                    axes[0].plot(xyz[:, 0], xyz[:, 1], label=label,
                                 color=color, linewidth=1.4)
                    axes[1].plot(elapsed, xyz[:, 2], label=label,
                                 color=color, linewidth=1.4)
                axes[0].scatter(ref[0, 0], ref[0, 1], color='black', s=20,
                                label='shared start', zorder=5)
                axes[1].scatter(0, ref[0, 2], color='black', s=20, zorder=5)
                axes[0].set(xlabel='x (m)', ylabel='y (m)', title='X-Y trajectory')
                axes[0].set_aspect('equal', adjustable='datalim')
                axes[1].set(xlabel='time from shared start (s)', ylabel='z (m)',
                            title='Height')
                axes[0].legend(fontsize=7)
                for axis in axes:
                    axis.grid(alpha=0.3)
                fig.suptitle(f'Same {end-start} captured frames; one first-pose alignment per estimate')
                out = Path(args.figure)
                out.parent.mkdir(parents=True, exist_ok=True)
                fig.savefig(out, format='svg')
                plt.close(fig)
                report['figure'] = str(out)
    value = json.dumps(report, indent=2) + '\n'
    if args.output:
        output = Path(args.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(value)
    print(value)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', type=Path, required=True)
    parser.add_argument('--variant', type=Path, required=True)
    parser.add_argument('--output')
    parser.add_argument('--figure', help='Optional SVG with a shared physical start pose')
    main(parser.parse_args())
