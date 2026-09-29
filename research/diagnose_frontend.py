"""Inspect the exact LEVIO feature detector and adjacent-frame matcher.

This is a controlled visual-front-end diagnostic, not a VIO trajectory run.
Only JPEG montages are intended for Git; per-frame CSV is ignored.
"""

import argparse
import csv
import json
import sys
from pathlib import Path

import cv2
import numpy as np
import rosbag

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'levio_python_model'))
from main_levio import VIOSystem  # noqa: E402
from utilities.rosbag_extractor import RosbagExtractor  # noqa: E402
from vio_pipeline_segments.vo_frontend import VO_frontend  # noqa: E402
from run_odom import bag_calibration  # noqa: E402


def geometry(kps, prev_kps, matches, K):
    if len(matches) < 8:
        return np.zeros(len(matches), dtype=bool), 0
    current = np.array([kps[m.queryIdx] for m in matches])
    previous = np.array([prev_kps[m.trainIdx] for m in matches])
    try:
        essential, mask = cv2.findEssentialMat(current, previous, K, cv2.RANSAC)
        if essential is None or mask is None:
            return np.zeros(len(matches), dtype=bool), 0
        inliers = mask.ravel().astype(bool)
        if inliers.sum() < 5:
            return inliers, 0
        count, _, _, _ = cv2.recoverPose(essential, current[inliers], previous[inliers], K)
        return inliers, int(count)
    except cv2.error:
        return np.zeros(len(matches), dtype=bool), 0


def annotated(image, kps):
    color = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    for x, y in kps:
        cv2.circle(color, (int(x), int(y)), 2, (220, 180, 0), -1)
    return color


def montage(previous, current, previous_kps, kps, matches, inliers, label,
            elapsed, output, inlier_label='E inliers'):
    height, width = current.shape
    upper = np.hstack((annotated(previous, previous_kps), annotated(current, kps)))
    lower = np.hstack((cv2.cvtColor(previous, cv2.COLOR_GRAY2BGR),
                       cv2.cvtColor(current, cv2.COLOR_GRAY2BGR)))
    for index, match in enumerate(matches[:70]):
        a = tuple(np.rint(previous_kps[match.trainIdx]).astype(int))
        b = tuple(np.rint(kps[match.queryIdx]).astype(int) + np.array([width, 0]))
        color = (60, 200, 60) if inliers[index] else (40, 60, 230)
        cv2.line(lower, a, b, color, 1, cv2.LINE_AA)
        cv2.circle(lower, a, 2, color, -1)
        cv2.circle(lower, b, 2, color, -1)
    image = np.vstack((upper, lower))
    cv2.rectangle(image, (0, 0), (2 * width, 28), (0, 0, 0), -1)
    title = (f'{label} t={elapsed:.1f}s | LEVIO GFTT+BRIEF | '
             f'keypoints {len(kps)} | Hamming<=30 {len(matches)} | '
             f'{inlier_label} {inliers.sum()}')
    cv2.putText(image, title, (8, 19), cv2.FONT_HERSHEY_SIMPLEX,
                0.55, (255, 255, 255), 1, cv2.LINE_AA)
    output.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(output), image, [cv2.IMWRITE_JPEG_QUALITY, 77])


def quantiles(values):
    arr = np.array(values, dtype=float)
    return {name: float(np.percentile(arr, percent)) for name, percent in
            [('p10', 10), ('median', 50), ('p90', 90)]}


def run(args):
    with rosbag.Bag(args.bag) as bag:
        if args.dataset == 'euroc':
            topic = '/cam0/image_raw'
            system = VIOSystem()
            K, D = system.K, system.D
        else:
            topic, _, K, D, _ = bag_calibration(bag, args.camera)

    extractor = RosbagExtractor(args.bag, topic)
    frontend = VO_frontend()
    rows = []
    previous = None
    next_time = None
    first_time = None
    snapshots = set()
    for raw, stamp in extractor.img_generator():
        if first_time is None:
            first_time = stamp
            next_time = stamp
        elapsed = stamp - first_time
        if args.max_seconds and elapsed > args.max_seconds:
            break
        if args.target_fps:
            if stamp + 1e-6 < next_time:
                continue
            while next_time <= stamp + 1e-6:
                next_time += 1.0 / args.target_fps
        image = cv2.undistort(raw, K, D)
        kps, descriptors = frontend.get_keypoints(image)
        cells = set((min(7, int(x * 8 / image.shape[1])),
                     min(5, int(y * 6 / image.shape[0]))) for x, y in kps)
        row = {
            'frame': len(rows), 'time_s': elapsed,
            'mean_gray': float(np.mean(image)),
            'laplacian_variance': float(cv2.Laplacian(image, cv2.CV_64F).var()),
            'keypoints': len(kps), 'occupied_cells_8x6': len(cells),
            'matches_hamming30': 0, 'essential_inliers': 0,
            'essential_inlier_fraction': 0.0, 'recover_pose_inliers': 0,
            'median_displacement_px': 0.0,
        }
        if previous is not None and descriptors is not None and previous[2] is not None:
            all_matches = sorted(frontend.matcher.match(descriptors, previous[2]),
                                 key=lambda match: match.distance)
            matches = [match for match in all_matches if match.distance <= 30]
            inliers, pose_inliers = geometry(kps, previous[1], matches, K)
            row['matches_hamming30'] = len(matches)
            row['essential_inliers'] = int(inliers.sum())
            row['essential_inlier_fraction'] = float(inliers.mean()) if len(matches) else 0.0
            row['recover_pose_inliers'] = pose_inliers
            if matches:
                shifts = [np.linalg.norm(kps[m.queryIdx] - previous[1][m.trainIdx])
                          for m in matches]
                row['median_displacement_px'] = float(np.median(shifts))
            for target in args.snapshot_seconds:
                if target not in snapshots and elapsed >= target:
                    figure = Path(args.figure_dir) / f'frontend_{args.label}_{target}s.jpg'
                    montage(previous[0], image, previous[1], kps, matches,
                            inliers, args.label, elapsed, figure)
                    snapshots.add(target)
        rows.append(row)
        previous = image, kps, descriptors
    extractor.bag.close()
    output = Path(args.output_dir)
    output.mkdir(parents=True, exist_ok=True)
    with (output / f'{args.label}.csv').open('w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    pairs = rows[1:]
    summary = {
        'label': args.label, 'bag': args.bag, 'dataset': args.dataset,
        'camera': args.camera if args.dataset == 'odom' else 'cam0',
        'image_topic': topic, 'target_fps': args.target_fps,
        'frames': len(rows), 'duration_s': rows[-1]['time_s'],
        'keypoints': quantiles([r['keypoints'] for r in rows]),
        'occupied_cells_8x6': quantiles([r['occupied_cells_8x6'] for r in rows]),
        'laplacian_variance': quantiles([r['laplacian_variance'] for r in rows]),
        'matches_hamming30': quantiles([r['matches_hamming30'] for r in pairs]),
        'essential_inliers': quantiles([r['essential_inliers'] for r in pairs]),
        'essential_inlier_fraction': quantiles([r['essential_inlier_fraction'] for r in pairs]),
        'recover_pose_inliers': quantiles([r['recover_pose_inliers'] for r in pairs]),
        'fraction_with_25_matches': sum(r['matches_hamming30'] >= 25 for r in pairs) / len(pairs),
        'fraction_with_15_E_inliers': sum(r['essential_inliers'] >= 15 for r in pairs) / len(pairs),
    }
    (output / f'{args.label}.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--bag', required=True)
    parser.add_argument('--dataset', choices=['odom', 'euroc'], required=True)
    parser.add_argument('--camera', choices=['color', 'infra1'], default='color')
    parser.add_argument('--label', required=True)
    parser.add_argument('--target-fps', type=float, default=20)
    parser.add_argument('--max-seconds', type=float, default=40)
    parser.add_argument('--snapshot-seconds', type=int, nargs='*', default=[10])
    parser.add_argument('--figure-dir', default='research/figures')
    parser.add_argument('--output-dir', default='research_results/frontend')
    run(parser.parse_args())
