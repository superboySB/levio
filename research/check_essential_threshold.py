"""Inspect a low-match runtime keyframe pair using the original OpenCV call."""

import argparse
import csv
import json
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import rosbag

from diagnose_frontend import geometry, montage
from run_odom import bag_calibration
from utilities.rosbag_extractor import RosbagExtractor
from vio_pipeline_segments.vo_frontend import VO_frontend


def main(args):
    run_dir = Path(args.run_dir)
    summary = json.loads((run_dir / 'summary.json').read_text())
    path = args.bag or summary['bag']
    with (run_dir / 'runtime_match_trace.csv').open(newline='') as handle:
        trace = list(csv.DictReader(handle))
    by_id = {int(row['frame_id']): row for row in trace}
    frame_id = args.frame
    if frame_id is None:
        failed = next((row for row in trace if row['essential_skipped'] == '1'), None)
        if failed is None:
            raise ValueError(f'No failed essential update in {run_dir}')
        frame_id = int(failed['frame_id'])
    if frame_id not in by_id or frame_id - 1 not in by_id:
        raise ValueError(f'Missing adjacent input frames for {frame_id}')
    keyframe_id = args.keyframe if args.keyframe is not None else int(
        by_id[frame_id]['keyframe_id'])
    if keyframe_id not in by_id:
        raise ValueError(f'Missing selected keyframe {keyframe_id}')
    timestamps = {index: float(by_id[index]['capture_time_s'])
                  for index in (keyframe_id, frame_id - 1, frame_id)}
    with rosbag.Bag(path) as bag:
        topic, _, K, D, _ = bag_calibration(bag, 'color')
    extractor = RosbagExtractor(path, topic)
    selected = {}
    first_time = float(trace[0]['capture_time_s'])
    for raw, stamp in extractor.img_generator():
        for index, wanted in timestamps.items():
            if index not in selected and abs(stamp - wanted) <= 1e-5:
                selected[index] = (cv2.undistort(raw, K, D), stamp)
        if len(selected) == len(timestamps):
            break
    extractor.bag.close()
    if len(selected) != len(timestamps):
        raise ValueError(f'Could not find runtime images for {timestamps}')
    frontend = VO_frontend()
    frames = {}
    for key, (image, stamp) in selected.items():
        kps, des = frontend.get_keypoints(image)
        frames[key] = SimpleNamespace(image=image, kps=kps, des=des, stamp=stamp)
    idx1, idx2, _ = frontend.get_matches(frames[frame_id], frames[keyframe_id],
                                          hamming_threshold=30)
    report = {'bag': path, 'run_dir': str(run_dir),
              'keyframe': keyframe_id, 'frame': frame_id,
              'matches': len(idx1),
              'time_gap_s': frames[frame_id].stamp - frames[keyframe_id].stamp,
              'research_essential_min_matches': 8,
              'runtime_essential_attempted': by_id[frame_id]['essential_attempted'] == '1',
              'runtime_pnp_accepted': by_id[frame_id]['pnp_accepted_by_model'] == '1'}
    if frame_id - 1 in frames:
        adjacent_idx, _, _ = frontend.get_matches(frames[frame_id],
                                                  frames[frame_id - 1],
                                                  hamming_threshold=30)
        report['adjacent_input_matches'] = len(adjacent_idx)
        report['adjacent_input_gap_s'] = (frames[frame_id].stamp -
                                          frames[frame_id - 1].stamp)
    if args.figure_prefix:
        for previous_id, suffix in ((frame_id - 1, 'adjacent'),
                                    (keyframe_id, 'selected_keyframe')):
            current, previous = frames[frame_id], frames[previous_id]
            if current.des is None or previous.des is None:
                matches = []
            else:
                matches = sorted(frontend.matcher.match(current.des, previous.des),
                                 key=lambda match: match.distance)
                matches = [match for match in matches if match.distance <= 30]
            mask, _ = geometry(current.kps, previous.kps, matches, K)
            figure = Path(f'{args.figure_prefix}_{suffix}.jpg')
            label = (f'{Path(path).stem} adjacent'
                     if suffix == 'adjacent' else
                     f'{Path(path).stem} selected KF')
            montage(previous.image, current.image, previous.kps, current.kps,
                    matches, mask, label, current.stamp - first_time, figure,
                    inlier_label='diagnostic E inliers')
    try:
        E, mask = cv2.findEssentialMat(frames[frame_id].kps[idx1],
                                       frames[keyframe_id].kps[idx2],
                                       K, cv2.RANSAC)
        report['essential_shape'] = list(E.shape) if E is not None else None
        report['ransac_inliers'] = int(np.count_nonzero(mask)) if mask is not None else 0
        if E is not None and mask is not None and np.count_nonzero(mask) >= 5:
            p1 = frames[frame_id].kps[idx1][mask.ravel() == 1]
            p2 = frames[keyframe_id].kps[idx2][mask.ravel() == 1]
            count, _, _, _ = cv2.recoverPose(E, p1, p2, K)
            report['recover_pose_inliers'] = int(count)
    except cv2.error as exc:
        report['opencv_error'] = str(exc).splitlines()[0]
    value = json.dumps(report, indent=2) + '\n'
    if args.output:
        path = Path(args.output)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(value)
    print(value)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--run-dir', default='research_results/run005_color_20hz')
    parser.add_argument('--bag', help='Override the bag path in summary.json')
    parser.add_argument('--keyframe', type=int)
    parser.add_argument('--frame', type=int)
    parser.add_argument('--output')
    parser.add_argument('--figure-prefix')
    main(parser.parse_args())
