"""Inspect a low-match runtime keyframe pair using the original OpenCV call."""

import argparse
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
    path = args.bag
    with rosbag.Bag(path) as bag:
        topic, _, K, D, _ = bag_calibration(bag, 'color')
    extractor = RosbagExtractor(path, topic)
    selected = {}
    next_time = None
    first_time = None
    index = 0
    for raw, stamp in extractor.img_generator():
        if next_time is None:
            first_time = next_time = stamp
        if stamp + 1e-6 < next_time:
            continue
        while next_time <= stamp + 1e-6:
            next_time += 0.05
        if index in (args.keyframe, args.frame - 1, args.frame):
            selected[index] = (cv2.undistort(raw, K, D), stamp)
        index += 1
        if index > args.frame:
            break
    extractor.bag.close()
    frontend = VO_frontend()
    frames = {}
    for key, (image, stamp) in selected.items():
        kps, des = frontend.get_keypoints(image)
        frames[key] = SimpleNamespace(image=image, kps=kps, des=des, stamp=stamp)
    idx1, idx2, _ = frontend.get_matches(frames[args.frame], frames[args.keyframe],
                                          hamming_threshold=30)
    report = {'bag': path, 'keyframe': args.keyframe, 'frame': args.frame,
              'matches': len(idx1),
              'time_gap_s': frames[args.frame].stamp - frames[args.keyframe].stamp}
    if args.frame - 1 in frames:
        adjacent_idx, _, _ = frontend.get_matches(frames[args.frame],
                                                  frames[args.frame - 1],
                                                  hamming_threshold=30)
        report['adjacent_input_matches'] = len(adjacent_idx)
        report['adjacent_input_gap_s'] = (frames[args.frame].stamp -
                                          frames[args.frame - 1].stamp)
    if args.figure_prefix:
        for previous_id, suffix in ((args.frame - 1, 'adjacent'),
                                    (args.keyframe, 'selected_keyframe')):
            current, previous = frames[args.frame], frames[previous_id]
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
                    inlier_label='usable pose inliers')
    try:
        E, mask = cv2.findEssentialMat(frames[args.frame].kps[idx1],
                                       frames[args.keyframe].kps[idx2],
                                       K, cv2.RANSAC)
        report['essential_shape'] = list(E.shape) if E is not None else None
        report['ransac_inliers'] = int(np.count_nonzero(mask)) if mask is not None else 0
        if E is not None and mask is not None and np.count_nonzero(mask) >= 5:
            p1 = frames[args.frame].kps[idx1][mask.ravel() == 1]
            p2 = frames[args.keyframe].kps[idx2][mask.ravel() == 1]
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
    parser.add_argument('--bag', default='research_data/run005.bag')
    parser.add_argument('--keyframe', type=int, default=225)
    parser.add_argument('--frame', type=int, default=231)
    parser.add_argument('--output')
    parser.add_argument('--figure-prefix')
    main(parser.parse_args())
