"""Input-only CLAHE control on two exact adjacent VIO capture frames.

The image transform matches run_odom.py --clahe. This diagnostic reports
pixel clipping and front-end correspondences; it does not score VIO motion.
"""

import argparse
import json
from pathlib import Path

import cv2
import numpy as np

from diagnose_frontend import geometry
from run_odom import bag_calibration
from utilities.rosbag_extractor import RosbagExtractor
from vio_pipeline_segments.vo_frontend import VO_frontend
from main_levio import VIOSystem
import rosbag


def read_adjacent(bag_path, topic, wanted):
    images = {}
    extractor = RosbagExtractor(str(bag_path), topic)
    try:
        for raw, stamp in extractor.img_generator():
            for frame, target in wanted.items():
                if frame not in images and abs(stamp - target) < 1e-5:
                    images[frame] = raw
            if len(images) == len(wanted):
                break
    finally:
        extractor.bag.close()
    if len(images) != len(wanted):
        raise ValueError(f'Missing exact capture stamps: {wanted}')
    return images


def evaluate(images, ids, K, D, enhance):
    frontend = VO_frontend()
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    observed = {}
    for frame in ids:
        image = images[frame]
        if enhance:
            image = clahe.apply(image)
        image = cv2.undistort(image, K, D)
        kps, des = frontend.get_keypoints(image)
        observed[frame] = (image, kps, des)
    previous, current = observed[ids[0]], observed[ids[1]]
    if current[2] is None or previous[2] is None:
        matches = []
    else:
        matches = [m for m in frontend.matcher.match(current[2], previous[2])
                   if m.distance <= 30]
    inliers, pose_inliers = geometry(current[1], previous[1], matches, K)
    image = current[0]
    return {
        'previous_keypoints': len(previous[1]),
        'current_keypoints': len(current[1]),
        'adjacent_hamming30_matches': len(matches),
        'adjacent_essential_ransac_inliers': int(inliers.sum()),
        'adjacent_recover_pose_inliers': pose_inliers,
        'current_mean_gray': float(np.mean(image)),
        'current_fraction_gray_ge_250': float(np.mean(image >= 250)),
        'current_fraction_gray_eq_255': float(np.mean(image == 255)),
        'current_fraction_gray_le_5': float(np.mean(image <= 5)),
        'current_laplacian_variance': float(cv2.Laplacian(image, cv2.CV_64F).var()),
    }


def main(args):
    run_dir = Path(args.run_dir)
    summary = json.loads((run_dir / 'summary.json').read_text())
    capture = np.atleast_1d(np.loadtxt(run_dir / 'capture_times.txt'))
    if args.frame < 1 or args.frame >= len(capture):
        raise ValueError('Frame must have a preceding captured input image')
    bag_path = Path(summary['bag'])
    if args.dataset == 'odom':
        with rosbag.Bag(str(bag_path)) as bag:
            topic, _, K, D, _ = bag_calibration(bag, summary['camera'])
    else:
        model = VIOSystem()
        topic, K, D = '/cam0/image_raw', model.K, model.D
    ids = (args.frame - 1, args.frame)
    wanted = {frame: capture[frame] for frame in ids}
    images = read_adjacent(bag_path, topic, wanted)
    raw = images[ids[1]]
    result = {
        'dataset': args.dataset, 'bag': str(bag_path), 'run_dir': str(run_dir),
        'adjacent_frame_ids': list(ids),
        'adjacent_capture_gap_ms': float(1000 * (capture[ids[1]] - capture[ids[0]])),
        'raw_current_fraction_gray_ge_250': float(np.mean(raw >= 250)),
        'raw_current_fraction_gray_eq_255': float(np.mean(raw == 255)),
        'raw_current_mean_gray': float(np.mean(raw)),
        'baseline': evaluate(images, ids, K, D, False),
        'clahe': evaluate(images, ids, K, D, True),
    }
    value = json.dumps(result, indent=2) + '\n'
    if args.output:
        out = Path(args.output)
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(value)
    print(value)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset', choices=['odom', 'euroc'], required=True)
    parser.add_argument('--run-dir', required=True)
    parser.add_argument('--frame', required=True, type=int)
    parser.add_argument('--output')
    main(parser.parse_args())
