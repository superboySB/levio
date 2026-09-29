"""Summarize the image pairs and estimator branches actually used at runtime."""

import argparse
import csv
import json
import statistics
from pathlib import Path


def summarize(path):
    with Path(path).open(newline='') as handle:
        rows = list(csv.DictReader(handle))
    compared = [row for row in rows if row['keyframe_id']]
    matches = [int(row['matches_hamming_threshold']) for row in compared]
    ages = [float(row['keyframe_age_s']) for row in compared]
    skipped = [row for row in compared if row['essential_skipped'] == '1']
    return {
        'run': Path(path).parent.name, 'frames': len(rows),
        'compared_frames': len(compared),
        'runtime_keyframe_matches_median': statistics.median(matches),
        'runtime_keyframe_age_median_s': statistics.median(ages),
        'runtime_keyframe_age_max_s': max(ages),
        'matches_below_8': sum(count < 8 for count in matches),
        'essential_skipped': len(skipped),
        'first_essential_skip_frame': (int(skipped[0]['frame_id']) if skipped else None),
        'pnp_accepted_by_model': sum(row['pnp_accepted_by_model'] == '1'
                                     for row in rows),
        'initialized_final': rows[-1]['initialized_after'] == '1',
        'last_keyframe_id': rows[-1]['keyframe_id'],
    }


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('traces', nargs='+', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    results = [summarize(path) for path in args.traces]
    value = json.dumps(results, indent=2) + '\n'
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(value)
    print(value)
