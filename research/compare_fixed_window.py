"""Compare the first equal-duration, initialized intervals without future alignment."""

import argparse
import json
from pathlib import Path

import numpy as np

from plot_results import aligned_samples


def summarize(run_dir, seconds):
    summary = json.loads((run_dir / 'summary.json').read_text())
    if not summary['comparison_after_initialization'].get(
            'metric_scale_valid', summary['vio_initialized']):
        return {'run': run_dir.name, 'status': 'no valid initialized interval'}
    _, times, reference, estimate, _ = aligned_samples(run_dir)
    take = times <= seconds + 1e-6
    if np.count_nonzero(take) < 2:
        return {'run': run_dir.name, 'status': 'fewer than two poses in requested window',
                'seconds_requested': seconds, 'matched_frames': int(np.count_nonzero(take))}
    ref = reference[take]
    est = estimate[take]
    error = np.linalg.norm(ref - est, axis=1)
    ref_length = float(np.linalg.norm(np.diff(ref, axis=0), axis=1).sum())
    est_steps = np.linalg.norm(np.diff(est, axis=0), axis=1)
    est_length = float(est_steps.sum())
    seconds_evaluated = float(times[take][-1])
    return {
        'run': run_dir.name,
        'status': 'complete' if seconds_evaluated >= seconds - 0.1
                  else 'shorter than requested window',
        'reference': 'EuRoC ground truth' if summary.get('dataset') == 'euroc'
                     else 'recorded fusion odometry; not proven ground truth',
        'seconds_requested': seconds, 'seconds_evaluated': seconds_evaluated,
        'window_complete': seconds_evaluated >= seconds - 0.1,
        'matched_frames': int(take.sum()),
        'first_pose_rmse_m': float(np.sqrt(np.mean(error ** 2))),
        'end_error_m': float(error[-1]),
        'reference_path_length_m': ref_length,
        'estimated_path_length_m': est_length,
        'estimated_step_p99_m': float(np.percentile(est_steps, 99)),
        'estimated_step_max_m': float(np.max(est_steps)),
        'endpoint_drift_rate_percent': float(error[-1] / ref_length * 100)
                                       if ref_length else None,
    }


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('run_dirs', nargs='+', type=Path)
    parser.add_argument('--seconds', type=float, default=10.0)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if args.seconds <= 0:
        parser.error('--seconds must be positive')
    results = [summarize(path, args.seconds) for path in args.run_dirs]
    value = json.dumps(results, indent=2) + '\n'
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(value)
    print(value)
