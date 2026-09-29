"""Print first-pose drift metrics for the selected 20 Hz runs."""

import json
from pathlib import Path

RUNS = ['run009', 'run003', 'run005', 'run007', 'run002', 'run010']

print('| Bag | Frames | Init frame | Skipped E | Matched | First-pose RMSE (m) | End error (m) | EDR (%) | Path ratio | Status |')
print('|---|---:|---:|---:|---:|---:|---:|---:|---:|---|')
for run in RUNS:
    path = Path('research_results') / f'{run}_color_20hz' / 'summary.json'
    if not path.exists():
        print(f'| {run} | pending | — | — | — | — | — | — | — | — |')
        continue
    data = json.loads(path.read_text())
    init = data['initialization_frame']
    metric = data['comparison_after_initialization']
    base = (f'| {run} | {data["frames_processed"]} | '
            f'{init if init is not None else "—"} | '
            f'{data["skipped_essential_frames"]} | '
            f'{metric.get("matched_frames", 0)} | ')
    if not metric.get('metric_scale_valid', False):
        print(base + '— | — | — | — | no valid metric interval |')
    else:
        print(base +
              f'{metric["initial_pose_aligned_rmse_m"]:.3f} | '
              f'{metric["initial_pose_aligned_end_error_m"]:.3f} | '
              f'{metric["endpoint_drift_rate_percent"]:.1f} | '
              f'{metric["path_length_ratio"]:.2f} | initialized |')
