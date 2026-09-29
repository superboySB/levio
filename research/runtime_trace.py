"""Record the frame pairs and pose branches selected by the running LEVIO model.

The wrappers call the original methods once and return their results unchanged.
Tracing is optional and does not alter the estimator's inputs or decisions.
"""

import csv
from pathlib import Path


class RuntimeMatchTrace:
    FIELDS = (
        'frame_id', 'capture_time_s', 'keyframe_id', 'keyframe_age_s',
        'matches_hamming_threshold', 'pnp_attempted', 'pnp_accepted_by_model',
        'essential_attempted', 'essential_succeeded', 'essential_skipped',
        'keyframes_after', 'initialized_after',
    )

    def __init__(self, system):
        self.system = system
        self.rows = []
        self.current = {}
        match = system.frontend.get_matches
        pnp = system.frontend.get_pose_ePnP
        essential = system.frontend.get_pose_essential

        def traced_match(frame, keyframe, *args, **kwargs):
            result = match(frame, keyframe, *args, **kwargs)
            self.current.update(frame_id=frame.id, keyframe_id=keyframe.id,
                                keyframe_age_s=frame.t - keyframe.t,
                                matches_hamming_threshold=len(result[0]))
            return result

        def traced_pnp(*args, **kwargs):
            result = pnp(*args, **kwargs)
            self.current.update(pnp_attempted=1,
                                pnp_accepted_by_model=int(not result[0]))
            return result

        def traced_essential(*args, **kwargs):
            result = essential(*args, **kwargs)
            self.current.update(essential_attempted=1,
                                essential_succeeded=int(result[0] is not None))
            return result

        system.frontend.get_matches = traced_match
        system.frontend.get_pose_ePnP = traced_pnp
        system.frontend.get_pose_essential = traced_essential

    def record(self, capture_time_s):
        row = dict.fromkeys(self.FIELDS, '')
        row.update(self.current)
        row['frame_id'] = self.system.graph.frames[-1].id
        row['capture_time_s'] = capture_time_s
        row['essential_skipped'] = int(bool(row['essential_attempted']) and
                                       not bool(row['essential_succeeded']))
        row['keyframes_after'] = len(self.system.graph.keyframes)
        row['initialized_after'] = int(self.system.graph.is_initialized)
        self.rows.append(row)
        self.current = {}

    def save(self, path):
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open('w', newline='') as handle:
            writer = csv.DictWriter(handle, fieldnames=self.FIELDS)
            writer.writeheader()
            writer.writerows(self.rows)
