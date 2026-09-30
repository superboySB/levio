"""Insert validated, reproducible cohort and segment tables into note.md."""

import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
RESULTS = ROOT / 'research_results'


def replace_section(text, name, payload):
    first = f'<!-- GENERATED {name} START -->'
    last = f'<!-- GENERATED {name} END -->'
    if text.count(first) != 1 or text.count(last) != 1:
        raise ValueError(f'note.md must contain one {name} marker pair')
    before, rest = text.split(first, 1)
    _, after = rest.split(last, 1)
    return before + first + '\n\n' + payload.strip() + '\n\n' + last + after


def main():
    cohort = json.loads((RESULTS / 'cohort_report.json').read_text())
    if (cohort['selected_runs'], cohort['audit_passed_runs'],
            cohort['complete_result_runs'], cohort['ready_runs']) != (18, 18, 18, 18):
        raise ValueError('The pinned 18-bag cohort has not fully passed')
    segment = json.loads((RESULTS / 'segment_report.json').read_text())
    if len(segment['reports']) != 19:
        raise ValueError('Expected EuRoC MH01 plus 18 odom sessions')
    windows = [row for report in segment['reports']
               for group in [*report['durations'].values(), report['reference_5m']]
               for row in group['windows']]
    if len(windows) != 264 or any(
            row['first_pose_error_m'] > 1e-9 for row in windows):
        raise ValueError('Expected 264 first-pose-anchored segments')
    for item in cohort['runs']:
        figure = ROOT / 'research/figures' / f'{item["label"]}_color_20hz.svg'
        if not figure.is_file() or figure.stat().st_size < 100:
            raise ValueError(f'Missing full-run figure: {figure}')
    cohort_md = (RESULTS / 'cohort_report.md').read_text()
    segment_md = (RESULTS / 'segment_report.md').read_text()
    # The protocol is already explained in Chinese immediately above this table.
    segment_md = segment_md[segment_md.index('| Bag |'):]
    note_path = ROOT / 'note.md'
    note = note_path.read_text()
    note = replace_section(note, 'COHORT', cohort_md)
    note = replace_section(note, 'SEGMENTS', segment_md)
    note_path.write_text(note)
    print(f'{note_path}: 18 bags, {len(windows)} first-pose-checked windows')


if __name__ == '__main__':
    main()
