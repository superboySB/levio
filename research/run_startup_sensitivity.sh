#!/usr/bin/env bash
# Replay an opt-in first-keyframe timestamp control on existing, audited bags.
set -euo pipefail
cd "$(dirname "$0")/.."

uid_gid="$(id -u):$(id -g)"
container() {
  docker run --rm --user "$uid_gid" -v "$PWD:/workspace/levio" \
    levio-research:py310 python "$@"
}

mkdir -p research_results/adaptation research/figures research_data
for label in run002 run005 run007 run010 run034; do
  test -f "research_data/$label.bag"
  test -f "research_results/${label}_color_20hz/summary.json"
  output="research_results/adaptation/${label}_preserve_full"
  container research/run_odom.py --bag "research_data/$label.bag" \
    --camera color --target-fps 20 --preserve-first-timestamp \
    --output "$output" --trace-output "$output/runtime_match_trace.csv" \
    > "research_data/${label}_preserve_full.log" 2>&1
  container research/compare_input_variants.py \
    --baseline "research_results/${label}_color_20hz" \
    --variant "$output" \
    --output "research_results/adaptation/${label}_preserve_common.json"
  container research/plot_results.py --run-dir "$output" \
    --output "research/figures/${label}_preserve_full.svg"
done
