#!/usr/bin/env bash
# Opt-in, input-only controls using previously downloaded and audited bags.
set -euo pipefail
cd "$(dirname "$0")/.."

uid_gid="$(id -u):$(id -g)"
container() {
  docker run --rm --user "$uid_gid" -v "$PWD:/workspace/levio" \
    levio-research:py310 python "$@"
}
for label in run005 run007 run021 run032; do
  test -f "research_data/$label.bag"
  test -f "research_results/${label}_color_20hz/summary.json"
done
test -f research_data/MH_01_easy.bag
test -f research_results/MH01_euroc_full/summary.json
mkdir -p research_data research_results/frontend research/figures

for label in run005 run007 run021; do
  output="research_results/${label}_clahe_20hz"
  container research/run_odom.py --bag "research_data/${label}.bag" \
    --camera color --target-fps 20 --clahe --output "$output" \
    --trace-output "$output/runtime_match_trace.csv" \
    > "research_data/${label}_clahe_20hz.log" 2>&1
  figure_option=()
  if [[ "$label" == run007 ]]; then
    figure_option=(--figure research/figures/run007_clahe_common_window.svg)
  fi
  container research/compare_input_variants.py \
    --baseline "research_results/${label}_color_20hz" --variant "$output" \
    --output "research_results/${label}_clahe_common.json" \
    "${figure_option[@]}"
done

# The 15 ms bounds leave every run007 selected image inside its raw IMU span;
# reference matching retains the unshifted camera capture stamps.
for shift in minus15 plus15; do
  if [[ "$shift" == minus15 ]]; then delta=-15; else delta=15; fi
  output="research_results/run007_offset_${shift}ms"
  container research/run_odom.py --bag research_data/run007.bag --camera color \
    --target-fps 20 --camera-imu-offset-ms="$delta" --output "$output" \
    --trace-output "$output/runtime_match_trace.csv" \
    > "research_data/run007_offset_${shift}ms.log" 2>&1
  container research/compare_input_variants.py \
    --baseline research_results/run007_color_20hz --variant "$output" \
    --output "research_results/run007_offset_${shift}ms_common.json"
done

for pair in \
  'euroc MH01_euroc_full 200 MH01' \
  'odom run005_color_20hz 227 run005' \
  'odom run007_color_20hz 200 run007' \
  'odom run021_color_20hz 2110 run021' \
  'odom run032_color_20hz 6403 run032'; do
  read -r dataset run frame label <<< "$pair"
  container research/check_clahe_frames.py --dataset "$dataset" \
    --run-dir "research_results/$run" --frame "$frame" \
    --output "research_results/frontend/${label}_clahe_frame${frame}.json"
done
