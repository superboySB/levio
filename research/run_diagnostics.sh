#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."

uid_gid="$(id -u):$(id -g)"
container() {
  docker run --rm --user "$uid_gid" -v "$PWD:/workspace/levio" \
    levio-research:py310 python "$@"
}

mkdir -p research_data research_results/frontend research_results/imu
mapfile -t default_runs < <(awk -F '\t' '!/^#/ && $5 == "default" { print $1 }' \
  research/metadata/bags.tsv)
trace_files=()
for label in "${default_runs[@]}"; do
  capture_times="research_results/${label}_color_20hz/capture_times.txt"
  trace="research_results/${label}_color_20hz/runtime_match_trace.csv"
  if [[ ! -s "$capture_times" || ! -s "$trace" ]]; then
    echo "Missing VIO output for $label; run research/run_all.sh before diagnostics" >&2
    exit 1
  fi
  trace_files+=("$trace")
done
if [[ ! -s research_results/MH01_euroc_full/capture_times.txt ||
      ! -s research_results/MH01_euroc_full/runtime_match_trace.csv ]]; then
  echo 'Missing EuRoC output; run the full traced EuRoC control before diagnostics' >&2
  exit 1
fi

# Whole-record adjacent-image audit uses the exact images consumed by VIO.
for label in "${default_runs[@]}"; do
  container research/diagnose_frontend.py --dataset odom \
    --bag "research_data/$label.bag" --camera color \
    --capture-times "research_results/${label}_color_20hz/capture_times.txt" \
    --label "${label}_full" --target-fps 20 --max-seconds 0 \
    --snapshot-seconds > "research_data/${label}_frontend_full.log" 2>&1
done
container research/check_essential_threshold.py \
  --run-dir research_results/run005_color_20hz --frame 227 \
  --output research_results/frontend/run005_low_match.json \
  --figure-prefix research/figures/run005_low_match \
  > research_data/run005_low_match.log 2>&1
container research/check_essential_threshold.py \
  --run-dir research_results/run004_color_20hz --frame 1088 \
  --output research_results/frontend/run004_low_match.json \
  --figure-prefix research/figures/run004_low_match \
  > research_data/run004_low_match.log 2>&1
container research/check_essential_threshold.py \
  --run-dir research_results/run003_color_20hz \
  --output research_results/frontend/run003_first_E_failure.json \
  --figure-prefix research/figures/run003_first_E \
  > research_data/run003_first_E_failure.log 2>&1
container research/check_essential_threshold.py \
  --run-dir research_results/run016_color_20hz \
  --output research_results/frontend/run016_first_E_failure.json \
  --figure-prefix research/figures/run016_first_E \
  > research_data/run016_first_E_failure.log 2>&1
container research/check_essential_threshold.py \
  --run-dir research_results/run021_color_20hz \
  --output research_results/frontend/run021_first_E_failure.json \
  --figure-prefix research/figures/run021_first_E_failure \
  > research_data/run021_first_E_failure.log 2>&1
container research/check_essential_threshold.py \
  --run-dir research_results/run030_color_20hz \
  --output research_results/frontend/run030_first_E.json \
  --figure-prefix research/figures/run030_first_E \
  > research_data/run030_first_E.log 2>&1
container research/check_essential_threshold.py \
  --run-dir research_results/run032_color_20hz \
  --output research_results/frontend/run032_first_E_failure.json \
  --figure-prefix research/figures/run032_first_E_failure \
  > research_data/run032_first_E_failure.log 2>&1
container research/check_essential_threshold.py \
  --run-dir research_results/run032_color_20hz --frame 2289 \
  --output research_results/frontend/run032_large_step_pair.json \
  --figure-prefix research/figures/run032_large_step_pair \
  > research_data/run032_large_step_pair.log 2>&1
container research/check_essential_threshold.py \
  --run-dir research_results/run036_color_20hz \
  --output research_results/frontend/run036_first_E.json \
  --figure-prefix research/figures/run036_first_E \
  > research_data/run036_first_E.log 2>&1

# Equal 35 s windows for a controlled front-end comparison.
for label in run005 run007; do
  container research/diagnose_frontend.py --dataset odom \
    --bag "research_data/$label.bag" --camera color \
    --capture-times "research_results/${label}_color_20hz/capture_times.txt" \
    --label "${label}_rgb" --target-fps 20 --max-seconds 35 \
    --snapshot-seconds 10 30 > "research_data/${label}_frontend_35.log" 2>&1
done
container research/diagnose_frontend.py --dataset euroc \
  --bag research_data/MH_01_easy.bag --label MH01_cam0 \
  --capture-times research_results/MH01_euroc_full/capture_times.txt \
  --target-fps 20 --max-seconds 35 --snapshot-seconds 10 30 \
  > research_data/MH01_frontend_35.log 2>&1

for label in run005 run007; do
  container research/diagnose_imu.py --dataset odom \
    --bag "research_data/$label.bag" --imu-topic /mavros/imu/data_raw \
    --max-seconds 35 --output "research_results/imu/${label}_raw.json" \
    > "research_data/${label}_imu_raw.log" 2>&1
done
container research/diagnose_imu.py --dataset odom --bag research_data/run007.bag \
  --imu-topic /mavros/imu/data --max-seconds 35 \
  --output research_results/imu/run007_filtered.json \
  > research_data/run007_imu_filtered.log 2>&1
container research/diagnose_imu.py --dataset euroc \
  --bag research_data/MH_01_easy.bag --imu-topic /imu0 --max-seconds 35 \
  --output research_results/imu/MH01_raw.json \
  > research_data/MH01_imu_raw.log 2>&1

for label in "${default_runs[@]}"; do
  container research/diagnose_init_time.py --bag "research_data/$label.bag" \
    --run-dir "research_results/${label}_color_20hz" \
    --output "research_results/imu/${label}_init_time.json" \
    > "research_data/${label}_init_time.log" 2>&1
done

container research/summarize_trace.py \
  "${trace_files[@]}" \
  research_results/MH01_euroc_full/runtime_match_trace.csv \
  --output research_results/runtime_trace_summary.json
run_dirs=(research_results/MH01_euroc_full)
for label in "${default_runs[@]}"; do
  run_dirs+=("research_results/${label}_color_20hz")
done
container research/compare_fixed_window.py --seconds 10 \
  "${run_dirs[@]}" --output research_results/first_10s_after_init.json
