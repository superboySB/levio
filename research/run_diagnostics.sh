#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."

uid_gid="$(id -u):$(id -g)"
container() {
  docker run --rm --user "$uid_gid" -v "$PWD:/workspace/levio" \
    levio-research:py310 python "$@"
}

mkdir -p research_data research_results/frontend research_results/imu
research/run_all.sh run023

# Whole-record adjacent-image audit: 20 Hz selection and identical feature code.
for label in run002 run003 run005 run007 run009 run010 run023; do
  container research/diagnose_frontend.py --dataset odom \
    --bag "research_data/$label.bag" --camera color \
    --label "${label}_full" --target-fps 20 --max-seconds 0 \
    --snapshot-seconds > "research_data/${label}_frontend_full.log" 2>&1
done
container research/check_essential_threshold.py \
  --output research_results/frontend/run005_first_E_failure.json \
  --figure-prefix research/figures/run005_first_E \
  > research_data/run005_first_E_failure.log 2>&1
container research/check_essential_threshold.py \
  --bag research_data/run003.bag --keyframe 704 --frame 705 \
  --output research_results/frontend/run003_first_E_failure.json \
  > research_data/run003_first_E_failure.log 2>&1

# Equal 35 s windows for a controlled front-end comparison.
for label in run005 run007; do
  container research/diagnose_frontend.py --dataset odom \
    --bag "research_data/$label.bag" --camera color \
    --label "${label}_rgb" --target-fps 20 --max-seconds 35 \
    --snapshot-seconds 10 30 > "research_data/${label}_frontend_35.log" 2>&1
done
container research/diagnose_frontend.py --dataset euroc \
  --bag research_data/MH_01_easy.bag --label MH01_cam0 \
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

for label in run002 run003 run005 run007 run009 run010; do
  container research/diagnose_init_time.py --bag "research_data/$label.bag" \
    --run-dir "research_results/${label}_color_20hz" \
    --output "research_results/imu/${label}_init_time.json" \
    > "research_data/${label}_init_time.log" 2>&1
done

container research/summarize_trace.py \
  research_results/run*_color_20hz/runtime_match_trace.csv \
  research_results/MH01_euroc_full/runtime_match_trace.csv \
  --output research_results/runtime_trace_summary.json
container research/compare_fixed_window.py --seconds 10 \
  research_results/MH01_euroc_full research_results/run003_color_20hz \
  research_results/run007_color_20hz research_results/run010_color_20hz \
  --output research_results/first_10s_after_init.json
