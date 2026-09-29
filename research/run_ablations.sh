#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."

uid_gid="$(id -u):$(id -g)"
container() {
  docker run --rm --user "$uid_gid" -v "$PWD:/workspace/levio" \
    levio-research:py310 python "$@"
}

mkdir -p research_data research_results research/figures
container research/run_odom.py --bag research_data/run002.bag --camera color \
  --target-fps 20 --max-frames 300 --preserve-first-timestamp \
  --output research_results/run002_preserve_300 \
  --trace-output research_results/run002_preserve_300/runtime_match_trace.csv \
  > research_data/run002_preserve_300.log 2>&1
container research/run_odom.py --bag research_data/run002.bag --camera color \
  --target-fps 20 --max-frames 300 --general-initialization \
  --output research_results/run002_general_300 \
  --trace-output research_results/run002_general_300/runtime_match_trace.csv \
  > research_data/run002_general_300.log 2>&1
container research/run_odom.py --bag research_data/run007.bag --camera color \
  --target-fps 20 --disable-optimization --output research_results/run007_noopt \
  --trace-output research_results/run007_noopt/runtime_match_trace.csv \
  > research_data/run007_noopt.log 2>&1

# Same bag, raw IMU, model and 15 Hz selection; camera stream/calibration change.
for camera in color infra1; do
  output="research_results/run007_${camera}_15hz"
  container research/run_odom.py --bag research_data/run007.bag \
    --camera "$camera" --target-fps 15 --output "$output" \
    --trace-output "$output/runtime_match_trace.csv" \
    > "research_data/run007_${camera}_15hz.log" 2>&1
  container research/plot_results.py --run-dir "$output" \
    --output "research/figures/run007_${camera}_15hz.svg"
done

# Same first 600 RGB frames and raw IMU; isolate post-initialization graph updates.
for mode in default noopt; do
  options=()
  if [[ "$mode" == noopt ]]; then options=(--disable-optimization); fi
  output="research_results/run010_${mode}_600"
  container research/run_odom.py --bag research_data/run010.bag --camera color \
    --target-fps 20 --max-frames 600 "${options[@]}" --output "$output" \
    --trace-output "$output/runtime_match_trace.csv" \
    > "research_data/run010_${mode}_600.log" 2>&1
done
container research/compare_fixed_window.py --seconds 10 \
  research_results/run010_default_600 research_results/run010_noopt_600 \
  --output research_results/run010_opt_ablation_10s.json
prefix=research_results/run010_default_600/online_frames.tum
cmp -n "$(wc -c < "$prefix")" "$prefix" \
  research_results/run010_color_20hz/online_frames.tum
echo 'run010 online 600-frame prefix matches the full run'

# This alternate IMU topic is a negative control: the recorded duplicate
# timestamp is expected to make GTSAM reject a zero integration interval.
if container research/run_odom.py --bag research_data/run007.bag \
    --camera color --target-fps 20 --imu-topic /mavros/imu/data \
    --output research_results/run007_filtered_imu \
    > research_data/run007_filtered_imu.log 2>&1; then
  echo 'Unexpected success with duplicate-timestamp filtered IMU' >&2
  exit 1
fi
[[ "$(< research_results/run007_filtered_imu/failure.txt)" == *'dt <=0'* ]]
