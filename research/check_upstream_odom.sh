#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."

upstream_commit='00d925f166bff859496fbda85049b2ce68bf7aa1'
git cat-file -e "${upstream_commit}^{commit}"
source_dir='research_results/upstream_odom_source'
mkdir -p research_data research_results
rm -rf -- "$source_dir"
mkdir -p "$source_dir"
git archive "$upstream_commit" levio_python_model | tar -x -C "$source_dir"
model="/workspace/levio/$source_dir/levio_python_model"
uid_gid="$(id -u):$(id -g)"

# One initialized run and one low-match run exercise the odom adapter with the
# original model, identical RGB decoding, calibration and IMU.
for label in run005 run007; do
  current="research_results/${label}_color_20hz"
  test -s "$current/online_frames.tum"
  upstream="research_results/${label}_upstream_control"
  if docker run --rm --user "$uid_gid" -v "$PWD:/workspace/levio" \
      -e "LEVIO_MODEL_DIR=$model" levio-research:py310 \
      python research/run_odom.py --bag "research_data/$label.bag" \
        --camera color --target-fps 20 --output "$upstream" \
        --trace-output "$upstream/runtime_match_trace.csv" \
      > "research_data/${label}_upstream_control.log" 2>&1; then
    status=0
  else
    status=$?
  fi
  if [[ "$label" == run005 ]]; then
    # main passes an empty graph-descriptor array into OpenCV at frame 201.
    [[ "$status" -eq 1 && -s "$upstream/failure.txt" ]]
    grep -Fq '_queryDescriptors.type() == trainDescType' \
      "$upstream/failure.txt"
    prefix="$upstream/online_frames.tum"
    [[ "$(wc -l < "$prefix")" -gt 50 ]]
    cmp -n "$(wc -c < "$prefix")" "$prefix" \
      "$current/online_frames.tum"
    echo "run005: upstream main fails after $(wc -l < "$prefix") online frames; research prefix is identical"
  else
    [[ "$status" -eq 0 ]]
    for name in frames keyframes online_frames; do
      cmp "$current/$name.tum" "$upstream/$name.tum"
      sha256sum "$current/$name.tum"
    done
  fi
done
echo 'run007 online and final poses match upstream main byte-for-byte'
