#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."

mkdir -p research_results/upstream_source research_data
git archive 00d925f166bff859496fbda85049b2ce68bf7aa1 \
  levio_python_model | tar -x -C research_results/upstream_source
uid_gid="$(id -u):$(id -g)"
model='/workspace/levio/research_results/upstream_source/levio_python_model'
for count in 300 0; do
  if [[ "$count" -eq 0 ]]; then
    suffix=full
    frame_arg=()
    current=research_results/MH01_euroc_full
  else
    suffix=300
    frame_arg=(--max-frames "$count")
    current=research_results/MH01_research_300
    docker run --rm --user "$uid_gid" -v "$PWD:/workspace/levio" \
      levio-research:py310 python research/run_euroc.py \
      "${frame_arg[@]}" --output "$current" \
      > research_data/MH01_research_300.log 2>&1
  fi
  upstream="research_results/MH01_upstream_${suffix}"
  docker run --rm --user "$uid_gid" -v "$PWD:/workspace/levio" \
    -e "LEVIO_MODEL_DIR=$model" levio-research:py310 \
    python research/run_euroc.py "${frame_arg[@]}" --output "$upstream" \
    > "research_data/MH01_upstream_${suffix}.log" 2>&1
  for name in frames keyframes online_frames; do
    cmp "$current/$name.tum" "$upstream/$name.tum"
    sha256sum "$current/$name.tum"
  done
done

prefix=research_results/MH01_research_300/online_frames.tum
cmp -n "$(wc -c < "$prefix")" "$prefix" \
  research_results/MH01_euroc_full/online_frames.tum
echo 'EuRoC online 300-frame prefix matches the full run'
