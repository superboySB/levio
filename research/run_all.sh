#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."
mkdir -p research_data research_results research_results/rgb_audit research/figures
image='levio-research:py310'
uid_gid="$(id -u):$(id -g)"

if [[ $# -eq 0 ]]; then
  mapfile -t default_runs < <(awk -F '\t' '!/^#/ && $5 == "default" { print $1 }' \
    research/metadata/bags.tsv)
  set -- "${default_runs[@]}"
fi
for label in "$@"; do
  if ! awk -F '\t' -v wanted="$label" '$1 == wanted { found = 1 } END { exit !found }' \
      research/metadata/bags.tsv; then
    echo "Unknown run: $label" >&2
    exit 2
  fi
  bag="research_data/$label.bag"
  output="research_results/${label}_color_20hz"
  test -f "$bag" || { echo "Missing $bag; run research/download_bags.sh" >&2; exit 1; }
  if docker run --rm --user "$uid_gid" -v "$PWD:/workspace/levio" "$image" \
      python research/audit_rgb.py --bag "$bag" \
      --output "research_results/rgb_audit/${label}.json" --require-vio-ready \
      > "research_data/${label}_rgb_audit.log" 2>&1; then
    :
  else
    status=$?
    if [[ "$status" -eq 2 ]]; then
      echo "Excluded $label: incomplete sensor/reference timeline; see research_results/rgb_audit/${label}.json" >&2
      exit 2
    fi
    cat "research_data/${label}_rgb_audit.log" >&2
    exit "$status"
  fi
  docker run --rm --user "$uid_gid" -v "$PWD:/workspace/levio" "$image" \
    python research/run_odom.py --bag "$bag" --camera color \
      --target-fps 20 --output "$output" \
      --trace-output "$output/runtime_match_trace.csv" \
    > "research_data/${label}_color_20hz.log" 2>&1
  docker run --rm --user "$uid_gid" -v "$PWD:/workspace/levio" "$image" \
    python research/plot_results.py --run-dir "$output" \
      --output "research/figures/${label}_color_20hz.svg"
done
