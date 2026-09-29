#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."
mkdir -p research_data
# Pin the Hugging Face dataset revision; per-bag SHA-256 is checked below.
base='https://huggingface.co/datasets/YangLiu1021/odom_dataset/resolve/6ac394eef603e8efbfdc9d15f718e85835eee6bb/raw/sessions'

download() {
  local label="$1" session="$2" expected="$3" path="research_data/$1.bag"
  if [[ -f "$path" ]] && printf '%s  %s\n' "$expected" "$path" | sha256sum -c - --status; then
    echo "$path already verified"
    return
  fi
  curl -L --fail --retry 3 --continue-at - \
    --output "$path" "$base/$session/bag.bag"
  printf '%s  %s\n' "$expected" "$path" | sha256sum -c -
}

if [[ $# -eq 0 ]]; then set -- run009 run003 run005 run007 run002 run010; fi
for label in "$@"; do
  case "$label" in
    run009) download run009 2026-05-14_002421_odom_run009 5e7ced6531f33df2ae13834fd2c3ac9dcadc6c573735085d317f7519642d8a3b ;;
    run007) download run007 2026-05-13_033707_odom_run007 dbc1278f00f9a098b65a7aeb12406174a89281758f0584e86f18caeb5d6b6027 ;;
    run003) download run003 2026-05-13_025750_odom_run003 d52dd44342b9418d9d41ae8e3c69aba3a7df3de10d6c95cf3dc85957f94ddf47 ;;
    run005) download run005 2026-05-13_031034_odom_run005 b663c05b97cce30a3176586e63a0eaa4bf13abd929a2bbfce065d11db4aad60e ;;
    run023) download run023 2026-05-15_034107_odom_run023 e0b20cb382fc310e7af25f36109e1f1f336f836a6bdc1689466fe32edec268d0 ;;
    run002) download run002 2026-05-13_025426_odom_run002 208402bbdd27aca8707b2e13e30a4c305f75803d6a98eaa92e768aea6ffd0182 ;;
    run010) download run010 2026-05-14_010514_odom_run010 9ce712312cc0d84c56e65d9d7a9030f7f9228e1331ac22ff3bb6a24ca76901ac ;;
    *) echo "Unknown session: $label" >&2; exit 2 ;;
  esac
done
