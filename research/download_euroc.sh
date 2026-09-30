#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."
mkdir -p research_data
path='research_data/MH_01_easy.bag'
expected='57f440ccd68ec8dc8f9461269f5909656b86198bac3adfd677b1fcc7a1428fa9'
url='https://huggingface.co/datasets/kavehsgh/EuRoC_MAV_Dataset_Machine_Hall_Easy_01/resolve/19434bff2188ded1943d3a01d5b5e6672afb117e/MH_01_easy.bag'
gt_path='research_data/MH_01_easy_gt.csv'
gt_expected='aae2d7c5684724c8afe364d2fd8499a6ed77a9169adf03736531f67d8e7b2117'
gt_url='https://raw.githubusercontent.com/rpng/open_vins/69488123ed9362dd44b6f28e7f4680abbff1442b/ov_data/euroc_mav/MH_01_easy.csv'

download() {
  local destination="$1" bytes="$2" checksum="$3" source="$4" actual_size
  if [[ -f "$destination" ]]; then
    actual_size=$(stat -c '%s' "$destination")
    if [[ "$actual_size" == "$bytes" ]] &&
      printf '%s  %s\n' "$checksum" "$destination" | sha256sum -c - --status; then
      echo "$destination already verified"
      return
    fi
    # A short file can be resumed. A full-size corrupt file must be restarted.
    if (( actual_size >= bytes )); then
      rm -f -- "$destination"
    fi
  fi
  curl --location --fail --retry 5 --retry-all-errors --retry-delay 5 \
    --continue-at - --output "$destination" "$source"
  actual_size=$(stat -c '%s' "$destination")
  if [[ "$actual_size" != "$bytes" ]]; then
    echo "$destination size mismatch: expected $bytes bytes, got $actual_size" >&2
    return 1
  fi
  if ! printf '%s  %s\n' "$checksum" "$destination" | sha256sum -c -; then
    rm -f -- "$destination"
    echo "$destination checksum mismatch; rerun the downloader" >&2
    return 1
  fi
}

download "$path" 2673818914 "$expected" "$url"
download "$gt_path" 6225269 "$gt_expected" "$gt_url"
