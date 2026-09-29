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

if [[ -f "$path" ]] && printf '%s  %s\n' "$expected" "$path" | sha256sum -c - --status; then
  echo "$path already verified"
else
  curl -L --fail --retry 3 --output "$path" "$url"
  printf '%s  %s\n' "$expected" "$path" | sha256sum -c -
fi
if [[ -f "$gt_path" ]] && printf '%s  %s\n' "$gt_expected" "$gt_path" | sha256sum -c - --status; then
  echo "$gt_path already verified"
else
  curl -L --fail --retry 3 --output "$gt_path" "$gt_url"
  printf '%s  %s\n' "$gt_expected" "$gt_path" | sha256sum -c -
fi
