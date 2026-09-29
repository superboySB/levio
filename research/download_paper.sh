#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."
mkdir -p research_data
url='https://arxiv.org/pdf/2602.03294v1'
path='research_data/levio_2602.03294v1.pdf'
expected='8d2232f1dde46327eb33e13a19b7e30c1b9099a1f791e85b5e35428597d163f6'

if [[ -f "$path" ]] && printf '%s  %s\n' "$expected" "$path" | sha256sum -c - --status; then
  echo "$path already verified"
  exit 0
fi

curl -L --fail --retry 3 --output "$path" "$url"
printf '%s  %s\n' "$expected" "$path" | sha256sum -c -
