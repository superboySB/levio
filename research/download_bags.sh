#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."
mkdir -p research_data
# The pinned session IDs, LFS byte sizes and SHA-256 hashes live in this repo.
manifest='research/metadata/bags.tsv'
revision='6ac394eef603e8efbfdc9d15f718e85835eee6bb'
base="https://huggingface.co/datasets/YangLiu1021/odom_dataset/resolve/$revision/raw/sessions"
if [[ "$(head -n 1 "$manifest")" != "# HF dataset revision: $revision" ]]; then
  echo "Manifest revision does not match the download URL" >&2
  exit 2
fi

download() {
  local label="$1" row session bytes expected selection sidecar path actual_size
  local before after attempt status
  row=$(awk -F '\t' -v wanted="$label" '$1 == wanted { print; exit }' "$manifest")
  if [[ -z "$row" ]]; then
    echo "Unknown session: $label" >&2
    return 2
  fi
  IFS=$'\t' read -r label session bytes expected selection <<< "$row"
  sidecar="research/metadata/$label/bag.bag.sha256"
  if [[ "$(awk '{ print $1 }' "$sidecar")" != "$expected" ]]; then
    echo "Manifest and SHA-256 sidecar disagree for $label" >&2
    return 2
  fi
  path="research_data/$label.bag"
  if [[ -f "$path" ]]; then
    actual_size=$(stat -c '%s' "$path")
    if [[ "$actual_size" == "$bytes" ]] &&
      printf '%s  %s\n' "$expected" "$path" | sha256sum -c - --status; then
      echo "$path already verified"
      return
    fi
    # Resume only short files; restarting a complete but corrupt file avoids
    # an endless checksum failure after curl resumes at its existing end.
    if (( actual_size >= bytes )); then
      rm -f -- "$path"
    fi
  fi
  # Retry outside curl: curl's internal retry can restart a resumed LFS file
  # and truncate gigabytes already received after a transient TLS failure.
  for attempt in {1..10}; do
    before=$(stat -c '%s' "$path" 2>/dev/null || echo 0)
    if curl --location --fail --retry 0 --continue-at - \
        --output "$path" "$base/$session/bag.bag"; then
      status=0
    else
      status=$?
    fi
    after=$(stat -c '%s' "$path" 2>/dev/null || echo 0)
    if (( after < before )); then
      echo "$path shrank from $before to $after bytes; aborting unsafe resume" >&2
      return 1
    fi
    if (( after > bytes )); then
      echo "$path exceeded pinned size $bytes bytes" >&2
      return 1
    fi
    if (( status == 0 && after == bytes )); then
      break
    fi
    if (( attempt == 10 )); then
      echo "$path remains incomplete after $attempt attempts (curl status $status)" >&2
      return 1
    fi
    sleep 5
  done
  actual_size=$(stat -c '%s' "$path")
  if [[ "$actual_size" != "$bytes" ]]; then
    echo "$path size mismatch: expected $bytes bytes, got $actual_size" >&2
    return 1
  fi
  if ! printf '%s  %s\n' "$expected" "$path" | sha256sum -c -; then
    rm -f -- "$path"
    echo "$path checksum mismatch; rerun the downloader" >&2
    return 1
  fi
}

# Optional bags remain available for exclusion auditing or candidate replacement.
if [[ $# -eq 0 ]]; then
  mapfile -t requested < <(awk -F '\t' '!/^#/ && $5 == "default" { print $1 }' "$manifest")
else
  requested=("$@")
fi
for label in "${requested[@]}"; do
  download "$label"
done
