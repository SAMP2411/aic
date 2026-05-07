#!/usr/bin/env bash
set -euo pipefail

if ! command -v nvidia-smi >/dev/null 2>&1; then
  echo "Host NVIDIA driver is not available."
  exit 1
fi

echo "Host GPU:"
nvidia-smi --query-gpu=name,driver_version --format=csv,noheader
echo

echo "Docker GPU probe:"
if docker run --rm --gpus all nvidia/cuda:12.2.0-base-ubuntu22.04 nvidia-smi >/tmp/aic_nvidia_probe.out 2>&1; then
  cat /tmp/aic_nvidia_probe.out
  echo
  echo "NVIDIA container support is working."
  exit 0
fi

cat /tmp/aic_nvidia_probe.out
echo
echo "NVIDIA container support is NOT working yet."
echo "You likely need the NVIDIA container toolkit configured for Docker."
exit 1
