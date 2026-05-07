#!/usr/bin/env bash
set -euo pipefail

if ! command -v isaaclab >/dev/null 2>&1; then
  echo "isaaclab command not found. Run this inside the Isaac Lab container." >&2
  exit 1
fi

TASK="${AIC_ISAAC_TASK:-AIC-Task-v0}"
NUM_ENVS="${AIC_ISAAC_NUM_ENVS:-64}"

exec isaaclab -p aic/aic_utils/aic_isaac/aic_isaaclab/scripts/rsl_rl/train.py \
  --task "$TASK" \
  --num_envs "$NUM_ENVS" \
  --enable_cameras
