#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

DATASET_REPO="${AIC_LEROBOT_DATASET_REPO:-your-hf-user/your_dataset}"
POLICY_TYPE="${AIC_LEROBOT_POLICY_TYPE:-act}"
OUTPUT_DIR="${AIC_LEROBOT_OUTPUT_DIR:-outputs/train/${POLICY_TYPE}_aic}"
JOB_NAME="${AIC_LEROBOT_JOB_NAME:-${POLICY_TYPE}_aic}"
DEVICE="${AIC_LEROBOT_DEVICE:-cuda}"
POLICY_REPO="${AIC_LEROBOT_POLICY_REPO:-your-hf-user/aic_${POLICY_TYPE}_policy}"

exec pixi run lerobot-train \
  --dataset.repo_id="$DATASET_REPO" \
  --policy.type="$POLICY_TYPE" \
  --output_dir="$OUTPUT_DIR" \
  --job_name="$JOB_NAME" \
  --policy.device="$DEVICE" \
  --wandb.enable="${AIC_LEROBOT_WANDB_ENABLE:-false}" \
  --policy.repo_id="$POLICY_REPO"
