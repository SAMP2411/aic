#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUN_ID="${1:-smoke_single_trial}"

export AIC_SKIP_BUILD="${AIC_SKIP_BUILD:-1}"
export AIC_ACT_MAX_TASK_SECONDS="${AIC_ACT_MAX_TASK_SECONDS:-2}"
export AIC_ACT_LOOP_HZ="${AIC_ACT_LOOP_HZ:-1}"
export AIC_EVAL_COMMAND="${AIC_EVAL_COMMAND:-gazebo_gui:=false launch_rviz:=false ground_truth:=false start_aic_engine:=true shutdown_on_aic_engine_exit:=true aic_engine_config_file:=/aic_engine_config/local_smoke_trial_1.yaml model_discovery_timeout_seconds:=30 model_configure_timeout_seconds:=60}"

exec "$ROOT_DIR/scripts/run_submission_eval.sh" "$RUN_ID"
