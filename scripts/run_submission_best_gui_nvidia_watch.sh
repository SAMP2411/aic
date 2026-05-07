#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUN_ID="${1:-best_gui_nvidia_watch_$(date +%Y%m%d_%H%M%S)}"

export AIC_EVAL_COMMAND="${AIC_EVAL_COMMAND:-gazebo_gui:=true launch_rviz:=false ground_truth:=false start_aic_engine:=true shutdown_on_aic_engine_exit:=false aic_engine_config_file:=/aic_engine_config/local_smoke_trial_1.yaml model_discovery_timeout_seconds:=30 model_configure_timeout_seconds:=60}"

exec "$ROOT_DIR/scripts/run_submission_best_gui_nvidia.sh" "$RUN_ID"
