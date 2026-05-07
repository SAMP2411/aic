#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUN_ID="${1:-best_gui_wslg_cpu_$(date +%Y%m%d_%H%M%S)}"
MAX_ATTEMPTS="${AIC_GUI_MAX_ATTEMPTS:-5}"

service docker start >/dev/null 2>&1 || true

export DISPLAY="${DISPLAY:-:0}"
export WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-wayland-0}"
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/0}"
xhost +local:root >/dev/null 2>&1 || true

export AIC_SKIP_BUILD="${AIC_SKIP_BUILD:-1}"
export AIC_SKIP_PULL="${AIC_SKIP_PULL:-1}"
export AIC_GUI_COMPOSE="${AIC_GUI_COMPOSE:-$ROOT_DIR/docker/docker-compose.submission-gui-wslg.yaml}"
export AIC_SUBMISSION_PROFILE="${AIC_SUBMISSION_PROFILE:-tf_smoother}"
export AIC_SCORING_TF_FINAL_OFFSET="${AIC_SCORING_TF_FINAL_OFFSET:--0.020}"
export AIC_SCORING_TF_HOLD_SECONDS="${AIC_SCORING_TF_HOLD_SECONDS:-8.0}"
export AIC_VISUAL_PUSH_SETTLE_SECONDS="${AIC_VISUAL_PUSH_SETTLE_SECONDS:-8.0}"
export AIC_SCORING_TF_COMPLETE_AFTER_HOLD="${AIC_SCORING_TF_COMPLETE_AFTER_HOLD:-1}"
export AIC_SCORING_TF_SEARCH_ON_MISS="${AIC_SCORING_TF_SEARCH_ON_MISS:-1}"
export AIC_ACT_MAX_WALL_SECONDS="${AIC_ACT_MAX_WALL_SECONDS:-900}"
export AIC_GUI_DETACHED_WAIT="${AIC_GUI_DETACHED_WAIT:-1}"
export AIC_GUI_WAIT_TIMEOUT_SECONDS="${AIC_GUI_WAIT_TIMEOUT_SECONDS:-1200}"
export AIC_GUI_DOWN_TIMEOUT_SECONDS="${AIC_GUI_DOWN_TIMEOUT_SECONDS:-20}"
export AIC_EVAL_COMMAND="${AIC_EVAL_COMMAND:-gazebo_gui:=true launch_rviz:=false ground_truth:=false start_aic_engine:=true shutdown_on_aic_engine_exit:=false aic_engine_config_file:=/aic_engine_config/local_smoke_trial_1.yaml model_discovery_timeout_seconds:=30 model_configure_timeout_seconds:=60}"

for attempt in $(seq 1 "$MAX_ATTEMPTS"); do
  if [[ "$attempt" == "1" ]]; then
    attempt_run_id="$RUN_ID"
  else
    attempt_run_id="${RUN_ID}_retry_${attempt}"
  fi

  echo "Starting visual WSLg CPU attempt $attempt/$MAX_ATTEMPTS: $attempt_run_id"
  docker compose \
    -f "$ROOT_DIR/docker/docker-compose.submission-cpu.yaml" \
    -f "$ROOT_DIR/docker/docker-compose.submission-gui-wslg.yaml" \
    down --remove-orphans >/dev/null 2>&1 || true

  if "$ROOT_DIR/scripts/run_submission_best_gui.sh" "$attempt_run_id"; then
    score_file="$ROOT_DIR/runs/$attempt_run_id/results/scoring.yaml"
    if [[ -f "$score_file" ]] && grep -q "Cable insertion successful" "$score_file"; then
      echo "Visual WSLg CPU run succeeded on attempt $attempt: $score_file"
      exit 0
    fi
    echo "Attempt $attempt wrote scoring.yaml but did not complete cable insertion; retrying." >&2
  else
    echo "Attempt $attempt failed before a successful score; retrying." >&2
  fi
done

echo "Visual WSLg CPU run failed after $MAX_ATTEMPTS attempts." >&2
exit 1
