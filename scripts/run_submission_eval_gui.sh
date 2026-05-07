#!/usr/bin/env bash
set -euo pipefail

if [[ -z "${DISPLAY:-}" ]]; then
  echo "DISPLAY is not set. Start an X11 session first." >&2
  exit 1
fi

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUN_ID="${1:-$(date +%Y%m%d_%H%M%S)_gui}"
RUN_DIR="$ROOT_DIR/runs/$RUN_ID"
RESULTS_DIR="$RUN_DIR/results"
BASE_COMPOSE="$ROOT_DIR/docker/docker-compose.submission-cpu.yaml"
GUI_COMPOSE_DEFAULT="$ROOT_DIR/docker/docker-compose.submission-gui.yaml"
GUI_COMPOSE="${AIC_GUI_COMPOSE:-$GUI_COMPOSE_DEFAULT}"
GUI_EXTRA_COMPOSE="${AIC_GUI_EXTRA_COMPOSE:-}"
TORCH_CACHE_DIR="$ROOT_DIR/.cache/torch-cache"
XAUTH_RUN_FILE="$RUN_DIR/.docker.xauth"
compose_args=()

mkdir -p "$RESULTS_DIR"
cd "$ROOT_DIR"

export DOCKER_BUILDKIT=1
export COMPOSE_DOCKER_CLI_BUILD=1

cleanup() {
  if [[ ${#compose_args[@]} -gt 0 ]]; then
    docker compose "${compose_args[@]}" down --remove-orphans >/dev/null 2>&1 || true
  fi
  rm -f "$XAUTH_RUN_FILE"
}
trap cleanup EXIT

if ! command -v xdpyinfo >/dev/null 2>&1 || ! xdpyinfo >/dev/null 2>&1; then
  echo "DISPLAY=$DISPLAY is not accessible from this shell. Open the same X11 session first." >&2
  exit 1
fi

if ! command -v xauth >/dev/null 2>&1; then
  echo "xauth is required for the GUI container to authenticate to your X server." >&2
  exit 1
fi

SOURCE_XAUTHORITY="${XAUTHORITY:-}"
if [[ -z "$SOURCE_XAUTHORITY" || ! -f "$SOURCE_XAUTHORITY" ]]; then
  SOURCE_XAUTHORITY="$(xauth info 2>/dev/null | awk -F': *' '/Authority file/ {print $2; exit}')"
fi

touch "$XAUTH_RUN_FILE"
chmod 600 "$XAUTH_RUN_FILE"
if [[ -n "$SOURCE_XAUTHORITY" && -f "$SOURCE_XAUTHORITY" ]]; then
  if ! xauth -f "$SOURCE_XAUTHORITY" nlist "$DISPLAY" \
    | sed -e 's/^..../ffff/' \
    | xauth -f "$XAUTH_RUN_FILE" nmerge - >/dev/null 2>&1; then
    cp "$SOURCE_XAUTHORITY" "$XAUTH_RUN_FILE"
  fi
  if [[ ! -s "$XAUTH_RUN_FILE" ]]; then
    cp "$SOURCE_XAUTHORITY" "$XAUTH_RUN_FILE"
  fi
else
  echo "No readable Xauthority file found; relying on xhost access for DISPLAY=$DISPLAY." >&2
fi

export AIC_XAUTHORITY_HOST_FILE="$XAUTH_RUN_FILE"

find_preferred_dri_pair() {
  local preferred_driver="${1:-i915}"
  local render_sys
  local fallback_card=""
  local fallback_render=""

  for render_sys in /sys/class/drm/renderD*; do
    [[ -e "$render_sys" ]] || continue

    local driver_path driver_name card_name candidate
    driver_path="$(readlink -f "$render_sys/device/driver" 2>/dev/null || true)"
    driver_name="$(basename "$driver_path")"
    card_name=""

    for candidate in "$render_sys"/device/drm/card*; do
      [[ -e "$candidate" ]] || continue
      candidate="$(basename "$candidate")"
      case "$candidate" in
        card[0-9]*)
          card_name="$candidate"
          break
          ;;
      esac
    done

    [[ -n "$card_name" ]] || continue

    if [[ -z "$fallback_card" ]]; then
      fallback_card="/dev/dri/$card_name"
      fallback_render="/dev/dri/$(basename "$render_sys")"
    fi

    if [[ "$driver_name" == "$preferred_driver" ]]; then
      printf '/dev/dri/%s\n/dev/dri/%s\n' "$card_name" "$(basename "$render_sys")"
      return 0
    fi
  done

  if [[ -n "$fallback_card" && -n "$fallback_render" ]]; then
    printf '%s\n%s\n' "$fallback_card" "$fallback_render"
    return 0
  fi

  return 1
}

mapfile -t _dri_pair < <(find_preferred_dri_pair "${AIC_EVAL_DRI_DRIVER_HINT:-i915}" || true)
if [[ ${#_dri_pair[@]} -lt 2 ]]; then
  if [[ -e /dev/dxg || -d /mnt/wslg ]]; then
    GUI_COMPOSE="${AIC_GUI_COMPOSE:-$ROOT_DIR/docker/docker-compose.submission-gui-wslg.yaml}"
    echo "No /dev/dri render node found; using WSLg GUI compose: $GUI_COMPOSE" >&2
  else
    echo "Could not resolve a preferred /dev/dri card/render pair for GUI acceleration." >&2
    exit 1
  fi
else
  export AIC_DRI_CARD_HOST_FILE="${_dri_pair[0]}"
  export AIC_DRI_RENDER_HOST_FILE="${_dri_pair[1]}"
fi

compose_args=(-f "$BASE_COMPOSE" -f "$GUI_COMPOSE")
if [[ -n "$GUI_EXTRA_COMPOSE" ]]; then
  compose_args+=(-f "$GUI_EXTRA_COMPOSE")
fi

printf 'run_id: %s\nrun_dir: %s\nresults_dir: %s\n' "$RUN_ID" "$RUN_DIR" "$RESULTS_DIR" \
  | tee "$RUN_DIR/summary.txt"

if [[ "${AIC_SKIP_PULL:-0}" == "1" ]] || \
   [[ "${AIC_SKIP_BUILD:-0}" == "1" ]] && docker image inspect ghcr.io/intrinsic-dev/aic/aic_eval:latest >/dev/null 2>&1; then
  echo "Using local ghcr.io/intrinsic-dev/aic/aic_eval:latest image; skipping pull." | tee "$RUN_DIR/pull.log"
else
  docker pull ghcr.io/intrinsic-dev/aic/aic_eval:latest 2>&1 | tee "$RUN_DIR/pull.log"
fi

AIC_RESULTS_HOST_DIR="$RESULTS_DIR" \
  docker compose "${compose_args[@]}" config >"$RUN_DIR/compose.rendered.yaml"

./scripts/prewarm_submission_cache.sh "$TORCH_CACHE_DIR" 2>&1 \
  | tee "$RUN_DIR/prewarm.log"

if [[ "${AIC_SKIP_BUILD:-0}" == "1" || "${AIC_SKIP_EVAL_BUILD:-0}" == "1" ]]; then
  echo "Skipping eval image build because AIC_SKIP_BUILD=1 or AIC_SKIP_EVAL_BUILD=1" | tee "$RUN_DIR/eval-build.log"
else
  AIC_RESULTS_HOST_DIR="$RESULTS_DIR" \
  AIC_TORCH_CACHE_HOST_DIR="$TORCH_CACHE_DIR" \
    docker compose "${compose_args[@]}" build eval 2>&1 | tee "$RUN_DIR/eval-build.log"
fi

if [[ "${AIC_SKIP_BUILD:-0}" != "1" ]]; then
  AIC_RESULTS_HOST_DIR="$RESULTS_DIR" \
  AIC_TORCH_CACHE_HOST_DIR="$TORCH_CACHE_DIR" \
    docker compose "${compose_args[@]}" build model 2>&1 | tee "$RUN_DIR/build.log"
else
  echo "Skipping image build because AIC_SKIP_BUILD=1" | tee "$RUN_DIR/build.log"
fi

docker image inspect my-solution:submission-act >"$RUN_DIR/model-image.inspect.json"

run_compose_detached_until_score() {
  local wait_timeout="${AIC_GUI_WAIT_TIMEOUT_SECONDS:-1200}"
  local down_timeout="${AIC_GUI_DOWN_TIMEOUT_SECONDS:-20}"
  local success_grace="${AIC_GUI_SUCCESS_GRACE_SECONDS:-3}"
  local start_ts now_ts status log_pid empty_running_checks

  AIC_RESULTS_HOST_DIR="$RESULTS_DIR" \
    AIC_TORCH_CACHE_HOST_DIR="$TORCH_CACHE_DIR" \
    docker compose "${compose_args[@]}" up -d 2>&1 \
    | tee "$RUN_DIR/compose.start.log"

  docker compose "${compose_args[@]}" logs -f --no-color >"$RUN_DIR/compose.log" 2>&1 &
  log_pid=$!
  start_ts="$(date +%s)"
  status="timeout"
  empty_running_checks=0

  while true; do
    if [[ -f "$RESULTS_DIR/scoring.yaml" ]]; then
      status="scored"
      sleep "$success_grace"
      break
    fi

    if ! docker compose "${compose_args[@]}" ps --status running -q | grep -q .; then
      if docker ps \
        --filter "name=aic-submission-cpu-" \
        --filter "status=running" \
        -q | grep -q .; then
        empty_running_checks=0
      else
        empty_running_checks=$((empty_running_checks + 1))
        if (( empty_running_checks >= 3 )); then
          status="containers_exited_before_score"
          break
        fi
      fi
    else
      empty_running_checks=0
    fi

    now_ts="$(date +%s)"
    if (( now_ts - start_ts >= wait_timeout )); then
      status="timed_out_waiting_for_score"
      break
    fi

    sleep 2
  done

  kill "$log_pid" >/dev/null 2>&1 || true
  wait "$log_pid" >/dev/null 2>&1 || true

  echo "detached_status=$status" | tee -a "$RUN_DIR/summary.txt"
  docker compose "${compose_args[@]}" ps -a >"$RUN_DIR/compose.ps-before-down.log" 2>&1 || true
  docker compose "${compose_args[@]}" down --timeout "$down_timeout" --remove-orphans \
    >"$RUN_DIR/down.log" 2>&1 || {
      echo "docker compose down failed; forcing container cleanup." | tee -a "$RUN_DIR/summary.txt"
      docker compose "${compose_args[@]}" rm -sf >"$RUN_DIR/down.force.log" 2>&1 || true
    }

  [[ "$status" == "scored" ]]
}

if [[ "${AIC_GUI_DETACHED_WAIT:-0}" == "1" ]]; then
  run_compose_detached_until_score
else
  AIC_RESULTS_HOST_DIR="$RESULTS_DIR" \
    AIC_TORCH_CACHE_HOST_DIR="$TORCH_CACHE_DIR" \
    docker compose "${compose_args[@]}" up --abort-on-container-exit 2>&1 \
    | tee "$RUN_DIR/compose.log"
fi

if [[ -f "$RESULTS_DIR/scoring.yaml" ]]; then
  echo "scoring_file=$RESULTS_DIR/scoring.yaml" | tee -a "$RUN_DIR/summary.txt"
  echo "Run completed and scoring.yaml was written." | tee -a "$RUN_DIR/summary.txt"
  echo "If warnings appeared after 'Finished scoring trial', they are simulator teardown noise and do not invalidate the score." \
    | tee -a "$RUN_DIR/summary.txt"
  ./scripts/print_score_summary.py "$RESULTS_DIR/scoring.yaml" | tee -a "$RUN_DIR/summary.txt"
else
  echo "scoring_file_missing=$RESULTS_DIR/scoring.yaml" | tee -a "$RUN_DIR/summary.txt"
  exit 1
fi
