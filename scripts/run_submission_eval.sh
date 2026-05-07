#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUN_ID="${1:-$(date +%Y%m%d_%H%M%S)}"
RUN_DIR="$ROOT_DIR/runs/$RUN_ID"
RESULTS_DIR="$RUN_DIR/results"
COMPOSE_FILE="$ROOT_DIR/docker/docker-compose.submission-cpu.yaml"
TORCH_CACHE_DIR="$ROOT_DIR/.cache/torch-cache"

mkdir -p "$RESULTS_DIR"
cd "$ROOT_DIR"

export DOCKER_BUILDKIT=1
export COMPOSE_DOCKER_CLI_BUILD=1

cleanup() {
  docker compose -f "$COMPOSE_FILE" down --remove-orphans >/dev/null 2>&1 || true
}
trap cleanup EXIT

printf 'run_id: %s\nrun_dir: %s\nresults_dir: %s\n' "$RUN_ID" "$RUN_DIR" "$RESULTS_DIR" \
  | tee "$RUN_DIR/summary.txt"

if [[ "${AIC_SKIP_PULL:-0}" == "1" ]] || \
   [[ "${AIC_SKIP_BUILD:-0}" == "1" ]] && docker image inspect ghcr.io/intrinsic-dev/aic/aic_eval:latest >/dev/null 2>&1; then
  echo "Using local ghcr.io/intrinsic-dev/aic/aic_eval:latest image; skipping pull." | tee "$RUN_DIR/pull.log"
else
  docker pull ghcr.io/intrinsic-dev/aic/aic_eval:latest 2>&1 | tee "$RUN_DIR/pull.log"
fi

AIC_RESULTS_HOST_DIR="$RESULTS_DIR" \
  docker compose -f "$COMPOSE_FILE" config >"$RUN_DIR/compose.rendered.yaml"

./scripts/prewarm_submission_cache.sh "$TORCH_CACHE_DIR" 2>&1 \
  | tee "$RUN_DIR/prewarm.log"

if [[ "${AIC_SKIP_BUILD:-0}" != "1" ]]; then
  AIC_RESULTS_HOST_DIR="$RESULTS_DIR" \
  AIC_TORCH_CACHE_HOST_DIR="$TORCH_CACHE_DIR" \
    docker compose -f "$COMPOSE_FILE" build model 2>&1 | tee "$RUN_DIR/build.log"
else
  echo "Skipping image build because AIC_SKIP_BUILD=1" | tee "$RUN_DIR/build.log"
fi

docker image inspect my-solution:submission-act >"$RUN_DIR/model-image.inspect.json"

AIC_RESULTS_HOST_DIR="$RESULTS_DIR" \
  AIC_TORCH_CACHE_HOST_DIR="$TORCH_CACHE_DIR" \
  docker compose -f "$COMPOSE_FILE" up --abort-on-container-exit 2>&1 \
  | tee "$RUN_DIR/compose.log"

if [[ -f "$RESULTS_DIR/scoring.yaml" ]]; then
  echo "scoring_file=$RESULTS_DIR/scoring.yaml" | tee -a "$RUN_DIR/summary.txt"
else
  echo "scoring_file_missing=$RESULTS_DIR/scoring.yaml" | tee -a "$RUN_DIR/summary.txt"
fi
