#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BATCH_ID="${1:-visual_consistency_$(date +%Y%m%d_%H%M%S)}"
ITERATIONS="${AIC_VISUAL_CONSISTENCY_ITERATIONS:-10}"
BATCH_DIR="$ROOT_DIR/runs/$BATCH_ID"
SUMMARY_TSV="$BATCH_DIR/summary.tsv"

mkdir -p "$BATCH_DIR"
cd "$ROOT_DIR"

printf 'iteration\tstatus\tattempts\tscore\tscore_file\trun_dir\n' >"$SUMMARY_TSV"

failures=0
for iteration in $(seq 1 "$ITERATIONS"); do
  iter_label="$(printf '%02d' "$iteration")"
  run_id="${BATCH_ID}_iter_${iter_label}"
  wrapper_log="$BATCH_DIR/iter_${iter_label}.wrapper.log"

  echo "=== Visual consistency iteration $iteration/$ITERATIONS: $run_id ===" | tee "$wrapper_log"

  if AIC_GUI_MAX_ATTEMPTS="${AIC_GUI_MAX_ATTEMPTS:-1}" \
    "$ROOT_DIR/scripts/run_submission_best_gui_wslg_cpu.sh" "$run_id" \
    2>&1 | tee -a "$wrapper_log"; then
    wrapper_status=0
  else
    wrapper_status=$?
  fi

  attempts="$(find "$ROOT_DIR/runs" -maxdepth 1 -type d -name "${run_id}*" | wc -l | tr -d ' ')"
  score_file=""
  final_run_dir=""
  for candidate in "$ROOT_DIR/runs/$run_id" "$ROOT_DIR"/runs/"${run_id}"_retry_*; do
    [[ -f "$candidate/results/scoring.yaml" ]] || continue
    if grep -q "Cable insertion successful" "$candidate/results/scoring.yaml"; then
      score_file="$candidate/results/scoring.yaml"
      final_run_dir="$candidate"
      break
    fi
  done

  if [[ -n "$score_file" ]]; then
    score="$(awk '/^total:/ {print $2; exit}' "$score_file")"
    status="success"
  else
    score=""
    status="failure"
  fi

  if docker ps -a --format '{{.Names}}' | grep -q '^aic-submission-cpu-'; then
    echo "AIC containers remained after iteration $iteration; cleaning them up." | tee -a "$wrapper_log"
    docker compose \
      -f "$ROOT_DIR/docker/docker-compose.submission-cpu.yaml" \
      -f "$ROOT_DIR/docker/docker-compose.submission-gui-wslg.yaml" \
      down --remove-orphans >/dev/null 2>&1 || true
    status="failure"
  fi

  if [[ "$wrapper_status" -ne 0 || "$status" != "success" ]]; then
    failures=$((failures + 1))
  fi

  printf '%s\t%s\t%s\t%s\t%s\t%s\n' \
    "$iteration" "$status" "$attempts" "$score" "$score_file" "$final_run_dir" \
    | tee -a "$SUMMARY_TSV"
done

successes=$((ITERATIONS - failures))
{
  echo
  echo "successes=$successes"
  echo "failures=$failures"
  echo "summary=$SUMMARY_TSV"
} | tee "$BATCH_DIR/final.txt"

[[ "$failures" -eq 0 ]]
