#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
STAMP="${1:-$(date +%Y%m%d_%H%M%S)}"
shift || true
PROFILES=("$@")
if [[ ${#PROFILES[@]} -eq 0 ]]; then
  PROFILES=(tf_smooth tf_smoother tf_gentle vision_then_act)
fi

RESULT_DIR="$ROOT_DIR/runs/tuning_$STAMP"
SUMMARY_TSV="$RESULT_DIR/summary.tsv"
mkdir -p "$RESULT_DIR"

printf "profile\trun_id\ttotal\ttier3\tduration\tforce\tjerk\n" >"$SUMMARY_TSV"

for profile in "${PROFILES[@]}"; do
  run_id="${profile}_${STAMP}"
  "$ROOT_DIR/scripts/run_submission_profile_smoke.sh" "$run_id" "$profile"
  python3 - "$ROOT_DIR/runs/$run_id/results/scoring.yaml" "$profile" "$run_id" >>"$SUMMARY_TSV" <<'PY'
import sys
import yaml
from pathlib import Path

score_path = Path(sys.argv[1])
profile = sys.argv[2]
run_id = sys.argv[3]
data = yaml.safe_load(score_path.read_text())
trial = data["trial_1"]
cats = trial["tier_2"]["categories"]
row = [
    profile,
    run_id,
    str(data["total"]),
    str(trial["tier_3"]["score"]),
    cats["duration"]["message"],
    cats["insertion force"]["message"],
    cats["trajectory smoothness"]["message"],
]
print("\t".join(row))
PY
done

if command -v column >/dev/null 2>&1; then
  column -t -s $'\t' "$SUMMARY_TSV"
else
  cat "$SUMMARY_TSV"
fi
