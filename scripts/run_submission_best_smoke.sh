#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUN_ID="${1:-best_smoke_$(date +%Y%m%d_%H%M%S)}"
# shellcheck source=/dev/null
source "$ROOT_DIR/scripts/submission_profiles.sh"
apply_submission_profile "${AIC_SUBMISSION_PROFILE:-tf_smoother}"

exec "$ROOT_DIR/scripts/run_submission_smoke.sh" "$RUN_ID"
