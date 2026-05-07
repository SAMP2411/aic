#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUN_ID="${1:-profile_smoke_$(date +%Y%m%d_%H%M%S)}"
PROFILE="${2:-tf_smooth}"

# shellcheck source=/dev/null
source "$ROOT_DIR/scripts/submission_profiles.sh"
apply_submission_profile "$PROFILE"

exec "$ROOT_DIR/scripts/run_submission_smoke.sh" "$RUN_ID"
