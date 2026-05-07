#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUN_ID="${1:-best_gui_hw_$(date +%Y%m%d_%H%M%S)}"

export AIC_EVAL_SOFTWARE_RENDERING="${AIC_EVAL_SOFTWARE_RENDERING:-0}"
export AIC_EVAL_MESA_DRIVER="${AIC_EVAL_MESA_DRIVER:-iris}"
export AIC_EVAL_GALLIUM_DRIVER="${AIC_EVAL_GALLIUM_DRIVER:-}"
export AIC_EVAL_DRI_DRIVER_HINT="${AIC_EVAL_DRI_DRIVER_HINT:-i915}"

exec "$ROOT_DIR/scripts/run_submission_best_gui.sh" "$RUN_ID"
