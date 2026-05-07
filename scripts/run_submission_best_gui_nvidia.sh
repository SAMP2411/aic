#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUN_ID="${1:-best_gui_nvidia_$(date +%Y%m%d_%H%M%S)}"

if command -v nvidia-smi >/dev/null 2>&1; then
  export AIC_GUI_EXTRA_COMPOSE="${AIC_GUI_EXTRA_COMPOSE:-$ROOT_DIR/docker/docker-compose.submission-gui-nvidia.yaml}"
elif [[ -e /dev/dxg || -d /mnt/wslg ]]; then
  echo "NVIDIA is not available in WSL; using WSLg GUI fallback without the NVIDIA compose overlay." >&2
  export AIC_GUI_COMPOSE="${AIC_GUI_COMPOSE:-$ROOT_DIR/docker/docker-compose.submission-gui-wslg.yaml}"
  unset AIC_GUI_EXTRA_COMPOSE
else
  export AIC_GUI_EXTRA_COMPOSE="${AIC_GUI_EXTRA_COMPOSE:-$ROOT_DIR/docker/docker-compose.submission-gui-nvidia.yaml}"
fi
export AIC_EVAL_SOFTWARE_RENDERING="${AIC_EVAL_SOFTWARE_RENDERING:-0}"
export AIC_EVAL_MESA_DRIVER="${AIC_EVAL_MESA_DRIVER:-}"
export AIC_EVAL_GALLIUM_DRIVER="${AIC_EVAL_GALLIUM_DRIVER:-}"
export AIC_EVAL_DRI_DRIVER_HINT="${AIC_EVAL_DRI_DRIVER_HINT:-nvidia}"
export AIC_EVAL_DRI_PRIME="${AIC_EVAL_DRI_PRIME:-}"
export AIC_EVAL_NV_PRIME_RENDER_OFFLOAD="${AIC_EVAL_NV_PRIME_RENDER_OFFLOAD:-1}"
export AIC_EVAL_GLX_VENDOR_LIBRARY_NAME="${AIC_EVAL_GLX_VENDOR_LIBRARY_NAME:-nvidia}"
export AIC_EVAL_VK_LAYER_NV_OPTIMUS="${AIC_EVAL_VK_LAYER_NV_OPTIMUS:-NVIDIA_only}"

exec "$ROOT_DIR/scripts/run_submission_best_gui.sh" "$RUN_ID"
