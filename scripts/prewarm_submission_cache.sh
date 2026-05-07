#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CACHE_DIR="${1:-$ROOT_DIR/.cache/torch-cache}"

mkdir -p "$CACHE_DIR"

docker run --rm \
  -v "$CACHE_DIR:/opt/torch-cache" \
  --entrypoint /bin/bash \
  my-solution:submission-act \
  -lc '
    export TORCH_HOME=/opt/torch-cache
    mkdir -p "$TORCH_HOME"
    cd /ws_aic/src/aic
    pixi run --as-is python - <<'"'"'PY'"'"'
from pathlib import Path
import os

os.environ.setdefault("TORCH_HOME", "/opt/torch-cache")

from torchvision.models import ResNet18_Weights, resnet18

resnet18(weights=ResNet18_Weights.IMAGENET1K_V1)

for path in sorted(Path("/opt/torch-cache").rglob("resnet18*.pth")):
    print(path)
PY
  '
