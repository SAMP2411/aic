#!/usr/bin/env bash
set -euo pipefail

OUTPUT_DIR="${1:-}"
CONTAINER_NAME="${2:-aic-submission-cpu-eval-1}"
TIMEOUT_SECONDS="${3:-20}"

if [[ -z "$OUTPUT_DIR" ]]; then
  echo "Usage: $0 <output_dir> [container_name] [timeout_seconds]" >&2
  exit 1
fi

mkdir -p "$OUTPUT_DIR"

docker exec "$CONTAINER_NAME" /bin/bash -lc "
set -euo pipefail
rm -rf /tmp/aic_camera_snapshots
mkdir -p /tmp/aic_camera_snapshots
set +u
. /ws_aic/install/setup.bash
set -u
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_ROUTER_CONFIG_URI=/aic_zenoh_config.json5
export ZENOH_CONFIG_OVERRIDE=';transport/shared_memory/enabled=false'
python3 - <<'PY'
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

OUTPUT_DIR = Path('/tmp/aic_camera_snapshots')
TIMEOUT_SECONDS = float(${TIMEOUT_SECONDS})
TOPICS = {
    'left_camera.png': '/left_camera/image',
    'center_camera.png': '/center_camera/image',
    'right_camera.png': '/right_camera/image',
}


class SnapshotNode(Node):
    def __init__(self) -> None:
        super().__init__('aic_camera_snapshot')
        self.frames = {}
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        for filename, topic in TOPICS.items():
            self.create_subscription(
                Image,
                topic,
                lambda msg, filename=filename: self._handle_image(filename, msg),
                qos,
            )

    def _handle_image(self, filename: str, msg: Image) -> None:
        if filename in self.frames:
            return
        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        # ROS image messages arrive as RGB, while OpenCV writes BGR.
        self.frames[filename] = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)


def main() -> int:
    rclpy.init()
    node = SnapshotNode()
    deadline = time.monotonic() + TIMEOUT_SECONDS
    try:
        while time.monotonic() < deadline and len(node.frames) < len(TOPICS):
            rclpy.spin_once(node, timeout_sec=0.5)
    finally:
        for filename, frame in node.frames.items():
            cv2.imwrite(str(OUTPUT_DIR / filename), frame)
        node.destroy_node()
        rclpy.shutdown()

    missing = sorted(set(TOPICS) - set(node.frames))
    if missing:
        print(f'Missing frames for: {missing}')
        return 1

    print(f'Saved frames to {OUTPUT_DIR}')
    return 0


raise SystemExit(main())
PY
"

docker cp "$CONTAINER_NAME:/tmp/aic_camera_snapshots/." "$OUTPUT_DIR/"
