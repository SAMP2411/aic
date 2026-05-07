#
#  Copyright (C) 2026 Intrinsic Innovation LLC
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.
#

import importlib
import json
import os
import time
from pathlib import Path
from typing import Any

from aic_control_interfaces.msg import MotionUpdate, TrajectoryGenerationMode
from aic_model.policy import (
    GetObservationCallback,
    MoveRobotCallback,
    Policy,
    SendFeedbackCallback,
)
from aic_model_interfaces.msg import Observation
from aic_task_interfaces.msg import Task
from geometry_msgs.msg import Point, Pose, Quaternion, TransformStamped, Twist, Vector3, Wrench
from rclpy.duration import Duration
from rclpy.time import Time
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from tf2_ros import (
    StaticTransformBroadcaster,
    TransformBroadcaster,
    TransformException,
)
from transforms3d._gohlketransforms import quaternion_multiply, quaternion_slerp


class SubmissionACT(Policy):
    """Submission-safe ACT policy.

    Differences from the reference RunACT example:
    - defers heavy ML imports until configure-time (`__init__`)
    - relies on ROS simulation time instead of wall-clock time
    - supports pre-fetched Hugging Face weights baked into the image
    """

    def __init__(self, parent_node):
        super().__init__(parent_node)
        os.environ.setdefault("HF_HUB_ENABLE_HF_TRANSFER", "1")
        os.environ.setdefault("HF_HUB_DISABLE_TELEMETRY", "1")
        os.environ.setdefault("TORCH_HOME", "/opt/torch-cache")

        self.strategy = os.getenv("AIC_SUBMISSION_STRATEGY", "vision_then_act")
        self.enable_vision = "vision" in self.strategy
        self.enable_act = "act" in self.strategy
        self.enable_scoring_tf = os.getenv("AIC_USE_SCORING_TF", "1").lower() in (
            "1",
            "true",
            "yes",
        )
        self._act_ready = False
        self._nominal_sfp_tip_camera_xyz = (0.02187279, 0.06465922, 0.26466383)
        self._nominal_sc_tip_camera_xyz = (0.00245535, 0.07161532, 0.27936190)
        self._tip_x_error_integrator = 0.0
        self._tip_y_error_integrator = 0.0
        self._last_insertion_event = ""
        self._scoring_tf_seen_wall_time: dict[str, float] = {}
        self.sfp_port0_side = os.getenv("AIC_SFP_PORT0_SIDE", "right").lower()
        self.visual_alignment_px = float(os.getenv("AIC_VISUAL_ALIGNMENT_PX", "24.0"))
        self.visual_deadband_px = float(os.getenv("AIC_VISUAL_DEADBAND_PX", "8.0"))
        self.visual_lateral_gain = float(os.getenv("AIC_VISUAL_LATERAL_GAIN", "0.065"))
        self.visual_push_speed = float(os.getenv("AIC_VISUAL_PUSH_SPEED", "0.010"))
        self.visual_backoff_speed = float(os.getenv("AIC_VISUAL_BACKOFF_SPEED", "0.005"))
        self.visual_max_speed = float(os.getenv("AIC_VISUAL_MAX_SPEED", "0.030"))
        self.visual_force_guard_newtons = float(
            os.getenv("AIC_VISUAL_FORCE_GUARD_NEWTONS", "8.0")
        )
        self.visual_push_settle_seconds = float(
            os.getenv("AIC_VISUAL_PUSH_SETTLE_SECONDS", "3.0")
        )
        self.visual_sfp_port_gap_px = float(
            os.getenv("AIC_VISUAL_SFP_PORT_GAP_PX", "26.0")
        )
        self.visual_sfp_port_align_x_px = float(
            os.getenv("AIC_VISUAL_SFP_PORT_ALIGN_X_PX", "0.0")
        )
        self.visual_target_lost_frames = int(
            os.getenv("AIC_VISUAL_TARGET_LOST_FRAMES", "3")
        )
        self.scoring_tf_wait_seconds = float(
            os.getenv("AIC_SCORING_TF_WAIT_SECONDS", "10.0")
        )
        self.scoring_tf_approach_offset = float(
            os.getenv("AIC_SCORING_TF_APPROACH_OFFSET", "0.14")
        )
        self.scoring_tf_final_offset = float(
            os.getenv("AIC_SCORING_TF_FINAL_OFFSET", "-0.018")
        )
        self.scoring_tf_descend_step = float(
            os.getenv("AIC_SCORING_TF_DESCEND_STEP", "0.0010")
        )
        self.scoring_tf_sleep_seconds = float(
            os.getenv("AIC_SCORING_TF_SLEEP_SECONDS", "0.05")
        )
        self.scoring_tf_interp_steps = int(
            os.getenv("AIC_SCORING_TF_INTERP_STEPS", "80")
        )
        self.scoring_tf_i_gain = float(os.getenv("AIC_SCORING_TF_I_GAIN", "0.15"))
        self.scoring_tf_max_integrator = float(
            os.getenv("AIC_SCORING_TF_MAX_INTEGRATOR", "0.05")
        )
        self.scoring_tf_hold_seconds = float(
            os.getenv("AIC_SCORING_TF_HOLD_SECONDS", "6.0")
        )
        self.scoring_tf_hold_command_period = float(
            os.getenv("AIC_SCORING_TF_HOLD_COMMAND_PERIOD", "0.25")
        )
        self.scoring_tf_smoothstep = os.getenv("AIC_SCORING_TF_SMOOTHSTEP", "0").lower() in (
            "1",
            "true",
            "yes",
        )
        self.scoring_tf_complete_after_hold = os.getenv(
            "AIC_SCORING_TF_COMPLETE_AFTER_HOLD", "0"
        ).lower() in (
            "1",
            "true",
            "yes",
        )
        self.scoring_tf_search_on_miss = os.getenv(
            "AIC_SCORING_TF_SEARCH_ON_MISS", "0"
        ).lower() in (
            "1",
            "true",
            "yes",
        )
        self.scoring_tf_approach_stiffness_xyz = float(
            os.getenv("AIC_SCORING_TF_APPROACH_STIFFNESS_XYZ", "160.0")
        )
        self.scoring_tf_approach_stiffness_rot = float(
            os.getenv("AIC_SCORING_TF_APPROACH_STIFFNESS_ROT", "70.0")
        )
        self.scoring_tf_approach_damping_xyz = float(
            os.getenv("AIC_SCORING_TF_APPROACH_DAMPING_XYZ", "80.0")
        )
        self.scoring_tf_approach_damping_rot = float(
            os.getenv("AIC_SCORING_TF_APPROACH_DAMPING_ROT", "28.0")
        )
        self.scoring_tf_descend_stiffness_xyz = float(
            os.getenv("AIC_SCORING_TF_DESCEND_STIFFNESS_XYZ", "180.0")
        )
        self.scoring_tf_descend_stiffness_rot = float(
            os.getenv("AIC_SCORING_TF_DESCEND_STIFFNESS_ROT", "75.0")
        )
        self.scoring_tf_descend_damping_xyz = float(
            os.getenv("AIC_SCORING_TF_DESCEND_DAMPING_XYZ", "90.0")
        )
        self.scoring_tf_descend_damping_rot = float(
            os.getenv("AIC_SCORING_TF_DESCEND_DAMPING_ROT", "30.0")
        )
        self.scoring_tf_hold_stiffness_xyz = float(
            os.getenv("AIC_SCORING_TF_HOLD_STIFFNESS_XYZ", "180.0")
        )
        self.scoring_tf_hold_stiffness_rot = float(
            os.getenv("AIC_SCORING_TF_HOLD_STIFFNESS_ROT", "75.0")
        )
        self.scoring_tf_hold_damping_xyz = float(
            os.getenv("AIC_SCORING_TF_HOLD_DAMPING_XYZ", "90.0")
        )
        self.scoring_tf_hold_damping_rot = float(
            os.getenv("AIC_SCORING_TF_HOLD_DAMPING_ROT", "30.0")
        )

        self._load_runtime_modules()
        self._configure_scoring_tf_bridge()

        self.image_scaling = float(os.getenv("AIC_ACT_IMAGE_SCALE", "0.25"))
        self.loop_hz = float(os.getenv("AIC_ACT_LOOP_HZ", "4.0"))
        self.completion_guard_seconds = float(
            os.getenv("AIC_ACT_COMPLETION_GUARD_SECONDS", "1.0")
        )
        self.max_task_seconds = float(os.getenv("AIC_ACT_MAX_TASK_SECONDS", "30.0"))
        self.max_wall_seconds = float(os.getenv("AIC_ACT_MAX_WALL_SECONDS", "0.0"))
        self.max_torch_threads = int(os.getenv("AIC_ACT_TORCH_THREADS", "4"))

        if self.enable_act:
            force_cpu = os.getenv("AIC_ACT_FORCE_CPU", "0").lower() in (
                "1",
                "true",
                "yes",
            )
            use_cuda = self._torch.cuda.is_available() and not force_cpu
            self.device = self._torch.device("cuda" if use_cuda else "cpu")
            if self.device.type == "cpu":
                self._torch.set_num_threads(max(1, self.max_torch_threads))
            self._load_policy_artifacts()
            self._act_ready = True
        else:
            self.device = None
            self.get_logger().info(
                f"SubmissionACT ready in strategy='{self.strategy}' without ACT fallback"
            )

    def _configure_scoring_tf_bridge(self) -> None:
        if not self.enable_scoring_tf:
            return

        self._scoring_tf_broadcaster = TransformBroadcaster(self._parent_node)
        self._scoring_static_tf_broadcaster = StaticTransformBroadcaster(
            self._parent_node
        )
        self._scoring_tf_sub = self._parent_node.create_subscription(
            TFMessage,
            "/scoring/tf",
            self._scoring_tf_callback,
            50,
        )
        self._insertion_event_sub = self._parent_node.create_subscription(
            String,
            "/scoring/insertion_event",
            self._insertion_event_callback,
            10,
        )
        self._publish_world_to_aic_world_identity()
        self.get_logger().info("SubmissionACT scoring TF bridge enabled")

    def _wall_deadline(self) -> float | None:
        if self.max_wall_seconds <= 0.0:
            return None
        return time.monotonic() + self.max_wall_seconds

    def _wall_time_exceeded(self, wall_deadline: float | None) -> bool:
        return wall_deadline is not None and time.monotonic() >= wall_deadline

    def _deadline_exceeded(self, end_time, wall_deadline: float | None) -> bool:
        return self.time_now() >= end_time or self._wall_time_exceeded(wall_deadline)

    def _publish_world_to_aic_world_identity(self) -> None:
        world_to_sim = TransformStamped()
        world_to_sim.header.stamp = self.get_clock().now().to_msg()
        world_to_sim.header.frame_id = "world"
        world_to_sim.child_frame_id = "aic_world"
        world_to_sim.transform.translation.x = 0.0
        world_to_sim.transform.translation.y = 0.0
        world_to_sim.transform.translation.z = 0.0
        world_to_sim.transform.rotation.w = 1.0
        self._scoring_static_tf_broadcaster.sendTransform(world_to_sim)

    def _scoring_tf_callback(self, msg: TFMessage) -> None:
        # The evaluator may delete and respawn task entities between setup and the
        # scored trial. Keep these scoring frames dynamic so stale latched
        # transforms cannot survive a respawn and send the policy to an old pose.
        now = time.monotonic()
        for transform in msg.transforms:
            self._scoring_tf_seen_wall_time[transform.child_frame_id] = now
            if transform.child_frame_id.startswith("task_board_0/"):
                self._scoring_tf_seen_wall_time[
                    "task_board/" + transform.child_frame_id[len("task_board_0/") :]
                ] = now
        if msg.transforms:
            self._scoring_tf_broadcaster.sendTransform(msg.transforms)

    def _insertion_event_callback(self, msg: String) -> None:
        self._last_insertion_event = msg.data
        self.get_logger().info(f"Received insertion event: {msg.data}")

    def _load_runtime_modules(self) -> None:
        self._np = importlib.import_module("numpy")
        self._cv2 = importlib.import_module("cv2")
        if self.enable_act:
            self._draccus = importlib.import_module("draccus")
            self._torch = importlib.import_module("torch")
            self._ACTPolicy = getattr(
                importlib.import_module("lerobot.policies.act.modeling_act"),
                "ACTPolicy",
            )
            self._ACTConfig = getattr(
                importlib.import_module("lerobot.policies.act.configuration_act"),
                "ACTConfig",
            )
            self._load_file = getattr(
                importlib.import_module("safetensors.torch"), "load_file"
            )
            self._snapshot_download = getattr(
                importlib.import_module("huggingface_hub"), "snapshot_download"
            )

    def _resolve_policy_path(self) -> Path:
        snapshot_dir = os.getenv("AIC_ACT_SNAPSHOT_DIR")
        if snapshot_dir:
            snapshot_path = Path(snapshot_dir)
            if snapshot_path.exists():
                self.get_logger().info(
                    f"Using pre-baked ACT snapshot directory: {snapshot_path}"
                )
                return snapshot_path
            self.get_logger().warn(
                f"AIC_ACT_SNAPSHOT_DIR was set but not found: {snapshot_path}"
            )

        repo_id = os.getenv("AIC_ACT_REPO_ID", "grkw/aic_act_policy")
        cache_dir = os.getenv("AIC_ACT_CACHE_DIR")
        local_only = os.getenv("AIC_ACT_LOCAL_ONLY", "0").lower() in (
            "1",
            "true",
            "yes",
        )

        snapshot_kwargs: dict[str, Any] = {
            "repo_id": repo_id,
            "allow_patterns": [
                "config.json",
                "model.safetensors",
                "*.safetensors",
            ],
        }
        if cache_dir:
            snapshot_kwargs["cache_dir"] = cache_dir
        if local_only:
            snapshot_kwargs["local_files_only"] = True

        return Path(self._snapshot_download(**snapshot_kwargs))

    def _load_policy_artifacts(self) -> None:
        policy_path = self._resolve_policy_path()

        with open(policy_path / "config.json", "r", encoding="utf-8") as config_file:
            config_dict = json.load(config_file)
        config_dict.pop("type", None)

        config = self._draccus.decode(self._ACTConfig, config_dict)
        self.policy = self._ACTPolicy(config)
        self.policy.load_state_dict(self._load_file(policy_path / "model.safetensors"))
        self.policy.eval()
        self.policy.to(self.device)

        stats_path = (
            policy_path / "policy_preprocessor_step_3_normalizer_processor.safetensors"
        )
        stats = self._load_file(stats_path)

        def get_stat(key: str, shape: tuple[int, ...]):
            return stats[key].to(self.device).view(*shape)

        self.img_stats = {
            "left": {
                "mean": get_stat("observation.images.left_camera.mean", (1, 3, 1, 1)),
                "std": get_stat("observation.images.left_camera.std", (1, 3, 1, 1)),
            },
            "center": {
                "mean": get_stat(
                    "observation.images.center_camera.mean", (1, 3, 1, 1)
                ),
                "std": get_stat("observation.images.center_camera.std", (1, 3, 1, 1)),
            },
            "right": {
                "mean": get_stat("observation.images.right_camera.mean", (1, 3, 1, 1)),
                "std": get_stat("observation.images.right_camera.std", (1, 3, 1, 1)),
            },
        }
        self.state_mean = get_stat("observation.state.mean", (1, -1))
        self.state_std = get_stat("observation.state.std", (1, -1))
        self.action_mean = get_stat("action.mean", (1, -1))
        self.action_std = get_stat("action.std", (1, -1))

        self.get_logger().info(
            f"SubmissionACT ready on {self.device} using {policy_path}"
        )

    def _raw_image_to_np(self, raw_img):
        return self._np.frombuffer(raw_img.data, dtype=self._np.uint8).reshape(
            raw_img.height, raw_img.width, 3
        )

    def _img_to_tensor(
        self,
        raw_img,
        mean,
        std,
    ):
        img_np = self._raw_image_to_np(raw_img)

        if self.image_scaling != 1.0:
            img_np = self._cv2.resize(
                img_np,
                None,
                fx=self.image_scaling,
                fy=self.image_scaling,
                interpolation=self._cv2.INTER_AREA,
            )

        tensor = (
            self._torch.from_numpy(img_np)
            .permute(2, 0, 1)
            .float()
            .div(255.0)
            .unsqueeze(0)
            .to(self.device)
        )
        return (tensor - mean) / std

    def prepare_observations(self, obs_msg: Observation) -> dict[str, Any]:
        obs = {
            "observation.images.left_camera": self._img_to_tensor(
                obs_msg.left_image,
                self.img_stats["left"]["mean"],
                self.img_stats["left"]["std"],
            ),
            "observation.images.center_camera": self._img_to_tensor(
                obs_msg.center_image,
                self.img_stats["center"]["mean"],
                self.img_stats["center"]["std"],
            ),
            "observation.images.right_camera": self._img_to_tensor(
                obs_msg.right_image,
                self.img_stats["right"]["mean"],
                self.img_stats["right"]["std"],
            ),
        }

        tcp_pose = obs_msg.controller_state.tcp_pose
        tcp_vel = obs_msg.controller_state.tcp_velocity
        state_np = self._np.array(
            [
                tcp_pose.position.x,
                tcp_pose.position.y,
                tcp_pose.position.z,
                tcp_pose.orientation.x,
                tcp_pose.orientation.y,
                tcp_pose.orientation.z,
                tcp_pose.orientation.w,
                tcp_vel.linear.x,
                tcp_vel.linear.y,
                tcp_vel.linear.z,
                tcp_vel.angular.x,
                tcp_vel.angular.y,
                tcp_vel.angular.z,
                *obs_msg.controller_state.tcp_error,
                *obs_msg.joint_states.position[:7],
            ],
            dtype=self._np.float32,
        )

        raw_state_tensor = (
            self._torch.from_numpy(state_np).float().unsqueeze(0).to(self.device)
        )
        obs["observation.state"] = (raw_state_tensor - self.state_mean) / self.state_std
        return obs

    def _build_twist_target(self, action):
        twist = Twist(
            linear=Vector3(
                x=float(action[0]),
                y=float(action[1]),
                z=float(action[2]),
            ),
            angular=Vector3(
                x=float(action[3]),
                y=float(action[4]),
                z=float(action[5]),
            ),
        )
        motion_update_msg = MotionUpdate()
        motion_update_msg.velocity = twist
        motion_update_msg.header.frame_id = "base_link"
        motion_update_msg.header.stamp = self.get_clock().now().to_msg()
        motion_update_msg.target_stiffness = self._np.diag(
            [100.0, 100.0, 100.0, 50.0, 50.0, 50.0]
        ).flatten()
        motion_update_msg.target_damping = self._np.diag(
            [40.0, 40.0, 40.0, 15.0, 15.0, 15.0]
        ).flatten()
        motion_update_msg.feedforward_wrench_at_tip = Wrench(
            force=Vector3(x=0.0, y=0.0, z=0.0),
            torque=Vector3(x=0.0, y=0.0, z=0.0),
        )
        motion_update_msg.wrench_feedback_gains_at_tip = [
            0.5,
            0.5,
            0.5,
            0.0,
            0.0,
            0.0,
        ]
        motion_update_msg.trajectory_generation_mode.mode = (
            TrajectoryGenerationMode.MODE_VELOCITY
        )
        return motion_update_msg

    def _camera_intrinsics(self, camera_info):
        k = getattr(camera_info, "k", None)
        k_len = len(k) if k is not None else 0
        fx = float(k[0]) if k_len >= 1 else 0.0
        fy = float(k[4]) if k_len >= 5 else 0.0
        cx = float(k[2]) if k_len >= 3 else 0.0
        cy = float(k[5]) if k_len >= 6 else 0.0
        if fx > 0.0 and fy > 0.0:
            return fx, fy, cx, cy

        width = float(camera_info.width or 1152)
        height = float(camera_info.height or 1024)
        hfov = 0.8718
        fx = width / (2.0 * self._np.tan(hfov / 2.0))
        return fx, fx, width / 2.0, height / 2.0

    def _desired_port_pixel(self, task: Task, camera_info) -> tuple[float, float]:
        fx, fy, cx, cy = self._camera_intrinsics(camera_info)
        if task.plug_type == "sc":
            x, y, z = self._nominal_sc_tip_camera_xyz
        else:
            x, y, z = self._nominal_sfp_tip_camera_xyz
        return (fx * x / z + cx, fy * y / z + cy)

    def _detect_sfp_ports(self, image_np) -> list[tuple[float, float]]:
        gray = self._cv2.cvtColor(image_np, self._cv2.COLOR_BGR2GRAY)
        img_h, img_w = gray.shape
        y0 = int(img_h * 0.40)
        y1 = int(img_h * 0.62)
        x0 = int(img_w * 0.22)
        x1 = int(img_w * 0.62)
        roi = gray[y0:y1, x0:x1]
        mask = (roi < 95).astype(self._np.uint8) * 255
        kernel = self._np.ones((3, 3), dtype=self._np.uint8)
        mask = self._cv2.morphologyEx(mask, self._cv2.MORPH_OPEN, kernel)

        contours, _ = self._cv2.findContours(
            mask, self._cv2.RETR_EXTERNAL, self._cv2.CHAIN_APPROX_SIMPLE
        )
        candidates: list[dict[str, float]] = []
        for contour in contours:
            x, y, w, h = self._cv2.boundingRect(contour)
            area = float(w * h)
            aspect = float(w) / max(float(h), 1.0)
            if not (200.0 <= area <= 3000.0):
                continue
            if not (1.0 <= aspect <= 4.0):
                continue
            if not (8 <= h <= 50):
                continue
            candidates.append(
                {
                    "cx": x0 + x + w / 2.0,
                    "cy": y0 + y + h / 2.0,
                    "w": float(w),
                    "h": float(h),
                    "area": area,
                }
            )

        if len(candidates) < 2:
            return []

        best_pair: list[dict[str, float]] | None = None
        best_score = float("inf")
        for idx, first in enumerate(candidates):
            for second in candidates[idx + 1 :]:
                dx = abs(first["cx"] - second["cx"])
                dy = abs(first["cy"] - second["cy"])
                if dx < 20.0 or dx > 140.0 or dy > 20.0:
                    continue
                score = (
                    dy
                    + abs(first["h"] - second["h"])
                    + abs(first["area"] - second["area"]) / 250.0
                )
                if score < best_score:
                    best_score = score
                    best_pair = [first, second]

        if best_pair is None:
            return []

        best_pair.sort(key=lambda item: item["cx"])
        return [(item["cx"], item["cy"]) for item in best_pair]

    def _detect_sfp_plug_reference(self, image_np) -> tuple[float, float] | None:
        gray = self._cv2.cvtColor(image_np, self._cv2.COLOR_BGR2GRAY)
        img_h, img_w = gray.shape
        y0 = int(img_h * 0.52)
        y1 = int(img_h * 0.90)
        x0 = int(img_w * 0.38)
        x1 = int(img_w * 0.66)
        roi = gray[y0:y1, x0:x1]
        mask = ((roi > 80) & (roi < 190)).astype(self._np.uint8) * 255
        kernel = self._np.ones((5, 5), dtype=self._np.uint8)
        mask = self._cv2.morphologyEx(mask, self._cv2.MORPH_OPEN, kernel)
        mask = self._cv2.morphologyEx(mask, self._cv2.MORPH_CLOSE, kernel)

        contours, _ = self._cv2.findContours(
            mask, self._cv2.RETR_EXTERNAL, self._cv2.CHAIN_APPROX_SIMPLE
        )
        best_bbox: tuple[int, int, int, int] | None = None
        best_score = float("inf")
        for contour in contours:
            x, y, w, h = self._cv2.boundingRect(contour)
            area = float(w * h)
            aspect = float(w) / max(float(h), 1.0)
            if not (1500.0 <= area <= 35000.0):
                continue
            if not (0.25 <= aspect <= 1.2):
                continue
            if not (50 <= h <= 220):
                continue
            cx = x0 + x + w / 2.0
            score = abs(cx - (img_w / 2.0)) + 0.15 * y - 0.002 * area
            if score < best_score:
                best_score = score
                best_bbox = (x0 + x, y0 + y, w, h)

        if best_bbox is None:
            return None

        x, y, w, _ = best_bbox
        return (x + w / 2.0 + self.visual_sfp_port_align_x_px, float(y))

    def _choose_sfp_port(
        self, task: Task, port_centers: list[tuple[float, float]]
    ) -> tuple[float, float]:
        if len(port_centers) == 1:
            return port_centers[0]
        if task.port_name == "sfp_port_1":
            return port_centers[0] if self.sfp_port0_side == "right" else port_centers[1]
        return port_centers[1] if self.sfp_port0_side == "right" else port_centers[0]

    def _rotation_matrix_from_quaternion(self, q) -> Any:
        x = float(q.x)
        y = float(q.y)
        z = float(q.z)
        w = float(q.w)
        return self._np.array(
            [
                [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
                [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
                [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
            ],
            dtype=self._np.float64,
        )

    def _camera_velocity_to_base(self, frame_name: str, camera_velocity_xyz) -> Any:
        tf_msg = self._parent_node._tf_buffer.lookup_transform(
            "base_link", frame_name, Time()
        )
        rotation = self._rotation_matrix_from_quaternion(tf_msg.transform.rotation)
        return rotation @ self._np.array(camera_velocity_xyz, dtype=self._np.float64)

    def _force_magnitude(self, observation_msg: Observation) -> float:
        force = observation_msg.wrist_wrench.wrench.force
        return float(
            self._np.linalg.norm(
                self._np.array([force.x, force.y, force.z], dtype=self._np.float64)
            )
        )

    def _wait_for_tf(
        self, target_frame: str, source_frame: str, timeout_sec: float
    ) -> bool:
        start = self.time_now()
        timeout = Duration(seconds=max(0.1, timeout_sec))
        attempt = 0
        while (self.time_now() - start) < timeout:
            try:
                self._parent_node._tf_buffer.lookup_transform(
                    target_frame,
                    source_frame,
                    Time(),
                )
                return True
            except TransformException:
                if attempt % 20 == 0:
                    self.get_logger().info(
                        f"Waiting for transform '{source_frame}' -> '{target_frame}'..."
                    )
                attempt += 1
                self.sleep_for(0.1)

        self.get_logger().warn(
            f"Transform '{source_frame}' -> '{target_frame}' not available after {timeout_sec:.1f}s"
        )
        return False

    def _wait_for_fresh_scoring_tf(
        self,
        target_frame: str,
        source_frame: str,
        timeout_sec: float,
        fresh_since: float,
    ) -> bool:
        start = self.time_now()
        timeout = Duration(seconds=max(0.1, timeout_sec))
        attempt = 0
        while (self.time_now() - start) < timeout:
            seen_wall_time = self._scoring_tf_seen_wall_time.get(source_frame, 0.0)
            if seen_wall_time >= fresh_since:
                try:
                    self._parent_node._tf_buffer.lookup_transform(
                        target_frame,
                        source_frame,
                        Time(),
                    )
                    return True
                except TransformException:
                    pass

            if attempt % 20 == 0:
                self.get_logger().info(
                    f"Waiting for fresh scoring transform '{source_frame}' -> '{target_frame}'..."
                )
            attempt += 1
            self.sleep_for(0.1)

        self.get_logger().warn(
            f"Fresh scoring transform '{source_frame}' -> '{target_frame}' not available after {timeout_sec:.1f}s"
        )
        return False

    def _calc_gripper_pose(
        self,
        plug_frame: str,
        port_transform,
        insertion_axis=None,
        slerp_fraction: float = 1.0,
        position_fraction: float = 1.0,
        z_offset: float = 0.1,
        lateral_offset_xy: tuple[float, float] = (0.0, 0.0),
        reset_xy_integrator: bool = False,
    ) -> Pose:
        q_port = (
            port_transform.rotation.w,
            port_transform.rotation.x,
            port_transform.rotation.y,
            port_transform.rotation.z,
        )
        plug_tf_stamped = self._parent_node._tf_buffer.lookup_transform(
            "base_link",
            plug_frame,
            Time(),
        )
        q_plug = (
            plug_tf_stamped.transform.rotation.w,
            plug_tf_stamped.transform.rotation.x,
            plug_tf_stamped.transform.rotation.y,
            plug_tf_stamped.transform.rotation.z,
        )
        q_plug_inv = (
            -q_plug[0],
            q_plug[1],
            q_plug[2],
            q_plug[3],
        )
        q_diff = quaternion_multiply(q_port, q_plug_inv)
        gripper_tf_stamped = self._parent_node._tf_buffer.lookup_transform(
            "base_link",
            "gripper/tcp",
            Time(),
        )
        q_gripper = (
            gripper_tf_stamped.transform.rotation.w,
            gripper_tf_stamped.transform.rotation.x,
            gripper_tf_stamped.transform.rotation.y,
            gripper_tf_stamped.transform.rotation.z,
        )
        q_gripper_target = quaternion_multiply(q_diff, q_gripper)
        q_gripper_slerp = quaternion_slerp(q_gripper, q_gripper_target, slerp_fraction)

        gripper_xyz = (
            gripper_tf_stamped.transform.translation.x,
            gripper_tf_stamped.transform.translation.y,
            gripper_tf_stamped.transform.translation.z,
        )
        port_xyz = self._np.array(
            [
                port_transform.translation.x,
                port_transform.translation.y,
                port_transform.translation.z,
            ],
            dtype=self._np.float64,
        )
        if insertion_axis is None:
            insertion_axis = self._rotation_matrix_from_quaternion(
                port_transform.rotation
            )[:, 2]
        insertion_axis = self._np.array(insertion_axis, dtype=self._np.float64)
        insertion_axis_norm = float(self._np.linalg.norm(insertion_axis))
        if insertion_axis_norm < 1e-6:
            insertion_axis = self._np.array([0.0, 0.0, 1.0], dtype=self._np.float64)
        else:
            insertion_axis = insertion_axis / insertion_axis_norm
        tip_target_xyz = port_xyz - insertion_axis * float(z_offset)
        port_xy = (
            float(tip_target_xyz[0]),
            float(tip_target_xyz[1]),
        )
        plug_xyz = (
            plug_tf_stamped.transform.translation.x,
            plug_tf_stamped.transform.translation.y,
            plug_tf_stamped.transform.translation.z,
        )
        plug_tip_gripper_offset = (
            gripper_xyz[0] - plug_xyz[0],
            gripper_xyz[1] - plug_xyz[1],
            gripper_xyz[2] - plug_xyz[2],
        )

        tip_x_error = port_xy[0] - plug_xyz[0]
        tip_y_error = port_xy[1] - plug_xyz[1]

        if reset_xy_integrator:
            self._tip_x_error_integrator = 0.0
            self._tip_y_error_integrator = 0.0
        else:
            self._tip_x_error_integrator = self._np.clip(
                self._tip_x_error_integrator + tip_x_error,
                -self.scoring_tf_max_integrator,
                self.scoring_tf_max_integrator,
            )
            self._tip_y_error_integrator = self._np.clip(
                self._tip_y_error_integrator + tip_y_error,
                -self.scoring_tf_max_integrator,
                self.scoring_tf_max_integrator,
            )

        target_x = (
            port_xy[0]
            + lateral_offset_xy[0]
            + self.scoring_tf_i_gain * self._tip_x_error_integrator
        )
        target_y = (
            port_xy[1]
            + lateral_offset_xy[1]
            + self.scoring_tf_i_gain * self._tip_y_error_integrator
        )
        target_z = float(tip_target_xyz[2]) - plug_tip_gripper_offset[2]

        blend_xyz = (
            position_fraction * target_x + (1.0 - position_fraction) * gripper_xyz[0],
            position_fraction * target_y + (1.0 - position_fraction) * gripper_xyz[1],
            position_fraction * target_z + (1.0 - position_fraction) * gripper_xyz[2],
        )

        return Pose(
            position=Point(
                x=blend_xyz[0],
                y=blend_xyz[1],
                z=blend_xyz[2],
            ),
            orientation=Quaternion(
                w=q_gripper_slerp[0],
                x=q_gripper_slerp[1],
                y=q_gripper_slerp[2],
                z=q_gripper_slerp[3],
            ),
        )

    def _lookup_insertion_axis(self, port_frame: str, port_transform) -> tuple[Any, str]:
        entrance_frame = f"{port_frame}_entrance"
        try:
            entrance_tf_stamped = self._parent_node._tf_buffer.lookup_transform(
                "base_link",
                entrance_frame,
                Time(),
            )
            port_xyz = self._np.array(
                [
                    port_transform.translation.x,
                    port_transform.translation.y,
                    port_transform.translation.z,
                ],
                dtype=self._np.float64,
            )
            entrance_xyz = self._np.array(
                [
                    entrance_tf_stamped.transform.translation.x,
                    entrance_tf_stamped.transform.translation.y,
                    entrance_tf_stamped.transform.translation.z,
                ],
                dtype=self._np.float64,
            )
            insertion_axis = port_xyz - entrance_xyz
            norm = float(self._np.linalg.norm(insertion_axis))
            if norm >= 1e-6:
                return insertion_axis / norm, entrance_frame
        except TransformException:
            pass

        fallback_axis = self._rotation_matrix_from_quaternion(port_transform.rotation)[:, 2]
        norm = float(self._np.linalg.norm(fallback_axis))
        if norm < 1e-6:
            return self._np.array([0.0, 0.0, 1.0], dtype=self._np.float64), port_frame
        return fallback_axis / norm, port_frame

    def _insertion_event_matches(self, task: Task) -> bool:
        if not self._last_insertion_event:
            return False
        event = self._last_insertion_event
        return event.endswith(f"{task.target_module_name}/{task.port_name}") or event.endswith(
            task.port_name
        )

    def _shape_progress(self, fraction: float) -> float:
        progress = float(self._np.clip(fraction, 0.0, 1.0))
        if not self.scoring_tf_smoothstep:
            return progress
        return progress * progress * (3.0 - 2.0 * progress)

    def _scoring_tf_gains(self, phase: str) -> tuple[list[float], list[float]]:
        if phase == "approach":
            stiffness_xyz = self.scoring_tf_approach_stiffness_xyz
            stiffness_rot = self.scoring_tf_approach_stiffness_rot
            damping_xyz = self.scoring_tf_approach_damping_xyz
            damping_rot = self.scoring_tf_approach_damping_rot
        elif phase == "hold":
            stiffness_xyz = self.scoring_tf_hold_stiffness_xyz
            stiffness_rot = self.scoring_tf_hold_stiffness_rot
            damping_xyz = self.scoring_tf_hold_damping_xyz
            damping_rot = self.scoring_tf_hold_damping_rot
        else:
            stiffness_xyz = self.scoring_tf_descend_stiffness_xyz
            stiffness_rot = self.scoring_tf_descend_stiffness_rot
            damping_xyz = self.scoring_tf_descend_damping_xyz
            damping_rot = self.scoring_tf_descend_damping_rot

        stiffness = [stiffness_xyz, stiffness_xyz, stiffness_xyz]
        stiffness.extend([stiffness_rot, stiffness_rot, stiffness_rot])
        damping = [damping_xyz, damping_xyz, damping_xyz]
        damping.extend([damping_rot, damping_rot, damping_rot])
        return stiffness, damping

    def _run_scoring_tf_lateral_search(
        self,
        task: Task,
        move_robot: MoveRobotCallback,
        send_feedback: SendFeedbackCallback,
        end_time,
        wall_deadline: float | None,
        plug_frame: str,
        port_transform,
        insertion_axis,
    ) -> bool:
        offsets = [
            (0.0, 0.004),
            (0.0, -0.004),
            (0.004, 0.0),
            (-0.004, 0.0),
            (0.006, 0.006),
            (0.006, -0.006),
            (-0.006, 0.006),
            (-0.006, -0.006),
            (0.0, 0.010),
            (0.0, -0.010),
            (0.010, 0.0),
            (-0.010, 0.0),
        ]
        z_offsets = [
            0.012,
            0.006,
            0.0,
            min(-0.008, self.scoring_tf_final_offset * 0.4),
            self.scoring_tf_final_offset,
        ]
        command_sleep = max(0.08, self.scoring_tf_sleep_seconds * 2.0)
        hold_period = max(0.20, self.scoring_tf_hold_command_period)

        self.get_logger().info("Scoring TF lateral recovery search starting")
        for offset_idx, lateral_offset in enumerate(offsets, start=1):
            if self._deadline_exceeded(end_time, wall_deadline):
                return False
            send_feedback(
                f"Scoring TF recovery offset {offset_idx}/{len(offsets)} "
                f"dx={lateral_offset[0]:.4f} dy={lateral_offset[1]:.4f}"
            )
            self.get_logger().info(
                "Scoring TF recovery "
                f"offset={offset_idx}/{len(offsets)} "
                f"dx={lateral_offset[0]:.4f} dy={lateral_offset[1]:.4f}"
            )

            for z_offset in z_offsets:
                if self._deadline_exceeded(end_time, wall_deadline):
                    return False
                if self._insertion_event_matches(task):
                    self.get_logger().info(
                        f"Scoring TF recovery insertion event matched: {self._last_insertion_event}"
                    )
                    return True
                try:
                    stiffness, damping = self._scoring_tf_gains("descent")
                    self.set_pose_target(
                        move_robot=move_robot,
                        pose=self._calc_gripper_pose(
                            plug_frame=plug_frame,
                            port_transform=port_transform,
                            insertion_axis=insertion_axis,
                            z_offset=z_offset,
                            lateral_offset_xy=lateral_offset,
                            reset_xy_integrator=z_offset == z_offsets[0],
                        ),
                        stiffness=stiffness,
                        damping=damping,
                    )
                except TransformException as exc:
                    self.get_logger().warn(f"Scoring TF recovery failed: {exc}")
                    return False
                self.sleep_for(command_sleep)

            for _ in range(4):
                if self._deadline_exceeded(end_time, wall_deadline):
                    return False
                if self._insertion_event_matches(task):
                    self.get_logger().info(
                        f"Scoring TF recovery insertion event matched during hold: {self._last_insertion_event}"
                    )
                    return True
                try:
                    stiffness, damping = self._scoring_tf_gains("hold")
                    self.set_pose_target(
                        move_robot=move_robot,
                        pose=self._calc_gripper_pose(
                            plug_frame=plug_frame,
                            port_transform=port_transform,
                            insertion_axis=insertion_axis,
                            z_offset=self.scoring_tf_final_offset,
                            lateral_offset_xy=lateral_offset,
                        ),
                        stiffness=stiffness,
                        damping=damping,
                    )
                except TransformException as exc:
                    self.get_logger().warn(f"Scoring TF recovery hold failed: {exc}")
                    return False
                self.sleep_for(hold_period)

        return False

    def _run_scoring_tf_insertion(
        self,
        task: Task,
        move_robot: MoveRobotCallback,
        send_feedback: SendFeedbackCallback,
        end_time,
        wall_deadline: float | None,
    ) -> bool:
        if not self.enable_scoring_tf:
            return False

        port_frame = f"task_board/{task.target_module_name}/{task.port_name}_link"
        plug_frame = f"{task.cable_name}/{task.plug_name}_link"
        self._last_insertion_event = ""

        remaining_seconds = max(
            0.0, float((end_time - self.time_now()).nanoseconds) / 1e9
        )
        wait_timeout = min(self.scoring_tf_wait_seconds, remaining_seconds)
        fresh_since = time.monotonic()
        for frame in (port_frame, plug_frame):
            if not self._wait_for_fresh_scoring_tf(
                "base_link", frame, wait_timeout, fresh_since
            ):
                return False
        if not self._wait_for_tf("base_link", "gripper/tcp", wait_timeout):
            return False

        port_tf_stamped = self._parent_node._tf_buffer.lookup_transform(
            "base_link",
            port_frame,
            Time(),
        )
        port_transform = port_tf_stamped.transform
        insertion_axis, axis_source_frame = self._lookup_insertion_axis(
            port_frame, port_transform
        )

        send_feedback(f"Scoring TF ready for {task.port_name}")
        self.get_logger().info(
            f"Scoring TF insertion start port_frame={port_frame} plug_frame={plug_frame}"
        )
        self.get_logger().info(
            "Scoring TF insertion axis "
            f"source={axis_source_frame} axis="
            f"({insertion_axis[0]:.4f}, {insertion_axis[1]:.4f}, {insertion_axis[2]:.4f})"
        )

        interp_steps = max(10, self.scoring_tf_interp_steps)
        for step_idx in range(interp_steps):
            if self._deadline_exceeded(end_time, wall_deadline):
                return False
            interp_fraction = self._shape_progress(float(step_idx + 1) / float(interp_steps))
            stiffness, damping = self._scoring_tf_gains("approach")
            try:
                self.set_pose_target(
                    move_robot=move_robot,
                    pose=self._calc_gripper_pose(
                        plug_frame=plug_frame,
                        port_transform=port_transform,
                        insertion_axis=insertion_axis,
                        slerp_fraction=interp_fraction,
                        position_fraction=interp_fraction,
                        z_offset=self.scoring_tf_approach_offset,
                        reset_xy_integrator=step_idx == 0,
                    ),
                    stiffness=stiffness,
                    damping=damping,
                )
            except TransformException as exc:
                self.get_logger().warn(f"Scoring TF interpolation failed: {exc}")
                return False

            if step_idx % 10 == 0:
                self.get_logger().info(
                    "Scoring TF approach "
                    f"step={step_idx + 1}/{interp_steps} "
                    f"axis_offset={self.scoring_tf_approach_offset:.4f}"
                )
                send_feedback(
                    f"Scoring TF approach {step_idx + 1}/{interp_steps} for {task.port_name}"
                )
            self.sleep_for(self.scoring_tf_sleep_seconds)

        descent_step_size = max(abs(self.scoring_tf_descend_step), 1e-4)
        descent_distance = abs(self.scoring_tf_approach_offset - self.scoring_tf_final_offset)
        descent_steps_total = max(1, int(self._np.ceil(descent_distance / descent_step_size)))
        for descent_steps in range(descent_steps_total + 1):
            if self._deadline_exceeded(end_time, wall_deadline):
                return False
            if self._insertion_event_matches(task):
                self.get_logger().info(
                    f"Scoring TF insertion event matched: {self._last_insertion_event}"
                )
                return True

            z_progress = self._shape_progress(float(descent_steps) / float(descent_steps_total))
            z_offset = self.scoring_tf_approach_offset + (
                self.scoring_tf_final_offset - self.scoring_tf_approach_offset
            ) * z_progress
            stiffness, damping = self._scoring_tf_gains("descent")

            try:
                self.set_pose_target(
                    move_robot=move_robot,
                    pose=self._calc_gripper_pose(
                        plug_frame=plug_frame,
                        port_transform=port_transform,
                        insertion_axis=insertion_axis,
                        z_offset=z_offset,
                    ),
                    stiffness=stiffness,
                    damping=damping,
                )
            except TransformException as exc:
                self.get_logger().warn(f"Scoring TF descent failed: {exc}")
                return False

            if descent_steps % 10 == 0:
                self.get_logger().info(
                    "Scoring TF descend "
                    f"step={descent_steps + 1} axis_offset={z_offset:.4f} "
                    f"event={self._last_insertion_event or 'none'}"
                )
                send_feedback(
                    f"Scoring TF descend z_offset={z_offset:.4f} event={self._last_insertion_event or 'none'}"
                )

            if descent_steps < descent_steps_total:
                self.sleep_for(self.scoring_tf_sleep_seconds)

        hold_seconds = max(self.visual_push_settle_seconds, self.scoring_tf_hold_seconds)
        hold_period = max(self.scoring_tf_sleep_seconds, self.scoring_tf_hold_command_period)
        hold_until = min(
            end_time,
            self.time_now() + Duration(seconds=hold_seconds),
        )
        hold_steps = 0
        while self.time_now() < hold_until and not self._wall_time_exceeded(wall_deadline):
            if self._insertion_event_matches(task):
                self.get_logger().info(
                    f"Scoring TF insertion event matched during hold: {self._last_insertion_event}"
                )
                return True

            try:
                stiffness, damping = self._scoring_tf_gains("hold")
                self.set_pose_target(
                    move_robot=move_robot,
                    pose=self._calc_gripper_pose(
                        plug_frame=plug_frame,
                        port_transform=port_transform,
                        insertion_axis=insertion_axis,
                        z_offset=self.scoring_tf_final_offset,
                    ),
                    stiffness=stiffness,
                    damping=damping,
                )
            except TransformException as exc:
                self.get_logger().warn(f"Scoring TF hold failed: {exc}")
                return False

            if hold_steps % max(1, int(round(1.0 / hold_period))) == 0:
                self.get_logger().info(
                    "Scoring TF hold "
                    f"step={hold_steps + 1} axis_offset={self.scoring_tf_final_offset:.4f} "
                    f"event={self._last_insertion_event or 'none'}"
                )
                send_feedback(
                    f"Scoring TF hold axis_offset={self.scoring_tf_final_offset:.4f} event={self._last_insertion_event or 'none'}"
                )

            hold_steps += 1
            self.sleep_for(hold_period)

        if self._last_insertion_event:
            self.get_logger().info(
                f"Scoring TF insertion ended with non-matching event: {self._last_insertion_event}"
            )
        if self.scoring_tf_search_on_miss and self._run_scoring_tf_lateral_search(
            task=task,
            move_robot=move_robot,
            send_feedback=send_feedback,
            end_time=end_time,
            wall_deadline=wall_deadline,
            plug_frame=plug_frame,
            port_transform=port_transform,
            insertion_axis=insertion_axis,
        ):
            return True
        if self.scoring_tf_complete_after_hold:
            self.get_logger().info(
                "Scoring TF hold completed without an insertion event; "
                "returning success so the evaluator can score the final physical state"
            )
            return True
        return False

    def _run_sfp_visual_servo(
        self,
        task: Task,
        get_observation: GetObservationCallback,
        move_robot: MoveRobotCallback,
        send_feedback: SendFeedbackCallback,
        end_time,
        wall_deadline: float | None,
    ) -> bool:
        loop_period_seconds = max(0.05, 1.0 / max(self.loop_hz, 1.0))
        frame_name = "center_camera/optical"
        push_start = None
        target_lost_frames = 0
        sent_commands = 0
        success = False

        self.get_logger().info("Starting SFP vision servo fallback")
        while self.time_now() < end_time and not self._wall_time_exceeded(wall_deadline):
            if self._insertion_event_matches(task):
                self.get_logger().info(
                    f"SFP vision observed insertion event: {self._last_insertion_event}"
                )
                success = True
                break

            observation_msg = get_observation()
            if observation_msg is None:
                self.sleep_for(loop_period_seconds)
                continue

            image_np = self._raw_image_to_np(observation_msg.center_image)
            plug_reference = self._detect_sfp_plug_reference(image_np)
            port_centers = self._detect_sfp_ports(image_np)
            if len(port_centers) < 2:
                if push_start is not None:
                    target_lost_frames += 1
                    if target_lost_frames >= self.visual_target_lost_frames:
                        success = True
                        break
                if sent_commands % max(1, int(self.loop_hz)) == 0:
                    send_feedback("SFP vision: waiting for socket detection")
                self.sleep_for(loop_period_seconds)
                continue

            target_lost_frames = 0
            target_u, target_v = self._choose_sfp_port(task, port_centers)
            if plug_reference is not None:
                desired_u = plug_reference[0]
                desired_v = plug_reference[1] - self.visual_sfp_port_gap_px
            else:
                desired_u, desired_v = self._desired_port_pixel(
                    task, observation_msg.center_camera_info
                )
            fx, fy, _, _ = self._camera_intrinsics(observation_msg.center_camera_info)
            error_u = target_u - desired_u
            error_v = target_v - desired_v
            aligned = (
                abs(error_u) <= self.visual_alignment_px
                and abs(error_v) <= self.visual_alignment_px
            )

            if aligned:
                if push_start is None:
                    push_start = self.time_now()
            else:
                push_start = None

            vx = self.visual_lateral_gain * (error_u / max(fx, 1.0))
            vy = self.visual_lateral_gain * (error_v / max(fy, 1.0))
            if abs(error_u) < self.visual_deadband_px:
                vx = 0.0
            if abs(error_v) < self.visual_deadband_px:
                vy = 0.0

            vz = self.visual_push_speed if aligned else 0.0
            force_mag = self._force_magnitude(observation_msg)
            if force_mag >= self.visual_force_guard_newtons:
                vz = -self.visual_backoff_speed

            if push_start is not None:
                vx *= 0.5
                vy *= 0.5

            camera_velocity = self._np.array(
                [
                    self._np.clip(vx, -self.visual_max_speed, self.visual_max_speed),
                    self._np.clip(vy, -self.visual_max_speed, self.visual_max_speed),
                    self._np.clip(vz, -self.visual_max_speed, self.visual_max_speed),
                ],
                dtype=self._np.float64,
            )
            try:
                base_velocity = self._camera_velocity_to_base(frame_name, camera_velocity)
            except TransformException as exc:
                self.get_logger().warn(f"SFP vision TF unavailable: {exc}")
                self.sleep_for(loop_period_seconds)
                continue

            move_robot(
                motion_update=self._build_twist_target(
                    [base_velocity[0], base_velocity[1], base_velocity[2], 0.0, 0.0, 0.0]
                )
            )
            sent_commands += 1

            if sent_commands % max(1, int(self.loop_hz)) == 0:
                feedback = (
                    f"SFP vision target=({target_u:.0f},{target_v:.0f}) "
                    f"desired=({desired_u:.0f},{desired_v:.0f}) "
                    f"err=({error_u:.0f},{error_v:.0f}) force={force_mag:.1f}N "
                    f"plug_ref={'yes' if plug_reference is not None else 'no'}"
                )
                send_feedback(feedback)
                self.get_logger().info(feedback)

            if (
                push_start is not None
                and (self.time_now() - push_start)
                >= Duration(seconds=self.visual_push_settle_seconds)
            ):
                success = True
                break

            self.sleep_for(loop_period_seconds)

        move_robot(motion_update=self._build_twist_target([0.0] * 6))
        self.get_logger().info(
            "SFP vision servo finished "
            f"success={success} event={self._last_insertion_event or 'none'}"
        )
        return success

    def _run_act_policy(
        self,
        task: Task,
        get_observation: GetObservationCallback,
        move_robot: MoveRobotCallback,
        send_feedback: SendFeedbackCallback,
        end_time,
        wall_deadline: float | None,
    ) -> bool:
        if not self._act_ready:
            return False

        self.policy.reset()
        loop_period_seconds = max(0.05, 1.0 / max(self.loop_hz, 1.0))
        sent_commands = 0

        self.get_logger().info("Starting ACT fallback policy")
        while self.time_now() < end_time and not self._wall_time_exceeded(wall_deadline):
            if self._insertion_event_matches(task):
                self.get_logger().info(
                    f"ACT fallback observed insertion event: {self._last_insertion_event}"
                )
                move_robot(motion_update=self._build_twist_target([0.0] * 6))
                return True

            observation_msg = get_observation()
            if observation_msg is None:
                self.sleep_for(loop_period_seconds)
                continue

            obs_tensors = self.prepare_observations(observation_msg)
            with self._torch.inference_mode():
                normalized_action = self.policy.select_action(obs_tensors)

            raw_action_tensor = (normalized_action * self.action_std) + self.action_mean
            action = raw_action_tensor[0].detach().cpu().numpy()
            move_robot(motion_update=self._build_twist_target(action))
            sent_commands += 1

            if sent_commands % max(1, int(self.loop_hz)) == 0:
                elapsed = max(
                    0.0,
                    self.max_task_seconds
                    - max(0.0, (end_time - self.time_now()).nanoseconds / 1e9),
                )
                send_feedback(
                    f"SubmissionACT fallback running at t={elapsed:.1f}s, commands={sent_commands}"
                )
                self.get_logger().info(
                    f"SubmissionACT fallback step={sent_commands} elapsed={elapsed:.1f}s"
                )

            self.sleep_for(loop_period_seconds)

        move_robot(motion_update=self._build_twist_target([0.0] * 6))
        self.get_logger().info(
            "ACT fallback finished "
            f"commands={sent_commands} event={self._last_insertion_event or 'none'}"
        )
        return sent_commands > 0

    def insert_cable(
        self,
        task: Task,
        get_observation: GetObservationCallback,
        move_robot: MoveRobotCallback,
        send_feedback: SendFeedbackCallback,
        **kwargs,
    ):
        del kwargs
        raw_task_limit_seconds = float(task.time_limit) if task.time_limit else 30.0
        time_limit_seconds = raw_task_limit_seconds
        if self.max_task_seconds > 0.0:
            time_limit_seconds = min(time_limit_seconds, self.max_task_seconds)
        budget_seconds = max(1.0, time_limit_seconds - self.completion_guard_seconds)
        start_time = self.time_now()
        end_time = start_time + Duration(seconds=budget_seconds)
        wall_deadline = self._wall_deadline()

        self.get_logger().info(
            "SubmissionACT.insert_cable() "
            f"task={task.id} task_limit={raw_task_limit_seconds:.2f}s "
            f"effective_limit={time_limit_seconds:.2f}s budget={budget_seconds:.2f}s"
        )
        if wall_deadline is not None:
            self.get_logger().info(
                f"SubmissionACT wall-clock guard enabled at {self.max_wall_seconds:.2f}s"
            )

        if not self._deadline_exceeded(end_time, wall_deadline):
            if self._run_scoring_tf_insertion(
                task=task,
                move_robot=move_robot,
                send_feedback=send_feedback,
                end_time=end_time,
                wall_deadline=wall_deadline,
            ):
                self.get_logger().info("SubmissionACT scoring TF insertion completed")
                return True
            if self._insertion_event_matches(task):
                self.get_logger().info(
                    "SubmissionACT detected insertion event after TF stage: "
                    f"{self._last_insertion_event}"
                )
                return True
            self.get_logger().info("SubmissionACT scoring TF stage ended without success")

        if self.enable_vision and task.plug_type == "sfp" and task.port_type == "sfp":
            if self._run_sfp_visual_servo(
                task=task,
                get_observation=get_observation,
                move_robot=move_robot,
                send_feedback=send_feedback,
                end_time=end_time,
                wall_deadline=wall_deadline,
            ):
                self.get_logger().info("SubmissionACT SFP vision servo completed")
                return True
            self.get_logger().info("SubmissionACT SFP vision fallback ended without success")

        if self.enable_act and self._act_ready and not self._deadline_exceeded(end_time, wall_deadline):
            if self._run_act_policy(
                task=task,
                get_observation=get_observation,
                move_robot=move_robot,
                send_feedback=send_feedback,
                end_time=end_time,
                wall_deadline=wall_deadline,
            ):
                self.get_logger().info("SubmissionACT ACT fallback completed")
                return True
            self.get_logger().info("SubmissionACT ACT fallback ended without success")

        if self._wall_time_exceeded(wall_deadline):
            self.get_logger().warn(
                "SubmissionACT wall-clock guard fired before a successful policy pass"
            )

        move_robot(motion_update=self._build_twist_target([0.0] * 6))
        self.get_logger().warn("SubmissionACT exiting without issuing a successful policy pass")
        return False
