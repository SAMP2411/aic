# Local Submission Workflow

This repository now includes a submission-oriented ACT policy and two compose flows:

- `docker/docker-compose.submission-cpu.yaml` for headless local evaluation
- `docker/docker-compose.submission-gui.yaml` as an overlay for Gazebo GUI
- `scripts/capture_eval_cameras.sh` for grabbing live left / center / right camera PNGs
- `scripts/run_submission_smoke.sh` for a one-trial local smoke run that writes
  `scoring.yaml` much sooner than the full three-trial qualification config

## What this covers

This workflow prepares and validates the **Qualification** submission path from the public toolkit:

- builds a participant image with a ROS 2 lifecycle node named `aic_model`
- pre-fetches the ACT checkpoint into the image for faster startup
- pre-warms Torch vision weights into a host cache mounted at runtime
- runs the official `aic_eval` image locally with the standard discovery/configure timeouts
- saves logs, rendered compose config, and `scoring.yaml` under `runs/<run_id>/`

> The public toolkit does **not** include private Flowstate access for the challenge's later Phase 1. That part only becomes available after qualification.

## Headless run

```bash
./scripts/run_submission_eval.sh
```

If you already have a working `my-solution:submission-act` image locally and only
want to iterate on runtime behavior, skip the rebuild step:

```bash
AIC_SKIP_BUILD=1 ./scripts/run_submission_eval.sh
```

By default, the local ACT policy caps its control loop to 30 simulated seconds
to match the reference `RunACT` behavior and keep local iterations practical.
Override that only if you explicitly want a longer local task budget:

```bash
AIC_ACT_MAX_TASK_SECONDS=60 ./scripts/run_submission_eval.sh
```

This creates a timestamped directory under `runs/` containing:

- `pull.log`
- `prewarm.log`
- `build.log`
- `compose.log`
- `compose.rendered.yaml`
- `model-image.inspect.json`
- `results/scoring.yaml`

The first run also creates a reusable Torch cache under `.cache/torch-cache/`.

## GUI run

Allow the container to use your X server:

```bash
xhost +local:root
```

Then start the GUI evaluation:

```bash
./scripts/run_submission_eval_gui.sh
```

For faster local iteration with an already-built image:

```bash
AIC_SKIP_BUILD=1 ./scripts/run_submission_eval_gui.sh
```

The GUI overlay defaults to Mesa software rendering (`llvmpipe`) because this
machine does not currently have the NVIDIA container runtime configured. If you
later install the NVIDIA container toolkit and want to try hardware rendering,
override the GUI defaults:

```bash
AIC_EVAL_SOFTWARE_RENDERING=0 AIC_EVAL_MESA_DRIVER= ./scripts/run_submission_eval_gui.sh
```

When finished, revoke the temporary X11 permission:

```bash
xhost -local:root
```

## Score summary

```bash
./scripts/print_score_summary.py runs/<run_id>/results/scoring.yaml
```

## Single-trial smoke run

For fast local validation, run the ACT image against a single-trial config with a
short local task budget:

```bash
./scripts/run_submission_smoke.sh
```

Defaults used by the smoke helper:

- `AIC_SKIP_BUILD=1`
- `AIC_ACT_MAX_TASK_SECONDS=2`
- `AIC_ACT_LOOP_HZ=1`
- `aic_engine_config_file:=/aic_engine_config/local_smoke_trial_1.yaml`

## Best-known tuned profile

The current best local scored result is `94.88897069758184` from
`runs/tf_smoother_02`, using a shaped TF-guided insertion profile with more
interpolation and smaller descent increments. That profile preserved reliable
completion while improving smoothness to `41.86 m/s^3` jerk.

Headless scored validation:

```bash
./scripts/run_submission_best_smoke.sh
```

Visual Gazebo run with the same tuned controller profile:

```bash
xhost +local:root
./scripts/run_submission_best_gui.sh
xhost -local:root
```

Faster local GUI run using host DRI acceleration instead of forced `llvmpipe`:

```bash
xhost +local:root
./scripts/run_submission_best_gui_hw.sh
xhost -local:root
```

Dedicated NVIDIA GPU GUI run:

```bash
xhost +local:root
./scripts/run_submission_best_gui_nvidia.sh
xhost -local:root
```

Use:

```bash
./scripts/check_nvidia_container_support.sh
```

to verify whether Docker can access the NVIDIA GPU yet. If it reports success,
`run_submission_best_gui_nvidia.sh` is the preferred visual path.

The tuned helper scripts default to:

- `AIC_SUBMISSION_STRATEGY=vision`
- `AIC_ACT_MAX_TASK_SECONDS=175`
- `AIC_SCORING_TF_SMOOTHSTEP=1`
- `AIC_SCORING_TF_DESCEND_STEP=0.0012`
- `AIC_SCORING_TF_INTERP_STEPS=32`
- `AIC_SCORING_TF_SLEEP_SECONDS=0.035`
- `AIC_SCORING_TF_I_GAIN=0.06`
- `AIC_SCORING_TF_MAX_INTEGRATOR=0.015`

Note: the `3.00s` or `5.00s` reported in `scoring.yaml` are the scored task
durations. The much longer local runtime you may observe on a weaker laptop is
wall-clock simulation time under software-rendered Gazebo.

## Profile-based tuning

Run a named smoke profile without retyping all tuning env vars:

```bash
./scripts/run_submission_profile_smoke.sh my_run tf_smooth
./scripts/run_submission_profile_smoke.sh my_run tf_smoother
./scripts/run_submission_profile_smoke.sh my_run_ml vision_then_act
```

Run the same named profiles visually in Gazebo:

```bash
xhost +local:root
./scripts/run_submission_profile_gui.sh my_gui tf_smooth
./scripts/run_submission_profile_gui.sh my_gui tf_smoother
xhost -local:root
```

Available profiles:

- `tf_smooth`: older stable TF-guided baseline
- `tf_smoother`: current best-known shaped-motion TF-guided profile
- `tf_gentle`: lower-force / gentler descent candidate
- `tf_fast`: faster / more aggressive descent candidate
- `vision_then_act`: `tf_smoother` plus ACT fallback enabled

Sweep multiple profiles and collect a summary table:

```bash
./scripts/tune_submission_profiles.sh
```

## ML and RL training launchers

ACT / LeRobot training example:

```bash
AIC_LEROBOT_DATASET_REPO=your-hf-user/your_dataset \
AIC_LEROBOT_POLICY_REPO=your-hf-user/aic_act_policy \
./scripts/train_act_lerobot_example.sh
```

Isaac Lab RSL-RL training example
This must be run inside the Isaac Lab container:

```bash
AIC_ISAAC_NUM_ENVS=64 \
./scripts/train_isaac_rsl_rl_example.sh
```

## Camera snapshots

Capture the current left / center / right camera views from a live evaluation:

```bash
./scripts/capture_eval_cameras.sh runs/<run_id>/snapshots
```

## Manual cache warmup

If you want to pre-warm the Torch cache separately from a run:

```bash
./scripts/prewarm_submission_cache.sh
```

## Useful debug override

For local debugging only, you can temporarily enable ground truth transforms:

```bash
AIC_EVAL_COMMAND="gazebo_gui:=true launch_rviz:=false ground_truth:=true start_aic_engine:=true shutdown_on_aic_engine_exit:=true model_discovery_timeout_seconds:=30 model_configure_timeout_seconds:=60" \
./scripts/run_submission_eval_gui.sh
```

Do not use `ground_truth:=true` for an actual submission image or compliance check.
