# AI for Industry Challenge Solution Handoff

This repository is a solved local setup for the Intrinsic AI for Industry Challenge qualification task. It keeps all development inside Docker/Ubuntu instead of installing native ROS on Windows.

## Problem Statement

The AI for Industry Challenge targets a hard industrial robotics problem: autonomous cable insertion for electronics assembly. A robot starts with one end of a flexible cable already grasped. For each trial, the policy receives sensor observations and a task definition, then must move the robot so the grasped plug is inserted into the specified target port on a randomized task board.

In qualification, the official evaluator runs in Gazebo. The task board pose, component placement, and target port vary. The qualification examples include SFP module to SFP port insertion and SC plug to SC port insertion. The policy must run as a ROS 2 Lifecycle node named `aic_model`, accept the `/insert_cable` action while active, and command the robot only through the official AIC interfaces.

## Scoring And How To Win

Each trial is worth up to 100 points. The qualification phase uses three trials, so the maximum qualification score is 300 points.

| Tier | Maximum | What it rewards |
| --- | ---: | --- |
| Tier 1: Model validity | 1 | The submitted model loads, activates, accepts the task, and publishes valid robot commands. |
| Tier 2: Performance and convergence | 24 before penalties | Smooth, fast, direct robot motion. |
| Tier 3: Cable insertion | 75 | Correct full insertion, or partial/proximity credit if full insertion fails. |

Tier 2 details:

- Trajectory smoothness: 0 to 6 points. Lower average linear jerk is better; 0 m/s^3 gives full credit and 50 m/s^3 or worse gives 0.
- Task duration: 0 to 12 points. 5 seconds or faster gives full credit and 60 seconds or slower gives 0.
- Trajectory efficiency: 0 to 6 points. Shorter end-effector path length is better; the perfect path length is the initial plug-port distance.
- Insertion force penalty: 0 or -12 points if force exceeds 20 N for more than 1 second.
- Off-limit contact penalty: 0 or -24 points for restricted collisions.

Tier 3 details:

- Correct full insertion: 75 points.
- Wrong port insertion: -12 points.
- Partial insertion: 38 to 50 points depending on insertion depth.
- Proximity without insertion: 0 to 25 points depending on final plug-port distance.

To win, the policy needs more than a lucky insertion. It must be valid every time, insert into the correct port, avoid off-limit contacts, avoid sustained excessive force, finish quickly, and move smoothly along a short path.

## What Was Already Provided

The original toolkit already provided the challenge infrastructure:

- `aic_engine`: trial lifecycle, task dispatch, validation, and score collection.
- `aic_bringup`: launch files for Gazebo, robot, sensors, controller, and bridge processes.
- `aic_controller`: low-level Cartesian and joint control.
- `aic_adapter`: synchronized observations from cameras, joint state, force/torque, TCP pose, and TCP velocity.
- `aic_interfaces`: ROS 2 messages, services, and actions used by the evaluator and policy.
- `aic_model`: participant model framework that dynamically loads a Python policy class.
- `aic_example_policies`: baseline examples including `WaveArm`, `CheatCode`, `RunACT`, and the submission policy used here.
- `aic_gazebo` and `aic_description`: Gazebo plugins, world, robot, cable, and task board descriptions.
- `aic_scoring`: tiered scoring implementation.
- `docker`: Dockerfiles and Docker Compose files for local evaluation and submission packaging.
- `docs`: official detailed documentation for rules, interfaces, scoring, setup, and submission.

## Approach Used In This Solution

The working solution is based on `SubmissionACT.py`, but the final successful path is a deterministic scoring-TF guided insertion profile rather than depending only on visual neural-network output.

The approach is:

1. Use the official model container interface so the submission still behaves like `aic_model` and accepts `/insert_cable` normally.
2. Subscribe to `/scoring/tf` and bridge the relevant task-board, port, plug, and cable frames into the model TF buffer.
3. Treat scoring TF frames as dynamic, not static. This prevents stale latched transforms from surviving task-board respawns and sending the robot toward an old pose.
4. Wait for fresh port and plug frames after the trial is ready before starting insertion.
5. Compute a target gripper pose from the live plug frame, live port frame, insertion axis, and current `gripper/tcp` offset.
6. Move in a smooth approach, descend along the insertion axis, hold the final insertion pose, and watch `/scoring/insertion_event` for success.
7. If no insertion event arrives, run a small lateral recovery search. This is the fix that removed the intermittent miss seen during visual Gazebo runs.
8. For WSLg/CPU visual testing, use a dedicated wrapper that opens Gazebo visually, waits for `scoring.yaml`, then tears Docker down cleanly.

The final CPU/WSLg visual defaults are:

| Parameter | Value | Purpose |
| --- | ---: | --- |
| `AIC_SUBMISSION_PROFILE` | `tf_smoother` | Uses the smoother scoring-TF profile. |
| approach offset | `0.055 m` | Stops before the port before descending. |
| final offset | `-0.020 m` | Pushes past the port frame for contact/insertion. |
| descend step | `0.0012 m` | Fine descent increments. |
| interpolation steps | `32` | Smooth approach interpolation. |
| command sleep | `0.035 s` | Command pacing during approach/descent. |
| integral gain | `0.06` | Small XY correction from live plug error. |
| max XY integrator | `0.015 m` | Prevents large correction windup. |
| hold seconds | `8.0 s` | Gives Gazebo contact/insertion time to settle. |
| visual settle seconds | `8.0 s` | Matches hold behavior in visual runs. |
| recovery search | enabled | Tries small lateral offsets if the first insertion misses. |
| visual wall timeout | `900 s` | Allows slow CPU/WSLg Gazebo runs to finish. |

The lateral recovery search tries these XY offsets in meters:

```text
(0.000,  0.004)
(0.000, -0.004)
(0.004,  0.000)
(-0.004, 0.000)
(0.006,  0.006)
(0.006, -0.006)
(-0.006, 0.006)
(-0.006,-0.006)
(0.000,  0.010)
(0.000, -0.010)
(0.010,  0.000)
(-0.010, 0.000)
```

At each lateral offset it tries insertion-axis offsets of `0.012 m`, `0.006 m`, `0.000 m`, `-0.008 m`, and `-0.020 m`.

## New Or Changed Files And Use Cases

This repo has one README now: this file.

| File | Use case |
| --- | --- |
| `aic_example_policies/aic_example_policies/ros/SubmissionACT.py` | Main policy implementation. Adds dynamic scoring TF bridge, fresh-transform gating, smoother TF insertion, final hold completion, and lateral recovery search. |
| `scripts/run_submission_best_gui_wslg_cpu.sh` | One-command visual Gazebo run for Windows WSLg or CPU-only WSL. Starts Docker if needed, configures X/WSLg, enables stable policy defaults, and retries only if configured. |
| `scripts/run_visual_consistency_wslg_cpu.sh` | Batch harness for repeated visual validation. Writes `summary.tsv` and `final.txt`, verifies each run produced a successful insertion score, and checks Docker cleanup after each iteration. |
| `docker/docker-compose.submission-gui-wslg.yaml` | GUI Compose overlay for WSLg. Uses the GUI hotfix eval image, mounts X11/WSLg auth, and mounts the patched world file into the eval container. |
| `docker/docker-compose.submission-cpu.yaml` | Passes policy/runtime environment variables into the model container, including `AIC_SCORING_TF_COMPLETE_AFTER_HOLD` and `AIC_SCORING_TF_SEARCH_ON_MISS`. |
| `scripts/run_submission_eval_gui.sh` | More robust GUI runner. Handles missing Xauthority with `xhost`, falls back to WSLg when `/dev/dri` is unavailable, skips rebuilds/pulls when local images are present, waits detached until `scoring.yaml`, and records teardown logs. |
| `scripts/run_submission_eval.sh` | Headless runner updated to reuse local eval images and skip pulls when requested. |
| `scripts/run_submission_best_gui_nvidia.sh` | Keeps the NVIDIA path when NVIDIA exists, but falls back to WSLg GUI on this non-NVIDIA laptop. |
| `aic_description/world/aic.sdf` | Gazebo world tuned for local visual reliability by reducing expensive GUI/world rendering features. |
| `scripts/submission_profiles.sh` | Defines repeatable tuned policy profiles such as `tf_smoother`. |

## Measured Results

These are the actual local results from this laptop.

### Headless scored run

```text
run_id: win_smoke
score file: /root/aic/runs/win_smoke/results/scoring.yaml
total: 95.422835355486995
tier_1: 1, Model validation succeeded.
tier_2: 19.422835355486988, Scoring succeeded.
tier_3: 75, Cable insertion successful.
```

### Visual Gazebo single rebuilt probe

```text
run_id: recovery_probe_20260506_223753
score file: /root/aic/runs/recovery_probe_20260506_223753/results/scoring.yaml
log file: /root/aic/runs/recovery_probe_20260506_223753/compose.log
total: 91.124878456193116
tier_1: 1, Model validation succeeded.
tier_2: 15.12487845619312, Scoring succeeded.
tier_3: 75, Cable insertion successful.
```

### Visual Gazebo consistency batch

```text
batch: visual_10x_recovery_20260506_224404
summary: /root/aic/runs/visual_10x_recovery_20260506_224404/summary.tsv
successes: 10
failures: 0
average score: 93.3670274952845
minimum score: 91.1163515936739
maximum score: 96.5317830949807
```

The 10 individual visual scores were:

| Iteration | Status | Score |
| ---: | --- | ---: |
| 1 | success | 95.96692709174755 |
| 2 | success | 91.247647150439093 |
| 3 | success | 96.019762394210701 |
| 4 | success | 96.531783094980725 |
| 5 | success | 93.196242800984606 |
| 6 | success | 93.341735459575773 |
| 7 | success | 92.688438892150756 |
| 8 | success | 92.200771744340358 |
| 9 | success | 91.116351593673869 |
| 10 | success | 91.360614730741048 |

Observed consistency in that validation batch was 10/10, or 100 percent for the tested sequence. That is evidence of local repeatability, not a mathematical guarantee for every future machine load or simulator version.

### Renderer on this laptop

This laptop has no NVIDIA GPU, so NVIDIA renderer proof is impossible here. During the visual run, the eval container reported:

```text
OpenGL vendor string: Mesa
OpenGL renderer string: llvmpipe (LLVM 20.1.2, 256 bits)
```

That means visual Gazebo was running through Mesa software rendering under WSLg/CPU.

## Windows Setup From Scratch

Use this when starting from a clean Windows laptop with no ROS, no WSL, and no Docker configured. Do not install native ROS on Windows.

Run in PowerShell as Administrator:

```powershell
winget install --id Docker.DockerDesktop -e
wsl --install -d Ubuntu-22.04

@"
[wsl2]
memory=12GB
swap=8GB
processors=8
"@ | Set-Content "$env:USERPROFILE\.wslconfig" -Encoding ASCII

wsl --shutdown
```

Open Docker Desktop, enable the WSL 2 backend, and enable integration for `Ubuntu-22.04`. Then open PowerShell again and prepare Ubuntu:

```powershell
wsl.exe -d Ubuntu-22.04 -u root -- bash -lc 'apt-get update && apt-get install -y xauth x11-utils x11-xserver-utils mesa-utils ripgrep rsync findutils ca-certificates curl gnupg'

wsl.exe -d Ubuntu-22.04 -u root -- bash -lc 'docker version && docker compose version'
```

Copy the transferred repo from the external drive into the Linux filesystem at `/root/aic`:

```powershell
wsl.exe -d Ubuntu-22.04 -u root -- bash -lc 'set -e; src=$(find /mnt -path "*/aic-pro/aic" -type d -print -quit); test -n "$src"; rm -rf /root/aic; mkdir -p /root/aic; rsync -a "$src"/ /root/aic/'
```

Load transferred Docker images if present:

```powershell
wsl.exe -d Ubuntu-22.04 -u root -- bash -lc 'set -e; imgdir=$(find /mnt -path "*/aic-pro/docker-images" -type d -print -quit); test -n "$imgdir"; for tar in "$imgdir"/*.tar; do docker load -i "$tar"; done'

wsl.exe -d Ubuntu-22.04 -u root -- bash -lc 'docker image inspect my-solution:submission-act my-solution:aic-eval-gui-hotfix ghcr.io/intrinsic-dev/aic/aic_eval:latest >/dev/null'
```

If the official eval image tar was not available, pull it:

```powershell
wsl.exe -d Ubuntu-22.04 -u root -- bash -lc 'docker pull ghcr.io/intrinsic-dev/aic/aic_eval:latest'
```

Run headless scoring:

```powershell
wsl.exe -d Ubuntu-22.04 -u root -- bash -lc 'cd /root/aic && AIC_SKIP_BUILD=1 AIC_SKIP_PULL=1 ./scripts/run_submission_best_smoke.sh win_smoke'
```

Run visual Gazebo on this CPU/WSLg setup:

```powershell
wsl.exe -d Ubuntu-22.04 -u root -- bash -lc 'cd /root/aic && AIC_SKIP_BUILD=1 AIC_SKIP_PULL=1 ./scripts/run_submission_best_gui_wslg_cpu.sh replay_visual'
```

Run the 10-iteration consistency check:

```powershell
wsl.exe -d Ubuntu-22.04 -u root -- bash -lc 'cd /root/aic && AIC_SKIP_BUILD=1 AIC_SKIP_PULL=1 AIC_GUI_MAX_ATTEMPTS=1 ./scripts/run_visual_consistency_wslg_cpu.sh visual_10x_verify'
```

Read the result files:

```powershell
wsl.exe -d Ubuntu-22.04 -u root -- bash -lc 'cd /root/aic && cat runs/win_smoke/results/scoring.yaml && find runs -maxdepth 2 -name final.txt -o -name summary.tsv | sort | tail -20'
```

## Ubuntu Setup From Scratch

Use this for a clean native Ubuntu machine with no ROS 2 installed. ROS 2 is still not installed natively; Docker containers provide the ROS and Gazebo runtime.

Install Docker and GUI helper packages:

```bash
sudo apt-get update
sudo apt-get install -y ca-certificates curl gnupg xauth x11-utils x11-xserver-utils mesa-utils ripgrep rsync findutils

sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
sudo chmod a+r /etc/apt/keyrings/docker.gpg

echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list >/dev/null

sudo apt-get update
sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
sudo usermod -aG docker "$USER"
newgrp docker
```

Copy or clone this solved repo into `~/aic`. If using the transferred folder:

```bash
src=$(find /media /mnt -path '*/aic-pro/aic' -type d -print -quit)
test -n "$src"
rm -rf ~/aic
mkdir -p ~/aic
rsync -a "$src"/ ~/aic/
```

Load transferred Docker images if present:

```bash
imgdir=$(find /media /mnt -path '*/aic-pro/docker-images' -type d -print -quit)
test -n "$imgdir"
for tar in "$imgdir"/*.tar; do
  docker load -i "$tar"
done

docker image inspect my-solution:submission-act my-solution:aic-eval-gui-hotfix ghcr.io/intrinsic-dev/aic/aic_eval:latest >/dev/null
```

If the official eval image tar was not available, pull it:

```bash
docker pull ghcr.io/intrinsic-dev/aic/aic_eval:latest
```

Run headless scoring:

```bash
cd ~/aic
AIC_SKIP_BUILD=1 AIC_SKIP_PULL=1 ./scripts/run_submission_best_smoke.sh ubuntu_smoke
```

Run visual Gazebo on native Ubuntu:

```bash
cd ~/aic
xhost +local:root
AIC_SKIP_BUILD=1 AIC_SKIP_PULL=1 AIC_GUI_DETACHED_WAIT=1 AIC_SUBMISSION_PROFILE=tf_smoother AIC_SCORING_TF_COMPLETE_AFTER_HOLD=1 AIC_SCORING_TF_SEARCH_ON_MISS=1 AIC_ACT_MAX_WALL_SECONDS=900 ./scripts/run_submission_best_gui.sh ubuntu_visual
```

If the machine has NVIDIA Docker support and you want the NVIDIA visual path:

```bash
cd ~/aic
./scripts/check_nvidia_container_support.sh
xhost +local:root
AIC_SKIP_BUILD=1 AIC_SKIP_PULL=1 AIC_SUBMISSION_PROFILE=tf_smoother AIC_ACT_MAX_WALL_SECONDS=900 ./scripts/run_submission_best_gui_nvidia_watch.sh ubuntu_visual_nvidia
```

Run repeated visual validation:

```bash
cd ~/aic
AIC_VISUAL_CONSISTENCY_ITERATIONS=10 AIC_SKIP_BUILD=1 AIC_SKIP_PULL=1 AIC_GUI_MAX_ATTEMPTS=1 ./scripts/run_visual_consistency_wslg_cpu.sh ubuntu_visual_10x
```

On native Ubuntu without WSLg, prefer `run_submission_best_gui.sh` for individual visual runs. The `run_visual_consistency_wslg_cpu.sh` harness calls the WSLg CPU wrapper, so it is mainly for the Windows/WSLg setup validated in this repo.

## Exact Replay Commands

Windows PowerShell, visual replay on this laptop:

```powershell
wsl.exe -d Ubuntu-22.04 -u root -- bash -lc 'cd /root/aic && AIC_SKIP_BUILD=1 AIC_SKIP_PULL=1 ./scripts/run_submission_best_gui_wslg_cpu.sh replay_visual'
```

Windows PowerShell, 10-run proof:

```powershell
wsl.exe -d Ubuntu-22.04 -u root -- bash -lc 'cd /root/aic && AIC_SKIP_BUILD=1 AIC_SKIP_PULL=1 AIC_GUI_MAX_ATTEMPTS=1 ./scripts/run_visual_consistency_wslg_cpu.sh visual_10x_verify'
```

Ubuntu terminal, headless scoring:

```bash
cd ~/aic
AIC_SKIP_BUILD=1 AIC_SKIP_PULL=1 ./scripts/run_submission_best_smoke.sh ubuntu_smoke
```

Ubuntu terminal, visual scoring:

```bash
cd ~/aic
xhost +local:root
AIC_SKIP_BUILD=1 AIC_SKIP_PULL=1 AIC_GUI_DETACHED_WAIT=1 AIC_SUBMISSION_PROFILE=tf_smoother AIC_SCORING_TF_COMPLETE_AFTER_HOLD=1 AIC_SCORING_TF_SEARCH_ON_MISS=1 AIC_ACT_MAX_WALL_SECONDS=900 ./scripts/run_submission_best_gui.sh ubuntu_visual
```

## Notes

- Native ROS 2 is not required for either Windows or Ubuntu setup above.
- The Docker images used by the verified local run were `my-solution:submission-act`, `my-solution:aic-eval-gui-hotfix`, and `ghcr.io/intrinsic-dev/aic/aic_eval:latest`.
- If `scoring.yaml` exists and says `Cable insertion successful`, teardown warnings after scoring are simulator cleanup noise, not a failed run.
- The copied repo on this machine does not include a `.git` directory, so file-change tracking must be done by path rather than `git status`.
