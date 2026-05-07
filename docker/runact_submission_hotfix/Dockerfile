FROM my-solution:submission-act

ENV HF_HOME="/opt/huggingface"
ENV HF_HUB_DISABLE_TELEMETRY="1"
ENV AIC_ACT_CACHE_DIR="/opt/huggingface"
ENV AIC_ACT_LOCAL_ONLY="1"
ENV AIC_ACT_SNAPSHOT_DIR="/opt/huggingface/aic_act_policy"
ENV TORCH_HOME="/opt/torch-cache"

COPY aic_example_policies/aic_example_policies/ros/SubmissionACT.py /ws_aic/src/aic/aic_example_policies/aic_example_policies/ros/SubmissionACT.py
COPY aic_example_policies/aic_example_policies/ros/SubmissionACT.py /ws_aic/src/aic/.pixi/envs/default/lib/python3.12/site-packages/aic_example_policies/ros/SubmissionACT.py

SHELL ["/bin/bash", "-c"]
RUN mkdir -p "$HF_HOME" "$TORCH_HOME" && \
    cd /ws_aic/src/aic && \
    pixi run --as-is python - <<'PY'
from pathlib import Path

from huggingface_hub import snapshot_download
from torchvision.models import ResNet18_Weights, resnet18

snapshot_path = Path(
    snapshot_download(
        repo_id="grkw/aic_act_policy",
        allow_patterns=["config.json", "*.safetensors"],
        cache_dir="/opt/huggingface",
    )
)

link_path = Path("/opt/huggingface/aic_act_policy")
if link_path.is_symlink() or link_path.is_file():
    link_path.unlink()
elif link_path.is_dir():
    for child in link_path.iterdir():
        if child.is_dir():
            import shutil

            shutil.rmtree(child)
        else:
            child.unlink()
    link_path.rmdir()
link_path.symlink_to(snapshot_path, target_is_directory=True)

resnet18(weights=ResNet18_Weights.IMAGENET1K_V1)
print(f"ACT snapshot: {snapshot_path}")
print(f"Stable link: {link_path}")
PY
