#!/usr/bin/env python
"""One-off dataset transform: replace `observation.state` with
[joint_state + downsampled Astra depth] so SmolVLA (or any policy that
hardcodes `batch[OBS_STATE]`, e.g. modeling_smolvla.py) actually sees the
depth signal - a *new* `observation.depth_summary` feature would just sit
unused, since SmolVLA only ever reads the one `observation.state` key.

Depth is a 240x320 spatial map (76,800 values/frame) - too big and
unstructured to hand a plain state MLP as-is, so this reshape-averages it
down to a 4x4 grid (16 values) that preserves rough "which corner is
close/far" signal without blowing up the state vector.

Decodes depth **per episode** (one `decode_video_frames` call per episode,
covering every frame in it) instead of per frame: `LeRobotDataset.__getitem__`
re-opens and re-seeks the video container on every single call (see
`dataset_reader.py`'s `_query_videos`), which is fine for training's shuffled
random access but is a reopen-per-frame anti-pattern for a full sequential
sweep over ~20k frames - the first version of this script hit exactly that
and got progressively slower as it went. Raw quantized depth (not
dequantized to real mm/m) is used directly - quantization is monotonic in
real depth, and this is only a coarse auxiliary signal that gets
dataset-stats-normalized before training anyway, so the physical unit
doesn't matter.

Two `modify_features` passes (lerobot's own dataset_tools.py) because it
validates new feature names against the *original* dataset.meta.features,
so you can't remove and re-add the same key in one call.

Usage (from the lerobot venv):
    cd ~/lerobot && uv run python ~/so101-bimanual-teleop/depth_to_state_feature.py \\
        --src_repo_id local/towel_half_fold_bimanual_no_depth \\
        --depth_repo_id local/towel_half_fold_bimanual_20260904_151949 \\
        --out_repo_id local/towel_half_fold_bimanual_depth_state
"""

import argparse

import numpy as np

from lerobot.datasets import LeRobotDataset
from lerobot.datasets.dataset_tools import modify_features
from lerobot.datasets.video_utils import decode_video_frames

GRID = 4  # depth downsampled to GRID x GRID
DEPTH_KEY = "observation.images.astra_depth"


def downsample_depth_batch(depth_bhw: np.ndarray, grid: int = GRID) -> np.ndarray:
    """(B, H, W) -> (B, grid*grid) via block-average. H, W must divide evenly by grid."""
    b, h, w = depth_bhw.shape
    bh, bw = h // grid, w // grid
    cropped = depth_bhw[:, : bh * grid, : bw * grid]
    return cropped.reshape(b, grid, bh, grid, bw).mean(axis=(2, 4)).reshape(b, -1)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--src_repo_id", required=True, help="Dataset to add the combined state to (no depth video)")
    parser.add_argument("--depth_repo_id", required=True, help="Dataset that still has observation.images.astra_depth")
    parser.add_argument("--out_repo_id", required=True)
    args = parser.parse_args()

    src = LeRobotDataset(args.src_repo_id)
    depth_src = LeRobotDataset(args.depth_repo_id)
    if len(src) != len(depth_src):
        raise ValueError(f"Frame count mismatch: {args.src_repo_id}={len(src)} vs {args.depth_repo_id}={len(depth_src)}")

    joint_names = src.meta.features["observation.state"]["names"]
    depth_names = [f"depth_{i}" for i in range(GRID * GRID)]

    print("Bulk-loading joint state (no video decode needed for this)...")
    state_col = src.hf_dataset["observation.state"]
    joint_state = np.stack([np.asarray(x, dtype=np.float32) for x in state_col])  # (N, n_joints)

    n_total = len(depth_src)
    combined = np.empty((n_total, len(joint_names) + len(depth_names)), dtype=np.float32)
    combined[:, : len(joint_names)] = joint_state

    meta = depth_src.meta
    n_episodes = meta.total_episodes
    print(f"Decoding depth for {n_episodes} episodes ({n_total} frames total)...")
    for ep_idx in range(n_episodes):
        ep = meta.episodes[ep_idx]
        from_ts = ep[f"videos/{DEPTH_KEY}/from_timestamp"]
        n_frames = ep["length"]
        query_ts = [from_ts + i / meta.fps for i in range(n_frames)]
        video_path = depth_src.root / meta.get_video_file_path(ep_idx, DEPTH_KEY)

        frames = decode_video_frames(video_path, query_ts, depth_src.tolerance_s, is_depth=True)  # (n_frames, 1, H, W)
        depth_bhw = frames.numpy()[:, 0].astype(np.float32)
        depth_summary = downsample_depth_batch(depth_bhw)

        row_from, row_to = ep["dataset_from_index"], ep["dataset_to_index"]
        combined[row_from:row_to, len(joint_names) :] = depth_summary
        print(f"  episode {ep_idx + 1}/{n_episodes} done ({row_to}/{n_total} frames)")

    print("Pass 1/2: removing old observation.state ...")
    step1 = modify_features(src, remove_features=["observation.state"], repo_id=f"{args.out_repo_id}_step1")

    print("Pass 2/2: adding combined [joints + depth] observation.state ...")
    modify_features(
        step1,
        add_features={
            "observation.state": (
                combined,
                {"dtype": "float32", "shape": [combined.shape[1]], "names": joint_names + depth_names},
            )
        },
        repo_id=args.out_repo_id,
    )
    print(f"Done: {args.out_repo_id} (state dim {combined.shape[1]} = {len(joint_names)} joints + {len(depth_names)} depth)")


if __name__ == "__main__":
    main()
