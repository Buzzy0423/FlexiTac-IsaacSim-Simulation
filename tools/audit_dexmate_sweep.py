"""Audit finished sweep records through the shared dex_wire offline reader.

Run in the robot environment with PYTHONPATH pointing to dex_wire/src.
"""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from dex_wire.data.offline import open_episode


def audit(root):
    records = []
    for path in sorted(root.glob("episode_*/trajectory.npz")):
        episode = open_episode(path.parent)
        clip = episode.window(8, 2, include_images=False)
        report = json.loads(path.with_name("episode_report.json").read_text())
        k = min(345, len(episode) - 1)
        frame = episode.frame(k)
        rows = clip["frames"]["source_state_row"]
        with np.load(path) as data:
            np.testing.assert_array_equal(clip["frames"]["observation.tactile"], data["tactile"][rows])
            hold_tactile = data["tactile"][data["phase"] == "grasp_hold"]
            hold_mean = hold_tactile.mean(0) if len(hold_tactile) else None
        hashes = {}
        for name in ("trajectory.npz", "camera_index.npz", "episode_report.json"):
            with (path.parent / name).open("rb") as source:
                hashes[name] = hashlib.file_digest(source, "sha256").hexdigest()
        records.append({
            "episode": path.parent.name, "variation": report["grasp_config"]["variation"],
            "physics_passed": report["passed"], "readable": True, "video_frames": len(episode),
            "window_camera_frames": len(clip["frames"]["timestamp"]),
            "window_signal_frames": len(clip["signals"]["timestamp"]),
            "decoded_frame_index": k,
            "decoded_rgb_shapes": {key: list(frame[key].shape) for key in episode.metadata["camera_keys"]},
            "mean_hold_tactile": hold_mean.tolist() if hold_mean is not None else None, "sha256": hashes,
        })
        print(f"Read and checked {path.parent.name}", flush=True)
    if not records:
        raise ValueError("No recorded episodes")
    base = records[0]["mean_hold_tactile"]
    for record in records:
        current = record.pop("mean_hold_tactile")
        difference = (float(np.abs(np.asarray(current)[:2] - np.asarray(base)[:2]).mean())
                      if current is not None and base is not None else None)
        record["left_pad_mean_abs_difference_from_baseline"] = difference
    result = {"scope": "Offline record alignment and four-camera decode; physics outcomes remain separate",
              "readable_count": len(records), "records": records}
    with (root / "offline_audit.json").open("x") as stream:
        stream.write(json.dumps(result, indent=2) + "\n")
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", type=Path)
    args = parser.parse_args()
    audit(args.root)
