"""Capture both real wrist cameras using the user's udev aliases and serial checks.

This reads UVC capabilities and validates capture, not geometric intrinsics.
Unknown K/distortion remain null in the profile. No robot motion is performed.
"""

import argparse
import concurrent.futures
import hashlib
import json
import subprocess
import time
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
PROFILE = ROOT / "Isaacsim_tactile_env/dexmate_wrist_cameras.json"


def command(*args):
    return subprocess.run(args, capture_output=True, text=True, check=True, timeout=10).stdout


def check_identity(camera):
    info = command("udevadm", "info", "--query=property", "--name=" + camera["device"])
    properties = dict(line.split("=", 1) for line in info.splitlines() if "=" in line)
    if properties.get("ID_SERIAL_SHORT") != camera["serial"]:
        raise ValueError(f"Unexpected camera serial on {camera['device']}")
    if ":capture:" not in properties.get("ID_V4L_CAPABILITIES", ""):
        raise ValueError(f"Not a video capture node: {camera['device']}")
    return properties


def capture(name, camera, settings, output, count, apply_controls):
    device = camera["device"]
    report = {"serial": camera["serial"], "device": device, "udev": check_identity(camera)}
    for key, flag in [("device_before", "--all"), ("formats", "--list-formats-ext"),
                      ("controls_before", "--list-ctrls-menus")]:
        report[key] = command("v4l2-ctl", "-d", device, flag)
    if apply_controls:
        values = ",".join(f"{key}={value}" for key, value in settings["controls"].items())
        command("v4l2-ctl", "-d", device, "--set-ctrl=" + values)
    video = cv2.VideoCapture(device, cv2.CAP_V4L2)
    try:
        if not video.isOpened():
            raise RuntimeError(f"Cannot open {device}")
        for prop, value in [
            (cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*settings["fourcc"])),
            (cv2.CAP_PROP_FRAME_WIDTH, settings["width"]),
            (cv2.CAP_PROP_FRAME_HEIGHT, settings["height"]),
            (cv2.CAP_PROP_FPS, settings["requested_fps"]),
            (cv2.CAP_PROP_BUFFERSIZE, settings["buffer_count"]),
        ]:
            if not video.set(prop, value):
                raise RuntimeError(f"Capture setting {prop}={value} rejected by {device}")
        actual = {
            "width": int(video.get(cv2.CAP_PROP_FRAME_WIDTH)),
            "height": int(video.get(cv2.CAP_PROP_FRAME_HEIGHT)),
            "reported_fps": video.get(cv2.CAP_PROP_FPS),
            "fourcc": int(video.get(cv2.CAP_PROP_FOURCC)),
        }
        if (actual["width"], actual["height"]) != (settings["width"], settings["height"]):
            raise RuntimeError(f"Unexpected image dimensions: {actual}")
        if actual["fourcc"] != cv2.VideoWriter_fourcc(*settings["fourcc"]):
            raise RuntimeError(f"Unexpected pixel format: {actual}")
        timestamps = []
        digests = []
        for _ in range(count + 15):
            ok, frame = video.read()
            timestamps.append(time.monotonic())
            if not ok:
                raise RuntimeError(f"Missing frame from {device}")
            digests.append(hashlib.sha256(frame.tobytes()).hexdigest())
        measured = timestamps[15:]
        actual.update(
            measured_host_read_fps=(len(measured) - 1) / (measured[-1] - measured[0]),
            max_host_frame_interval_s=float(np.max(np.diff(measured))),
            captured_frames=len(timestamps), warmup_frames=15,
            unique_decoded_frames=len(set(digests[15:])),
            host_read_timestamps_s=timestamps,
            timestamp_semantics="Host read completion, not hardware exposure time or cross-camera synchronization",
        )
        actual["within_5_percent_of_requested_fps"] = (
            abs(actual["measured_host_read_fps"] / settings["requested_fps"] - 1) <= 0.05
        )
        report["capture"] = actual
        report["intrinsics"] = camera["intrinsics"]
        if not cv2.imwrite(str(output / f"{name}.png"), frame):
            raise RuntimeError("Failed to save frame")
    finally:
        video.release()
    report["device_after"] = command("v4l2-ctl", "-d", device, "--all")
    (output / f"{name}.json").write_text(json.dumps(report, indent=2) + "\n")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", type=Path, default=PROFILE)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--frames", type=int, default=120)
    parser.add_argument("--apply-controls", action="store_true", help="Apply the saved profile's UVC controls")
    args = parser.parse_args()
    if args.frames < 30:
        parser.error("Use at least 30 measured frames")
    profile = json.loads(args.profile.read_text())
    settings = profile["capture"]
    if settings["crop"] is not None or settings["mirror"] or settings["rotation_degrees"] != 0:
        raise ValueError("This probe saves native images; crop/mirror/rotation are unsupported")
    # Check both identities before changing capture settings on either device.
    for camera in profile["cameras"].values():
        check_identity(camera)
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output / "tested_profile.json").write_bytes(args.profile.read_bytes())
    report = {"profile_sha256": hashlib.sha256(args.profile.read_bytes()).hexdigest(), "cameras": {}}
    errors = {}
    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
        jobs = {name: pool.submit(capture, name, camera, settings, args.output, args.frames, args.apply_controls)
                for name, camera in profile["cameras"].items()}
        for name, job in jobs.items():
            try:
                report["cameras"][name] = job.result()
            except Exception as exc:
                errors[name] = str(exc)
    report["errors"] = errors
    report["status"] = "capture_failed" if errors else "captured"
    (args.output / "probe.json").write_text(json.dumps(report, indent=2) + "\n")
    if errors:
        raise RuntimeError(errors)
    frames = []
    for name, result in report["cameras"].items():
        frame = cv2.imread(str(args.output / f"{name}.png"))
        frame = cv2.resize(frame, (960, 540))
        frame = cv2.copyMakeBorder(frame, 45, 0, 0, 0, cv2.BORDER_CONSTANT, value=(30, 30, 30))
        label = f"{name} | {result['serial']} | {result['capture']['measured_host_read_fps']:.2f} fps"
        cv2.putText(frame, label, (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
        frames.append(frame)
    cv2.imwrite(str(args.output / "wrist_cameras.jpg"), np.hstack(frames))
    print(json.dumps({name: {"measured_fps": result["capture"]["measured_host_read_fps"],
                             "intrinsics_status": result["intrinsics"]["status"]}
                      for name, result in report["cameras"].items()}, indent=2))


if __name__ == "__main__":
    main()
