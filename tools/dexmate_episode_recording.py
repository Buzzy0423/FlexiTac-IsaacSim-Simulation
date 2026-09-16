"""Save synchronized 30 Hz demos; retain 120 Hz samples in memory for physics checks."""

import json
import subprocess

import numpy as np
from PIL import Image, ImageOps


class VideoWriter:
    def __init__(self, path, shape):
        height, width = shape[:2]
        self.log = path.with_suffix(".ffmpeg.log").open("w")
        self.process = subprocess.Popen(
            ["ffmpeg", "-v", "error", "-y", "-f", "rawvideo", "-pixel_format", "rgb24",
             "-video_size", f"{width}x{height}", "-framerate", "30", "-i", "pipe:0", "-an",
             "-c:v", "libx264", "-threads", "2", "-preset", "veryfast", "-crf", "18",
             "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(path)],
            stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=self.log,
        )
        self.count = 0

    def append(self, image):
        self.process.stdin.write(np.ascontiguousarray(image).tobytes())
        self.count += 1

    def close(self):
        self.process.stdin.close()
        status = self.process.wait(timeout=60)
        self.log.close()
        if status:
            raise RuntimeError(f"FFmpeg exited with {status}")


class EpisodeWriter:
    def __init__(self, output, env, reset_obs):
        output.mkdir()
        self.output, self.env = output, env
        self.frames, self.camera_rows = [], []
        self.names = list(reset_obs["rgb"])
        self.videos = {n: VideoWriter(output / f"{n}.mp4", image.shape) for n, image in reset_obs["rgb"].items()}
        self.closed = False
        for name, image in reset_obs["rgb"].items():
            Image.fromarray(image).save(output / f"reset_{name}.png")
        self.reset_state = {k: np.asarray(reset_obs[k]).tolist() for k in (
            "joint_position", "joint_velocity", "object_position_m", "object_orientation_wxyz",
            "object_velocity_m_s", "object_angular_velocity_rad_s", "tactile",
        )}

    def append(self, obs, action, reward, terminated, truncated, info):
        row = {k: obs[k] for k in (
            "step", "time_s", "sim_time_s", "physics_step", "joint_position", "joint_velocity",
            "object_position_m", "object_orientation_wxyz", "object_velocity_m_s", "object_angular_velocity_rad_s",
            "gripper_pose", "object_in_gripper_m", "tactile", "rgb_step",
        )}
        row.update(action=action.copy(), phase=info["phase"], phase_step=info["phase_step"],
                   finger_loaded=info["finger_loaded"], tactile_peak=obs["tactile"].max(axis=(1, 2)),
                   reward=reward, terminated=terminated, truncated=truncated, task_success=info["task_success"])
        self.frames.append(row)
        if obs["rgb_updated"]:
            for name, video in self.videos.items():
                video.append(obs["rgb"][name])
            self.camera_rows.append({
                "state_row": len(self.frames) - 1, "step": obs["step"], "time_s": obs["time_s"],
                "sim_time_s": obs["sim_time_s"],
                "render_sim_time_s": [obs["rgb_sim_time_s"][n] for n in self.names],
                "reference_time": [obs["rgb_reference_time"][n] for n in self.names],
                "camera_to_world": [obs["rgb_camera_to_world"][n] for n in self.names],
            })

    def close(self):
        if self.closed:
            return
        self.closed = True
        errors = []
        for video in self.videos.values():
            try:
                video.close()
            except Exception as error:
                errors.append(str(error))
        if errors:
            raise RuntimeError("; ".join(errors))

    def finish(self):
        from dexmate_grasp_regression import evaluate_grasp
        from dexmate_tactile_preview import export_views

        self.close()
        data = {k: np.asarray([f[k] for f in self.frames]) for k in self.frames[0]}
        cameras = {k: np.asarray([f[k] for f in self.camera_rows]) for k in self.camera_rows[0]}
        # Validate the full physics history, including any failure between camera ticks.
        report = evaluate_grasp(data, self.env.report)
        recorded = {k: v[cameras["state_row"]] for k, v in data.items()}
        np.savez_compressed(self.output / "trajectory.npz", **recorded,
                            joint_names=np.asarray(self.env.robot.dof_names),
                            action_names=np.asarray(self.env.action_names))
        saved_cameras = {**cameras, "state_row": np.arange(len(self.camera_rows))}
        np.savez_compressed(self.output / "camera_index.npz", **saved_cameras, camera_names=np.asarray(self.names))
        delta = cameras["render_sim_time_s"] - cameras["sim_time_s"][:, None]
        frame_counts = {}
        for name in self.names:
            count = subprocess.check_output([
                "ffprobe", "-v", "error", "-count_frames", "-select_streams", "v:0",
                "-show_entries", "stream=nb_read_frames", "-of", "csv=p=0", str(self.output / f"{name}.mp4"),
            ], text=True).strip()
            frame_counts[name] = int(count)
        sync_checks = {
            "physics_step_sequence": bool(np.array_equal(data["physics_step"], np.arange(1, len(data["step"]) + 1))),
            "internal_state_120hz": bool(np.allclose(np.diff(data["time_s"]), 1 / 120, atol=1e-6)),
            "recorded_state_30hz": bool(np.allclose(np.diff(recorded["time_s"]), 1 / 30, atol=1e-6)),
            "camera_every_four_steps": bool(np.array_equal(cameras["step"], np.arange(4, len(data["step"]) + 1, 4))),
            "camera_state_alignment": bool(np.array_equal(data["step"][cameras["state_row"]], cameras["step"])),
            "saved_camera_state_alignment": bool(np.array_equal(recorded["step"][saved_cameras["state_row"]],
                                                                 cameras["step"])),
            "camera_render_clock_alignment": bool(np.max(np.abs(delta)) < 1e-5),
            "camera_render_references_advance": bool(np.all(np.diff(
                cameras["reference_time"][..., 0] / cameras["reference_time"][..., 1], axis=0) > 0)),
            "four_camera_clocks_match": bool(np.max(np.ptp(cameras["render_sim_time_s"], axis=1)) < 1e-6),
            "encoded_frame_counts_match": all(v == len(self.camera_rows) for v in frame_counts.values()),
        }
        report.update(sync_checks=sync_checks, camera_frame_counts=frame_counts,
                      max_render_clock_error_s=float(np.max(np.abs(delta))),
                      api_success=bool(data["terminated"][-1] and data["task_success"][-1]
                                       and not data["truncated"][-1]))
        report["passed"] = bool(report["passed"] and all(sync_checks.values()) and report["api_success"])
        report["reset_state"] = self.reset_state
        report["validation_frame_count"] = len(data["step"])
        report["validation_hz"] = 120
        report["frame_count"] = len(recorded["step"])
        report["final_physics_state"] = {k: data[k][-1].item() for k in
                                         ("step", "time_s", "terminated", "truncated", "task_success")}
        report["grasp_config"] = self.env.report["grasp_config"]
        report["data_contract"] = {
            "version": 2, "physics_hz": 120, "signal_hz": 30, "camera_hz": 30,
            "video_codec": "H.264 CRF18 yuv420p (lossy)",
            "action": "Position targets in action_names order, radians except Lift in metres; mimic followers excluded",
            "alignment": "trajectory row contains state after applying that row's action for one physics step",
            "sampling": "Every fourth physics step at the camera timestamp; no averaging or interpolation",
            "action_scope": "Sampled 120 Hz command, not a command held for the full 1/30 s recording interval",
            "video_index": "Zero-based video frame maps to camera_index.state_row; reset images stored separately",
            "camera_parameters": "Nominal, not physically calibrated",
            "camera_models": self.env.report.get("camera_models", {}),
            "camera_to_world": "USD camera frame: +X right, +Y up, -Z forward; matrices use column vectors",
        }
        (self.output / "episode_report.json").write_text(json.dumps(report, indent=2) + "\n")
        export_views(recorded["tactile"], recorded["phase"], recorded["phase_step"], recorded["time_s"], self.output)
        return report


def make_review(output, *, four_cameras=False, make_gif=True):
    """Preview RGB above synchronized tactile, using the recorded frame index."""
    import cv2
    from dexmate_tactile_preview import render_frame
    from PIL import ImageDraw

    with np.load(output / "trajectory.npz") as data, np.load(output / "camera_index.npz") as index:
        rows = index["state_row"]
        tactile = data["tactile"][rows]
        times, phases = data["time_s"][rows], data["phase"][rows]
        if not np.array_equal(data["step"][rows], index["step"]):
            raise ValueError("Preview camera and tactile steps do not match")
    candidates = np.flatnonzero(phases == "grasp_hold")
    contact_frame = (int(candidates[len(candidates) // 2]) if len(candidates)
                     else int(np.argmax(tactile.sum(axis=(1, 2, 3)))))
    names = ["head_left", "head_right", "wrist_left", "wrist_right"] if four_cameras else ["observer"]
    captures = [cv2.VideoCapture(str(output / f"{name}.mp4")) for name in names]
    stem = "four_cameras_preview" if four_cameras else "observer_preview"
    report = json.loads((output / "episode_report.json").read_text())
    case = report.get("grasp_config", {}).get("variation", {}).get("name", "baseline")
    video = VideoWriter(output / f"{stem}.mp4", (900, 960, 3))
    try:
        for k, grids in enumerate(tactile):
            canvas = Image.new("RGB", (960, 900), "white")
            for i, capture in enumerate(captures):
                ok, pixels = capture.read()
                if not ok:
                    raise ValueError(f"{names[i]} ended before indexed frame {k}")
                rgb = Image.fromarray(cv2.cvtColor(pixels, cv2.COLOR_BGR2RGB))
                size = (480, 360) if four_cameras else (960, 720)
                xy = ((i % 2) * 480, (i // 2) * 360) if four_cameras else (0, 0)
                fitted = ImageOps.contain(rgb, size, Image.Resampling.LANCZOS)
                canvas.paste(fitted, (xy[0] + (size[0] - fitted.width) // 2,
                                      xy[1] + (size[1] - fitted.height) // 2))
                draw = ImageDraw.Draw(canvas)
                draw.rectangle((xy[0], xy[1], xy[0] + 120, xy[1] + 18), fill="black")
                draw.text((xy[0] + 4, xy[1] + 3), names[i], fill="white")
            heat = render_frame(grids, f"{case} | {phases[k]} | t={times[k]:.3f} s")
            canvas.paste(heat.resize((960, 180), Image.Resampling.LANCZOS), (0, 720))
            video.append(np.asarray(canvas))
            if k == contact_frame:
                canvas.save(output / f"{stem}.png")
        if any(capture.read()[0] for capture in captures):
            raise ValueError("Video has more frames than the camera index")
    finally:
        for capture in captures:
            capture.release()
        video.close()
    if not make_gif:
        return
    subprocess.run([
        "ffmpeg", "-v", "error", "-y", "-i", str(output / f"{stem}.mp4"), "-filter_complex",
        "fps=10,split[a][b];[a]palettegen[p];[b][p]paletteuse",
        "-loop", "0", str(output / ("four_cameras.gif" if four_cameras else "observer.gif")),
    ], check=True)
