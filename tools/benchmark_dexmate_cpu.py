"""Replay an immutable 120 Hz reference grasp to benchmark threads and verify all internal samples."""

import argparse
import cProfile
import faulthandler
import hashlib
import json
import pstats
import sys
import time
import traceback
from pathlib import Path

import numpy as np
from dexmate_episode_recording import EpisodeWriter
from dexmate_performance import Timings

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from Isaacsim_tactile_env.dexmate_env import DexmateEnv  # noqa: E402


def run(args):
    faulthandler.enable()
    faulthandler.dump_traceback_later(120, repeat=True)
    with np.load(args.reference, allow_pickle=False) as data:
        reference = dict(data)
    if not np.array_equal(reference["step"], np.arange(1, 2161)):
        raise ValueError("Expected a full 2160-step 120 Hz reference, not a 30 Hz recording")
    source_report = json.loads(args.reference.with_name("episode_report.json").read_text())
    env, writer = None, None
    profiler, timings = cProfile.Profile(), Timings()
    summary = {"reference": str(args.reference.resolve()),
               "reference_sha256": hashlib.sha256(args.reference.read_bytes()).hexdigest(),
               "profile_enabled": args.profile, "status": "starting"}
    status = 1
    try:
        with timings.measure("startup"):
            env = DexmateEnv(args.output, workaround_r535_vulkan=args.workaround_r535_vulkan,
                             cpu_threads=args.cpu_threads, physics_threads=args.physics_threads,
                             disable_viewport_updates=args.disable_viewport_updates,
                             physics_mode=args.physics_mode, ccd=args.ccd)
        print("Runtime:", json.dumps(env.runtime), flush=True)
        if env.action_names != reference["action_names"].tolist():
            raise ValueError("Reference action order does not match environment")
        summary["episodes"] = []
        for episode_id in range(args.episodes):
            with timings.measure("reset"):
                obs, _ = env.reset(variation=source_report["grasp_config"].get("variation"))
            if episode_id == 0:
                if args.start_gate is not None:
                    ready = {"monotonic": time.monotonic(), "startup_seconds": timings.seconds["startup"],
                             "initial_reset_seconds": timings.seconds["reset"],
                             "driver_shader_cache": env.runtime["driver_shader_cache"]}
                    temporary_ready = args.output / "ready.json.tmp"
                    temporary_ready.write_text(json.dumps(ready))
                    temporary_ready.replace(args.output / "ready.json")
                    deadline = time.monotonic() + args.gate_timeout
                    while not args.start_gate.exists():
                        if time.monotonic() >= deadline:
                            raise TimeoutError("Parallel benchmark start gate timed out")
                        time.sleep(.05)
                summary["collection_started_monotonic"] = time.monotonic()
            writer = EpisodeWriter(args.output / f"episode_{episode_id:03d}", env, obs)
            previous = None
            for phase, action in zip(reference["phase"], reference["action"], strict=True):
                if phase != previous:
                    print(f"episode={episode_id} {phase}", flush=True)
                    previous = phase
                if args.profile:
                    profiler.enable()
                try:
                    with timings.measure("env_step"):
                        obs, reward, terminated, truncated, info = env.step(action, phase=str(phase))
                finally:
                    if args.profile:
                        profiler.disable()
                with timings.measure("record_append"):
                    writer.append(obs, action, reward, terminated, truncated, info)
                if terminated or truncated:
                    break
            comparisons = {}
            for key in ("joint_position", "joint_velocity", "object_position_m", "object_orientation_wxyz",
                        "gripper_pose", "tactile"):
                actual = np.asarray([frame[key] for frame in writer.frames])
                expected = reference[key][:len(actual)]
                comparisons[key] = {"shape": list(actual.shape),
                                    "max_abs_difference": float(np.max(np.abs(actual - expected)))}
            complete = len(writer.frames) == len(reference["step"])
            with timings.measure("record_finish"):
                result = writer.finish()
            writer = None
            episode = {k: v for k, v in result.items() if k != "reset_state"}
            summary["episodes"].append({"episode_id": episode_id, "episode": episode,
                                        "reference_complete": complete, "full_rate_comparison": comparisons})
            # Retain the original single-episode keys for historical report consumers.
            if episode_id == 0:
                summary.update(episode=episode, full_rate_comparison=comparisons)
        summary["collection_finished_monotonic"] = time.monotonic()
        passed = all(item["episode"]["passed"] and item["reference_complete"] for item in summary["episodes"])
        summary["status"] = "passed" if passed else "failed"
        status = 0 if passed else 2
    except BaseException:
        summary["status"], summary["error"] = "failed", traceback.format_exc()
        traceback.print_exc()
    finally:
        faulthandler.cancel_dump_traceback_later()
        args.output.mkdir(parents=True, exist_ok=True)
        if args.profile and profiler.getstats():
            profiler.dump_stats(str(args.output / "step_profile.pstats"))
            stats = pstats.Stats(profiler)
            rows = [{"function": f"{file}:{line}:{name}", "calls": nc,
                     "self_seconds": tt, "inclusive_seconds": ct}
                    for (file, line, name), (_, nc, tt, ct, _) in stats.stats.items()]
            summary["profile_top_self"] = sorted(rows, key=lambda row: -row["self_seconds"])[:40]
            summary["profile_physics_and_contacts"] = [row for row in rows if any(
                key in row["function"] for key in ("simulate", "fetch_results", "on_contacts", "record_contact_phase"))]
        summary["timings"] = timings.snapshot()
        if env is not None:
            summary.update(runtime=env.runtime, env_timings=env.timings.snapshot(),
                           render_flush_histogram=env.flush_histogram)
        (args.output / "benchmark.json").write_text(json.dumps(summary, indent=2) + "\n")
        if writer is not None:
            writer.close()
        if env is not None:
            env.close(exit_code=status)
    return status


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reference", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--cpu-threads", type=int, default=32)
    parser.add_argument("--physics-threads", type=int)
    parser.add_argument("--physics-mode", choices=("cpu", "gpu"), default="cpu")
    parser.add_argument("--ccd", action=argparse.BooleanOptionalAction, default=None)
    parser.add_argument("--disable-viewport-updates", action="store_true")
    parser.add_argument("--profile", action="store_true")
    parser.add_argument("--episodes", type=int, default=1)
    parser.add_argument("--start-gate", type=Path, help="Wait after initial reset until this file exists")
    parser.add_argument("--gate-timeout", type=float, default=300)
    parser.add_argument("--workaround-r535-vulkan", action="store_true")
    arguments = parser.parse_args()
    if arguments.output.exists():
        parser.error("output must be new")
    if arguments.cpu_threads < 1 or (arguments.physics_threads is not None and arguments.physics_threads < 1):
        parser.error("thread counts must be positive")
    if arguments.episodes < 1 or arguments.gate_timeout <= 0:
        parser.error("episodes and gate timeout must be positive")
    if arguments.physics_mode == "gpu" and arguments.ccd is True:
        parser.error("GPU dynamics requires CCD disabled")
    raise SystemExit(run(arguments))
