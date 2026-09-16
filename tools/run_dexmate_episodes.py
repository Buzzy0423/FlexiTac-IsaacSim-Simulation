"""Validate repeated reset/grasp episodes and record five synchronized videos per episode."""

import argparse
import json
import sys
import traceback
from pathlib import Path

import numpy as np
from dexmate_episode_recording import EpisodeWriter, make_review
from dexmate_grasp_motion import LeftArmKinematics, interpolate_path
from dexmate_performance import Timings
from dexmate_variations import pose_sweep
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from Isaacsim_tactile_env.dexmate_env import DexmateEnv  # noqa: E402


def grasp_actions(env, initial_observation):
    kin = LeftArmKinematics(env.scene["source"], env.report["workspace"]["robot"])
    yaw = env.report["grasp_config"].get("yaw_deg", 0)
    kin.orientation = Rotation.from_euler("z", yaw, degrees=True).as_matrix() @ kin.orientation
    xyz = np.asarray(env.report["grasp_config"]["gripper_base_position_m"])
    pre = xyz + [0, 0, .12]
    pre_q = kin.solve(pre, np.array([1.5, .1, .1, -1.8, -.1, .3, .1]))
    approach = kin.line(pre, xyz, pre_q)
    lift = kin.line(xyz, xyz + [0, 0, .05], approach[-1])
    arm_ids = [env.robot.get_dof_index(f"L_arm_j{i}") for i in range(1, 8)]
    finger_ids = [env.robot.get_dof_index(f"L_gripper_j{i}") for i in (1, 2)]
    phases = [
        ("grasp_preposition", np.array([initial_observation["joint_position"][arm_ids], pre_q]), .4, 480),
        ("grasp_approach", approach, .4, 360),
        ("grasp_close", np.array([approach[-1]] * 2), 0., 180),
        ("grasp_lift", lift, 0., 240),
        ("grasp_hold", np.array([lift[-1]] * 2), 0., 240),
        ("grasp_lower", lift[::-1], 0., 240),
        ("grasp_release", np.array([approach[-1]] * 2), .4, 180),
        ("grasp_retreat", approach[::-1], .4, 240),
    ]
    for phase, path, opening, count in phases:
        for step in range(count):
            desired = env.initial.copy()
            desired[arm_ids] = interpolate_path(path, (step + 1) / count)
            desired[finger_ids] = opening
            yield phase, desired[env.driven_ids]


def check_reset(obs, previous):
    checks = {
        "clock_zero": obs["step"] == 0 and obs["physics_step"] == 0 and obs["time_s"] == 0,
        "tactile_zero": bool(obs["tactile"].max() < 1e-6),
        "object_stationary": bool(np.linalg.norm(obs["object_velocity_m_s"]) < .001
                                  and np.linalg.norm(obs["object_angular_velocity_rad_s"]) < .01),
        "fresh_reset_images": obs["rgb_updated"] and obs["rgb_step"] == 0,
    }
    deviations = {}
    if previous is not None:
        for key, tolerance in (("joint_position", .001), ("joint_velocity", .001),
                               ("object_position_m", .0001), ("object_orientation_wxyz", .0001)):
            deviations[key] = float(np.max(np.abs(obs[key] - previous[key])))
            checks[f"repeat_{key}"] = deviations[key] < tolerance
    return {"checks": checks, "max_deviations": deviations, "passed": all(checks.values()),
            "object_position_m": obs["object_position_m"].tolist(),
            "object_velocity_m_s": obs["object_velocity_m_s"].tolist(),
            "object_angular_velocity_rad_s": obs["object_angular_velocity_rad_s"].tolist(),
            "camera_time_s": obs["rgb_sim_time_s"], "physics_time_s": obs["sim_time_s"]}


def run(args):
    env, writer = None, None
    cases = pose_sweep() if args.pose_sweep else [{"name": "baseline"}] * args.episodes
    summary = {"status": "starting", "episodes": [], "reset_checks": [], "cases": cases}
    status = 1
    timings = Timings()
    try:
        with timings.measure("startup"):
            env = DexmateEnv(args.output, workspace=args.workspace,
                             workaround_r535_vulkan=args.workaround_r535_vulkan,
                             render_mode=args.render_mode, diagnostic_cameras=args.diagnostic_cameras,
                             zero_delay=args.zero_delay, cpu_threads=args.cpu_threads,
                             physics_threads=args.physics_threads,
                             disable_viewport_updates=not args.viewport_updates)
        references, plans = {}, {}
        summary["motion_plan_cache_hits"] = 0
        for episode, case in enumerate(cases):
            with timings.measure("reset"):
                obs, _ = env.reset(seed=0, variation=case)
            signature = (*case.get("offset_xy_m", [0, 0]), case.get("yaw_deg", 0))
            reset = check_reset(obs, references.get(signature))
            reset["variation"] = case
            target = np.asarray(env.report["grasp_config"]["position_m"])
            reset["checks"]["requested_object_xy"] = bool(
                np.max(np.abs(obs["object_position_m"][:2] - target[:2])) < .001)
            target_q = np.asarray(env.report["grasp_config"]["orientation_wxyz"])
            reset["checks"]["requested_object_orientation"] = bool(
                abs(np.dot(obs["object_orientation_wxyz"], target_q)) > .9999)
            reset["passed"] = all(reset["checks"].values())
            summary["reset_checks"].append(reset)
            if not reset["passed"]:
                raise RuntimeError(f"Reset check failed: {reset}")
            references.setdefault(signature, obs)
            if args.smoke:
                for _ in range(8):
                    obs, _, _, _, _ = env.step(env.initial[env.driven_ids])
                print("Camera render times:", obs["rgb_sim_time_s"], "physics:", obs["sim_time_s"], flush=True)
                continue
            output = env.output / f"episode_{episode:03d}"
            with timings.measure("writer_setup"):
                writer = EpisodeWriter(output, env, obs)
            with timings.measure("motion_plan"):
                plan_key = (signature, obs["joint_position"].tobytes())
                if plan_key not in plans:
                    plans[plan_key] = list(grasp_actions(env, obs))
                else:
                    summary["motion_plan_cache_hits"] += 1
                actions = plans[plan_key]
            previous_phase = None
            for phase, action in actions:
                if phase != previous_phase:
                    print(f"Episode {episode}: {phase}", flush=True)
                    previous_phase = phase
                with timings.measure("env_step"):
                    obs, reward, terminated, truncated, info = env.step(action, phase=phase)
                with timings.measure("record_append"):
                    writer.append(obs, action, reward, terminated, truncated, info)
                if terminated or truncated:
                    break
            with timings.measure("record_finish"):
                result = writer.finish()
            writer = None
            summary["episodes"].append({k: v for k, v in result.items() if k != "reset_state"})
            (env.output / "run_report.json").write_text(json.dumps(summary, indent=2) + "\n")
            if not result["passed"] and not args.pose_sweep:
                raise RuntimeError(f"Episode {episode} failed; inspect {output / 'episode_report.json'}")
            if not args.no_preview and (episode == 0 or args.pose_sweep):
                with timings.measure("preview"):
                    make_review(output, four_cameras=args.pose_sweep, make_gif=episode == 0)
            print(f"Episode {episode} {case['name']} passed={result['passed']}: "
                  f"{result.get('hold_lift_range_m')}", flush=True)
        summary["passed_count"] = sum(r["passed"] for r in summary["episodes"])
        summary["status"] = "passed" if all(r["passed"] for r in summary["episodes"]) else "completed_with_failures"
        status = 0 if summary["status"] == "passed" else 2
    except BaseException:
        summary["status"] = "failed"
        summary["error"] = traceback.format_exc()
        traceback.print_exc()
    finally:
        if writer is not None:
            writer.close()
        args.output.mkdir(parents=True, exist_ok=True)
        summary["performance"] = {"wall_clock_sections": timings.snapshot(),
                                  "note": "Nested timings are inclusive; do not sum parent and child sections"}
        if env is not None:
            summary["performance"].update(runtime=env.runtime, env_sections=env.timings.snapshot(),
                                          render_flush_histogram=env.flush_histogram)
        (args.output / "run_report.json").write_text(json.dumps(summary, indent=2) + "\n")
        if env is not None:
            env.close(exit_code=status)
    return status


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--workspace", type=Path, help="Optional isolated candidate workspace configuration")
    parser.add_argument("--episodes", type=int, default=3)
    parser.add_argument("--workaround-r535-vulkan", action="store_true")
    parser.add_argument("--smoke", action="store_true", help="Only initialize, reset and step eight times per episode")
    parser.add_argument("--render-mode", choices=("physics", "camera"), default="camera",
                        help="Render every physics step, or once every four physics steps at 30 Hz")
    parser.add_argument("--diagnostic-cameras", action="store_true",
                        help="Also render three unused placement views (legacy performance baseline)")
    parser.add_argument("--zero-delay", action=argparse.BooleanOptionalAction, default=True,
                        help="Use Isaac Sim's synchronous render completion settings")
    parser.add_argument("--no-preview", action="store_true", help="Skip composite MP4/GIF; keep all five source videos")
    parser.add_argument("--cpu-threads", type=int, default=32, help="Global Carbonite/TBB worker limit")
    parser.add_argument("--physics-threads", type=int, help="PhysX workers; omit to keep the installed default")
    parser.add_argument("--viewport-updates", action=argparse.BooleanOptionalAction, default=False,
                        help="Render the unused GUI viewport in addition to the recorded cameras")
    parser.add_argument("--pose-sweep", action="store_true",
                        help="Run ten bounded XY/yaw cases, including a final baseline repeat")
    args = parser.parse_args()
    if args.episodes < 1:
        parser.error("episodes must be positive")
    if args.cpu_threads < 1 or (args.physics_threads is not None and args.physics_threads < 1):
        parser.error("thread counts must be positive")
    if args.output.exists():
        parser.error("output must be a new directory; existing evidence is never overwritten")
    return run(args)


if __name__ == "__main__":
    raise SystemExit(main())
