"""Single-scene Dexmate environment with reset/step and explicitly timed RGB observations.

Create before importing other Isaac Sim APIs. Action: position targets for action_names,
in rad (revolute) or metres (Lift). Follower mimic joints are excluded.
"""

import copy
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from types import SimpleNamespace

import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def validate_action(action, lower, upper):
    value = np.asarray(action, dtype=np.float32)
    if value.shape != lower.shape or not np.isfinite(value).all():
        raise ValueError(f"Expected finite action of shape {lower.shape}")
    if np.any(value < lower - 1e-6) or np.any(value > upper + 1e-6):
        raise ValueError("Joint target exceeds URDF limits")
    return value.copy()


class GraspProgress:
    """Success requires sustained supported lift followed by stable table release."""

    def __init__(self, baseline_z):
        self.baseline_z = baseline_z
        self.hold_steps = 0
        self.release_steps = 0
        self.held = False

    def update(self, position, velocity, loaded, tactile, finger_positions):
        lifted = .04 < position[2] - self.baseline_z < .06
        supported = all(loaded) and np.all(tactile[:2].max(axis=(1, 2)) > .001)
        self.hold_steps = self.hold_steps + 1 if lifted and supported else 0
        self.held = self.held or self.hold_steps >= 240
        released = (self.held and abs(position[2] - self.baseline_z) < .003
                    and np.linalg.norm(velocity) < .02 and not any(loaded)
                    and tactile[:2].max() < 1e-6 and np.all(np.asarray(finger_positions) > .35))
        self.release_steps = self.release_steps + 1 if released else 0
        return self.release_steps >= 60


class DexmateEnv:
    physics_hz = 120
    camera_hz = 30

    def __init__(self, output, workspace=None, *, max_episode_steps=2160, workaround_r535_vulkan=False,
                 render_mode="camera", diagnostic_cameras=False, zero_delay=True,
                 cpu_threads=32, physics_threads=None, disable_viewport_updates=True,
                 physics_mode="cpu", ccd=None):
        from isaacsim import SimulationApp

        sys.path.insert(0, str(ROOT / "tools"))
        from dexmate_performance import Timings
        from validate_dexmate_scene import create_scene, driver_arguments

        if render_mode not in ("physics", "camera"):
            raise ValueError("render_mode must be physics or camera")
        if physics_mode not in ("cpu", "gpu") or (physics_mode == "gpu" and ccd is True):
            raise ValueError("physics_mode must be cpu or gpu; GPU dynamics requires CCD disabled")
        if cpu_threads < 1 or (physics_threads is not None and physics_threads < 1):
            raise ValueError("Thread counts must be positive")
        self.render_mode = render_mode
        self.timings = Timings()
        self.flush_histogram = {}
        self.output = Path(output).resolve()
        self.output.mkdir(parents=True, exist_ok=False)
        workspace = Path(workspace or ROOT / "Isaacsim_tactile_env/dexmate_workspace.json")
        self.report = {"status": "starting", "workspace": json.loads(workspace.read_text())}
        args = SimpleNamespace(output=self.output, workspace=workspace, four_cameras=True, table_proxy=False,
                               grasp_regression=True, contact_regression=False, raw_collisions=False,
                               disable_self_collision=False, workaround_r535_vulkan=workaround_r535_vulkan)
        args.diagnostic_cameras = diagnostic_cameras
        args.physics_mode, args.ccd = physics_mode, ccd
        extra_args = driver_arguments(args, self.report)
        if physics_threads is not None:
            extra_args.append(f"--/persistent/physics/numThreads={physics_threads}")
        if zero_delay:
            # Same render-completion settings as Isaac Sim 5.1's zero_delay experience.
            extra_args += ["--/app/hydraEngine/waitIdle=1",
                           "--/app/updateOrder/checkForHydraRenderComplete=1000"]
        self.app = SimulationApp({"headless": True, "extra_args": extra_args,
                                  "limit_cpu_threads": cpu_threads,
                                  "disable_viewport_updates": disable_viewport_updates})
        from isaacsim.core.utils.extensions import enable_extension

        enable_extension("isaacsim.asset.importer.urdf")
        self.scene = create_scene(args, self.report, self._save_scene_report)
        self.baseline_grasp_config = copy.deepcopy(self.report["grasp_config"])
        self.world, self.robot = self.scene["world"], self.scene["robot"]
        import carb
        from isaacsim.core.simulation_manager import SimulationManager

        settings = carb.settings.get_settings()
        self.runtime = {"physics_device": SimulationManager.get_physics_sim_device(),
                        "physics_mode_requested": physics_mode,
                        "gpu_dynamics_enabled": SimulationManager.is_gpu_dynamics_enabled(),
                        "broadphase": SimulationManager.get_broadphase_type(),
                        "ccd_enabled": SimulationManager.is_ccd_enabled(),
                        "solver_type": SimulationManager.get_solver_type(),
                        "suppress_readback": settings.get("/physics/suppressReadback"),
                        "gpu_found_lost_aggregate_pairs_capacity":
                            self.world.get_physics_context().get_gpu_found_lost_aggregate_pairs_capacity(),
                        "backend": self.world.backend, "render_mode": render_mode,
                        "diagnostic_cameras": diagnostic_cameras, "zero_delay": zero_delay,
                        "disable_viewport_updates": disable_viewport_updates,
                        "driver_shader_cache": settings.get("/rtx/shaderDb/driverShaderCachePath"),
                        "thread_settings": {key: settings.get(key) for key in (
                            "/plugins/carb.tasking.plugin/threadCount", "/persistent/physics/numThreads",
                            "/plugins/omni.tbb.globalcontrol/maxThreadCount")}}
        if self.runtime["gpu_dynamics_enabled"] != (physics_mode == "gpu"):
            raise RuntimeError("Actual PhysX dynamics does not match requested physics_mode")
        self.cube = self.scene["grasp_cube"]
        self.sensor = self.scene["tactile"].sensor
        self.initial = self.scene["initial"].copy()
        self.driven_ids = self.scene["driven_ids"]
        self.action_names = [self.robot.dof_names[i] for i in self.driven_ids]
        joints = {j.get("name"): j for j in ET.parse(self.scene["source"]).getroot().findall("joint")}
        self.action_lower = np.array([float(joints[n].find("limit").get("lower")) for n in self.action_names])
        self.action_upper = np.array([float(joints[n].find("limit").get("upper")) for n in self.action_names])
        self.robot.set_joints_default_state(positions=self.initial, velocities=np.zeros_like(self.initial))
        self.max_episode_steps = max_episode_steps
        self.episode_id = -1
        self.done = True
        self.closed = False
        self._setup_cameras()

    def _save_scene_report(self):
        (self.output / "scene_setup.json").write_text(json.dumps(self.report, indent=2) + "\n")

    def _setup_cameras(self):
        import omni.replicator.core as rep
        from isaacsim.core.nodes.bindings import _isaacsim_core_nodes

        self.core_clock = _isaacsim_core_nodes.acquire_interface()
        self.readers = dict(self.scene["camera_readers"])
        products = dict(self.scene["camera_products"])
        camera = rep.create.camera(position=(.9, 1.05, 1.25), look_at=(.30, .22, .92), clipping_range=(.01, 20.))
        products["observer"] = rep.create.render_product(camera, (800, 600))
        self.readers["observer"] = rep.AnnotatorRegistry.get_annotator("rgb")
        self.readers["observer"].attach(products["observer"])
        self.clocks = {}
        self.camera_prims = {}
        for name, product in products.items():
            clock = rep.AnnotatorRegistry.get_annotator("ReferenceTime")
            clock.attach(product)
            self.clocks[name] = clock
            camera_path = self.world.stage.GetPrimAtPath(product.path).GetRelationship("camera").GetTargets()[0]
            self.camera_prims[name] = self.world.stage.GetPrimAtPath(camera_path)

    def _capture(self):
        from pxr import Usd, UsdGeom

        # Kit pipelines several render frames. Drain it with render-only updates;
        # never advance physics to make a delayed image look current.
        physics_step = self.world.current_time_step_index
        for flush in range(10):
            current = []
            fresh = True
            for name, clock in self.clocks.items():
                ref = clock.get_data()
                if "referenceTimeNumerator" not in ref:
                    break
                rational = (int(ref["referenceTimeNumerator"]), int(ref["referenceTimeDenominator"]))
                previous = self.rgb_reference_time.get(name)
                if previous is not None:
                    fresh = fresh and rational[0] * previous[1] > previous[0] * rational[1]
                current.append(float(self.core_clock.get_sim_time_at_time(rational)))
            aligned = np.max(np.abs(np.asarray(current) - self.world.current_time)) < 1e-5 if current else False
            if len(current) == len(self.clocks) and aligned and fresh:
                break
            with self.timings.measure("render_flush"):
                self.world.render()
        else:
            raise RuntimeError(f"Camera frames did not catch up to physics time {self.world.current_time}: {current}")
        if self.world.current_time_step_index != physics_step:
            raise RuntimeError("Camera synchronization advanced physics")
        self.render_flushes = flush
        self.flush_histogram[self.render_flushes] = self.flush_histogram.get(self.render_flushes, 0) + 1
        images, times, references = {}, {}, {}
        for name, reader in self.readers.items():
            image = np.asarray(reader.get_data())
            if image.ndim != 3 or image.shape[2] < 3:
                raise RuntimeError(f"Missing camera frame: {name}")
            images[name] = image[..., :3].copy()
            images[name].setflags(write=False)
            ref = self.clocks[name].get_data()
            rational = (int(ref["referenceTimeNumerator"]), int(ref["referenceTimeDenominator"]))
            references[name] = rational
            times[name] = float(self.core_clock.get_sim_time_at_time(rational))
        self.rgb = images
        self.rgb_sim_time = times
        self.rgb_reference_time = references
        cache = UsdGeom.XformCache(Usd.TimeCode.Default())
        self.rgb_camera_to_world = {
            n: np.asarray(cache.GetLocalToWorldTransform(p)).T.copy() for n, p in self.camera_prims.items()
        }
        self.rgb_step = self.step_count
        self.rgb_episode_time = self.world.current_time - self.time_origin

    def reset(self, *, seed=None, variation=None):
        from dexmate_variations import resolve_grasp_config

        from isaacsim.core.utils.types import ArticulationAction

        if self.closed:
            raise RuntimeError("Environment is closed")
        config = resolve_grasp_config(self.baseline_grasp_config, variation or {"name": "baseline"})
        self.report["grasp_config"] = config
        self.episode_id += 1
        self.world.reset()
        self.robot.set_joint_positions(self.initial)
        self.robot.set_joint_velocities(np.zeros_like(self.initial))
        self.robot.set_solver_position_iteration_count(32)
        self.robot.set_solver_velocity_iteration_count(4)
        # Explicitly restore all free objects, including velocity. No object pose is changed by step().
        self.cube.set_world_pose(np.asarray(config["position_m"]), np.asarray(config["orientation_wxyz"]))
        for body in (self.cube, self.world.scene.get_object("test_cube")):
            body.set_linear_velocity(np.zeros(3))
            body.set_angular_velocity(np.zeros(3))
        self.robot.apply_action(ArticulationAction(joint_positions=self.initial[self.driven_ids],
                                                  joint_indices=self.driven_ids))
        self.report["physics_phase"] = {"name": "reset_settle", "step": -1}
        for settle_step in range(120):
            self.world.step(render=self.render_mode == "physics" or (settle_step + 1) % 4 == 0)
        # Publish reset at rest after settling; solver residual velocity must not
        # become inherited momentum in the next episode.
        self.robot.set_joint_velocities(np.zeros_like(self.initial))
        for body in (self.cube, self.world.scene.get_object("test_cube")):
            body.set_linear_velocity(np.zeros(3))
            body.set_angular_velocity(np.zeros(3))
        self.report["phase_contacts"] = {}
        self.report["first_self_contacts"] = {}
        self.scene["contact_pairs"].clear()
        self.step_count, self.phase_step, self.phase = 0, -1, None
        self.time_origin = self.world.current_time
        self.physics_origin = self.world.current_time_step_index
        self.rgb, self.rgb_sim_time, self.rgb_reference_time = {}, {}, {}
        self.progress = GraspProgress(float(self.cube.get_world_pose()[0][2]))
        self.done = False
        self._capture()
        observation = self._observe(rgb_updated=True)
        return observation, {"episode_id": self.episode_id, "seed": seed, "randomization": False,
                             "variation": config["variation"],
                             "settle_steps": 120, "task_success": False}

    def _observe(self, rgb_updated):
        from pxr import Usd, UsdGeom

        position, quaternion = self.cube.get_world_pose()
        prim = self.world.stage.GetPrimAtPath("/World/Dexmate/L_gripper_base")
        pose = np.asarray(UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())).T
        with self.timings.measure("tactile"):
            tactile = self.sensor.update()
        return {
            "episode_id": self.episode_id, "step": self.step_count,
            "time_s": self.world.current_time - self.time_origin, "sim_time_s": self.world.current_time,
            "physics_step": self.world.current_time_step_index - self.physics_origin,
            "joint_position": self.robot.get_joint_positions().copy(),
            "joint_velocity": self.robot.get_joint_velocities().copy(),
            "object_position_m": position.copy(), "object_orientation_wxyz": quaternion.copy(),
            "object_velocity_m_s": self.cube.get_linear_velocity().copy(),
            "object_angular_velocity_rad_s": self.cube.get_angular_velocity().copy(),
            "gripper_pose": pose, "object_in_gripper_m": (np.linalg.inv(pose) @ np.r_[position, 1])[:3],
            "tactile": tactile, "rgb": self.rgb, "rgb_updated": rgb_updated,
            "rgb_step": self.rgb_step, "rgb_time_s": self.rgb_episode_time,
            "rgb_sim_time_s": self.rgb_sim_time.copy(), "rgb_reference_time": self.rgb_reference_time.copy(),
            "rgb_camera_to_world": {k: v.copy() for k, v in self.rgb_camera_to_world.items()},
        }

    def step(self, action, *, phase="rollout"):
        from dexmate_grasp_regression import contact_flags

        from isaacsim.core.utils.types import ArticulationAction

        if self.closed or self.done:
            raise RuntimeError("Call reset() before stepping a new episode")
        action = validate_action(action, self.action_lower, self.action_upper)
        self.phase_step = self.phase_step + 1 if phase == self.phase else 0
        self.phase = phase
        self.report["physics_phase"] = {"name": phase, "step": self.phase_step}
        before = self.world.current_time_step_index
        self.robot.apply_action(ArticulationAction(joint_positions=action, joint_indices=self.driven_ids))
        render = self.render_mode == "physics" or (self.step_count + 1) % 4 == 0
        with self.timings.measure("world_step"):
            with self.timings.measure("physics_and_render" if render else "physics_only"):
                self.world.step(render=render)
        if self.world.current_time_step_index != before + 1:
            raise RuntimeError("step() must advance exactly one physics step")
        self.step_count += 1
        updated = self.step_count % 4 == 0
        if updated:
            with self.timings.measure("capture"):
                self._capture()
        with self.timings.measure("observe"):
            obs = self._observe(updated)
        loaded = contact_flags(self.report, phase, self.phase_step)
        ids = [self.robot.get_dof_index(f"L_gripper_j{i}") for i in (1, 2)]
        success = self.progress.update(obs["object_position_m"], obs["object_velocity_m_s"], loaded,
                                       obs["tactile"], obs["joint_position"][ids])
        failed = (not np.isfinite(obs["joint_position"]).all()
                  or obs["object_position_m"][2] < self.progress.baseline_z - .1)
        horizon = self.step_count >= self.max_episode_steps
        terminated = bool(failed or (horizon and success))
        truncated = bool(horizon and not success and not failed)
        self.done = terminated or truncated
        info = {"task_success": bool(success), "failure": bool(failed), "phase": phase,
                "phase_step": self.phase_step, "finger_loaded": np.asarray(loaded), "action": action}
        return obs, float(success and horizon and not failed), terminated, truncated, info

    def close(self, exit_code=0):
        if self.closed:
            return
        import omni.kit.app

        self.closed = True
        self._save_scene_report()
        self.scene["contact_subscription"] = None
        omni.kit.app.get_app().post_quit(exit_code)
        self.app.close(skip_cleanup=False)
