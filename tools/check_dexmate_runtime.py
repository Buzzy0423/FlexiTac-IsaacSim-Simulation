"""Check Isaac Sim startup, CPU PhysX and an actual RTX RGB render before robot import."""

import argparse
import importlib.metadata
import json
import subprocess
import traceback
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--workaround-r535-vulkan",
        action="store_true",
        help="NVIDIA documented version-reporting workaround, only for R535.256+",
    )
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    report = {"isaacsim": importlib.metadata.version("isaacsim"), "status": "starting"}

    def save():
        (args.output / "runtime.json").write_text(json.dumps(report, indent=2) + "\n")

    save()
    app = None
    try:
        from isaacsim import SimulationApp

        extra_args = []
        if args.workaround_r535_vulkan:
            driver = (
                subprocess.check_output(
                    ["nvidia-smi", "--query-gpu=driver_version", "--format=csv,noheader"], text=True
                )
                .strip()
                .splitlines()[0]
            )
            parts = [int(part) for part in driver.split(".")]
            if parts[0] != 535 or parts[1] < 256:
                raise ValueError(f"R535 Vulkan workaround does not apply to driver {driver}")
            extra_args.append("--/rtx/verifyDriverVersion/enabled=false")
            report["r535_vulkan_workaround"] = {"actual_driver": driver, "enabled": True}
        app = SimulationApp(
            {"headless": True, "width": 640, "height": 480, "extra_args": extra_args}
        )
        report["status"] = "app_started"
        save()
        import numpy as np
        from PIL import Image

        import omni.replicator.core as rep
        from isaacsim.core.api import World
        from isaacsim.core.api.objects import DynamicCuboid, FixedCuboid
        from pxr import UsdLux

        world = World(stage_units_in_meters=1.0, physics_dt=1 / 120, rendering_dt=1 / 60)
        report["status"] = "world_created"
        save()
        world.scene.add(
            FixedCuboid(
                prim_path="/World/Ground",
                name="ground",
                position=np.array([0.0, 0.0, -0.05]),
                scale=np.array([4.0, 4.0, 0.1]),
                color=np.array([0.3, 0.3, 0.3]),
            )
        )
        cube = world.scene.add(
            DynamicCuboid(
                prim_path="/World/TestCube",
                name="test_cube",
                position=np.array([0.0, 0.0, 0.5]),
                scale=np.array([0.1, 0.1, 0.1]),
                color=np.array([0.2, 0.6, 0.9]),
                mass=0.1,
            )
        )
        light = UsdLux.DomeLight.Define(world.stage, "/World/Light")
        light.CreateIntensityAttr(1000.0)
        camera = rep.create.camera(position=(1.0, 1.0, 0.8), look_at=(0, 0, 0.1))
        product = rep.create.render_product(camera, (640, 480))
        rgb = rep.AnnotatorRegistry.get_annotator("rgb")
        rgb.attach(product)
        report["status"] = "scene_created"
        save()
        world.reset()
        report["status"] = "physics_started"
        save()
        for _ in range(240):
            world.step(render=True)
        position, _ = cube.get_world_pose()
        report["cube_final_position_m"] = position.tolist()
        report["physics_passed"] = bool(abs(position[2] - 0.05) < 0.01)
        rep.orchestrator.step(rt_subframes=4)
        pixels = np.asarray(rgb.get_data())
        report["rgb_shape"] = list(pixels.shape)
        report["render_passed"] = bool(pixels.size and pixels[..., :3].std() > 1.0)
        if pixels.size:
            Image.fromarray(pixels).save(args.output / "runtime.png")
        report["status"] = "passed" if report["physics_passed"] and report["render_passed"] else "failed"
        save()
        if report["status"] != "passed":
            raise RuntimeError("Isaac Sim physics/render check failed; see runtime.json and launch log")
    except BaseException:
        report["status"] = "failed"
        report["error"] = traceback.format_exc()
        save()
        raise
    finally:
        if app is not None:
            import omni.kit.app

            omni.kit.app.get_app().post_quit(0 if report.get("status") == "passed" else 1)
            app.close()


if __name__ == "__main__":
    main()
