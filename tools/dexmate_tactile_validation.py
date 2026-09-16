"""Record tactile output from each physics step and validate contact/release phases."""

import importlib.util
import json
import sys
from pathlib import Path

import numpy as np
from dexmate_tactile_preview import export_views


def load_adapter():
    name = "dexmate_flexitac"
    if name not in sys.modules:
        path = Path(__file__).resolve().parents[1] / "Isaacsim_tactile_env/dexmate_flexitac.py"
        spec = importlib.util.spec_from_file_location(name, path)
        module = importlib.util.module_from_spec(spec)
        sys.modules[name] = module
        spec.loader.exec_module(module)
    return sys.modules[name]


class TactileRecorder:
    def __init__(self, stage, assets, cfg, probes, output, world):
        paths = [*cfg["target_prim_paths"], *(str(p.prim.GetPath()) for p in probes.values())]
        self.sensor = load_adapter().DexmateFlexiTac(stage, "/World/Dexmate", assets, cfg, paths)
        self.output = output
        self.frames, self.phases, self.steps = [], [], []
        self.world = world
        self.times, self.physics_steps = [], []

    def sample(self, phase):
        self.frames.append(self.sensor.update())
        self.phases.append(phase["name"])
        self.steps.append(phase["step"])
        self.times.append(self.world.current_time)
        self.physics_steps.append(self.world.current_time_step_index)

    def finish(self, report):
        frames, phases, steps = np.asarray(self.frames), np.asarray(self.phases), np.asarray(self.steps)
        np.savez_compressed(self.output / "tactile_sequence.npz", tactile=frames, phase=phases, step=steps,
                            slots=np.asarray(load_adapter().SLOTS), local_points_m=self.sensor.local_points,
                            time_s=np.asarray(self.times), physics_step=np.asarray(self.physics_steps))
        stats = {}
        peaks = frames.max(axis=(2, 3))
        for phase in dict.fromkeys(self.phases):
            indices = np.flatnonzero(phases == phase)
            stats[phase] = {
                "frames": len(indices), "peak_per_finger": peaks[indices].max(0).tolist(),
                "tail_peak_per_finger": peaks[indices[-20:]].max(0).tolist(),
                "first_nonzero_step": [
                    int(steps[indices[peaks[indices, finger] > 1e-5][0]])
                    if np.any(peaks[indices, finger] > 1e-5) else None for finger in range(4)
                ],
            }
        checks = {
            "finite_and_normalized": bool(np.isfinite(frames).all() and frames.min() >= 0 and frames.max() <= 1),
            "free_space_zero": all(
                max(s["peak_per_finger"]) <= 1e-6 for p, s in stats.items() if p.startswith("free_")
            ),
            "one_sample_per_physics_step_within_phases": bool(
                np.all(np.diff(self.physics_steps)[phases[1:] == phases[:-1]] == 1)
            ),
        }
        if "probe_release" in stats:
            hold = np.maximum(stats["probe_close"]["tail_peak_per_finger"],
                              stats["probe_shift_close"]["tail_peak_per_finger"])
            checks["all_four_respond_when_loaded"] = bool(np.all(hold > 0.1))
            checks["all_four_zero_after_release"] = max(stats["probe_release"]["tail_peak_per_finger"]) <= 1e-6
        report["tactile"] = dict(self.sensor.metadata(), phase_statistics=stats, checks=checks,
                                 passed=all(checks.values()), frame_count=len(frames),
                                 physics_dt_s=self.world.get_physics_dt(),
                                 recorded_time_range_s=[self.times[0], self.times[-1]])
        (self.output / "tactile_metadata.json").write_text(json.dumps(report["tactile"], indent=2) + "\n")
        export_views(frames, phases, steps, np.asarray(self.times), self.output)


def create_recorder(stage, assets, scene, probes, output, world):
    cfg = scene.get("tactile", {})
    return TactileRecorder(stage, assets, cfg, probes, output, world) if cfg.get("enabled", False) else None


def sample_tactile(recorder, report):
    if recorder is not None:
        recorder.sample(report["physics_phase"])


def finish_tactile(recorder, report):
    if recorder is not None:
        recorder.finish(report)
