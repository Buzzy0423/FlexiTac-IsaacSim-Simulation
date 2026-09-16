"""Measure independent Isaac Sim processes sharing one GPU, with a synchronized start.

Run through run_dexmate_sim.sh in the dexmate_isaacsim Conda environment.
Each worker replays the same full-rate reference; this is a throughput experiment,
not a dataset of distinct demonstrations or a vectorized Isaac Lab environment.
"""

import argparse
import json
import os
import signal
import subprocess
import time
import traceback
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def physx_errors(log_text):
    return [line for line in log_text.splitlines()
            if "[Error]" in line and ("[omni.physx" in line or "PhysX error:" in line)]


def summarize_workers(reports, native_errors=None):
    episodes = [episode for report in reports for episode in report["episodes"]]
    elapsed = (max(report["collection_finished_monotonic"] for report in reports)
               - min(report["collection_started_monotonic"] for report in reports))
    complete = all(item["reference_complete"] for item in episodes)
    passed = sum(bool(item["episode"]["passed"] and item["reference_complete"]) for item in episodes)
    physical_passed = passed
    if native_errors and any(native_errors.values()):
        passed = 0  # The run is invalid even when the scripted grasp checks passed.
    return {
        "episode_count": len(episodes), "passed_count": passed,
        "physical_passed_count": physical_passed,
        "full_reference_complete": complete,
        "all_numeric_samples_identical": complete and all(
            field["max_abs_difference"] == 0 for item in episodes for field in item["full_rate_comparison"].values()),
        "collection_wall_seconds": elapsed,
        "successful_episodes_per_minute": 60 * passed / elapsed,
        "collection_seconds_per_successful_episode": elapsed / passed if passed else None,
        "aggregate_simulated_seconds_per_wall_second": 18 * passed / elapsed,
        "worker_core_seconds": [report["timings"]["env_step"]["seconds"] for report in reports],
        "start_skew_seconds": (max(report["collection_started_monotonic"] for report in reports)
                               - min(report["collection_started_monotonic"] for report in reports)),
    }


def gpu_sample():
    result = subprocess.run(
        ["nvidia-smi", "--query-gpu=index,memory.used,memory.total,utilization.gpu,utilization.memory",
         "--format=csv,noheader,nounits"], capture_output=True, text=True, timeout=5, check=True)
    return [{"gpu": int(values[0]), "memory_used_mib": int(values[1]), "memory_total_mib": int(values[2]),
             "gpu_utilization_percent": int(values[3]), "memory_utilization_percent": int(values[4])}
            for line in result.stdout.splitlines() if (values := line.split(","))]


def run(args):
    args.output.mkdir(parents=True, exist_ok=False)
    gate = args.output / "start.flag"
    started = time.monotonic()
    processes, streams, telemetry = [], [], []
    summary = {"status": "starting", "workers": args.workers, "episodes_per_worker": args.episodes,
               "worker_startup": [],
               "startup_mode": "sequential loading, synchronized parallel collection",
               "cpu_threads_per_worker": args.cpu_threads, "physics_threads_per_worker": args.physics_threads,
               "physics_mode": args.physics_mode, "ccd_requested": args.ccd,
               "scope": "Independent processes, one shared GPU, identical 120 Hz reference replay; five RGB streams"}
    try:
        for worker in range(args.workers):
            output = args.output / f"worker_{worker:02d}"
            command = [str(ROOT / "tools/run_dexmate_sim.sh"), str(ROOT / "tools/benchmark_dexmate_cpu.py"),
                       "--output", str(output), "--reference", str(args.reference),
                       "--episodes", str(args.episodes), "--cpu-threads", str(args.cpu_threads),
                       "--physics-threads", str(args.physics_threads), "--disable-viewport-updates",
                       "--physics-mode", args.physics_mode,
                       "--start-gate", str(gate), "--gate-timeout", str(args.timeout)]
            if args.workaround_r535_vulkan:
                command.append("--workaround-r535-vulkan")
            if args.ccd is not None:
                command.append("--ccd" if args.ccd else "--no-ccd")
            stream = (args.output / f"worker_{worker:02d}.log").open("x")
            streams.append(stream)
            launched = time.monotonic()
            processes.append(subprocess.Popen(command, cwd=ROOT, stdout=stream, stderr=subprocess.STDOUT,
                                              start_new_session=True))
            # Load each scene independently, reusing the installed driver's persistent
            # shader cache, then release all collectors together.
            while not (output / "ready.json").exists():
                if any(process.poll() is not None for process in processes):
                    raise RuntimeError("Worker exited during scene loading; inspect worker logs")
                if time.monotonic() - started > args.timeout:
                    raise TimeoutError("Scene loading exceeded the total timeout")
                telemetry.append({"elapsed_seconds": time.monotonic() - started,
                                  "collecting": False, "gpus": gpu_sample()})
                time.sleep(1)
            ready = json.loads((output / "ready.json").read_text())
            summary["worker_startup"].append({"worker": worker, **ready,
                                               "process_to_ready_seconds": ready["monotonic"] - launched})
            print(f"Worker {worker} ready after {ready['monotonic'] - launched:.2f}s "
                  f"(environment {ready['startup_seconds']:.2f}s, reset {ready['initial_reset_seconds']:.2f}s)",
                  flush=True)
        summary["worker_pids"] = [process.pid for process in processes]
        while True:
            now = time.monotonic()
            if now - started > args.timeout:
                raise TimeoutError("Parallel benchmark exceeded its total timeout")
            exits = [process.poll() for process in processes]
            if any(code not in (None, 0) for code in exits):
                raise RuntimeError(f"Worker failed: exit codes {exits}; inspect worker logs")
            if not gate.exists():
                if any(code is not None for code in exits):
                    raise RuntimeError("Worker exited before the start gate")
                if all((args.output / f"worker_{worker:02d}/ready.json").exists()
                       for worker in range(args.workers)):
                    gate.touch(exist_ok=False)
                    summary["startup_to_gate_seconds"] = time.monotonic() - started
                    print(f"All {args.workers} workers ready; recording begins", flush=True)
            telemetry.append({"elapsed_seconds": now - started, "collecting": gate.exists(), "gpus": gpu_sample()})
            if all(code is not None for code in exits):
                break
            time.sleep(1)
        reports = [json.loads((args.output / f"worker_{worker:02d}/benchmark.json").read_text())
                   for worker in range(args.workers)]
        summary["physx_errors"] = {f"worker_{worker:02d}": physx_errors(
            (args.output / f"worker_{worker:02d}.log").read_text()) for worker in range(args.workers)}
        summary.update(summarize_workers(reports, summary["physx_errors"]))
        accepted = all(report["status"] == "passed" for report in reports) and not any(summary["physx_errors"].values())
        summary["status"] = "passed" if accepted else "failed"
    except BaseException:
        summary.update(status="failed", error=traceback.format_exc())
        traceback.print_exc()
    finally:
        # Terminate only the process groups created here, including any video encoders.
        for process in processes:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGTERM)
        for process in processes:
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        for stream in streams:
            stream.close()
        summary["total_wall_seconds"] = time.monotonic() - started
        summary["exit_codes"] = [process.returncode for process in processes]
        if telemetry:
            summary["peak_gpu_memory_mib"] = max(gpu["memory_used_mib"] for row in telemetry for gpu in row["gpus"])
            active = [gpu["gpu_utilization_percent"] for row in telemetry if row["collecting"] for gpu in row["gpus"]]
            summary["mean_collection_gpu_utilization_percent"] = sum(active) / len(active) if active else None
        (args.output / "gpu_telemetry.json").write_text(json.dumps(telemetry, indent=2) + "\n")
        (args.output / "parallel_benchmark.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2), flush=True)
    return 0 if summary["status"] == "passed" else 2


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workers", type=int, choices=(1, 2, 4), required=True)
    parser.add_argument("--episodes", type=int, default=2)
    parser.add_argument("--cpu-threads", type=int, default=8)
    parser.add_argument("--physics-threads", type=int, default=8)
    parser.add_argument("--physics-mode", choices=("cpu", "gpu"), default="cpu")
    parser.add_argument("--ccd", action=argparse.BooleanOptionalAction, default=None)
    parser.add_argument("--timeout", type=float, default=600)
    parser.add_argument("--reference", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--workaround-r535-vulkan", action="store_true")
    arguments = parser.parse_args()
    arguments.output = arguments.output.resolve()
    arguments.reference = arguments.reference.resolve(strict=True)
    if min(arguments.episodes, arguments.cpu_threads, arguments.physics_threads, arguments.timeout) <= 0:
        parser.error("episode count, threads and timeout must be positive")
    if arguments.physics_mode == "gpu" and arguments.ccd is True:
        parser.error("GPU dynamics requires CCD disabled")
    raise SystemExit(run(arguments))
