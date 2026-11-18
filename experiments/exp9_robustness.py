"""
experiments/exp9_robustness.py - Robustness sweeps for heartbeat loss, delay, and timeout.

Evaluations on tiny4 (4 robots) across 10 seeds:
  1. Base Baseline:
     - v2: loss=0, delay=0, timeout=15 (healthy, robot0) -> 20 runs
     - greedy: baseline (healthy, robot0) -> 20 runs (greedy ignores heartbeat beliefs)
  2. Message Loss Sweep (v2 only):
     - loss in [0.05, 0.10, 0.20], delay=0, timeout=15 (healthy, robot0) -> 60 runs
  3. Transmission Delay Sweep (v2 only):
     - delay in [1, 3, 5], loss=0.0, timeout=15 (healthy, robot0) -> 60 runs
  4. Timeout Sweep (v2 only):
     - timeout in [5, 10, 30], loss=0.0, delay=0 (healthy, robot0) -> 60 runs

Total runs: 20 + 20 + 60 + 60 + 60 = 220 runs.
Resumable with ResultStore('exp9_raw'). Run:
    python experiments/exp9_robustness.py --workers 6
"""
import argparse
import csv
import sys
from multiprocessing import Pool
from pathlib import Path

_ROOT_DIR = Path(__file__).resolve().parent.parent
for _p in [str(_ROOT_DIR / "core"), str(_ROOT_DIR / "experiments"), str(_ROOT_DIR)]:
    if _p not in sys.path:
        sys.path.insert(0, _p)

import numpy as np
from multi_robot_vecenv import MultiRobotVecEnv
from pathfinding import GridPathfinder
from lossy_fault_layer import LossyFaultLayer
from auction_layer_v2 import AuctionLayerV2
from exp3_greedy_baselines import greedy_assign
from exp_utils import ResultStore, RESULTS_DIR, mean_ci95

SEEDS = [1, 42, 100, 2024, 7, 11, 23, 99, 314, 777]
N_STEPS = 20000
FAILURE_STEP = 500
BID = "bid_head_trained.pt"
ENV_ID = "rware-tiny-4ag-v2"
ROBOT_TYPES = [0, 1, 2, 0]

CONDITIONS = {"none": (), "robot0": (0,)}


def build_specs():
    specs = []

    # 1. Base setting (loss=0, delay=0, timeout=15): v2 and greedy
    for fname, failed in CONDITIONS.items():
        for seed in SEEDS:
            specs.append(("v2", fname, failed, seed, "base", "baseline", 0.0, 0, 15))
            specs.append(("greedy", fname, failed, seed, "base", "baseline", 0.0, 0, 15))

    # 2. Loss sweep (v2 only: 0.05, 0.10, 0.20)
    for loss in [0.05, 0.10, 0.20]:
        for fname, failed in CONDITIONS.items():
            for seed in SEEDS:
                specs.append(("v2", fname, failed, seed, "loss", loss, loss, 0, 15))

    # 3. Delay sweep (v2 only: 1, 3, 5)
    for delay in [1, 3, 5]:
        for fname, failed in CONDITIONS.items():
            for seed in SEEDS:
                specs.append(("v2", fname, failed, seed, "delay", delay, 0.0, delay, 15))

    # 4. Timeout sweep (v2 only: 5, 10, 30)
    for timeout in [5, 10, 30]:
        for fname, failed in CONDITIONS.items():
            for seed in SEEDS:
                specs.append(("v2", fname, failed, seed, "timeout", timeout, 0.0, 0, timeout))

    return specs


def key_of(spec):
    system, fname, _, seed, sweep_name, sweep_val, _, _, _ = spec
    return f"{system}|{fname}|{sweep_name}_{sweep_val}|{seed}"


def _init():
    import torch
    torch.set_num_threads(1)


def run_one(spec):
    system, fname, failed, seed, sweep_name, sweep_val, loss, delay, timeout = spec

    vec = MultiRobotVecEnv(env_id=ENV_ID, robot_types=list(ROBOT_TYPES), seed=seed)
    obs = vec.reset()
    env = vec.hetero_env
    path = GridPathfinder(env)

    # Always create LossyFaultLayer, even for healthy runs, to test spurious failure detection!
    fault = LossyFaultLayer(env, loss=loss, delay=delay, timeout=timeout, seed=seed)

    auction = None
    if system == "v2":
        auction = AuctionLayerV2(env, bid_model_path=BID)

    for step in range(N_STEPS):
        if step == FAILURE_STEP and failed:
            for r in failed:
                fault.inject_failure(r)

        bf = fault.step()

        if system == "greedy":
            pursue, tasks = greedy_assign(env, True)
        elif system == "v2":
            pursue, _, tasks = auction.step(obs, belief_failed=bf)

        actions = path.get_actions_for_pursuit(obs, pursue, tasks)
        actions = fault.get_actions_override(actions)

        obs, _, _, _ = vec.step(actions)
        if auction is not None:
            auction.clear_stale_beliefs()

    m, mm = sum(env.match_count), sum(env.mismatch_count)
    vec.close()

    det_delay = fault.detect_step.get(0, None)
    if det_delay is not None:
        det_delay -= FAILURE_STEP

    res = dict(
        deliveries=int(m + mm),
        match_rate=(m / (m + mm) * 100) if (m + mm) else 0.0,
        false_positive_steps=fault.false_positive_steps,
        det_delay=det_delay,
    )
    return key_of(spec), res


def summarize(store):
    print("\n" + "=" * 90)
    print("EXP 9 ROBUSTNESS EVALUATION SUMMARY (10 Seeds, mean +/- 95% CI)")
    print("=" * 90)

    rows = []

    # Define the sweep presentation order
    eval_groups = [
        ("BASE BASELINE", [("base", "baseline", 0.0, 0, 15)], ["v2", "greedy"]),
        ("LOSS SWEEP", [( "loss", 0.05, 0.05, 0, 15), ("loss", 0.10, 0.10, 0, 15), ("loss", 0.20, 0.20, 0, 15)], ["v2"]),
        ("DELAY SWEEP", [("delay", 1, 0.0, 1, 15), ("delay", 3, 0.0, 3, 15), ("delay", 5, 0.0, 5, 15)], ["v2"]),
        ("TIMEOUT SWEEP", [("timeout", 5, 0.0, 0, 5), ("timeout", 10, 0.0, 0, 10), ("timeout", 30, 0.0, 0, 30)], ["v2"]),
    ]

    for group_name, param_list, systems in eval_groups:
        print(f"\n--- {group_name} ---")
        for sweep_name, sweep_val, loss, delay, timeout in param_list:
            for system in systems:
                for fname in ["none", "robot0"]:
                    keys = [f"{system}|{fname}|{sweep_name}_{sweep_val}|{s}" for s in SEEDS]
                    if not all(store.has(k) for k in keys):
                        continue
                    runs = [store.get(k) for k in keys]

                    delivs = [r["deliveries"] for r in runs]
                    dm, dsd, dci = mean_ci95(delivs)

                    matches = [r["match_rate"] for r in runs]
                    mm = float(np.mean(matches))

                    fps = [r["false_positive_steps"] for r in runs]
                    fpm, _, fpci = mean_ci95(fps)

                    delays = [r["det_delay"] for r in runs if r["det_delay"] is not None]
                    delay_m, _, delay_ci = mean_ci95(delays) if delays else (None, None, None)

                    # Compute seed-matched retention against the system's baseline healthy
                    retention_str = ""
                    rm, rci = None, None
                    if fname == "robot0":
                        base_keys = [f"{system}|none|base_baseline|{s}" for s in SEEDS]
                        if all(store.has(b) for b in base_keys):
                            ret = [100.0 * store.get(k)["deliveries"] / store.get(b)["deliveries"]
                                   for k, b in zip(keys, base_keys)]
                            rm, _, rci = mean_ci95(ret)
                            retention_str = f" | ret {rm:5.1f}% +/- {rci:4.1f}%"

                    delay_str = f" | det_delay {delay_m:4.1f} +/- {delay_ci:3.1f}" if delay_m is not None else ""
                    print(f"[{sweep_name:7s} = {str(sweep_val):8s}] {system:6s} {fname:6s} | "
                          f"deliv {dm:6.1f} +/- {dci:4.1f} | match {mm:5.1f}% | "
                          f"FP_steps {fpm:5.1f} +/- {fpci:4.1f}{delay_str}{retention_str}")

                    rows.append(dict(
                        sweep=sweep_name,
                        val=sweep_val,
                        loss=loss,
                        delay=delay,
                        timeout=timeout,
                        system=system,
                        condition=fname,
                        deliveries=dm,
                        deliv_ci95=dci,
                        match_rate=mm,
                        fp_steps=fpm,
                        fp_ci95=fpci,
                        det_delay=delay_m,
                        det_delay_ci95=delay_ci,
                        retention=rm,
                        retention_ci95=rci
                    ))

    if rows:
        summary_csv = RESULTS_DIR / "exp9_summary.csv"
        names = list(rows[0].keys())
        with open(summary_csv, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=names)
            w.writeheader()
            w.writerows(rows)
        print(f"\nSummary saved to: {summary_csv}")


def main(workers):
    store = ResultStore("exp9_raw")
    specs = build_specs()
    todo = [s for s in specs if not store.has(key_of(s))]
    print(f"Total specs: {len(specs)} | To execute: {len(todo)} | Already cached: {len(specs) - len(todo)}")

    if todo:
        print(f"Launching worker pool with {workers} parallel processes...")
        with Pool(workers, initializer=_init) as pool:
            for i, (k, res) in enumerate(pool.imap_unordered(run_one, todo), 1):
                store.set(k, res)
                det_info = f"det_delay={res['det_delay']}" if res['det_delay'] is not None else ""
                print(f"[{i:3d}/{len(todo):3d}] {k}: {res['deliveries']} deliv, {res['match_rate']:.1f}% match, "
                      f"FP={res['false_positive_steps']} {det_info}", flush=True)

    summarize(store)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--workers", type=int, default=6)
    main(ap.parse_args().workers)
