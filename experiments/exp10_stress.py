"""
experiments/exp10_stress.py - Stress-testing warehouse swarm coordination to empirical breaking points.

Sweeps on tiny4 (4 robots, 20,000 steps, 10 seeds):
  1. Stress Grid (160 runs):
     - tau in [5, 15] x loss in [0.40, 0.60, 0.80, 0.90]
     - Conditions: healthy ('none') and robot 0 fails at step 500 ('robot0')
  2. Radio Outage Sweep (30 runs):
     - Temporary radio muting of Robot 0 at step 500 for mute_len in [10, 20, 40] steps
     - Condition: healthy ('none' - robot is healthy but silent)
     - Baseline tau=15, loss=0.0
  3. Lossy Task Claims Sweep (30 runs):
     - claim_loss in [0.05, 0.10, 0.20] in AuctionLayerV2 peer conflict resolution
     - Condition: healthy ('none')
     - Baseline tau=15, loss=0.0

Total new runs: 160 + 30 + 30 = 220 runs.
Resumable with ResultStore('exp10_raw'). Run:
    python experiments/exp10_stress.py --workers 6
"""
import argparse
import csv
import json
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

    # 1. Stress Grid: tau in [5, 15] x loss in [0.4, 0.6, 0.8, 0.9]
    for tau in [5, 15]:
        for loss in [0.40, 0.60, 0.80, 0.90]:
            tag = f"tau{tau}_loss{loss:.2f}"
            for fname, failed in CONDITIONS.items():
                for seed in SEEDS:
                    specs.append((
                        "v2", fname, failed, seed, "grid", tag,
                        loss, 0, tau, None, 0, 0, 0.0
                    ))

    # 2. Radio Outage: mute_len in [10, 20, 40] on healthy condition
    for mlen in [10, 20, 40]:
        tag = f"len{mlen}"
        for seed in SEEDS:
            specs.append((
                "v2", "none", (), seed, "outage", tag,
                0.0, 0, 15, 0, FAILURE_STEP, mlen, 0.0
            ))

    # 3. Lossy Claims: claim_loss in [0.05, 0.10, 0.20] on healthy condition
    for closs in [0.05, 0.10, 0.20]:
        tag = f"closs{closs:.2f}"
        for seed in SEEDS:
            specs.append((
                "v2", "none", (), seed, "claims", tag,
                0.0, 0, 15, None, 0, 0, closs
            ))

    return specs


def key_of(spec):
    system, fname, _, seed, sweep_name, sweep_val, _, _, _, _, _, _, _ = spec
    return f"{system}|{fname}|{sweep_name}_{sweep_val}|{seed}"


def _init():
    import torch
    torch.set_num_threads(1)


def run_one(spec):
    (
        system, fname, failed, seed, sweep_name, sweep_val,
        loss, delay, timeout, mute_robot, mute_start, mute_len, claim_loss
    ) = spec

    vec = MultiRobotVecEnv(env_id=ENV_ID, robot_types=list(ROBOT_TYPES), seed=seed)
    obs = vec.reset()
    env = vec.hetero_env
    path = GridPathfinder(env)

    fault = LossyFaultLayer(
        env,
        loss=loss,
        delay=delay,
        timeout=timeout,
        seed=seed,
        mute_robot=mute_robot,
        mute_start=mute_start,
        mute_len=mute_len,
    )

    auction = AuctionLayerV2(
        env,
        bid_model_path=BID,
        claim_loss=claim_loss,
        seed=seed,
    )

    for step in range(N_STEPS):
        if step == FAILURE_STEP and failed:
            for r in failed:
                fault.inject_failure(r)

        bf = fault.step()
        pursue, _, tasks = auction.step(obs, belief_failed=bf)

        actions = path.get_actions_for_pursuit(obs, pursue, tasks)
        actions = fault.get_actions_override(actions)

        obs, _, _, _ = vec.step(actions)
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
        dual_claims=auction.dual_claims,
    )
    return key_of(spec), res


def import_baselines(store):
    """Seed exp10 with identical baseline runs from exp9 if available."""
    exp9_file = RESULTS_DIR / "exp9_raw.json"
    if not exp9_file.exists():
        return
    with open(exp9_file, "r") as f:
        exp9_raw = json.load(f)

    for fname in ["none", "robot0"]:
        for s in SEEDS:
            k9 = f"v2|{fname}|base_baseline|{s}"
            k10 = f"v2|{fname}|base_baseline|{s}"
            if not store.has(k10) and k9 in exp9_raw:
                val = dict(exp9_raw[k9])
                val["dual_claims"] = 0
                store.set(k10, val)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--workers", type=int, default=6)
    args = parser.parse_args()

    store = ResultStore("exp10_raw")
    import_baselines(store)

    specs = build_specs()
    pending = [s for s in specs if not store.has(key_of(s))]
    print(f"Exp 10 Stress Evaluation: {len(specs)} total specs, {len(pending)} pending (workers={args.workers})")

    if pending:
        if args.workers > 1:
            with Pool(args.workers, initializer=_init) as p:
                for k, v in p.imap_unordered(run_one, pending):
                    store.set(k, v)
        else:
            _init()
            for s in pending:
                k, v = run_one(s)
                store.set(k, v)

    print("\nCompleted all runs. Analyzing results...")


if __name__ == "__main__":
    main()

