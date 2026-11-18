"""
exp7_auction_v2.py - auction v2 under the Exp 2b protocol.
4 robots [0,1,2,0], 10 seeds, 20,000 steps, failures at step 500.
Compares with exp2b_raw.json (original) and exp3_raw.json (greedy_eligible).
Run:  python exp7_auction_v2.py [--n-seeds 10] [--workers 4]
"""
import argparse
import csv
import sys
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path

_ROOT_DIR = Path(__file__).resolve().parent.parent
for _p in [str(_ROOT_DIR / "core"), str(_ROOT_DIR / "experiments"), str(_ROOT_DIR)]:
    if _p not in sys.path:
        sys.path.insert(0, _p)

import numpy as np
from multi_robot_vecenv import MultiRobotVecEnv
from auction_layer_v2 import AuctionLayerV2
from pathfinding import GridPathfinder
from fault_layer import FaultLayer
from exp_utils import ResultStore, RESULTS_DIR, mean_ci95

ALL_SEEDS = [1, 42, 100, 2024, 7, 11, 23, 99, 314, 777]
N_STEPS, FAILURE_STEP = 20000, 500
FAILURES = {"none": [], "robot0_fast": [0], "robot1_heavy": [1], "robot1+2": [1, 2]}


def run_worker_task(args):
    fname, failed, seed = args
    vec = MultiRobotVecEnv(env_id="rware-tiny-4ag-v2", robot_types=[0, 1, 2, 0], seed=seed)
    obs = vec.reset()
    env = vec.hetero_env
    auction = AuctionLayerV2(env, bid_model_path="bid_head_trained.pt")
    path = GridPathfinder(env)
    fault = FaultLayer(env)
    for step in range(N_STEPS):
        if step == FAILURE_STEP:
            for r in failed:
                fault.inject_failure(r)
        bf = fault.step()
        pursue, _, tasks = auction.step(obs, belief_failed=bf)
        acts = fault.get_actions_override(path.get_actions_for_pursuit(obs, pursue, tasks))
        obs, _, _, _ = vec.step(acts)
        auction.clear_stale_beliefs()
    m, mm = sum(env.match_count), sum(env.mismatch_count)
    vec.close()
    return fname, seed, dict(deliveries=int(m + mm), match_rate=(m / (m + mm) * 100) if (m + mm) else 0.0)


def main(n_seeds, workers=1):
    seeds = ALL_SEEDS[:n_seeds]
    store = ResultStore("exp7_raw")

    pending = []
    for fname, failed in FAILURES.items():
        for s in seeds:
            key = f"v2|{fname}|{s}"
            if not store.has(key):
                pending.append((fname, failed, s))

    if pending:
        print(f"Executing {len(pending)} pending runs with {workers} worker process(es)...")
        if workers > 1:
            with ProcessPoolExecutor(max_workers=workers) as executor:
                futures = {executor.submit(run_worker_task, item): item for item in pending}
                for fut in as_completed(futures):
                    fname, s, r = fut.result()
                    key = f"v2|{fname}|{s}"
                    store.set(key, r)
                    print(f"[v2|{fname}] seed={s}: {r['deliveries']} deliveries, {r['match_rate']:.1f}% match")
        else:
            for item in pending:
                fname, s, r = run_worker_task(item)
                key = f"v2|{fname}|{s}"
                store.set(key, r)
                print(f"[v2|{fname}] seed={s}: {r['deliveries']} deliveries, {r['match_rate']:.1f}% match")

    base = {s: store.get(f"v2|none|{s}")["deliveries"] for s in seeds}
    rows = []
    for fname in FAILURES:
        runs = [store.get(f"v2|{fname}|{s}") for s in seeds]
        d = [r["deliveries"] for r in runs]
        ret = [100 * r["deliveries"] / base[s] for r, s in zip(runs, seeds)]
        dm, dsd, dci = mean_ci95(d)
        rm, _, rci = mean_ci95(ret)
        rows.append(dict(failed=fname, deliveries_mean=dm, deliveries_ci95=dci,
                         retention_mean=rm, retention_ci95=rci,
                         match_mean=float(np.mean([r["match_rate"] for r in runs]))))
    with open(RESULTS_DIR / "exp7_summary.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader(); w.writerows(rows)
    print("\n=== EXP 7 (auction v2) SUMMARY ===")
    for r in rows:
        print(f"{r['failed']:13s} deliveries {r['deliveries_mean']:.1f} +/- {r['deliveries_ci95']:.1f} | "
              f"retention {r['retention_mean']:.1f}% +/- {r['retention_ci95']:.1f} | match {r['match_mean']:.1f}%")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--n-seeds", type=int, default=10)
    ap.add_argument("--workers", type=int, default=1)
    args = ap.parse_args()
    main(args.n_seeds, args.workers)
