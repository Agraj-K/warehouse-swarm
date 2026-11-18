"""
Exp 4 - No-communication ablation under the SAME protocol as Exp 2
(4 robots, 10 seeds, 20,000 steps, failures at step 500).

Uses experiments/auction_layer_nocomm.py (COMM_RANGE = 0). Failed robots are
still forced to NOOP physically; what is missing is peers' ability to detect
the failure and reallocate. Retention is paired by seed against the no-comm
healthy run, and also reported against the with-communication healthy run
from Exp 2 (results/exp2_raw.json) when that file exists.

Run:  python exp4_nocomm_ablation.py [--n-seeds 10]
"""
import argparse
import csv
import json
import sys
from pathlib import Path

_ROOT_DIR = Path(__file__).resolve().parent.parent
_CORE_DIR = _ROOT_DIR / "core"
_EXP_DIR = _ROOT_DIR / "experiments"
for _p in [str(_CORE_DIR), str(_EXP_DIR), str(_ROOT_DIR)]:
    if _p not in sys.path:
        sys.path.insert(0, _p)

import numpy as np
from multi_robot_vecenv import MultiRobotVecEnv
from auction_layer_nocomm import NoCommAuctionLayer
from pathfinding import GridPathfinder
from fault_layer import FaultLayer
from exp_utils import ResultStore, RESULTS_DIR, mean_ci95

ALL_SEEDS = [1, 42, 100, 2024, 7, 11, 23, 99, 314, 777]
N_STEPS = 20000
FAILURE_STEP = 500
ROBOT_TYPES = [0, 1, 2, 0]
CONDITIONS = {"0_failures": [], "1_failure": [1], "2_failures": [1, 2]}


def run(failed, seed):
    vec_env = MultiRobotVecEnv(env_id="rware-tiny-4ag-v2", robot_types=ROBOT_TYPES, seed=seed)
    obs = vec_env.reset()
    env = vec_env.hetero_env
    auction = NoCommAuctionLayer(env, bid_model_path="bid_head_trained.pt")
    pathfinder = GridPathfinder(env)
    fault = FaultLayer(env)
    for step in range(N_STEPS):
        if step == FAILURE_STEP:
            for r in failed:
                fault.inject_failure(r)
        fault.step()
        pursue, bids, tasks = auction.step(obs)
        actions = pathfinder.get_actions_for_pursuit(obs, pursue, tasks)
        actions = fault.get_actions_override(actions)
        obs, _, _, _ = vec_env.step(actions)
    m, mm = sum(env.match_count), sum(env.mismatch_count)
    vec_env.close()
    return dict(deliveries=int(m + mm), match_rate=(m / (m + mm) * 100) if (m + mm) else 0.0)


def main(n_seeds):
    seeds = ALL_SEEDS[:n_seeds]
    store = ResultStore("exp4_raw")
    for cname, failed in CONDITIONS.items():
        for seed in seeds:
            key = f"{cname}|{seed}"
            if store.has(key):
                continue
            r = run(failed, seed)
            store.set(key, r)
            print(f"[nocomm|{cname}] seed={seed}: {r['deliveries']} deliveries, {r['match_rate']:.1f}% match")

    exp2_path = RESULTS_DIR / "exp2_raw.json"
    exp2 = json.loads(exp2_path.read_text()) if exp2_path.exists() else {}

    base = {s: store.get(f"0_failures|{s}")["deliveries"] for s in seeds}
    rows = []
    for cname in CONDITIONS:
        runs = [store.get(f"{cname}|{s}") for s in seeds]
        d = [r["deliveries"] for r in runs]
        ret = [100 * r["deliveries"] / base[s] for r, s in zip(runs, seeds) if base[s] > 0]
        dm, dsd, dci = mean_ci95(d)
        rm, _, rci = mean_ci95(ret)
        row = dict(condition=cname, n_seeds=len(seeds), deliveries_mean=dm, deliveries_sd=dsd,
                   deliveries_ci95=dci, retention_mean=rm, retention_ci95=rci,
                   match_mean=float(np.mean([r["match_rate"] for r in runs])))
        # paired comparison with the with-communication system from Exp 2
        if exp2 and all(f"{cname}|{s}" in exp2 for s in seeds):
            comm = [exp2[f"{cname}|{s}"]["deliveries"] for s in seeds]
            diff = [c - n for c, n in zip(comm, d)]
            dmean, dsd2, dci2 = mean_ci95(diff)
            row.update(comm_minus_nocomm_mean=dmean, comm_minus_nocomm_ci95=dci2)
        rows.append(row)

    keys = sorted({k for r in rows for k in r.keys()}, key=lambda k: list(rows[0].keys()).index(k) if k in rows[0] else 99)
    path = RESULTS_DIR / "exp4_summary.csv"
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader(); w.writerows(rows)

    print("\n=== EXP 4 NO-COMM SUMMARY (mean +/- 95% CI) ===")
    for r in rows:
        extra = ""
        if "comm_minus_nocomm_mean" in r:
            extra = f" | with-comm minus no-comm {r['comm_minus_nocomm_mean']:.1f} +/- {r['comm_minus_nocomm_ci95']:.1f}"
        print(f"{r['condition']:10s} deliveries {r['deliveries_mean']:.1f} +/- {r['deliveries_ci95']:.1f} | "
              f"retention {r['retention_mean']:.1f}% +/- {r['retention_ci95']:.1f} | match {r['match_mean']:.1f}%{extra}")
    print(f"\nSaved {path}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--n-seeds", type=int, default=10)
    main(ap.parse_args().n_seeds)
