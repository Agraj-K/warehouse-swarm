"""
exp8_final_suite.py - final comparison, SEEDED environment, parallel.

Systems : greedy (eligible), nocomm, fixed (original auction + failed-sender
          filter), v2 (retargeting + filter)
Configs : tiny4 (4 robots, 4 failure conditions), tiny2, tiny8,
          small16, medium16 (healthy)
10 seeds x 20,000 steps. Resumable. Run:
    python exp8_final_suite.py --workers 4
Place in warehouse-swarm/experiments/ next to the other scripts.
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
from fault_layer import FaultLayer
from auction_layer_fixed import FixedAuctionLayer
from auction_layer_v2 import AuctionLayerV2
from auction_layer_nocomm import NoCommAuctionLayer
from exp3_greedy_baselines import greedy_assign
from exp_utils import ResultStore, RESULTS_DIR, mean_ci95

SEEDS = [1, 42, 100, 2024, 7, 11, 23, 99, 314, 777]
N_STEPS, FAILURE_STEP = 20000, 500
BID = "bid_head_trained.pt"
T16 = [0, 1, 2] * 5 + [0]
T8 = [0, 1, 2, 0, 1, 2, 0, 1]

CONFIGS = {
    "tiny4":    ("rware-tiny-4ag-v2",    [0, 1, 2, 0]),
    "tiny2":    ("rware-tiny-2ag-v2",    [0, 1]),
    "tiny8":    ("rware-tiny-8ag-v2",    T8),
    "small16":  ("rware-small-16ag-v2",  T16),
    "medium16": ("rware-medium-16ag-v2", T16),
}
FAIL = {"none": (), "robot0": (0,), "robot1": (1,), "robot1+2": (1, 2)}
ALL_SYSTEMS = ["greedy", "nocomm", "fixed", "v2"]


def build_specs():
    specs = []
    for cfg in CONFIGS:
        systems = ALL_SYSTEMS if cfg == "tiny4" else ["greedy", "nocomm", "v2"]
        fails = FAIL if cfg == "tiny4" else {"none": ()}
        for system in systems:
            for fname, failed in fails.items():
                for seed in SEEDS:
                    specs.append((system, cfg, fname, failed, seed))
    return specs


def key_of(spec):
    system, cfg, fname, _, seed = spec
    return f"{system}|{cfg}|{fname}|{seed}"


def _init():
    import torch
    torch.set_num_threads(1)


def run_one(spec):
    system, cfg, fname, failed, seed = spec
    env_id, types = CONFIGS[cfg]
    vec = MultiRobotVecEnv(env_id=env_id, robot_types=list(types), seed=seed)
    obs = vec.reset()
    env = vec.hetero_env
    path = GridPathfinder(env)
    fault = FaultLayer(env) if failed else None
    auction = None
    if system == "nocomm":
        auction = NoCommAuctionLayer(env, bid_model_path=BID)
    elif system == "fixed":
        auction = FixedAuctionLayer(env, bid_model_path=BID)
    elif system == "v2":
        auction = AuctionLayerV2(env, bid_model_path=BID)

    for step in range(N_STEPS):
        bf = None
        if fault is not None:
            if step == FAILURE_STEP:
                for r in failed:
                    fault.inject_failure(r)
            bf = fault.step()
        if system == "greedy":
            pursue, tasks = greedy_assign(env, True)
        elif system == "nocomm":
            pursue, _, tasks = auction.step(obs)
        else:
            pursue, _, tasks = auction.step(obs, belief_failed=bf)
        actions = path.get_actions_for_pursuit(obs, pursue, tasks)
        if fault is not None:
            actions = fault.get_actions_override(actions)
        obs, _, _, _ = vec.step(actions)
        if auction is not None:
            auction.clear_stale_beliefs()

    m, mm = sum(env.match_count), sum(env.mismatch_count)
    vec.close()
    return key_of(spec), dict(deliveries=int(m + mm),
                              match_rate=(m / (m + mm) * 100) if (m + mm) else 0.0)


def summarize(store):
    rows = []
    for system in ALL_SYSTEMS:
        for cfg in CONFIGS:
            for fname in FAIL:
                keys = [f"{system}|{cfg}|{fname}|{s}" for s in SEEDS]
                if not all(store.has(k) for k in keys):
                    continue
                runs = [store.get(k) for k in keys]
                d = [r["deliveries"] for r in runs]
                dm, dsd, dci = mean_ci95(d)
                row = dict(system=system, config=cfg, failed=fname, n=len(SEEDS),
                           deliveries=dm, sd=dsd, ci95=dci,
                           match=float(np.mean([r["match_rate"] for r in runs])))
                base_keys = [f"{system}|{cfg}|none|{s}" for s in SEEDS]
                if cfg == "tiny4" and all(store.has(k) for k in base_keys):
                    ret = [100 * store.get(k)["deliveries"] / store.get(b)["deliveries"]
                           for k, b in zip(keys, base_keys)]
                    rm, _, rci = mean_ci95(ret)
                    row.update(retention=rm, retention_ci95=rci)
                rows.append(row)
    if not rows:
        return
    names = sorted({k for r in rows for k in r}, key=lambda k: list(rows[0]).index(k) if k in rows[0] else 99)
    with open(RESULTS_DIR / "exp8_summary.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=names)
        w.writeheader(); w.writerows(rows)
    print("\n=== EXP 8 FINAL SUMMARY (mean +/- 95% CI, seeded) ===")
    for r in rows:
        ret = f" | retention {r['retention']:.1f}% +/- {r['retention_ci95']:.1f}" if "retention" in r else ""
        print(f"{r['system']:7s} {r['config']:9s} {r['failed']:9s} deliveries {r['deliveries']:7.1f} +/- {r['ci95']:5.1f} "
              f"| match {r['match']:.1f}%{ret}")
    # paired (seed-matched) v2 minus nocomm on tiny4
    print("\nSeed-matched v2 minus nocomm (tiny4):")
    for fname in FAIL:
        try:
            diff = [store.get(f"v2|tiny4|{fname}|{s}")["deliveries"] - store.get(f"nocomm|tiny4|{fname}|{s}")["deliveries"]
                    for s in SEEDS]
            m, _, ci = mean_ci95(diff)
            print(f"  {fname:9s} {m:7.1f} +/- {ci:5.1f}")
        except KeyError:
            pass


def main(workers):
    store = ResultStore("exp8_raw")
    todo = [s for s in build_specs() if not store.has(key_of(s))]
    print(f"{len(todo)} runs to do ({len(build_specs()) - len(todo)} cached)")
    if todo:
        with Pool(workers, initializer=_init) as pool:
            for i, (k, res) in enumerate(pool.imap_unordered(run_one, todo), 1):
                store.set(k, res)
                print(f"[{i}/{len(todo)}] {k}: {res['deliveries']} deliveries, {res['match_rate']:.1f}% match", flush=True)
    summarize(store)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--workers", type=int, default=4)
    main(ap.parse_args().workers)
