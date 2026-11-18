"""
Exp 3 - Greedy baselines under the SAME protocol as Exp 2
(4 robots [0,1,2,0], 10 seeds, 20,000 steps, failures at step 500).

  greedy_nearest  : nearest robot takes the nearest task, NO eligibility check
                    (the original naive baseline)
  greedy_eligible : nearest *eligible* robot takes the nearest task, no bidding,
                    no communication  (separates the effect of eligibility from
                    the effect of bidding + claims)

Failed robots are forced to NOOP through FaultLayer, but greedy has no
recovery mechanism, as in the original baseline.

Run:  python exp3_greedy_baselines.py [--n-seeds 10]
"""
import argparse
import csv
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
from pathfinding import GridPathfinder
from fault_layer import FaultLayer
from exp_utils import ResultStore, RESULTS_DIR, mean_ci95

ALL_SEEDS = [1, 42, 100, 2024, 7, 11, 23, 99, 314, 777]
N_STEPS = 20000
FAILURE_STEP = 500
ROBOT_TYPES = [0, 1, 2, 0]
CONDITIONS = {"0_failures": [], "1_failure": [1], "2_failures": [1, 2]}


def dist(a, b):
    return abs(a.x - b.x) + abs(a.y - b.y)


def greedy_assign(env, eligible_only):
    agents = env.unwrapped.agents
    queue = list(env.unwrapped.request_queue)
    n = len(agents)
    nearest = [None] * n
    pursue = [False] * n
    if not queue:
        return pursue, nearest

    target_shelf = [None] * n
    for i, agent in enumerate(agents):
        best_d, best_s = None, None
        for shelf in queue:
            if eligible_only:
                tt = env.shelf_task_type.get(shelf, 2)
                if env.robot_types[i] not in env.ACCEPTABLE_ROBOTS[tt]:
                    continue
            d = dist(agent, shelf)
            if best_d is None or d < best_d:
                best_d, best_s = d, shelf
        if best_s is not None:
            nearest[i] = (best_s.x, best_s.y)
            target_shelf[i] = best_s

    best_for_task = {}
    for i, agent in enumerate(agents):
        s = target_shelf[i]
        if s is None:
            continue
        key = (s.x, s.y)
        d = dist(agent, s)
        if key not in best_for_task or d < best_for_task[key][0] or \
           (d == best_for_task[key][0] and i < best_for_task[key][1]):
            best_for_task[key] = (d, i)
    for key, (d, winner) in best_for_task.items():
        pursue[winner] = True
    return pursue, nearest


def run(variant, failed, seed):
    vec_env = MultiRobotVecEnv(env_id="rware-tiny-4ag-v2", robot_types=ROBOT_TYPES, seed=seed)
    obs = vec_env.reset()
    env = vec_env.hetero_env
    pathfinder = GridPathfinder(env)
    fault = FaultLayer(env)
    eligible_only = (variant == "greedy_eligible")
    for step in range(N_STEPS):
        if step == FAILURE_STEP:
            for r in failed:
                fault.inject_failure(r)
        fault.step()
        pursue, nearest = greedy_assign(env, eligible_only)
        actions = pathfinder.get_actions_for_pursuit(obs, pursue, nearest)
        actions = fault.get_actions_override(actions)
        obs, _, _, _ = vec_env.step(actions)
    m, mm = sum(env.match_count), sum(env.mismatch_count)
    vec_env.close()
    return dict(deliveries=int(m + mm), valid=int(m), mismatched=int(mm),
                match_rate=(m / (m + mm) * 100) if (m + mm) else 0.0)


def main(n_seeds):
    seeds = ALL_SEEDS[:n_seeds]
    store = ResultStore("exp3_raw")
    for variant in ["greedy_nearest", "greedy_eligible"]:
        for cname, failed in CONDITIONS.items():
            for seed in seeds:
                key = f"{variant}|{cname}|{seed}"
                if store.has(key):
                    continue
                r = run(variant, failed, seed)
                store.set(key, r)
                print(f"[{variant}|{cname}] seed={seed}: {r['deliveries']} deliveries, "
                      f"{r['valid']} valid, {r['match_rate']:.1f}% match")

    rows = []
    for variant in ["greedy_nearest", "greedy_eligible"]:
        base = {s: store.get(f"{variant}|0_failures|{s}")["deliveries"] for s in seeds}
        for cname in CONDITIONS:
            runs = [store.get(f"{variant}|{cname}|{s}") for s in seeds]
            d = [r["deliveries"] for r in runs]
            v = [r["valid"] for r in runs]
            ret = [100 * r["deliveries"] / base[s] for r, s in zip(runs, seeds) if base[s] > 0]
            dm, dsd, dci = mean_ci95(d)
            vm, _, vci = mean_ci95(v)
            rm, _, rci = mean_ci95(ret)
            rows.append(dict(variant=variant, condition=cname, n_seeds=len(seeds),
                             deliveries_mean=dm, deliveries_sd=dsd, deliveries_ci95=dci,
                             valid_mean=vm, valid_ci95=vci,
                             match_mean=float(np.mean([r["match_rate"] for r in runs])),
                             retention_mean=rm, retention_ci95=rci))
    path = RESULTS_DIR / "exp3_summary.csv"
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader(); w.writerows(rows)
    print("\n=== EXP 3 SUMMARY (mean +/- 95% CI) ===")
    for r in rows:
        print(f"{r['variant']:16s} {r['condition']:10s} deliveries {r['deliveries_mean']:.1f} +/- {r['deliveries_ci95']:.1f} | "
              f"valid {r['valid_mean']:.1f} +/- {r['valid_ci95']:.1f} | match {r['match_mean']:.1f}% | "
              f"retention {r['retention_mean']:.1f}% +/- {r['retention_ci95']:.1f}")
    print(f"\nSaved {path}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--n-seeds", type=int, default=10)
    main(ap.parse_args().n_seeds)
