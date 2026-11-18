"""
Exp 6 / 7 - Zone-based coordination, 20,000-step protocol, 10 seeds, 95% CIs.

Configurations (all healthy unless stated):
  flat_8_tiny        8 robots, flat, 11x10            (same as Exp 5, 8 robots)
  flat_16_small      16 robots, flat, 20x10           <- like-for-like control
  hier_16_small_4z   16 robots, 4 zones, 20x10
  flat_16_medium     16 robots, flat, 20x16           <- like-for-like control
  hier_16_medium_2z  16 robots, 2 zones, 20x16        (zone-size sensitivity)
  hier_16_medium_4z  16 robots, 4 zones, 20x16
  hier_16_medium_8z  16 robots, 8 zones, 20x16        (zone-size sensitivity)
  hier_16_medium_4z_zonefail
        4 zones, 20x16, all heavy_load robots in robot-1's zone fail at step 500
        (tests the handoff when a zone loses its only eligible robots)

IMPORTANT - metric definitions (state them in the report):
  * flat  : comm events = number of robot pairs within COMM_RANGE per step
            (geometry-based), divided by robots.
  * zoned : comm events as counted by HierarchicalCoordinator
            (broadcasts that reach a peer inside the same zone).
  These two counters are not identical, so compare flat vs zoned mainly on
  deliveries and match rate, and use the communication numbers as indicative.

Run:  python exp6_zone_scaling.py [--n-seeds 10] [--only hier_16_medium_4z]
Resumable; writes results/exp6_summary.csv
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
from zone_partitioner import ZonePartitioner
from hierarchical_coordinator import HierarchicalCoordinator
from auction_layer import DecentralizedAuctionLayer
from pathfinding import GridPathfinder
from exp_utils import ResultStore, RESULTS_DIR, mean_ci95

ALL_SEEDS = [1, 42, 100, 2024, 7, 11, 23, 99, 314, 777]
N_STEPS = 20000
FAILURE_STEP = 500
T16 = [0, 1, 2] * 5 + [0]
T8 = [0, 1, 2, 0, 1, 2, 0, 1]

CONFIGS = {
    "flat_8_tiny":       dict(env="rware-tiny-8ag-v2",    types=T8,  mode="flat"),
    "flat_16_small":     dict(env="rware-small-16ag-v2",  types=T16, mode="flat"),
    "hier_16_small_4z":  dict(env="rware-small-16ag-v2",  types=T16, mode="hier", zones=4),
    "flat_16_medium":    dict(env="rware-medium-16ag-v2", types=T16, mode="flat"),
    "hier_16_medium_2z": dict(env="rware-medium-16ag-v2", types=T16, mode="hier", zones=2),
    "hier_16_medium_4z": dict(env="rware-medium-16ag-v2", types=T16, mode="hier", zones=4),
    "hier_16_medium_8z": dict(env="rware-medium-16ag-v2", types=T16, mode="hier", zones=8),
    "hier_16_medium_4z_zonefail": dict(env="rware-medium-16ag-v2", types=T16, mode="hier",
                                       zones=4, zonefail=True),
}


def run_flat(cfg, seed):
    vec_env = MultiRobotVecEnv(env_id=cfg["env"], robot_types=cfg["types"], seed=seed)
    obs = vec_env.reset()
    env = vec_env.hetero_env
    auction = DecentralizedAuctionLayer(env, bid_model_path="bid_head_trained.pt")
    pathfinder = GridPathfinder(env)
    n = vec_env.n_robots
    events = []
    for step in range(N_STEPS):
        pursue, bids, tasks = auction.step(obs)
        actions = pathfinder.get_actions_for_pursuit(obs, pursue, tasks)
        obs, _, _, _ = vec_env.step(actions)
        auction.clear_stale_beliefs()
        ag = env.unwrapped.agents
        pairs = sum(1 for i in range(n) for j in range(i + 1, n)
                    if abs(ag[i].x - ag[j].x) + abs(ag[i].y - ag[j].y) <= DecentralizedAuctionLayer.COMM_RANGE)
        events.append(pairs)
    m, mm = sum(env.match_count), sum(env.mismatch_count)
    vec_env.close()
    return dict(deliveries=int(m + mm), match_rate=(m / (m + mm) * 100) if (m + mm) else 0.0,
                comm_per_robot_per_step=float(np.mean(events) / n), handoff_rate=0.0, n_robots=n)


def run_hier(cfg, seed):
    vec_env = MultiRobotVecEnv(env_id=cfg["env"], robot_types=cfg["types"], seed=seed)
    obs = vec_env.reset()
    env = vec_env.hetero_env
    ua = env.unwrapped
    grid_w, grid_h = ua.grid_size[1], ua.grid_size[0]
    partitioner = ZonePartitioner(grid_w, grid_h, n_zones=cfg["zones"])
    zone_assignment = partitioner.assign_robots_to_zones(ua.agents)
    coordinator = HierarchicalCoordinator(env, partitioner, zone_assignment,
                                          bid_model_path="bid_head_trained.pt")

    to_fail = []
    if cfg.get("zonefail"):
        z = zone_assignment[1]
        to_fail = [r for r, zz in zone_assignment.items() if zz == z and cfg["types"][r] == 1]

    for step in range(N_STEPS):
        if step == FAILURE_STEP:
            for r in to_fail:
                coordinator.inject_failure(r)
        pursue, bids, tasks = coordinator.step(obs)
        actions = coordinator.pathfinder.get_actions_for_pursuit(obs, pursue, tasks)
        actions = coordinator.get_fault_action_overrides(actions)
        obs, _, _, _ = vec_env.step(actions)
        coordinator.clear_stale_beliefs()

    m, mm = coordinator.total_match_count(), coordinator.total_mismatch_count()
    seen = coordinator.total_tasks_seen
    out = dict(deliveries=int(m + mm), match_rate=(m / (m + mm) * 100) if (m + mm) else 0.0,
               comm_per_robot_per_step=float(coordinator.avg_comm_per_robot_per_step()),
               handoff_rate=(coordinator.cross_zone_handoffs / seen * 100) if seen else 0.0,
               n_robots=vec_env.n_robots, failed_robots=to_fail)
    vec_env.close()
    return out


def main(n_seeds, only):
    seeds = ALL_SEEDS[:n_seeds]
    store = ResultStore("exp6_raw")
    names = [only] if only else list(CONFIGS)
    for name in names:
        cfg = CONFIGS[name]
        for seed in seeds:
            key = f"{name}|{seed}"
            if store.has(key):
                continue
            try:
                r = run_flat(cfg, seed) if cfg["mode"] == "flat" else run_hier(cfg, seed)
            except Exception as e:
                print(f"[{name}] seed={seed}: FAILED {type(e).__name__}: {e}")
                continue
            store.set(key, r)
            print(f"[{name}] seed={seed}: {r['deliveries']} deliveries, {r['match_rate']:.1f}% match, "
                  f"comm/robot/step {r['comm_per_robot_per_step']:.3f}, handoff {r['handoff_rate']:.1f}%")

    rows = []
    for name in CONFIGS:
        runs = [store.get(f"{name}|{s}") for s in seeds if store.has(f"{name}|{s}")]
        if not runs:
            continue
        dm, dsd, dci = mean_ci95([r["deliveries"] for r in runs])
        cm, _, cci = mean_ci95([r["comm_per_robot_per_step"] for r in runs])
        hm, _, hci = mean_ci95([r["handoff_rate"] for r in runs])
        rows.append(dict(config=name, n_robots=runs[0]["n_robots"], n_seeds=len(runs),
                         deliveries_mean=dm, deliveries_sd=dsd, deliveries_ci95=dci,
                         match_mean=float(np.mean([r["match_rate"] for r in runs])),
                         comm_mean=cm, comm_ci95=cci, handoff_mean=hm, handoff_ci95=hci))
    path = RESULTS_DIR / "exp6_summary.csv"
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader(); w.writerows(rows)
    print("\n=== EXP 6 SUMMARY (mean +/- 95% CI) ===")
    for r in rows:
        print(f"{r['config']:28s} {r['n_robots']:2d} robots | deliveries {r['deliveries_mean']:.1f} +/- {r['deliveries_ci95']:.1f} | "
              f"match {r['match_mean']:.1f}% | comm/robot/step {r['comm_mean']:.3f} +/- {r['comm_ci95']:.3f} | "
              f"handoff {r['handoff_mean']:.1f}% +/- {r['handoff_ci95']:.1f}")
    print(f"\nSaved {path}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--n-seeds", type=int, default=10)
    ap.add_argument("--only", choices=list(CONFIGS), default=None)
    a = ap.parse_args()
    main(a.n_seeds, a.only)
