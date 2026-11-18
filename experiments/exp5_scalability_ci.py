"""
Exp 5 - Fleet scalability (2 / 4 / 8 robots) with more seeds and 95% CIs.

Same pipeline as phase7_scalability_experiment.py (no failures), but:
  * 10 seeds by default
  * resumable (each finished run is saved)
  * mean, SD and 95% CI for total and per-robot deliveries
  * speed-up vs. 2 robots and parallel efficiency
      efficiency = (deliveries_N / deliveries_2) / (N / 2)
  * writes results/exp5_summary.csv and results/fig4_5_scalability.png

Run from the experiments folder:   python exp5_scalability_ci.py
"""
import csv
import sys
from pathlib import Path

_ROOT_DIR = Path(__file__).resolve().parent.parent
_CORE_DIR = _ROOT_DIR / "core"
for _p in [str(_CORE_DIR), str(_ROOT_DIR)]:
    if _p not in sys.path:
        sys.path.insert(0, _p)

import numpy as np
from multi_robot_vecenv import MultiRobotVecEnv
from auction_layer import DecentralizedAuctionLayer
from pathfinding import GridPathfinder
from exp_utils import ResultStore, RESULTS_DIR, mean_ci95, apply_style, MAIN_COLOR, ACCENT_COLOR

SEEDS = [1, 42, 100, 2024, 7, 11, 23, 99, 314, 777]
N_STEPS = 20000

SWARM_CONFIGS = {
    "2_robots": ("rware-tiny-2ag-v2", [0, 1]),
    "4_robots": ("rware-tiny-4ag-v2", [0, 1, 2, 0]),
    "8_robots": ("rware-tiny-8ag-v2", [0, 1, 2, 0, 1, 2, 0, 1]),
}


def run_seed(env_id, robot_types, seed):
    vec_env = MultiRobotVecEnv(env_id=env_id, robot_types=robot_types, seed=seed)
    obs = vec_env.reset()
    env = vec_env.hetero_env

    auction = DecentralizedAuctionLayer(env, bid_model_path="bid_head_trained.pt")
    pathfinder = GridPathfinder(env)

    for step in range(N_STEPS):
        pursue, bids, nearest_tasks = auction.step(obs)
        actions = pathfinder.get_actions_for_pursuit(obs, pursue, nearest_tasks)
        obs, rewards, dones, infos = vec_env.step(actions)
        auction.clear_stale_beliefs()

    matches = sum(env.match_count)
    mismatches = sum(env.mismatch_count)
    total = matches + mismatches
    match_rate = (matches / total * 100) if total > 0 else 0.0
    n_robots = vec_env.n_robots
    vec_env.close()
    return int(total), float(match_rate), int(n_robots)


def main():
    store = ResultStore("exp5_raw")
    print(f"Exp 5: {len(SWARM_CONFIGS)} fleet sizes x {len(SEEDS)} seeds x {N_STEPS} steps "
          f"(finished runs are skipped on re-run)\n")

    for cfg, (env_id, types) in SWARM_CONFIGS.items():
        for seed in SEEDS:
            key = f"{cfg}|{seed}"
            if store.has(key):
                print(f"  [{cfg}] seed={seed}: cached {store.get(key)}")
                continue
            try:
                d, m, n = run_seed(env_id, types, seed)
            except Exception as e:
                print(f"  [{cfg}] seed={seed}: FAILED ({e})")
                continue
            store.set(key, {"deliveries": d, "match_rate": m, "n_robots": n})
            print(f"  [{cfg}] seed={seed}: {d} deliveries, {m:.1f}% match")

    rows = []
    for cfg in SWARM_CONFIGS:
        runs = [store.get(f"{cfg}|{s}") for s in SEEDS if store.has(f"{cfg}|{s}")]
        if not runs:
            continue
        n = runs[0]["n_robots"]
        dels = [r["deliveries"] for r in runs]
        per_robot = [d / n for d in dels]
        mrs = [r["match_rate"] for r in runs]
        dm, dsd, dci = mean_ci95(dels)
        pm, psd, pci = mean_ci95(per_robot)
        mm, _, mci = mean_ci95(mrs)
        rows.append(dict(config=cfg, n_robots=n, n_seeds=len(runs),
                         total_mean=dm, total_sd=dsd, total_ci95=dci,
                         per_robot_mean=pm, per_robot_sd=psd, per_robot_ci95=pci,
                         match_mean=mm, match_ci95=mci))

    ref = rows[0]  # smallest fleet is the reference
    for r in rows:
        r["speedup_vs_smallest"] = r["total_mean"] / ref["total_mean"]
        r["parallel_efficiency"] = r["speedup_vs_smallest"] / (r["n_robots"] / ref["n_robots"])

    csv_path = RESULTS_DIR / "exp5_summary.csv"
    with open(csv_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)

    print("\n=== SCALABILITY SUMMARY (mean +/- 95% CI) ===")
    for r in rows:
        print(f"{r['n_robots']} robots: total {r['total_mean']:.1f} +/- {r['total_ci95']:.1f} | "
              f"per-robot {r['per_robot_mean']:.2f} +/- {r['per_robot_ci95']:.2f} | "
              f"speed-up {r['speedup_vs_smallest']:.2f}x | efficiency {r['parallel_efficiency']*100:.0f}% | "
              f"match {r['match_mean']:.1f}%")

    # ---------------- plot (Fig 4.5) ----------------
    plt = apply_style()
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 4))
    ns = [r["n_robots"] for r in rows]

    ax1.errorbar(ns, [r["total_mean"] for r in rows], yerr=[r["total_ci95"] for r in rows],
                 marker="o", capsize=5, color=MAIN_COLOR, label="Measured")
    ax1.plot(ns, [ref["total_mean"] * n / ref["n_robots"] for n in ns],
             ls="--", color="gray", label="Ideal linear scaling")
    ax1.set_xticks(ns); ax1.set_xlabel("Number of robots")
    ax1.set_ylabel(f"Total deliveries per {N_STEPS:,} steps")
    ax1.set_title("Total throughput"); ax1.legend()

    ax2.errorbar(ns, [r["per_robot_mean"] for r in rows], yerr=[r["per_robot_ci95"] for r in rows],
                 marker="s", capsize=5, color=ACCENT_COLOR)
    ax2.set_xticks(ns); ax2.set_xlabel("Number of robots")
    ax2.set_ylabel("Deliveries per robot"); ax2.set_title("Per-robot throughput")

    fig.savefig(RESULTS_DIR / "fig4_5_scalability.png")
    print(f"\nSaved {csv_path} and results/fig4_5_scalability.png")


if __name__ == "__main__":
    main()
