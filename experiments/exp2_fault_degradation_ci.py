"""
Exp 2 - Fault-degradation curve with more seeds and 95% confidence intervals.

Same pipeline and failure conditions as phase6_degradation_experiment.py
(bidding + eligibility gate + A* + heartbeat + reallocation, 4 robots,
failures injected at step 500), but:
  * 10 seeds by default (edit SEEDS)
  * every finished (condition, seed) run is saved, so you can Ctrl+C and resume
  * reports mean, SD and 95% CI for deliveries and match rate
  * retention % is computed PAIRED by seed (condition / same-seed baseline),
    then averaged, with a CI
  * writes results/exp2_summary.csv and results/fig4_2_fault_degradation.png

Run from the experiments folder:   python exp2_fault_degradation_ci.py
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
from fault_layer import FaultLayer
from exp_utils import ResultStore, RESULTS_DIR, mean_ci95, apply_style, MAIN_COLOR, ACCENT_COLOR

SEEDS = [1, 42, 100, 2024, 7, 11, 23, 99, 314, 777]   # first 4 = your original seeds
N_STEPS = 20000
FAILURE_STEP = 500
ROBOT_TYPES = [0, 1, 2, 0]

FAILURE_CONDITIONS = {
    "0_failures": [],
    "1_failure": [1],        # heavy_load fails
    "2_failures": [1, 2],    # heavy_load + balanced fail
}
LABELS = {"0_failures": "0 failures", "1_failure": "1 failure (25%)", "2_failures": "2 failures (50%)"}


def run_condition(failed_robots, seed):
    vec_env = MultiRobotVecEnv(env_id="rware-tiny-4ag-v2", robot_types=ROBOT_TYPES, seed=seed)
    obs = vec_env.reset()
    env = vec_env.hetero_env

    auction = DecentralizedAuctionLayer(env, bid_model_path="bid_head_trained.pt")
    pathfinder = GridPathfinder(env)
    fault_layer = FaultLayer(env)

    injected = False
    for step in range(N_STEPS):
        if not injected and step == FAILURE_STEP and failed_robots:
            for r in failed_robots:
                fault_layer.inject_failure(r)
            injected = True

        belief_failed = fault_layer.step()
        pursue, bids, nearest_tasks = auction.step(obs, belief_failed=belief_failed)
        actions = pathfinder.get_actions_for_pursuit(obs, pursue, nearest_tasks)
        actions = fault_layer.get_actions_override(actions)

        obs, rewards, dones, infos = vec_env.step(actions)
        auction.clear_stale_beliefs()

    matches = sum(env.match_count)
    mismatches = sum(env.mismatch_count)
    total = matches + mismatches
    match_rate = (matches / total * 100) if total > 0 else 0.0
    vec_env.close()
    return int(total), float(match_rate)


def main():
    store = ResultStore("exp2_raw")
    print(f"Exp 2: {len(FAILURE_CONDITIONS)} conditions x {len(SEEDS)} seeds x {N_STEPS} steps "
          f"(finished runs are skipped on re-run)\n")

    for cond, failed in FAILURE_CONDITIONS.items():
        for seed in SEEDS:
            key = f"{cond}|{seed}"
            if store.has(key):
                print(f"  [{cond}] seed={seed}: cached {store.get(key)}")
                continue
            d, m = run_condition(failed, seed)
            store.set(key, {"deliveries": d, "match_rate": m})
            print(f"  [{cond}] seed={seed}: {d} deliveries, {m:.1f}% match")

    # ---------------- summarise ----------------
    base = {s: store.get(f"0_failures|{s}")["deliveries"] for s in SEEDS}
    rows = []
    for cond in FAILURE_CONDITIONS:
        dels = [store.get(f"{cond}|{s}")["deliveries"] for s in SEEDS]
        mrs = [store.get(f"{cond}|{s}")["match_rate"] for s in SEEDS]
        ret = [100.0 * store.get(f"{cond}|{s}")["deliveries"] / base[s]
               for s in SEEDS if base[s] > 0]
        dm, dsd, dci = mean_ci95(dels)
        mm, msd, mci = mean_ci95(mrs)
        rm, rsd, rci = mean_ci95(ret)
        rows.append(dict(condition=cond, n_seeds=len(SEEDS),
                         deliveries_mean=dm, deliveries_sd=dsd, deliveries_ci95=dci,
                         match_mean=mm, match_ci95=mci,
                         retention_mean=rm, retention_sd=rsd, retention_ci95=rci))

    csv_path = RESULTS_DIR / "exp2_summary.csv"
    with open(csv_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)

    print("\n=== DEGRADATION SUMMARY (mean +/- 95% CI) ===")
    for r in rows:
        print(f"{r['condition']:11s} deliveries {r['deliveries_mean']:.1f} +/- {r['deliveries_ci95']:.1f} "
              f"(SD {r['deliveries_sd']:.1f}) | retention {r['retention_mean']:.1f}% +/- {r['retention_ci95']:.1f} "
              f"| match {r['match_mean']:.1f}%")

    # ---------------- plot (Fig 4.2) ----------------
    plt = apply_style()
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 4))
    x = np.arange(len(rows))
    names = [LABELS[r["condition"]] for r in rows]

    ax1.bar(x, [r["deliveries_mean"] for r in rows], yerr=[r["deliveries_ci95"] for r in rows],
            capsize=5, color=MAIN_COLOR)
    ax1.set_xticks(x); ax1.set_xticklabels(names)
    ax1.set_ylabel(f"Deliveries per {N_STEPS:,} steps"); ax1.set_title("Throughput under failure")

    ax2.errorbar(x, [r["retention_mean"] for r in rows], yerr=[r["retention_ci95"] for r in rows],
                 marker="o", capsize=5, color=ACCENT_COLOR)
    ax2.axhline(100, color="gray", ls="--", lw=1)
    ax2.set_xticks(x); ax2.set_xticklabels(names)
    ax2.set_ylabel("% of healthy baseline"); ax2.set_ylim(0, 115)
    ax2.set_title("Throughput retention (paired by seed)")

    fig.savefig(RESULTS_DIR / "fig4_2_fault_degradation.png")
    print(f"\nSaved {csv_path} and results/fig4_2_fault_degradation.png")


if __name__ == "__main__":
    main()
