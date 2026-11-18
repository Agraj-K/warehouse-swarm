"""
experiments/control_no_beliefs.py

Control experiment: v2 with heartbeat beliefs disabled (bf=None).
Failure is still injected and action override still fires, so robot 0 really dies.
Only the belief propagation into the auction is suppressed.

Compares per-seed against:
  - v2 normal (from exp10_raw.json base_baseline|robot0)
  - no-comm   (from exp8_raw.json nocomm|tiny4|robot0)

10 seeds, 20,000 steps each.
"""
import sys
import json
import math
from pathlib import Path
from multiprocessing import Pool

_ROOT_DIR = Path(__file__).resolve().parent.parent
for _p in [str(_ROOT_DIR / "core"), str(_ROOT_DIR / "experiments"), str(_ROOT_DIR)]:
    if _p not in sys.path:
        sys.path.insert(0, _p)

from multi_robot_vecenv import MultiRobotVecEnv
from pathfinding import GridPathfinder
from lossy_fault_layer import LossyFaultLayer
from auction_layer_v2 import AuctionLayerV2

SEEDS = [1, 42, 100, 2024, 7, 11, 23, 99, 314, 777]
N_STEPS = 20000
FAILURE_STEP = 500
ENV_ID = "rware-tiny-4ag-v2"
ROBOT_TYPES = [0, 1, 2, 0]
RESULTS_DIR = _ROOT_DIR / "experiments" / "results"


def mean_ci95(vals):
    n = len(vals)
    m = sum(vals) / n
    if n < 2:
        return m, 0.0
    var = sum((x - m) ** 2 for x in vals) / (n - 1)
    t_crit = 2.262
    return m, t_crit * math.sqrt(var / n)


def _init():
    import torch
    torch.set_num_threads(1)


def run_seed(s):
    vec = MultiRobotVecEnv(env_id=ENV_ID, robot_types=list(ROBOT_TYPES), seed=s)
    obs = vec.reset()
    env = vec.hetero_env
    path = GridPathfinder(env)

    fault = LossyFaultLayer(env, seed=s)
    auction = AuctionLayerV2(env, seed=s)

    for step in range(N_STEPS):
        if step == FAILURE_STEP:
            fault.inject_failure(0)

        fault.step()  # run the fault layer but ignore its beliefs
        pursue, _, tasks = auction.step(obs, belief_failed=None)  # <-- beliefs disabled

        actions = path.get_actions_for_pursuit(obs, pursue, tasks)
        actions = fault.get_actions_override(actions)  # robot 0 still physically stuck
        obs, _, _, _ = vec.step(actions)
        auction.clear_stale_beliefs()

    m, mm = sum(env.match_count), sum(env.mismatch_count)
    vec.close()
    return s, int(m + mm)


if __name__ == "__main__":
    with Pool(6, initializer=_init) as p:
        results = dict(p.map(run_seed, SEEDS))

    ctrl_delivs = [results[s] for s in SEEDS]

    # Load reference data
    with open(RESULTS_DIR / "exp10_raw.json") as f:
        d10 = json.load(f)
    with open(RESULTS_DIR / "exp8_raw.json") as f:
        d8 = json.load(f)

    v2_delivs   = [d10[f"v2|robot0|base_baseline|{s}"]["deliveries"] for s in SEEDS]
    nocomm_delivs = [d8[f"nocomm|tiny4|robot0|{s}"]["deliveries"] for s in SEEDS]

    ctrl_m, ctrl_ci = mean_ci95(ctrl_delivs)
    v2_m, v2_ci     = mean_ci95(v2_delivs)
    nc_m, nc_ci     = mean_ci95(nocomm_delivs)

    # Paired differences
    diff_ctrl_vs_nc  = [c - nc for c, nc in zip(ctrl_delivs, nocomm_delivs)]
    diff_v2_vs_nc    = [v - nc for v, nc in zip(v2_delivs,   nocomm_delivs)]
    diff_ctrl_vs_v2  = [c - v  for c, v  in zip(ctrl_delivs, v2_delivs)]

    dcn_m, dcn_ci = mean_ci95(diff_ctrl_vs_nc)
    dvn_m, dvn_ci = mean_ci95(diff_v2_vs_nc)
    dcv_m, dcv_ci = mean_ci95(diff_ctrl_vs_v2)

    print("=" * 80)
    print("BELIEFS-DISABLED CONTROL (v2, bf=None, Robot 0 fails, 10 seeds)")
    print("=" * 80)
    print(f"\nSystem            | Deliveries        | vs No-comm (95% CI)")
    print("-" * 70)
    print(f"No-comm (Exp 8)   | {nc_m:>6.1f} ± {nc_ci:<6.1f} | —")
    print(f"v2 normal (Exp 8) | {v2_m:>6.1f} ± {v2_ci:<6.1f} | {dvn_m:>+6.1f} ± {dvn_ci:.1f} [{dvn_m-dvn_ci:.1f}, {dvn_m+dvn_ci:.1f}]")
    print(f"v2 bf=None (ctrl) | {ctrl_m:>6.1f} ± {ctrl_ci:<6.1f} | {dcn_m:>+6.1f} ± {dcn_ci:.1f} [{dcn_m-dcn_ci:.1f}, {dcn_m+dcn_ci:.1f}]")
    print()
    print(f"Paired ctrl vs v2-normal: {dcv_m:>+6.1f} ± {dcv_ci:.1f} [{dcv_m-dcv_ci:.1f}, {dcv_m+dcv_ci:.1f}]")
    print()
    print("Per-seed raw deliveries:")
    print(f"  {'Seed':<6} {'No-comm':>8} {'v2 normal':>10} {'bf=None':>10} {'diff v2-nc':>12} {'diff bf=None-nc':>16}")
    for s in SEEDS:
        nc = d8[f"nocomm|tiny4|robot0|{s}"]["deliveries"]
        v2 = d10[f"v2|robot0|base_baseline|{s}"]["deliveries"]
        ct = results[s]
        print(f"  {s:<6} {nc:>8} {v2:>10} {ct:>10} {v2-nc:>12} {ct-nc:>16}")

    # Save per-seed results for later reference
    out_path = RESULTS_DIR / "control_no_beliefs.json"
    with open(out_path, "w") as f:
        json.dump({str(s): results[s] for s in SEEDS}, f, indent=2)
    print(f"\nSaved to {out_path}")

