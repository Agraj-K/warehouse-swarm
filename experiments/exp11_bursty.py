"""
experiments/exp11_bursty.py - Gilbert-Elliott bursty channel stress test.

GE parameters: (p_gb=0.01, p_bg=0.1, l_good=0.01, l_bad=0.9)
  Average loss = l_good * p_bg/(p_gb+p_bg) + l_bad * p_gb/(p_gb+p_bg) ≈ 9.1%
  Average burst length = 1/p_bg = 10 steps.

Compare against independent 10% loss (Exp 9 baseline already run).

Runs:
  tau in [5, 15] x condition in [none, robot0] x 10 seeds = 40 runs
  Plus greedy no-comm reference from exp8 (free, no rerun needed).

Output: experiments/results/exp11_raw.json and exp11_summary.csv
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

from multi_robot_vecenv import MultiRobotVecEnv
from pathfinding import GridPathfinder
from lossy_fault_layer import LossyFaultLayer
from auction_layer_v2 import AuctionLayerV2
from exp_utils import ResultStore, RESULTS_DIR, mean_ci95

SEEDS = [1, 42, 100, 2024, 7, 11, 23, 99, 314, 777]
N_STEPS = 20000
FAILURE_STEP = 500
ENV_ID = "rware-tiny-4ag-v2"
ROBOT_TYPES = [0, 1, 2, 0]

# GE channel: ~9.1% average loss, bursts of ~10 steps
GE_PARAMS = (0.01, 0.1, 0.01, 0.9)   # (p_gb, p_bg, l_good, l_bad)

CONDITIONS = {"none": (), "robot0": (0,)}


def build_specs():
    specs = []
    for tau in [5, 15]:
        for fname, failed in CONDITIONS.items():
            for seed in SEEDS:
                tag = f"ge_tau{tau}"
                specs.append((fname, failed, seed, tau, tag))
    return specs


def key_of(spec):
    fname, _, seed, tau, tag = spec
    return f"v2|{fname}|{tag}|{seed}"


def _init():
    import torch
    torch.set_num_threads(1)


def run_one(spec):
    fname, failed, seed, tau, tag = spec

    vec = MultiRobotVecEnv(env_id=ENV_ID, robot_types=list(ROBOT_TYPES), seed=seed)
    obs = vec.reset()
    env = vec.hetero_env
    path = GridPathfinder(env)

    fault = LossyFaultLayer(env, timeout=tau, seed=seed, ge=GE_PARAMS)
    auction = AuctionLayerV2(env, seed=seed)

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
    )
    return key_of(spec), res


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--workers", type=int, default=6)
    args = parser.parse_args()

    store = ResultStore("exp11_raw")
    specs = build_specs()
    pending = [s for s in specs if not store.has(key_of(s))]
    print(f"Exp 11 GE Bursty: {len(specs)} total, {len(pending)} pending (workers={args.workers})")

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

    # Print comparison table
    with open(RESULTS_DIR / "exp9_raw.json") as f:
        d9 = json.load(f)
    with open(RESULTS_DIR / "exp11_raw.json") as f:
        d11 = json.load(f)
    with open(RESULTS_DIR / "exp8_raw.json") as f:
        d8 = json.load(f)

    nocomm = [d8[f"nocomm|tiny4|robot0|{s}"]["deliveries"] for s in SEEDS]
    nc_m, _, nc_ci = mean_ci95(nocomm)

    print("\n" + "=" * 100)
    print("EXP 11: GE BURSTY (avg ~9% loss, burst=10 steps) vs Exp 9 INDEPENDENT 10% LOSS")
    print(f"{'Setting':<26} | {'Healthy Deliv':<16} | {'Fail Deliv':<16} | {'Retention %':<14} | {'FP Steps':<12} | {'Det Delay'}")
    print("-" * 100)

    for tau in [5, 15]:
        # GE bursty results
        ge_h  = [d11[f"v2|none|ge_tau{tau}|{s}"]["deliveries"] for s in SEEDS]
        ge_f  = [d11[f"v2|robot0|ge_tau{tau}|{s}"]["deliveries"] for s in SEEDS]
        ge_fp = [d11[f"v2|none|ge_tau{tau}|{s}"]["false_positive_steps"] for s in SEEDS]
        ge_fp_f = [d11[f"v2|robot0|ge_tau{tau}|{s}"]["false_positive_steps"] for s in SEEDS]
        ge_det = [d11[f"v2|robot0|ge_tau{tau}|{s}"]["det_delay"] for s in SEEDS if d11[f"v2|robot0|ge_tau{tau}|{s}"]["det_delay"] is not None]
        ge_ret = [f / h * 100 for f, h in zip(ge_f, ge_h)]

        hm, _, hci = mean_ci95(ge_h)
        fm, _, fci = mean_ci95(ge_f)
        rm, _, rci = mean_ci95(ge_ret)
        fpm, _, fpci = mean_ci95(ge_fp)
        dm, _, dci = mean_ci95(ge_det)

        print(f"GE bursty tau={tau:<13} | {hm:>5.1f} ± {hci:<6.1f}  | {fm:>5.1f} ± {fci:<6.1f}  | {rm:>5.1f}% ± {rci:.1f}% | {fpm:>8.0f}     | {dm:.1f} ± {dci:.1f}")

        # Exp 9 independent 10% loss for comparison
        ind_key_h = f"v2|none|loss_0.1|{{}}"
        ind_key_f = f"v2|robot0|loss_0.1|{{}}"
        # exp9 key format: v2|none|loss_0.10|seed
        try:
            ind_h  = [d9[f"v2|none|loss_0.10|{s}"]["deliveries"] for s in SEEDS]
            ind_f  = [d9[f"v2|robot0|loss_0.10|{s}"]["deliveries"] for s in SEEDS]
            ind_fp = [d9[f"v2|none|loss_0.10|{s}"]["false_positive_steps"] for s in SEEDS]
            ind_det = [d9[f"v2|robot0|loss_0.10|{s}"]["det_delay"] for s in SEEDS if d9[f"v2|robot0|loss_0.10|{s}"]["det_delay"] is not None]
            ind_ret = [f / h * 100 for f, h in zip(ind_f, ind_h)]

            ihm, _, ihci = mean_ci95(ind_h)
            ifm, _, ifci = mean_ci95(ind_f)
            irm, _, irci = mean_ci95(ind_ret)
            ifpm, _, ifpci = mean_ci95(ind_fp)
            idm, _, idci = mean_ci95(ind_det) if ind_det else (0, 0, 0)

            print(f"Indep 10% tau={tau:<13} | {ihm:>5.1f} ± {ihci:<6.1f}  | {ifm:>5.1f} ± {ifci:<6.1f}  | {irm:>5.1f}% ± {irci:.1f}% | {ifpm:>8.0f}     | {idm:.1f} ± {idci:.1f}")
        except KeyError:
            print(f"Indep 10% tau={tau:<13} | (Exp 9 did not vary tau, used tau=15 only)")

    print(f"\nNo-comm reference (Robot 0 fails): {nc_m:.1f} ± {nc_ci:.1f}")

    # Write CSV
    rows = []
    for tau in [5, 15]:
        for fname in ["none", "robot0"]:
            for s in SEEDS:
                k = f"v2|{fname}|ge_tau{tau}|{s}"
                rows.append({"tau": tau, "condition": fname, "seed": s, **d11[k]})
    csv_path = RESULTS_DIR / "exp11_summary.csv"
    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["tau", "condition", "seed", "deliveries", "match_rate", "false_positive_steps", "det_delay"])
        writer.writeheader()
        writer.writerows(rows)
    print(f"Wrote {csv_path}")


if __name__ == "__main__":
    main()

