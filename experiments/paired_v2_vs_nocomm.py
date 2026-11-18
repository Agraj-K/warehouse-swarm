"""
experiments/paired_v2_vs_nocomm.py - Paired statistical comparison of v2 under stress vs no-comm.
"""
import json
import math
from pathlib import Path

_ROOT_DIR = Path(__file__).resolve().parent.parent
RESULTS_DIR = _ROOT_DIR / "experiments" / "results"
SEEDS = [1, 42, 100, 2024, 7, 11, 23, 99, 314, 777]


def mean_ci95(vals):
    n = len(vals)
    if n == 0:
        return 0.0, 0.0
    m = sum(vals) / n
    if n == 1:
        return m, 0.0
    var = sum((x - m) ** 2 for x in vals) / (n - 1)
    t_crit = 2.262 if n == 10 else 1.96
    ci = t_crit * math.sqrt(var / n)
    return m, ci


def main():
    with open(RESULTS_DIR / "exp8_raw.json", "r") as f:
        d8 = json.load(f)

    with open(RESULTS_DIR / "exp10_raw.json", "r") as f:
        d10 = json.load(f)

    nocomm_delivs = [d8[f"nocomm|tiny4|robot0|{s}"]["deliveries"] for s in SEEDS]
    nc_m, nc_ci = mean_ci95(nocomm_delivs)

    base_v2 = [d10[f"v2|robot0|base_baseline|{s}"]["deliveries"] for s in SEEDS]
    diff_base = [v - nc for v, nc in zip(base_v2, nocomm_delivs)]
    db_m, db_ci = mean_ci95(diff_base)

    print("=" * 105)
    print("PAIRED COMPARISON: v2 UNDER STRESS vs NO-COMM BASELINE (10 SEEDS)")
    print(f"No-comm deliveries (Robot 0 fails): {nc_m:.1f} +/- {nc_ci:.1f}")
    print(f"Baseline v2 advantage over No-comm: +{db_m:.1f} +/- {db_ci:.1f} (Exp 8 reference: +111.9 +/- 52.6)")
    print("=" * 105)
    print(f"{'Setting':<24} | {'v2 Deliveries':<16} | {'Paired Advantage (95% CI)':<30} | {'CI Excludes 0?'}")
    print("-" * 105)

    for tau in [5, 15]:
        for loss in [0.40, 0.60, 0.80, 0.90]:
            tag = f"grid_tau{tau}_loss{loss:.2f}"
            v2_delivs = [d10[f"v2|robot0|{tag}|{s}"]["deliveries"] for s in SEEDS]
            diffs = [v - nc for v, nc in zip(v2_delivs, nocomm_delivs)]

            vm, vci = mean_ci95(v2_delivs)
            dm, dci = mean_ci95(diffs)
            excl_zero = (dm - dci) > 0
            excl_str = "YES (P < 0.05)" if excl_zero else "Includes 0"

            lbl = f"tau={tau}, loss={int(loss*100)}%"
            print(f"{lbl:<24} | {vm:>6.1f} +/- {vci:<6.1f} | {dm:>+6.1f} +/- {dci:<6.1f} [{dm-dci:>5.1f}, {dm+dci:>5.1f}] | {excl_str}")


if __name__ == "__main__":
    main()

