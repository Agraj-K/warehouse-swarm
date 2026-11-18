"""
experiments/analyze_stress.py - Paired statistical analysis of Exp 10 stress experiments.

Computes:
  1. Per-seed paired difference: delta_deliv = deliv(setting, s) - deliv(baseline, s)
  2. Per-seed retention: ret(s) = deliv(fail, s) / deliv(healthy, s) * 100%
  3. Paired delta_retention = ret(setting, s) - ret(baseline, s)
  4. 95% Student-t CI (df=9, t_crit=2.262) on all paired deltas.
  5. Statistical degradation flag (CI upper bound < 0, meaning statistically significant drop).
"""
import csv
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
    # Student's t critical value for df=9 (alpha=0.05) is ~2.262
    t_crit = 2.262 if n == 10 else 1.96
    ci = t_crit * math.sqrt(var / n)
    return m, ci


def run_analysis():
    raw_path = RESULTS_DIR / "exp10_raw.json"
    if not raw_path.exists():
        print(f"File not found: {raw_path}")
        return

    with open(raw_path, "r") as f:
        data = json.load(f)

    # 1. Baseline vectors
    base_healthy = [data[f"v2|none|base_baseline|{s}"]["deliveries"] for s in SEEDS]
    base_fail = [data[f"v2|robot0|base_baseline|{s}"]["deliveries"] for s in SEEDS]
    base_ret = [f / h * 100 for f, h in zip(base_fail, base_healthy)]

    bh_m, bh_ci = mean_ci95(base_healthy)
    bf_m, bf_ci = mean_ci95(base_fail)
    br_m, br_ci = mean_ci95(base_ret)

    print("=" * 110)
    print("EXP 10 STRESS ANALYSIS & PAIRED DEGRADATION TEST")
    print(f"Canonical Baseline (v2, tau=15, loss=0, delay=0):")
    print(f"  Healthy Deliveries: {bh_m:.1f} +/- {bh_ci:.1f}")
    print(f"  Robot 0 Fail Deliveries: {bf_m:.1f} +/- {bf_ci:.1f}")
    print(f"  Retention: {br_m:.1f}% +/- {br_ci:.1f}%")
    print("=" * 110)

    summary_rows = []

    # Helper to evaluate a sweep group
    def eval_group(group_name, settings):
        print(f"\n>>> {group_name.upper()} <<<")
        print(f"{'Setting':<22} | {'Healthy Deliv':<16} | {'Fail Deliv':<16} | {'Retention %':<16} | {'Paired dRet (95% CI)':<22} | {'FP Steps':<10} | {'Sig Drop?'}")
        print("-" * 125)

        for tag, label, has_fail in settings:
            h_delivs = []
            f_delivs = []
            rets = []
            d_rets = []
            fp_steps = []
            dual_claims_list = []
            match_rates = []

            for s in SEEDS:
                hk = f"v2|none|{tag}|{s}"
                h_item = data.get(hk)
                if not h_item:
                    continue
                h_deliv = h_item["deliveries"]
                h_delivs.append(h_deliv)
                fp_steps.append(h_item.get("false_positive_steps", 0))
                dual_claims_list.append(h_item.get("dual_claims", 0))
                match_rates.append(h_item.get("match_rate", 0.0))

                if has_fail:
                    fk = f"v2|robot0|{tag}|{s}"
                    f_item = data.get(fk)
                    if f_item:
                        f_deliv = f_item["deliveries"]
                        f_delivs.append(f_deliv)
                        ret = (f_deliv / h_deliv) * 100.0
                        rets.append(ret)
                        # paired difference with baseline retention for seed s
                        base_r = (data[f"v2|robot0|base_baseline|{s}"]["deliveries"] /
                                  data[f"v2|none|base_baseline|{s}"]["deliveries"]) * 100.0
                        d_rets.append(ret - base_r)

            hm, hci = mean_ci95(h_delivs)
            fpm, fpci = mean_ci95(fp_steps)
            dcm, dcci = mean_ci95(dual_claims_list)
            mrm, mrci = mean_ci95(match_rates)

            if has_fail and rets:
                fm, fci = mean_ci95(f_delivs)
                rm, rci = mean_ci95(rets)
                drm, drci = mean_ci95(d_rets)
                # Significant degradation if CI is strictly negative
                is_sig = (drm + drci) < 0.0
                sig_str = "YES (P<0.05)" if is_sig else "No"
                dret_str = f"{drm:+.2f} +/- {drci:.2f}%"
                fdel_str = f"{fm:.1f} +/- {fci:.1f}"
                ret_str = f"{rm:.1f}% +/- {rci:.1f}%"
            else:
                fdel_str = "N/A"
                ret_str = "N/A"
                dret_str = "N/A"
                # Check healthy paired degradation
                d_h = [h - b for h, b in zip(h_delivs, base_healthy)]
                dhm, dhci = mean_ci95(d_h)
                is_sig = (dhm + dhci) < 0.0
                sig_str = "YES (P<0.05)" if is_sig else "No"

            print(f"{label:<22} | {hm:>6.1f} +/- {hci:<6.1f} | {fdel_str:<16} | {ret_str:<16} | {dret_str:<22} | {fpm:>5.1f}     | {sig_str}")

            summary_rows.append({
                "group": group_name,
                "label": label,
                "tag": tag,
                "healthy_mean": round(hm, 1),
                "healthy_ci": round(hci, 1),
                "fail_mean": round(fm, 1) if has_fail and rets else "N/A",
                "fail_ci": round(fci, 1) if has_fail and rets else "N/A",
                "retention_mean": round(rm, 2) if has_fail and rets else "N/A",
                "retention_ci": round(rci, 2) if has_fail and rets else "N/A",
                "paired_delta_ret_mean": round(drm, 2) if has_fail and rets else "N/A",
                "paired_delta_ret_ci": round(drci, 2) if has_fail and rets else "N/A",
                "false_positive_steps_mean": round(fpm, 1),
                "false_positive_steps_ci": round(fpci, 1),
                "dual_claims_mean": round(dcm, 1),
                "match_rate_mean": round(mrm, 2),
                "sig_degradation": is_sig,
            })

    # Sweep 1: Stress Grid
    tau5_settings = [
        (f"grid_tau5_loss{l:.2f}", f"tau=5, loss={int(l*100)}%", True)
        for l in [0.40, 0.60, 0.80, 0.90]
    ]
    eval_group("Stress Grid: tau=5 (Aggressive Timeout)", tau5_settings)

    tau15_settings = [
        (f"grid_tau15_loss{l:.2f}", f"tau=15, loss={int(l*100)}%", True)
        for l in [0.40, 0.60, 0.80, 0.90]
    ]
    eval_group("Stress Grid: tau=15 (Default Timeout)", tau15_settings)

    # Sweep 2: Radio Outage
    outage_settings = [
        (f"outage_len{mlen}", f"Mute Robot 0 ({mlen} steps)", False)
        for mlen in [10, 20, 40]
    ]
    eval_group("Radio Outage Test (Healthy Fleet, Step 500)", outage_settings)

    # Sweep 3: Lossy Task Claims
    claim_settings = [
        (f"claims_closs{cl:.2f}", f"Claim Loss {int(cl*100)}%", False)
        for cl in [0.05, 0.10, 0.20]
    ]
    eval_group("Lossy Task Claims Sweep (Healthy Fleet)", claim_settings)

    # Write summary CSV
    csv_path = RESULTS_DIR / "exp10_summary.csv"
    with open(csv_path, "w", newline="") as f:
        fieldnames = [
            "group", "label", "tag", "healthy_mean", "healthy_ci",
            "fail_mean", "fail_ci", "retention_mean", "retention_ci",
            "paired_delta_ret_mean", "paired_delta_ret_ci",
            "false_positive_steps_mean", "false_positive_steps_ci",
            "dual_claims_mean", "match_rate_mean", "sig_degradation"
        ]
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(summary_rows)

    print(f"\nWrote summary CSV to {csv_path}")


if __name__ == "__main__":
    run_analysis()

