"""
make_figures.py - Generates publication-quality figures for B.Tech project report.

Figures:
  - fig4_1: Bid head training curves (train/val BCE and val MAE vs epoch)
  - fig4_2: Deliveries under failure (4 robots, 11x10 floor, 10 seeds)
  - fig4_3: Seed-matched retention under failure (4 robots, 11x10 floor, 10 seeds)
  - fig4_4: Seed-matched paired difference (v2 minus no-comm with 95% CI)
  - fig4_5: Fleet scaling and parallel efficiency (11x10 floor, 2/4/8 robots)

Outputs saved to figures/ as both PNG (300 dpi) and PDF.
"""

import json
import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy import stats

# -------------------------------------------------------------------------
# Paths & Setup
# -------------------------------------------------------------------------
ROOT_DIR = Path(__file__).resolve().parent
RESULTS_DIR = ROOT_DIR / "experiments" / "results"
FIG_DIR = ROOT_DIR / "figures"
FIG_DIR.mkdir(parents=True, exist_ok=True)

# -------------------------------------------------------------------------
# Palette & Style Configuration
# -------------------------------------------------------------------------
# Consistent colors across all figures:
# Greedy: Amber/Dark Orange (#d97706), No-comm: Slate/Steel Blue (#475569), v2: Royal Blue (#2563eb)
COLORS = {
    "greedy": "#d97706",
    "nocomm": "#475569",
    "v2": "#2563eb",
}
LABELS = {
    "greedy": "Greedy (eligible)",
    "nocomm": "No-comm",
    "v2": "v2 (Final)",
}

def setup_style():
    plt.rcParams.update({
        "font.family": "DejaVu Sans",
        "font.size": 10,
        "axes.titlesize": 10,
        "axes.labelsize": 10.5,
        "xtick.labelsize": 9.5,
        "ytick.labelsize": 9.5,
        "legend.fontsize": 9.5,
        "axes.spines.top": False,
        "axes.spines.right": False,
        "axes.grid": True,
        "grid.alpha": 0.25,
        "grid.linestyle": ":",
        "figure.autolayout": False,
    })

setup_style()


# -------------------------------------------------------------------------
# Fig 4.1: Bid Head Training Curves
# -------------------------------------------------------------------------
def make_fig4_1():
    csv_path = RESULTS_DIR / "exp1_history.csv"
    if not csv_path.exists():
        raise FileNotFoundError(f"Missing training history: {csv_path}")

    df = pd.read_csv(csv_path)

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(7.0, 3.3))

    # Panel 1: BCE loss
    ax1.plot(df["epoch"], df["train_bce"], label="Train BCE", color="#64748b", lw=1.6)
    ax1.plot(df["epoch"], df["val_bce"], label="Val BCE", color="#2563eb", lw=1.8)
    ax1.axhline(0.3831, color="#dc2626", ls="--", lw=1.2, label="Entropy floor (0.3831)")
    ax1.set_xlabel("Epoch")
    ax1.set_ylabel("Binary Cross-Entropy Loss")
    ax1.set_xlim(1, 60)
    ax1.set_ylim(0.375, 0.415)
    ax1.legend(loc="upper right", frameon=False, fontsize=8.5)

    # Panel 2: MAE
    ax2.plot(df["epoch"], df["val_mae"], label="Validation MAE", color="#059669", lw=1.8)
    ax2.axhline(0.2780, color="#dc2626", ls="--", lw=1.2, label="Mean baseline (0.2780)")
    ax2.set_xlabel("Epoch")
    ax2.set_ylabel("Mean Absolute Error")
    ax2.set_xlim(1, 60)
    ax2.set_ylim(-0.01, 0.30)
    ax2.legend(loc="upper right", frameon=False, fontsize=8.5)

    fig.tight_layout()
    for ext in ["png", "pdf"]:
        fig.savefig(FIG_DIR / f"fig4_1.{ext}", dpi=300)
    plt.close(fig)
    print("Saved fig4_1 (Bid head training curves)")


# -------------------------------------------------------------------------
# Fig 4.2: Deliveries Under Failure
# -------------------------------------------------------------------------
def make_fig4_2(df_summary):
    fig, ax = plt.subplots(figsize=(6.2, 3.8))

    conditions = ["none", "robot0", "robot1", "robot1+2"]
    cond_labels = [
        "Healthy\n(No failure)",
        "Robot 0 fails\n(fast_light)",
        "Robot 1 fails\n(heavy_load)",
        "Robots 1+2 fail\n(heavy + bal.)",
    ]
    systems = ["greedy", "nocomm", "v2"]

    x = np.arange(len(conditions))
    width = 0.26
    offsets = [-width, 0, width]

    for sys_name, offset in zip(systems, offsets):
        means = []
        cis = []
        for c in conditions:
            row = df_summary[
                (df_summary["system"] == sys_name) &
                (df_summary["config"] == "tiny4") &
                (df_summary["failed"] == c)
            ].iloc[0]
            means.append(row["deliveries"])
            cis.append(row["ci95"])

        ax.bar(
            x + offset,
            means,
            width=width,
            yerr=cis,
            capsize=3.5,
            error_kw={"elinewidth": 1.2, "ecolor": "#1e293b", "capthick": 1.2},
            label=LABELS[sys_name],
            color=COLORS[sys_name],
            edgecolor="#1e293b",
            linewidth=0.6,
            alpha=0.92,
        )

    ax.set_ylabel("Deliveries (20,000 steps)")
    ax.set_xticks(x)
    ax.set_xticklabels(cond_labels)
    ax.set_ylim(0, 1450)
    ax.legend(loc="upper right", frameon=False)

    fig.tight_layout()
    for ext in ["png", "pdf"]:
        fig.savefig(FIG_DIR / f"fig4_2.{ext}", dpi=300)
    plt.close(fig)
    print("Saved fig4_2 (Deliveries under failure)")


# -------------------------------------------------------------------------
# Fig 4.3: Seed-Matched Retention
# -------------------------------------------------------------------------
def make_fig4_3(df_summary):
    fig, ax = plt.subplots(figsize=(6.0, 3.6))

    conditions = ["robot0", "robot1", "robot1+2"]
    cond_labels = [
        "Robot 0 fails\n(fast_light)",
        "Robot 1 fails\n(heavy_load)",
        "Robots 1+2 fail\n(heavy + balanced)",
    ]
    systems = ["greedy", "nocomm", "v2"]

    x = np.arange(len(conditions))
    width = 0.25
    offsets = [-width, 0, width]

    # Reference line at 100%
    ax.axhline(100.0, color="#94a3b8", ls="--", lw=1.2, label="Healthy baseline (100%)", zorder=1)

    for sys_name, offset in zip(systems, offsets):
        means = []
        cis = []
        for c in conditions:
            row = df_summary[
                (df_summary["system"] == sys_name) &
                (df_summary["config"] == "tiny4") &
                (df_summary["failed"] == c)
            ].iloc[0]
            means.append(row["retention"])
            cis.append(row["retention_ci95"])

        ax.bar(
            x + offset,
            means,
            width=width,
            yerr=cis,
            capsize=3.5,
            error_kw={"elinewidth": 1.2, "ecolor": "#1e293b", "capthick": 1.2},
            label=LABELS[sys_name],
            color=COLORS[sys_name],
            edgecolor="#1e293b",
            linewidth=0.6,
            alpha=0.92,
            zorder=2,
        )

    ax.set_ylabel("Seed-Matched Retention (%)")
    ax.set_xticks(x)
    ax.set_xticklabels(cond_labels)
    ax.set_ylim(0, 110)
    ax.legend(loc="upper right", frameon=False)

    fig.tight_layout()
    for ext in ["png", "pdf"]:
        fig.savefig(FIG_DIR / f"fig4_3.{ext}", dpi=300)
    plt.close(fig)
    print("Saved fig4_3 (Seed-matched retention)")


# -------------------------------------------------------------------------
# Fig 4.4: Seed-Matched Paired Difference (v2 minus No-comm)
# -------------------------------------------------------------------------
def make_fig4_4(raw_data):
    seeds = [1, 42, 100, 2024, 7, 11, 23, 99, 314, 777]
    conditions = ["none", "robot0", "robot1", "robot1+2"]
    cond_labels = [
        "Healthy\n(No failure)",
        "Robot 0 fails\n(fast_light)",
        "Robot 1 fails\n(heavy_load)",
        "Robots 1+2 fail\n(heavy + bal.)",
    ]

    means = []
    cis = []
    excludes_zero = []

    for cond in conditions:
        diffs = []
        for s in seeds:
            k_v2 = f"v2|tiny4|{cond}|{s}"
            k_nc = f"nocomm|tiny4|{cond}|{s}"
            diffs.append(raw_data[k_v2]["deliveries"] - raw_data[k_nc]["deliveries"])

        arr = np.array(diffs, dtype=float)
        m = float(np.mean(arr))
        se = float(stats.sem(arr))
        ci = float(se * stats.t.ppf(0.975, len(arr) - 1))

        means.append(m)
        cis.append(ci)
        # CI excludes zero if both bounds have the same sign
        excludes_zero.append((m - ci > 0) or (m + ci < 0))

    fig, ax = plt.subplots(figsize=(6.0, 3.4))

    ax.axhline(0, color="#64748b", ls="--", lw=1.2, zorder=1)

    x = np.arange(len(conditions))

    # Plot points with error bars
    for i in range(len(conditions)):
        m = means[i]
        ci = cis[i]
        excl = excludes_zero[i]

        # Distinct visual treatment: filled blue square vs hollow circle
        marker = "s" if excl else "o"
        mfc = "#2563eb" if excl else "white"
        mec = "#2563eb"
        label = "CI excludes 0 (p < 0.05)" if (excl and i == 1) else ("CI spans 0" if (not excl and i == 0) else None)

        ax.errorbar(
            x[i],
            m,
            yerr=ci,
            fmt=marker,
            markersize=7.5,
            mfc=mfc,
            mec=mec,
            mew=1.8,
            ecolor="#1e293b",
            elinewidth=1.5,
            capsize=4.5,
            capthick=1.5,
            zorder=3,
            label=label,
        )

        # Annotation text above/below point
        sign = "+" if m > 0 else ""
        va = "bottom" if m >= 0 else "top"
        y_text = m + (ci + 7 if m >= 0 else -ci - 7)
        ax.text(
            x[i],
            y_text,
            f"{sign}{m:.1f} ± {ci:.1f}",
            ha="center",
            va=va,
            fontsize=8.5,
            fontweight="bold" if excl else "normal",
            color="#1e293b",
        )

    ax.set_ylabel("Deliveries Difference (v2 − No-comm)")
    ax.set_xticks(x)
    ax.set_xticklabels(cond_labels)
    ax.set_ylim(-70, 195)
    ax.legend(loc="upper left", frameon=False, fontsize=9)

    fig.tight_layout()
    for ext in ["png", "pdf"]:
        fig.savefig(FIG_DIR / f"fig4_4.{ext}", dpi=300)
    plt.close(fig)
    print("Saved fig4_4 (Seed-matched paired difference)")


# -------------------------------------------------------------------------
# Fig 4.5: Fleet Scaling & Parallel Efficiency (11x10 floor)
# -------------------------------------------------------------------------
def make_fig4_5(df_summary):
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(7.2, 3.4))

    robot_counts = [2, 4, 8]
    config_keys = ["tiny2", "tiny4", "tiny8"]

    # Panel 1: Throughput scaling
    for sys_name in ["greedy", "nocomm", "v2"]:
        means = []
        cis = []
        for cfg in config_keys:
            row = df_summary[
                (df_summary["system"] == sys_name) &
                (df_summary["config"] == cfg) &
                (df_summary["failed"] == "none")
            ].iloc[0]
            means.append(row["deliveries"])
            cis.append(row["ci95"])

        means = np.array(means)
        cis = np.array(cis)

        marker = "s" if sys_name == "v2" else "o"
        ax1.plot(
            robot_counts,
            means,
            marker=marker,
            markersize=5.5,
            lw=1.8,
            color=COLORS[sys_name],
            label=LABELS[sys_name],
        )
        ax1.fill_between(
            robot_counts,
            means - cis,
            means + cis,
            color=COLORS[sys_name],
            alpha=0.15,
        )

    # Ideal linear scaling reference anchored at 2-robot v2 baseline
    v2_base = df_summary[
        (df_summary["system"] == "v2") &
        (df_summary["config"] == "tiny2") &
        (df_summary["failed"] == "none")
    ].iloc[0]["deliveries"]
    ideal_linear = [v2_base * (n / 2.0) for n in robot_counts]
    ax1.plot(
        robot_counts,
        ideal_linear,
        color="#94a3b8",
        ls="--",
        lw=1.3,
        label="Ideal linear (from 2-robot)",
        zorder=1,
    )

    ax1.set_xlabel("Number of Robots")
    ax1.set_ylabel("Deliveries (20,000 steps)")
    ax1.set_xticks(robot_counts)
    ax1.set_xlim(1.5, 8.5)
    ax1.set_ylim(500, 3500)
    ax1.legend(loc="upper left", frameon=False, fontsize=8.5)

    # Panel 2: v2 Parallel Efficiency
    # Speed-up vs 2 robots: S(N) = Deliveries(N) / Deliveries(2)
    # Parallel efficiency: E(N) = S(N) / (N / 2) = Deliveries(N) / (Deliveries(2) * (N / 2))
    efficiencies = []
    for cfg, n in zip(config_keys, robot_counts):
        deliv = df_summary[
            (df_summary["system"] == "v2") &
            (df_summary["config"] == cfg) &
            (df_summary["failed"] == "none")
        ].iloc[0]["deliveries"]
        speedup = deliv / v2_base
        eff = speedup / (n / 2.0) * 100.0
        efficiencies.append(eff)

    ax2.plot(
        robot_counts,
        efficiencies,
        marker="s",
        markersize=6,
        lw=1.8,
        color=COLORS["v2"],
        label="v2 Parallel Efficiency",
    )
    ax2.axhline(100, color="#94a3b8", ls="--", lw=1.2, label="100% Ideal Efficiency")

    for n, eff in zip(robot_counts, efficiencies):
        ax2.text(
            n,
            eff - 4.5,
            f"{eff:.1f}%",
            ha="center",
            va="top",
            fontsize=9,
            fontweight="bold",
            color=COLORS["v2"],
        )

    ax2.set_xlabel("Number of Robots")
    ax2.set_ylabel("Parallel Efficiency (%)")
    ax2.set_xticks(robot_counts)
    ax2.set_xlim(1.5, 8.5)
    ax2.set_ylim(60, 110)
    ax2.legend(loc="lower left", frameon=False, fontsize=8.5)

    fig.tight_layout()
    for ext in ["png", "pdf"]:
        fig.savefig(FIG_DIR / f"fig4_5.{ext}", dpi=300)
    plt.close(fig)
    print("Saved fig4_5 (Fleet scaling and parallel efficiency)")


# -------------------------------------------------------------------------
# Verification & Self-Check Against Target Report Values
# -------------------------------------------------------------------------
def verify_plotted_values(df_summary, raw_data):
    print("\n" + "=" * 80)
    print("STEP 4: VERIFICATION OF PLOTTED NUMBERS AGAINST REPORT VALUES")
    print("=" * 80)

    # 1. Healthy 4 robots
    print("\n[1] Healthy 4 Robots (Deliveries):")
    targets_h = {"v2": 1285.6, "nocomm": 1288.0, "greedy": 1272.0}
    for sys_k, target in targets_h.items():
        val = df_summary[
            (df_summary["system"] == sys_k) &
            (df_summary["config"] == "tiny4") &
            (df_summary["failed"] == "none")
        ].iloc[0]["deliveries"]
        diff = abs(val - target)
        status = "MATCH" if diff < 0.1 else f"MISMATCH ({diff:.2f})"
        print(f"  {sys_k:7s}: Plotted = {val:.1f}, Report = {target:.1f} -> [{status}]")

    # 2. Robot 0 fails (Deliveries & Retention)
    print("\n[2] Robot 0 Fails (Deliveries & Retention):")
    targets_r0 = {
        "v2": (1105.5, 86.1),
        "nocomm": (993.6, 77.3),
        "greedy": (703.1, 55.4),
    }
    for sys_k, (t_del, t_ret) in targets_r0.items():
        row = df_summary[
            (df_summary["system"] == sys_k) &
            (df_summary["config"] == "tiny4") &
            (df_summary["failed"] == "robot0")
        ].iloc[0]
        v_del, v_ret = row["deliveries"], row["retention"]
        stat_del = "MATCH" if abs(v_del - t_del) < 0.1 else "MISMATCH"
        stat_ret = "MATCH" if abs(v_ret - t_ret) < 0.1 else "MISMATCH"
        print(f"  {sys_k:7s}: Deliveries = {v_del:.1f} (target {t_del:.1f}) [{stat_del}], "
              f"Retention = {v_ret:.1f}% (target {t_ret:.1f}%) [{stat_ret}]")

    # 3. Robot 1 fails
    print("\n[3] Robot 1 Fails (Deliveries):")
    targets_r1 = {"v2": 356.4, "nocomm": 353.7, "greedy": 313.9}
    for sys_k, target in targets_r1.items():
        val = df_summary[
            (df_summary["system"] == sys_k) &
            (df_summary["config"] == "tiny4") &
            (df_summary["failed"] == "robot1")
        ].iloc[0]["deliveries"]
        status = "MATCH" if abs(val - target) < 0.1 else "MISMATCH"
        print(f"  {sys_k:7s}: Plotted = {val:.1f}, Report = {target:.1f} -> [{status}]")

    # 4. Robots 1+2 fail
    print("\n[4] Robots 1+2 Fail (Deliveries):")
    targets_r12 = {"v2": 339.7, "nocomm": 343.7, "greedy": 184.6}
    for sys_k, target in targets_r12.items():
        val = df_summary[
            (df_summary["system"] == sys_k) &
            (df_summary["config"] == "tiny4") &
            (df_summary["failed"] == "robot1+2")
        ].iloc[0]["deliveries"]
        status = "MATCH" if abs(val - target) < 0.1 else "MISMATCH"
        print(f"  {sys_k:7s}: Plotted = {val:.1f}, Report = {target:.1f} -> [{status}]")

    # 5. v2 minus No-comm differences
    print("\n[5] v2 minus No-comm Differences (Deliveries):")
    seeds = [1, 42, 100, 2024, 7, 11, 23, 99, 314, 777]
    for cond, target_m, target_ci in [("none", -2.4, 50.8), ("robot0", 111.9, 52.6)]:
        diffs = [
            raw_data[f"v2|tiny4|{cond}|{s}"]["deliveries"] - raw_data[f"nocomm|tiny4|{cond}|{s}"]["deliveries"]
            for s in seeds
        ]
        m = float(np.mean(diffs))
        se = float(stats.sem(diffs))
        ci = float(se * stats.t.ppf(0.975, len(diffs) - 1))
        status = "MATCH" if (abs(m - target_m) < 0.1 and abs(ci - target_ci) < 0.1) else "MISMATCH"
        print(f"  {cond:8s}: Plotted = {m:+.1f} ± {ci:.1f}, Report = {target_m:+.1f} ± {target_ci:.1f} -> [{status}]")

    # 6. Scaling v2
    print("\n[6] Scaling v2 (Deliveries):")
    targets_scale = [("tiny2", 2, 828.8), ("tiny4", 4, 1285.6), ("tiny8", 8, 2672.6)]
    for cfg, n, target in targets_scale:
        val = df_summary[
            (df_summary["system"] == "v2") &
            (df_summary["config"] == cfg) &
            (df_summary["failed"] == "none")
        ].iloc[0]["deliveries"]
        status = "MATCH" if abs(val - target) < 0.1 else "MISMATCH"
        print(f"  {n} robots ({cfg}): Plotted = {val:.1f}, Report = {target:.1f} -> [{status}]")

    # 7. Bid head training final metrics
    print("\n[7] Bid Head Training Final Metrics (Epoch 60):")
    df_train = pd.read_csv(RESULTS_DIR / "exp1_history.csv")
    last_row = df_train.iloc[-1]
    val_bce = last_row["val_bce"]
    val_mae = last_row["val_mae"]
    stat_bce = "MATCH" if abs(val_bce - 0.3839) < 0.0001 else "MISMATCH"
    stat_mae = "MATCH" if abs(val_mae - 0.0019) < 0.0001 else "MISMATCH"
    print(f"  val_bce: Plotted = {val_bce:.4f}, Report = 0.3839 -> [{stat_bce}]")
    print(f"  val_mae: Plotted = {val_mae:.4f}, Report = 0.0019 -> [{stat_mae}]")
    print("=" * 80)


def main():
    print("Loading data files...")
    df_summary = pd.read_csv(RESULTS_DIR / "exp8_summary.csv")
    with open(RESULTS_DIR / "exp8_raw.json", "r") as f:
        raw_data = json.load(f)

    print("Generating figures...")
    make_fig4_1()
    make_fig4_2(df_summary)
    make_fig4_3(df_summary)
    make_fig4_4(raw_data)
    make_fig4_5(df_summary)

    verify_plotted_values(df_summary, raw_data)
    print("\nAll 5 figures generated and verified in figures/ (PNG @ 300dpi + PDF).")


if __name__ == "__main__":
    main()

