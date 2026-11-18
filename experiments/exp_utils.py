"""
Shared helpers for the report experiments (Exp 1, 2, 5).
Put this file in  warehouse-swarm/experiments/  next to the experiment scripts.

- mean_ci95(): mean, sample SD and 95% CI half-width (Student t)
- ResultStore: JSON file that saves every finished run, so a long experiment
  can be stopped and resumed without redoing finished (condition, seed) runs
- apply_style(): one matplotlib style + one colour per robot type, so all
  three teammates' figures look the same
"""
import json
import math
from pathlib import Path

import numpy as np

RESULTS_DIR = Path(__file__).resolve().parent / "results"
RESULTS_DIR.mkdir(exist_ok=True)

# one colour per robot type (agree on these with the team)
TYPE_COLORS = {"fast_light": "#1f77b4", "heavy_load": "#d62728", "balanced": "#2ca02c"}
MAIN_COLOR = "#374151"
ACCENT_COLOR = "#e07b00"


def mean_ci95(values):
    """Return (mean, sample SD, 95% CI half-width). Uses Student t."""
    v = np.asarray(values, dtype=float)
    n = len(v)
    if n == 0:
        return float("nan"), float("nan"), float("nan")
    mean = float(v.mean())
    if n < 2:
        return mean, 0.0, 0.0
    sd = float(v.std(ddof=1))
    try:
        from scipy import stats
        tcrit = float(stats.t.ppf(0.975, n - 1))
    except ImportError:  # fallback: normal approximation (slightly narrow for small n)
        tcrit = 1.96
    return mean, sd, tcrit * sd / math.sqrt(n)


class ResultStore:
    """Tiny JSON store: key -> value. Saves after every set()."""

    def __init__(self, name):
        self.path = RESULTS_DIR / f"{name}.json"
        self.data = json.loads(self.path.read_text()) if self.path.exists() else {}

    def has(self, key):
        return key in self.data

    def get(self, key):
        return self.data[key]

    def set(self, key, value):
        self.data[key] = value
        self.path.write_text(json.dumps(self.data, indent=2))


def apply_style():
    import matplotlib.pyplot as plt
    plt.rcParams.update({
        "font.family": "DejaVu Sans",
        "font.size": 11,
        "axes.titlesize": 12,
        "axes.labelsize": 11,
        "axes.spines.top": False,
        "axes.spines.right": False,
        "axes.grid": True,
        "grid.alpha": 0.25,
        "figure.dpi": 120,
        "savefig.dpi": 300,
        "savefig.bbox": "tight",
        "legend.frameon": False,
    })
    return plt
