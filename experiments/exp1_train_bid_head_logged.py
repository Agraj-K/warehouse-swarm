"""
Exp 1 - Neural bid head training, with FULL per-epoch logging and the loss curve.

Same model / data / hyper-parameters as experiments/train_bid_head.py, but:
  * logs train BCE, val BCE, val MAE and learning rate for every epoch
  * saves the history to results/exp1_history.csv
  * plots the loss curve + MAE curve (Fig 4.1)
  * computes two reference lines you can quote in the report:
      - BCE floor: with continuous labels, BCE cannot reach 0; its minimum is
        the mean binary entropy of the labels. Compare 0.187 against this.
      - MAE of always predicting the training mean (the "no learning" baseline)
  * does NOT overwrite bid_head_trained.pt unless you pass --save-model,
    because the other experiments depend on the existing trained model.

Run from the experiments folder:   python exp1_train_bid_head_logged.py
"""
import argparse
import csv
import os
import sys
from pathlib import Path

_ROOT_DIR = Path(__file__).resolve().parent.parent
_CORE_DIR = _ROOT_DIR / "core"
_MODELS_DIR = _ROOT_DIR / "models_and_data"
for _p in [str(_CORE_DIR), str(_ROOT_DIR)]:
    if _p not in sys.path:
        sys.path.insert(0, _p)

import numpy as np
import torch
import torch.nn as nn
from torch.utils.data import TensorDataset, DataLoader

from policy_network import CapabilityConditionedPolicy
from exp_utils import RESULTS_DIR, apply_style, MAIN_COLOR, ACCENT_COLOR

DATA_FILE = str(_MODELS_DIR / "bid_training_data.npz")
EPOCHS = 60
BATCH_SIZE = 256
LEARNING_RATE = 3e-3
VAL_SPLIT = 0.15
STEP_SIZE = 20
GAMMA = 0.3


def binary_entropy_floor(y):
    """Minimum achievable BCE for soft labels y in [0,1]: mean of -[y ln y + (1-y) ln(1-y)]."""
    y = np.clip(y, 1e-7, 1 - 1e-7)
    return float(np.mean(-(y * np.log(y) + (1 - y) * np.log(1 - y))))


def main(seed, save_model):
    if not os.path.exists(DATA_FILE):
        raise FileNotFoundError(f"Training data not found at {DATA_FILE}")

    torch.manual_seed(seed)
    np.random.seed(seed)

    data = np.load(DATA_FILE)
    obs = torch.tensor(data["obs"], dtype=torch.float32)
    labels = torch.tensor(data["labels"], dtype=torch.float32).unsqueeze(1)

    n_total = len(obs)
    n_val = int(n_total * VAL_SPLIT)
    n_train = n_total - n_val
    perm = torch.randperm(n_total)
    tr_idx, va_idx = perm[:n_train], perm[n_train:]

    train_loader = DataLoader(TensorDataset(obs[tr_idx], labels[tr_idx]),
                              batch_size=BATCH_SIZE, shuffle=True)
    val_loader = DataLoader(TensorDataset(obs[va_idx], labels[va_idx]),
                            batch_size=BATCH_SIZE, shuffle=False)

    # reference lines for the report
    val_labels_np = labels[va_idx].numpy().ravel()
    train_mean = float(labels[tr_idx].mean())
    mae_mean_baseline = float(np.mean(np.abs(val_labels_np - train_mean)))
    bce_floor = binary_entropy_floor(val_labels_np)
    print(f"Samples: {n_train} train / {n_val} val")
    print(f"Reference - BCE floor (label entropy): {bce_floor:.4f}")
    print(f"Reference - MAE of predicting the mean: {mae_mean_baseline:.4f}\n")

    model = CapabilityConditionedPolicy(obs_dim=80)
    optimizer = torch.optim.Adam(model.parameters(), lr=LEARNING_RATE)
    scheduler = torch.optim.lr_scheduler.StepLR(optimizer, step_size=STEP_SIZE, gamma=GAMMA)
    bce_fn, mae_fn = nn.BCELoss(), nn.L1Loss()

    history = []
    for epoch in range(1, EPOCHS + 1):
        model.train()
        tr_losses = []
        for xb, yb in train_loader:
            bid, _ = model(xb)
            loss = bce_fn(bid, yb)
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            tr_losses.append(loss.item())

        lr_used = optimizer.param_groups[0]["lr"]
        scheduler.step()

        model.eval()
        v_bce, v_mae = [], []
        with torch.no_grad():
            for xb, yb in val_loader:
                bid, _ = model(xb)
                v_bce.append(bce_fn(bid, yb).item())
                v_mae.append(mae_fn(bid, yb).item())

        row = dict(epoch=epoch, train_bce=float(np.mean(tr_losses)),
                   val_bce=float(np.mean(v_bce)), val_mae=float(np.mean(v_mae)), lr=lr_used)
        history.append(row)
        print(f"Epoch {epoch:2d}/{EPOCHS} | train_bce {row['train_bce']:.4f} | "
              f"val_bce {row['val_bce']:.4f} | val_mae {row['val_mae']:.4f} | lr {lr_used:.5f}")

    # ---- save history -------------------------------------------------
    csv_path = RESULTS_DIR / "exp1_history.csv"
    with open(csv_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(history[0].keys()))
        w.writeheader()
        w.writerows(history)

    # ---- plot (Fig 4.1) -----------------------------------------------
    plt = apply_style()
    ep = [h["epoch"] for h in history]
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4))

    ax1.plot(ep, [h["train_bce"] for h in history], color=MAIN_COLOR, label="Train BCE")
    ax1.plot(ep, [h["val_bce"] for h in history], color=ACCENT_COLOR, label="Validation BCE")
    ax1.axhline(bce_floor, color="gray", ls="--", lw=1, label=f"BCE floor ({bce_floor:.3f})")
    for e in range(STEP_SIZE, EPOCHS, STEP_SIZE):
        ax1.axvline(e, color="gray", ls=":", lw=0.8)
    ax1.set_xlabel("Epoch"); ax1.set_ylabel("BCE loss")
    ax1.set_title("Bid head: loss vs. epoch"); ax1.legend()

    ax2.plot(ep, [h["val_mae"] for h in history], color=ACCENT_COLOR, label="Validation MAE")
    ax2.axhline(mae_mean_baseline, color="gray", ls="--", lw=1,
                label=f"Predict-the-mean MAE ({mae_mean_baseline:.3f})")
    ax2.set_xlabel("Epoch"); ax2.set_ylabel("MAE")
    ax2.set_title("Bid head: validation MAE vs. epoch"); ax2.legend()

    fig.savefig(RESULTS_DIR / "fig4_1_bid_head_training.png")
    print(f"\nSaved {csv_path} and results/fig4_1_bid_head_training.png")

    last = history[-1]
    print(f"Final: train_bce {last['train_bce']:.4f} | val_bce {last['val_bce']:.4f} | "
          f"val_mae {last['val_mae']:.4f}  (floor {bce_floor:.4f}, mean-baseline MAE {mae_mean_baseline:.4f})")

    if save_model:
        torch.save(model.state_dict(), str(_MODELS_DIR / "bid_head_trained.pt"))
        torch.save(model.state_dict(), str(_CORE_DIR / "bid_head_trained.pt"))
        print("Model weights OVERWRITTEN (--save-model). Re-run other experiments if you do this.")
    else:
        torch.save(model.state_dict(), str(RESULTS_DIR / "bid_head_exp1_copy.pt"))
        print("Existing bid_head_trained.pt left untouched (a copy is in results/).")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--save-model", action="store_true")
    args = ap.parse_args()
    main(args.seed, args.save_model)
