"""
Render a high-resolution publication-quality figure of the Warehouse Swarm GUI
(Fig 4.8) directly matching visualization/run_simulation_gui.py styling.
"""
import sys
from pathlib import Path

_ROOT_DIR = Path(__file__).resolve().parent.parent
_CORE_DIR = _ROOT_DIR / "core"
_EXP_DIR = _ROOT_DIR / "experiments"
_VIS_DIR = _ROOT_DIR / "visualization"
for _p in [str(_CORE_DIR), str(_EXP_DIR), str(_VIS_DIR), str(_ROOT_DIR)]:
    if _p not in sys.path:
        sys.path.insert(0, _p)

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from multi_robot_vecenv import MultiRobotVecEnv
from zone_partitioner import ZonePartitioner
from hierarchical_coordinator import HierarchicalCoordinator
from exp_utils import RESULTS_DIR

ROBOT_COLORS = {
    0: "#228be6",  # fast_light -> Blue
    1: "#fa5252",  # heavy_load -> Red
    2: "#40c057",  # balanced   -> Green
}
ZONE_COLORS = ["#f8f9fa", "#f1f3f5", "#e9ecef", "#dee2e6"]


def generate_gui_figure(steps=35, seed=42):
    n_robots = 16
    robot_types = [0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0]
    vec_env = MultiRobotVecEnv(env_id="rware-small-16ag-v2", robot_types=robot_types, seed=seed)
    obs = vec_env.reset()
    env = vec_env.hetero_env
    ua = env.unwrapped
    grid_h, grid_w = ua.grid_size[0], ua.grid_size[1]

    partitioner = ZonePartitioner(grid_w, grid_h, n_zones=4)
    zone_assignment = partitioner.assign_robots_to_zones(ua.agents)
    coordinator = HierarchicalCoordinator(env, partitioner, zone_assignment, bid_model_path="bid_head_trained.pt")

    for step in range(steps):
        pursue, bids, nearest_tasks = coordinator.step(obs)
        actions = coordinator.pathfinder.get_actions_for_pursuit(obs, pursue, nearest_tasks)
        actions = coordinator.get_fault_action_overrides(actions)
        obs, rewards, dones, infos = vec_env.step(actions)
        coordinator.clear_stale_beliefs()

    # Create figure
    fig, (ax_main, ax_stats) = plt.subplots(1, 2, figsize=(14, 8), gridspec_kw={"width_ratios": [3.2, 1.2]})
    fig.patch.set_facecolor("#212529")
    ax_main.set_facecolor("#1a1d20")
    ax_stats.set_facecolor("#2b3035")

    # Draw zones
    for z in range(4):
        col, row = z % 2, z // 2
        x1 = col * partitioner.zone_col_width
        y1 = row * partitioner.zone_row_height
        w = partitioner.zone_col_width
        h = partitioner.zone_row_height
        rect = patches.Rectangle((x1, y1), w, h, linewidth=2, edgecolor="#adb5bd",
                                 facecolor=ZONE_COLORS[z], alpha=0.35, zorder=1)
        ax_main.add_patch(rect)
        ax_main.text(x1 + 1.2, y1 + h - 1.0, f"ZONE {z}", color="#adb5bd",
                     fontsize=12, fontweight="bold", alpha=0.8, zorder=2)

    # Draw goals
    for gx, gy in ua.goals:
        rect = patches.Rectangle((gx, gy), 1, 1, linewidth=1.5, edgecolor="#37b24d",
                                 facecolor="#8ce99a", alpha=0.8, zorder=2)
        ax_main.add_patch(rect)
        ax_main.text(gx + 0.5, gy + 0.5, "GOAL", color="#2b8a3e", fontsize=7,
                     fontweight="bold", ha="center", va="center", zorder=3)

    # Draw shelves
    active_shelves = set(ua.request_queue)
    for shelf in ua.shelfs:
        sx, sy = shelf.x, shelf.y
        is_req = shelf in active_shelves
        fill_c = "#ffd43b" if is_req else "#adb5bd"
        edge_c = "#e67700" if is_req else "#868e96"
        rect = patches.Rectangle((sx + 0.1, sy + 0.1), 0.8, 0.8, linewidth=1.2,
                                 edgecolor=edge_c, facecolor=fill_c, zorder=3)
        ax_main.add_patch(rect)
        if is_req:
            task_t = env.shelf_task_type.get(shelf, 2)
            char = ["U", "H", "S"][task_t]
            ax_main.text(sx + 0.5, sy + 0.5, char, color="#343a40",
                         fontsize=8, fontweight="bold", ha="center", va="center", zorder=4)

    # Draw robots
    failed_flags = coordinator.fault_layers[0].actual_failed
    n_carrying = 0
    for i, agent in enumerate(ua.agents):
        rx, ry = agent.x, agent.y
        r_type = robot_types[i]
        is_failed = failed_flags[i]
        col = "#495057" if is_failed else ROBOT_COLORS[r_type]

        circle = patches.Circle((rx + 0.5, ry + 0.5), 0.42, facecolor=col,
                                edgecolor="#ffffff", linewidth=1.5, zorder=5)
        ax_main.add_patch(circle)

        if agent.carrying_shelf is not None:
            n_carrying += 1
            ring = patches.Circle((rx + 0.5, ry + 0.5), 0.25, facecolor="#ffe066",
                                  edgecolor="#f08c00", linewidth=1.5, zorder=6)
            ax_main.add_patch(ring)

        ax_main.text(rx + 0.5, ry + 0.5, str(i),
                     color="#212529" if agent.carrying_shelf else "white",
                     fontsize=8, fontweight="bold", ha="center", va="center", zorder=7)

    ax_main.set_xlim(0, grid_w)
    ax_main.set_ylim(0, grid_h)
    ax_main.set_aspect("equal")
    ax_main.invert_yaxis()
    ax_main.set_title("Warehouse Floor Plan — 16 Robots in 4 Partitioned Zones",
                      color="white", fontsize=14, pad=12, fontweight="bold")
    ax_main.set_xticks(range(0, grid_w + 1, 2))
    ax_main.set_yticks(range(0, grid_h + 1, 2))
    ax_main.tick_params(colors="#adb5bd")
    for spine in ax_main.spines.values():
        spine.set_color("#495057")

    # Stats panel
    ax_stats.axis("off")
    tot_del = sum(env.match_count) + sum(env.mismatch_count)
    match_rate = (sum(env.match_count) / tot_del * 100) if tot_del > 0 else 100.0

    stats_text = (
        "LIVE SIMULATION METRICS\n"
        "───────────────────────────────\n\n"
        f"Step Count: {steps}\n"
        f"Active Robots: {n_robots}\n"
        f"Spatial Zones: 4\n\n"
        "PERFORMANCE:\n"
        f"  Total Deliveries: {tot_del}\n"
        f"  Match Rate: {match_rate:.1f}%\n"
        f"  Carrying Pods: {n_carrying}\n"
        f"  Cross-Zone Handoffs: {coordinator.cross_zone_handoffs}\n"
        f"  Comm Events/Robot: {coordinator.avg_comm_per_robot_per_step():.2f}\n\n"
        "ROBOT LEGEND:\n"
        "  ● Fast Light (Type 0)\n"
        "  ● Heavy Load (Type 1)\n"
        "  ● Balanced (Type 2)\n\n"
        "TASK LEGEND:\n"
        "  [U] Urgent Light\n"
        "  [H] Heavy Shelf\n"
        "  [S] Standard Delivery\n"
    )

    ax_stats.text(0.08, 0.95, stats_text, color="#f8f9fa", fontfamily="DejaVu Sans",
                  fontsize=10.5, verticalalignment="top", linespacing=1.4)

    out_file = RESULTS_DIR / "fig4_8_gui_simulation.png"
    plt.tight_layout()
    fig.savefig(out_file, dpi=200, bbox_inches="tight", facecolor=fig.get_facecolor())
    plt.close(fig)
    vec_env.close()
    print(f"Successfully generated {out_file}")


if __name__ == "__main__":
    generate_gui_figure()
