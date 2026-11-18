"""
DecentralizedAuctionLayer with RECEIVER-SIDE FILTERING of failed senders.

Why: in core/auction_layer.py, step() clears stale beliefs/claims from robots a
peer believes have failed, and THEN lets every robot (including the failed one)
broadcast its bid and claim again in the same step. The failed robot's bid
re-enters the peers' tables, so the stale claim is never actually released and
peers keep deferring to a robot that cannot move.

Fix: a receiver j ignores bids and claims from a sender i that j currently
believes has failed (belief_failed[j][i]). This stays fully decentralized:
it only uses j's own local belief from the heartbeat layer.

Place this file in warehouse-swarm/experiments/.
"""
import sys
from pathlib import Path

_ROOT_DIR = Path(__file__).resolve().parent.parent
_CORE_DIR = _ROOT_DIR / "core"
for _p in [str(_CORE_DIR), str(_ROOT_DIR)]:
    if _p not in sys.path:
        sys.path.insert(0, _p)

from auction_layer import DecentralizedAuctionLayer


class FixedAuctionLayer(DecentralizedAuctionLayer):
    def step(self, obs_batch, belief_failed=None):
        if belief_failed is not None:
            self.clear_stale_claims_from_failed_robots(belief_failed)

        agents = self.env.unwrapped.agents
        bids = self.compute_bids(obs_batch)

        def ignores(j, i):
            return belief_failed is not None and bool(belief_failed[j][i])

        nearest_task_per_robot = [
            self._nearest_eligible_task(agents[i], self.env.robot_types[i])
            for i in range(self.n_robots)
        ]

        # ---- bid broadcasts -------------------------------------------------
        for i, agent_i in enumerate(agents):
            task_i = nearest_task_per_robot[i]
            if task_i is None:
                continue
            self.bid_broadcasts_made += 1
            changed_someone = False
            for j, agent_j in enumerate(agents):
                if i == j or self._distance(agent_i, agent_j) > self.COMM_RANGE:
                    continue
                if ignores(j, i):          # <-- the fix
                    continue
                current_best = self.belief[j].get(task_i)
                if current_best is None or bids[i] > current_best[0]:
                    old_winner = current_best[1] if current_best else None
                    self.belief[j][task_i] = (bids[i], i)
                    if old_winner != i:
                        changed_someone = True
            if changed_someone:
                self.bid_broadcasts_useful += 1

        for i in range(self.n_robots):
            task_i = nearest_task_per_robot[i]
            if task_i is None:
                continue
            current_best = self.belief[i].get(task_i)
            if current_best is None or bids[i] > current_best[0]:
                self.belief[i][task_i] = (bids[i], i)

        provisional_pursue = []
        for i in range(self.n_robots):
            task_i = nearest_task_per_robot[i]
            if task_i is None:
                provisional_pursue.append(False)
                continue
            best_bid, best_robot = self.belief[i][task_i]
            provisional_pursue.append(best_robot == i)

        # ---- claim broadcasts -----------------------------------------------
        for i, agent_i in enumerate(agents):
            task_i = nearest_task_per_robot[i]
            if task_i is None or not provisional_pursue[i]:
                continue
            self.claim_broadcasts_made += 1
            changed_someone = False
            for j, agent_j in enumerate(agents):
                if i == j or self._distance(agent_i, agent_j) > self.COMM_RANGE:
                    continue
                if ignores(j, i):          # <-- the fix
                    continue
                would_have_pursued = (
                    nearest_task_per_robot[j] == task_i and provisional_pursue[j]
                )
                self.known_claims[j][task_i] = i
                if would_have_pursued and j != i:
                    changed_someone = True
            if changed_someone:
                self.claim_broadcasts_useful += 1

        for i in range(self.n_robots):
            if provisional_pursue[i] and nearest_task_per_robot[i] is not None:
                self.known_claims[i][nearest_task_per_robot[i]] = i

        final_pursue = []
        for i in range(self.n_robots):
            task_i = nearest_task_per_robot[i]
            if task_i is None:
                final_pursue.append(False)
                continue
            claimer = self.known_claims[i].get(task_i)
            if claimer is not None and claimer != i:
                final_pursue.append(False)
            else:
                final_pursue.append(provisional_pursue[i])

        return final_pursue, bids, nearest_task_per_robot
