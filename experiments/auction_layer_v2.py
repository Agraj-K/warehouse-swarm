"""
auction_layer_v2.py - robots that lose a claim retarget to their next-best
unclaimed eligible task instead of idling.

Robots are processed in descending bid order (a deterministic stand-in for
the auction converging). Robot i takes its nearest eligible task that no
earlier robot within COMM_RANGE (and not believed failed) has already taken.
Place in warehouse-swarm/experiments/.
"""
import sys
from pathlib import Path

import numpy as np

_ROOT_DIR = Path(__file__).resolve().parent.parent
for _p in [str(_ROOT_DIR / "core"), str(_ROOT_DIR)]:
    if _p not in sys.path:
        sys.path.insert(0, _p)

from auction_layer import DecentralizedAuctionLayer


class AuctionLayerV2(DecentralizedAuctionLayer):
    def __init__(self, hetero_env, bid_model_path="bid_head_trained.pt", claim_loss=0.0, seed=0):
        super().__init__(hetero_env, bid_model_path=bid_model_path)
        self.claim_loss = claim_loss
        self.rng = np.random.default_rng(seed)
        self.dual_claims = 0

    def _ranked_eligible(self, agent, rtype):
        queue = list(self.env.unwrapped.request_queue)
        out = []
        for shelf in queue:
            tt = self.env.shelf_task_type.get(shelf, 2)
            if rtype in self.env.ACCEPTABLE_ROBOTS[tt]:
                d = abs(agent.x - shelf.x) + abs(agent.y - shelf.y)
                out.append((d, (shelf.x, shelf.y)))
        out.sort(key=lambda t: t[0])
        return out

    def step(self, obs_batch, belief_failed=None):
        if belief_failed is not None:
            self.clear_stale_claims_from_failed_robots(belief_failed)
        agents = self.env.unwrapped.agents
        n = self.n_robots
        bids = self.compute_bids(obs_batch)
        bid_vals = [float(np.asarray(bids[i]).reshape(-1)[0]) for i in range(n)]
        ranked = [self._ranked_eligible(agents[i], self.env.robot_types[i]) for i in range(n)]

        # sanity check: task key format must match the base class
        for i in range(n):
            base = self._nearest_eligible_task(agents[i], self.env.robot_types[i])
            if base is not None and ranked[i] and base not in {t for _, t in ranked[i]}:
                raise RuntimeError(f"task key mismatch: base {base!r} vs ranked {ranked[i][0][1]!r}")

        order = sorted(range(n), key=lambda i: (-bid_vals[i], i))
        claims = {}
        chosen = [None] * n
        for i in order:
            for _, task in ranked[i]:
                blocked = False
                for j in claims.get(task, []):
                    if self.claim_loss > 0 and self.rng.random() < self.claim_loss:
                        continue  # missed peer claim packet
                    near = abs(agents[i].x - agents[j].x) + abs(agents[i].y - agents[j].y) <= self.COMM_RANGE
                    believed_failed = belief_failed is not None and bool(belief_failed[i][j])
                    if near and not believed_failed:
                        blocked = True
                        break
                if not blocked:
                    chosen[i] = task
                    if task in claims and len(claims[task]) > 0:
                        self.dual_claims += 1
                    claims.setdefault(task, []).append(i)
                    break

        pursue = [c is not None for c in chosen]
        tasks = [c if c is not None else (ranked[i][0][1] if ranked[i] else None)
                 for i, c in enumerate(chosen)]
        return pursue, bids, tasks
