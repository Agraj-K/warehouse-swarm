import numpy as np
from fault_layer import FaultLayer


class LossyFaultLayer(FaultLayer):
    """FaultLayer with heartbeat loss, delay, temporal radio outage, and a configurable timeout.

    Parameters
    ----------
    loss : float
        Independent per-packet drop probability (used when ge=None).
    delay : int
        Fixed per-packet delivery delay in steps.
    timeout : int
        Heartbeat timeout threshold tau.
    seed : int
        RNG seed.
    mute_robot : int or None
        Robot index to silence temporarily (healthy but radio-silent).
    mute_start : int
        Step at which the radio outage begins.
    mute_len : int
        Duration of the radio outage in steps.
    ge : tuple or None
        Gilbert-Elliott two-state channel parameters:
        (p_good_to_bad, p_bad_to_good, loss_good, loss_bad).
        When set, overrides `loss`. Each (i, j) link has its own Markov state.
        Example: (0.01, 0.1, 0.01, 0.9) gives ~9% average loss in bursts of ~10 steps.
    """

    def __init__(self, hetero_env, loss=0.0, delay=0, timeout=15, seed=0,
                 mute_robot=None, mute_start=0, mute_len=0, ge=None):
        super().__init__(hetero_env)
        self.loss = loss
        self.delay = delay
        self.HEARTBEAT_TIMEOUT = timeout
        self.mute_robot = mute_robot
        self.mute_start = mute_start
        self.mute_len = mute_len
        self.ge = ge  # None or (p_gb, p_bg, l_good, l_bad)
        self.rng = np.random.default_rng(seed)
        self.link_bad = {}   # (i, j) -> bool: current GE channel state
        self.pending = []    # (deliver_at_step, receiver j, sender i)
        self.t = 0
        self.false_positive_steps = 0   # healthy robot believed failed
        self.detect_step = {}           # failed robot -> first step anyone flags it

    def _packet_dropped(self, i, j):
        """Return True if the heartbeat from i to j should be dropped this step."""
        if self.ge is not None:
            p_gb, p_bg, l_good, l_bad = self.ge
            bad = self.link_bad.get((i, j), False)
            if bad:
                if self.rng.random() < p_bg:
                    bad = False
            else:
                if self.rng.random() < p_gb:
                    bad = True
            self.link_bad[(i, j)] = bad
            return self.rng.random() < (l_bad if bad else l_good)
        elif self.loss > 0:
            return self.rng.random() < self.loss
        return False

    def step(self):
        agents = self.env.unwrapped.agents
        n = self.n_robots
        for i in range(n):
            for j in range(n):
                if i != j:
                    self.steps_since_heard[i][j] += 1
        for i, ai in enumerate(agents):
            if self.actual_failed[i]:
                continue
            if (self.mute_robot is not None
                    and i == self.mute_robot
                    and self.mute_start <= self.t < self.mute_start + self.mute_len):
                continue  # temporary radio outage: operational but radio silent
            for j, aj in enumerate(agents):
                if i != j and self._distance(ai, aj) <= self.COMM_RANGE:
                    if self._packet_dropped(i, j):
                        continue
                    self.pending.append((self.t + self.delay, j, i))
        due = [p for p in self.pending if p[0] <= self.t]
        self.pending = [p for p in self.pending if p[0] > self.t]
        for _, j, i in due:
            self.steps_since_heard[j][i] = 0
        for i in range(n):
            for j in range(n):
                if i != j:
                    self.belief_failed[i][j] = self.steps_since_heard[i][j] > self.HEARTBEAT_TIMEOUT
                    if self.belief_failed[i][j]:
                        if not self.actual_failed[j]:
                            self.false_positive_steps += 1
                        elif j not in self.detect_step:
                            self.detect_step[j] = self.t
        self.t += 1
        return self.belief_failed
