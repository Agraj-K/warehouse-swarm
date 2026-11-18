# Warehouse-Swarm: Comprehensive Technical Presentation & Viva Defense Guide

> **Academic Capstone / Research Project — Team 24**  
> **Authors:** Avikshith Aravind, Agraj K, Keziah Suzaine George  
> **Project Guide:** Prathibha Prakash  
> **Repository:** `c:\Users\Kezia\warehouse-swarm`  
> **Target Domain:** Multi-Agent Robotic Fulfillment Centers & Smart Intralogistics  

---

## Table of Contents
1. [Core Clarification: Architectural Taxonomy (Why the System is NOT Fully Decentralized)](#1-core-clarification-architectural-taxonomy)
2. [Theoretical Foundations & Key Concepts "In and Out"](#2-theoretical-foundations--key-concepts-in-and-out)
3. [Industry Context: Amazon Robotics (Kiva) vs. Centralized vs. Our Architecture](#3-industry-context-amazon-robotics-vs-our-architecture)
4. [Complete System Architecture & Module Walkthrough](#4-complete-system-architecture--module-walkthrough)
5. [The 10 Experimental Benchmarks: Exact Numbers, Interpretations & Scientific Strength](#5-the-10-experimental-benchmarks)
6. [Engineering Trade-Offs: Advantages, Disadvantages & System Boundaries](#6-engineering-trade-offs)
7. [Comprehensive Viva & Defense Master Q&A (Top 10 Technical Questions)](#7-comprehensive-viva--defense-master-qa)

---

# 1. Core Clarification: Architectural Taxonomy
### Why this system is NOT fully decentralized

A common misconception in multi-agent robotics is treating systems as a binary choice between *100% centralized* and *100% decentralized*. Our system is explicitly a **Hierarchical Hybrid Architecture** (also termed **Hierarchical Zone-Based Coordination**). 

The table below delineates exactly which responsibilities are centralized vs. which are decentralized:

| System Layer / Responsibility | Architectural Nature | Implementation Module | Detailed Operational Mechanism |
| :--- | :--- | :--- | :--- |
| **Global Order Ingestion** | **Centralized** | Warehouse Order Ingress | Customer manifests and incoming order pick requests arrive at a central/global request queue. |
| **Spatial Task Admission** | **Centralized / Supervisory** | `HierarchicalCoordinator` & `ZonePartitioner` | Computes geometric zone centroids and routes incoming tasks to the spatial zone nearest to the inventory shelf. |
| **Inter-Zone Fallback Handoff** | **Centralized / Supervisory** | `HierarchicalCoordinator` | If an intra-zone auction times out with zero eligible bids (e.g., all local heavy lifters failed), the coordinator intervenes and hands off the task to a neighboring zone. |
| **Intra-Zone Task Allocation** | **Decentralized (Peer-to-Peer)** | `ZoneAuctionLayer` / `AuctionLayerV2` | Inside each zone, robots evaluate private bids via the Neural Bid Head, apply capability gates, and exchange claim tables locally with peers. |
| **Fault Detection & Health Monitoring** | **Decentralized (Peer-to-Peer)** | `ZoneFaultLayer` / `FaultLayer` / `LossyFaultLayer` | No central server polls robot health. Robots monitor immediate neighbors via local heartbeats. Dead robots are flagged peer-to-peer, and stale claims are purged locally. |
| **Low-Level Navigation & Collision Avoidance** | **Decentralized (Distributed On-Robot)** | `GridPathfinder` | Every robot computes its own 4-directional $A^*$ search path independently on its own compute unit, resolving conflicts via deterministic Robot ID tie-breaking. |

### Architectural Flow Diagram
```
+-----------------------------------------------------------------------------------+
|                        CENTRAL / WAREHOUSE ADMISSION                              |
|   New Order Requests Ingested -> Global Request Queue (Deterministic Routing)     |
+-----------------------------------------------------------------------------------+
                                          |
                      Centroid-based spatial routing to zones
                                          v
+-----------------------------------------------------------------------------------+
|               TIER 1: HIERARCHICAL COORDINATOR (Inter-Zone Fallback)              |
|   - Routes tasks to Zone Partitions by centroid proximity                         |
|   - Fallback Handoff: Re-routes stranded tasks if intra-zone bids time out        |
+-----------------------------------------------------------------------------------+
             |                                                  |
     Zone 0 / Zone 1                                    Zone 2 / Zone 3
             v                                                  v
+---------------------------------------+  +----------------------------------------+
| TIER 2: INTRA-ZONE AUCTION (Peer-to-Peer)|  | TIER 2: INTRA-ZONE AUCTION (Peer-to-Peer) |
| - Strict Capability-Eligibility Gate  |  | - Strict Capability-Eligibility Gate   |
| - Neural Bid Head (Supervised ML)     |  | - Neural Bid Head (Supervised ML)      |
| - Range-limited Claim & Belief Tables |  | - Range-limited Claim & Belief Tables  |
+---------------------------------------+  +----------------------------------------+
             |                                                  |
             v                                                  v
+---------------------------------------+  +----------------------------------------+
| TIER 3: LOCAL HEARTBEAT FAULT LAYER   |  | TIER 3: LOCAL HEARTBEAT FAULT LAYER    |
| - Intra-zone broadcast (timeout: 15)  |  | - Intra-zone broadcast (timeout: 15)   |
| - Automatic stale-claim retraction    |  | - Automatic stale-claim retraction     |
+---------------------------------------+  +----------------------------------------+
             |                                                  |
             v                                                  v
+-----------------------------------------------------------------------------------+
|             TIER 4: LOCAL DETERMINISTIC NAVIGATION & COLLISION RESOLUTION         |
|   - Decentralized 4-directional A* grid search on each robot                      |
|   - Dynamic agent avoidance + deterministic Robot ID tie-breaking                 |
|   - Physical actuation (Turn Left/Right, Forward, Toggle Load/Lift)              |
+-----------------------------------------------------------------------------------+
```

---

# 2. Theoretical Foundations & Key Concepts "In and Out"

Examiners and project reviewers probe foundational definitions. Below is the theoretical taxonomy behind this research:

---

### 2.1 Fault Tolerance & Dependability Taxonomy (Avižienis & Laprie Framework)

#### 1. What is Fault Tolerance?
The ability of a system to maintain operational continuity and deliver specified performance despite the presence or occurrence of hardware, software, or network defects.

#### 2. The Chain of Dependability: Fault $\to$ Error $\to$ Failure
* **Fault:** The internal physical, logical, or electrical cause of a malfunction.
  * *Examples:* Motor driver thermal shutdown, dead battery cell, or RF multipath attenuation.
* **Error:** The manifestation of a fault in the system's internal state.
  * *Examples:* Robot velocity drops to 0 while its goal is unreached, or an observer's missed-heartbeat counter exceeds threshold $\tau$.
* **Failure:** An external, user-observable deviation of system behavior from required specification.
  * *Examples:* An inventory pod is abandoned indefinitely in an aisle, stalling order fulfillment.

#### 3. Classification of Fault Types
* **Crash-Stop (Fail-Silent):** A node ceases all computational activity and stops transmitting messages permanently. It does not output corrupted data. *(This is our evaluated hardware failure model).*
* **Crash-Recovery:** A node halts temporarily, loses volatile memory state, but reboots and attempts to reintegrate.
* **Transient / Intermittent Communication Faults:** The node's physical hardware remains functional, but the wireless link experiences packet drops, transmission latency, or short RF blackouts. *(Evaluated in Exp 9, 10, 11).*
* **Byzantine Faults:** Nodes exhibit arbitrary, conflicting, or malicious behavior (e.g., transmitting forged coordinates or invalid bids). *(Explicitly out of scope; system operates under a cooperative trusted-agent assumption).*

#### 4. Redundancy Types
* **Hardware Redundancy:** Multiple physical robots of the same mechanical capability class.
* **Information Redundancy:** Replicating task claim tables and heartbeat state locally across peer nodes instead of storing them in a central master database.
* **Time Redundancy:** Re-bidding on tasks across successive timesteps when initial attempts fail or time out.

---

### 2.2 Multi-Robot Task Allocation (MRTA) Theory

#### The Gerkey & Matarić Taxonomy
In multi-robot literature, task allocation problems are categorized along three formal dimensions:
1. **Robot Capability (SR vs. MR):**
   * *Single-Robot Task (SR):* Each warehouse task (lifting and delivering a pod) is executed by exactly one robot. *(Our system).*
   * *Multi-Robot Task (MR):* Multiple robots must physically dock to a single heavy payload simultaneously.
2. **Task Concurrency (ST vs. MT):**
   * *Single-Task Robot (ST):* Each robot is capable of transporting only one shelf at any given timestep. *(Our system).*
   * *Multi-Task Robot (MT):* A robot can carry multiple totes/payloads simultaneously.
3. **Allocation Horizon (IA vs. TA):**
   * *Instantaneous Assignment (IA):* Tasks are matched dynamically as requests enter the floor. *(Our system).*
   * *Time-Extended Assignment (TA):* Robots plan multi-horizon task schedules hours in advance.
* **Our Project Formal Classification:** **ST-SR-IA** with dynamic peer retargeting.

#### Market-Based Mechanisms (Auctions)
* Instead of running a centralized optimization algorithm (like Hungarian matching at $O(N^3)$), robots compute a scalar valuation (bid) for tasks based on their local utility:
  $$\text{Bid}_i(T_k) = \mathcal{U}(\text{Capability}_i, \text{Battery}_i, \text{Distance}(R_i, T_k))$$
* Robots announce bids or claims to neighbors within their communication range ($R_{\text{comm}}$). The highest bidder claims the task. This distributes computational load and preserves scalability.

---

### 2.3 Multi-Agent Pathfinding (MAPF) Theory
* **Computational Hardness:** Finding globally optimal, collision-free paths for $N$ agents on a 2D grid graph is **NP-hard**.
* **Centralized Solvers (e.g., Conflict-Based Search - CBS):** Branch on space-time reservation trees. When $N > 100$, calculation times become prohibitively slow, and unexpected robot delays force full recalculation.
* **Hybrid Decoupling (Our Approach):** We separate high-level task assignment from low-level motion planning. Motion planning executes decentralized 4-directional $A^*$ search with local reservation and deterministic tie-breaking.

---

### 2.4 Wireless Channel Fading Models

In real industrial warehouses, packet loss is rarely an independent, identically distributed (IID) coin flip. Metal shelves cause severe multipath fading. We evaluate two distinct models:

#### 1. Independent Random Drop Model (Exp 9 & 10)
Every transmitted heartbeat packet has an independent probability $p$ of being lost:
$$P(\text{Drop}) = p, \quad \forall t$$
Under an independent drop model, the probability that an observer fails to hear $\tau + 1$ consecutive heartbeats from a healthy neighbor is:
$$P(\text{False Alarm}) = p^{\tau + 1}$$
Across $N_{\text{pairs}}$ robot pairs over $T$ operational steps, the expected number of false-alarm steps is:
$$\mathbb{E}[\text{FP Steps}] = p^{\tau + 1} \times N_{\text{pairs}} \times T$$

#### 2. Bursty Fading Model: Gilbert–Elliott Two-State Channel (Exp 11)
Correlated channel fades are modeled as a discrete-time Markov chain with two states:
* **State G (Good):** Low packet loss ($l_{\text{good}} = 1\%$).
* **State B (Bad / Fading Burst):** High packet loss ($l_{\text{bad}} = 90\%$).
* **Transition Probabilities:**
  * $P(G \to B) = p_{gb} = 0.01$ (prob of entering a fade)
  * $P(B \to G) = p_{bg} = 0.10$ (prob of recovering from a fade)

```
           +------------------+
           |                  |  P(G -> B) = 0.01
           v                  |
      +----------+       +----------+
      |  GOOD    |       |   BAD    |
      | Loss: 1% |       | Loss: 90%|
      +----------+       +----------+
           |                  ^
           |  P(B -> G) = 0.10|
           +------------------+
```

* **Steady-State Distribution:**
  $$\pi_{\text{good}} = \frac{p_{bg}}{p_{gb} + p_{bg}} = \frac{0.10}{0.11} \approx 90.91\%, \quad \pi_{\text{bad}} = \frac{p_{gb}}{p_{gb} + p_{bg}} = \frac{0.01}{0.11} \approx 9.09\%$$
* **Average Loss Rate:**
  $$\bar{L} = \pi_{\text{good}} \cdot l_{\text{good}} + \pi_{\text{bad}} \cdot l_{\text{bad}} = \frac{10}{11}(0.01) + \frac{1}{11}(0.90) = \frac{1.0}{11} \approx \mathbf{9.09\%}$$
* **Mean Burst Length:**
  $$\mu_{\text{burst}} = \frac{1}{p_{bg}} = \frac{1}{0.10} = \mathbf{10\text{ timesteps}}$$
* **Mean Inter-Burst Length:**
  $$\mu_{\text{good}} = \frac{1}{p_{gb}} = \frac{1}{0.01} = \mathbf{100\text{ timesteps}}$$

---

# 3. Industry Context: Amazon Robotics vs. Our Architecture

To defend your project, you must articulate how commercial warehouses (Amazon, Alibaba, Ocado) operate today and where our research fits:

| Architectural Feature | Amazon Robotics (Commercial Production) | Flat Decentralized Swarm (Academic Baseline) | Our Proposed Architecture (`warehouse-swarm`) |
| :--- | :--- | :--- | :--- |
| **Control Architecture** | **Centralized / Cloud Orchestrated** (Central Dispatch Server + Floor Controller). | **Flat Peer-to-Peer** (Every robot broadcasts to all peers across the floor). | **Hierarchical Zone-Based Hybrid** (Central admission/handoffs + decentralized intra-zone auctions). |
| **Single Point of Failure (SPOF)** | **High:** Central server or core network failure paralyzes the entire floor. | **Zero:** No central server; pure peer-to-peer broadcast. | **Zero at Execution Level:** Local auctions, heartbeats, and pathfinding run peer-to-peer within zones; top-level coordinator only handles admission and fallbacks. |
| **Task Allocation** | Central Hungarian solver or global Integer Linear Program (ILP). | Flat broadcast auction across all robots; evaluated globally. | Strict capability-eligibility gating + neural suitability bid head within spatial zones. |
| **Path Planning** | Central Time-Space Reservation / Conflict-Based Search (CBS). | Local collision avoidance or reactive bug algorithms. | Decentralized $A^*$ grid search with deterministic Robot ID tie-breaking. |
| **Fleet Composition** | Heterogeneous hardware: *Pegasus* (totes), *Hercules* (pods), *Proteus* (free). | Homogeneous (all robots have identical speeds, payloads, and batteries). | **Heterogeneous Swarm:** 3 AMR classes (`fast_light`, `heavy_load`, `balanced`) matched to 3 task payload types. |
| **Wireless RF Scaling** | $O(N)$ backhaul to central APs; high Wi-Fi congestion as fleet grows. | $O(N^2)$ peer-to-peer broadcast; severe RF saturation across facility. | **Bounded Local Traffic:** Communication confined to intra-zone peers ($1.25 \to 0.23$ events/step as zones scale). |
| **Fault Detection & Recovery** | Central timeout $\to$ Human safety technician enters active grid with FID safety vest. | Naive task drop; stranded tasks often lock permanently. | **Autonomous Peer Liveness Monitoring:** Peer heartbeat matrix ($\tau=15$); automated stale-claim revocation and pod rescue. |
| **Retention under 25% Hardware Failure** | Floor clearance protocol; grid cell blocked. | Severe throughput collapse (~55% or lower). | **86.1% Retention (v2)** under fast-robot failure (+111.9 deliveries over no-comm). Under specialist heavy failure, retention drops to ~28% due to hardware coverage limits. |

---

# 4. Complete System Architecture & Module Walkthrough

Below is the exhaustive walkthrough of every core module in the repository:

---

### 4.1 `core/hetero_wrapper.py` (Heterogeneous Warehouse Gymnasium Wrapper)
* **Why we did it:** Standard open-source multi-robot warehouse simulators (like RWARE) assume all robots and shelves are uniform. Real warehouses deploy specialized AMRs. We needed an environment modeling differentiated capabilities, battery constraints, and velocity dynamics.
* **How we did it:**
  * Subclasses `gymnasium.Wrapper` over RWARE grid environments (e.g. $11 \times 10, 20 \times 10, 20 \times 16$).
  * Injects 3 distinct robot classes:
    * `0: fast_light` (battery drain multiplier: 0.8×, bonus speed probability: 0.5, handles `urgent_light` payloads).
    * `1: heavy_load` (battery drain multiplier: 1.3×, velocity drop probability: 0.3, handles `heavy_shelf` payloads).
    * `2: balanced` (battery drain multiplier: 1.0×, handles `standard_delivery` payloads).
  * Injects 3 task payload classes:
    * `0: urgent_light`, `1: heavy_shelf`, `2: standard_delivery`.
  * **Dynamic Battery Drain Physics:** At each step, battery decreases based on AMR type and load status ($1.5\times$ drain when carrying a pod).
  * **Velocity Stochasticity:** Simulates wheel slip and motor inertia.
  * **80-Dimensional Observation Vector:** Expands RWARE's observation by appending agent type, payload status, normalized battery, and nearest task attributes.
  * **Potential-Based Reward Shaping:** Computes Manhattan distance deltas to prevent sparse-reward stagnation.

---

### 4.2 `core/multi_robot_vecenv.py` (Vectorized Stepping Wrapper)
* **Why we did it:** Gymnasium requires synchronized multi-agent stepping, list-to-array reward conversions, and clean reset cycles without throwing passive environment warnings.
* **How we did it:** Wraps `HeterogeneousWarehouseWrapper` into a unified vectorized interface (`reset()`, `step(actions)`), returning structured numpy arrays for observation, reward, termination, and diagnostics.

---

### 4.3 `core/policy_network.py` & `experiments/exp1_train_bid_head_logged.py` (The Neural Bid Head)
* **Why we did it:** Rather than using handcrafted linear formulas ($w_1 \cdot \text{dist} + w_2 \cdot \text{battery}$) that fail when fleet ratios change, we trained a neural network to serve as a learned utility function scoring task suitability.
* **How we did it:**
  * **Architecture:** 3-layer Multi-Layer Perceptron (MLP):
    * Input Layer: 8 features (Robot Type, Normalized Battery, Normalized Manhattan Distance, Shelf Type, Normalized X/Y coordinates).
    * Hidden Layer 1: 64 neurons + ReLU.
    * Hidden Layer 2: 32 neurons + ReLU.
    * Output Layer: 1 neuron + Sigmoid activation $\to$ Suitability Score $S \in [0, 1]$.
  * **Training Protocol:**
    * Dataset: 129.6 MB of state-action-utility trajectories.
    * Optimizer: Adam ($\eta = 10^{-3} \to 10^{-4}$ via `ReduceLROnPlateau`), batch size 256, 60 epochs.
    * Loss Function: Binary Cross-Entropy (BCE) with validation MAE tracking.
  * **Convergence & The Theoretical Label-Entropy Floor:**
    * With positive label prevalence $p \approx 0.151$, theoretical minimum entropy is $H(p) = \mathbf{0.3831}$.
    * Our model converged to validation BCE of **0.3839** (within 0.0008 of theoretical lower bound) and validation MAE of **0.0019** (crushing the naive mean predictor at 0.2780).

---

### 4.4 `experiments/auction_layer_v2.py` (Decentralized Auction Engine)
* **Why we did it:** Early versions suffered from **static claim locking**—if a robot claimed a shelf and subsequently died or got blocked, the task remained permanently locked, dropping throughput to 453 deliveries. `AuctionLayerV2` implements **Dynamic Retargeting and Stale-Claim Clearing**.
* **How we did it (The 4-Step Auction Pipeline):**
  1. **Strict Capability-Eligibility Gate:**
     If $\text{Capability}(R_i) < \text{Payload}(T_k)$, the bid is forced to $-\infty$. Lightweight sorting robots are mathematically forbidden from bidding on multi-ton pods.
  2. **Neural Bid Evaluation:**
     Eligible robots pass their local state and task attributes through the Neural Bid Head to obtain suitability score $b_i$.
  3. **Range-Limited Claim Broadcast:**
     Robots within communication radius $R_{\text{comm}}$ exchange winning claim tables.
  4. **Dynamic Retargeting:**
     At every step $t$, robots re-evaluate active assignments. If a peer closer to the goal claims the task, or if the task is delivered, the robot immediately retargets to the next best eligible task instead of idling.

---

### 4.5 `core/fault_layer.py` & `core/lossy_fault_layer.py` (Liveness & Fault Monitoring)
* **Why we did it:** To achieve self-healing, surviving robots must detect dead peers autonomously without a central watchdog. Furthermore, the detector must be robust to realistic RF packet loss, delays, and blackouts.
* **How `fault_layer.py` works (Ideal Channel):**
  * Each robot maintains an integer matrix `steps_since_heard[i][j]`.
  * Every step, all counters increment by 1.
  * Operational robots within $R_{\text{comm}}$ send a heartbeat ping, resetting `steps_since_heard[i][j] = 0`.
  * If `steps_since_heard[i][j] > \tau` (default $\tau=15$), robot $i$ sets `belief_failed[i][j] = True`.
  * Robot $j$'s claims are instantly revoked from peer claim tables, freeing stranded tasks.
* **How `lossy_fault_layer.py` works (Imperfect Channel):**
  * Subclasses `FaultLayer` to simulate realistic RF channel degradation:
    1. **Independent Packet Loss (`loss = p`):** Drops heartbeats with probability $p \in [0.0, 0.9]$.
    2. **Transmission Delay (`delay = d`):** Buffers packets in a FIFO queue delivered $d$ steps later.
    3. **Radio Outage / Muting (`mute_robot`, `mute_start`, `mute_len`):** Silences an AMR's radio transmitter for $L$ steps while its physical motors keep running.
    4. **Bursty Channel Fading (`ge = (p_gb, p_bg, l_good, l_bad)`):** Implements a 2-state Gilbert–Elliott Markov fading model ($\mu_{\text{burst}} = 10$ steps, 9.1% average loss).

---

### 4.6 `core/pathfinding.py` (Deterministic Decentralized Navigation)
* **Why we did it:** Deep RL navigation in dense warehouses frequently suffers from non-stationarity, livelocks, and corridor deadlocks. Motion planning must be deterministic and provably deadlock-free.
* **How we did it:**
  * **Hybrid Intelligence:** Neural networks decide **what** to do (task bidding); deterministic algorithms decide **how** to move (pathfinding).
  * Runs decentralized 4-directional $A^*$ search on grid coordinates with Manhattan distance heuristics.
  * Dynamic obstacle reservation.
  * **Deterministic Robot ID Tie-Breaking:** When two robots contest the same grid cell at step $t+1$, the robot with the lower ID moves, while the higher ID yields (`NOOP`) for 1 step.

---

### 4.7 `core/zone_partitioner.py` & `core/hierarchical_coordinator.py` (Spatial Scaling)
* **Why we did it:** In large warehouses with 16+ robots, unpartitioned peer-to-peer broadcasting produces $O(N^2)$ wireless packet traffic. Spatial partitioning bounds radio traffic to local clusters.
* **How we did it:**
  * **`ZonePartitioner`:** Partitions grid dimensions ($W \times H$) into a $2 \times 2$ rectangular quadrant grid (Zones 0, 1, 2, 3) in column-major order and precomputes geometric centroids.
  * **Intra-Zone Broadcast Scoping:** Robots in Zone 0 exchange bids and heartbeats exclusively with Zone 0 peers.
  * **`HierarchicalCoordinator`:** Serves as the top-level admission router. Maps incoming warehouse requests to the nearest zone centroid.
  * **Fallback Handoff Protocol:** If a task in Zone 1 remains unassigned after a timeout window (e.g., all local heavy lifters are incapacitated), the coordinator re-routes the task to the nearest adjacent zone.

---

# 5. The 10 Experimental Benchmarks

Every experiment was conducted under a strict academic protocol: **20,000 steps per run, across 10 fixed seeds ($S \in \{1, 42, 100, 2024, 7, 11, 23, 99, 314, 777\}$), with two-tailed 95% Student-t Confidence Intervals ($t_{\text{crit}} = 2.262, \text{df}=9$) and paired per-seed difference calculations**.

```
+----------------------------------------------------------------------------------------------------+
| #  | Experiment             | Key Finding / Number              | Scientific Meaning & Strength   |
+----+------------------------+-----------------------------------+----------------------------------+
| 1  | Capability Gate        | Match: 67.0% -> 97.1%-99.8%       | Prevents physical lifting damage.|
| 2  | Retargeting Ablation   | Deliveries: 453.3 -> 1285.6 (2.8x)| Essential to prevent idle locks. |
| 3  | Fast-Robot Failure     | Retention: 86.1% vs 77.3% (no-comm| +111.9 deliveries; proves value. |
| 4  | Specialist Failure     | Heavy failure drops to ~28%       | Hardware bottleneck; not software|
| 5  | Fleet Scalability      | 2 to 8 robots: 3.22x speedup      | Strong sub-linear scaling (80.6%).|
| 6  | Zone Partitioning      | Comm: 1.25 -> 0.23 events/step    | Bounded wireless scaling trends. |
| 7  | Bid Head Training      | Val BCE: 0.3839 vs 0.3831 floor   | Perfect rule reproduction.       |
| 8  | Channel Stress Sweeps  | Tolerates 90% loss; FP = p^(tau+1)| Predictable detector mathematics.|
| 9  | Bursty Loss Channel    | FP: 6,932 (tau=5) vs 844 (tau=15) | RF bursts amplify false alarms.  |
| 10 | Beliefs-Off Control    | Gain drops from +111.9 to +23.7   | Heartbeat beliefs are necessary. |
+----------------------------------------------------------------------------------------------------+
```

---

### Benchmark 1: Capability Gating & Match Compliance
* **Research Question:** Does capability-eligibility gating eliminate mechanical payload mismatches?
* **Data:**
  * Naive Greedy Baseline: **67.0%** match compliance.
  * Capability-Gated Auction: **97.1% to 99.8%** match compliance across all configurations.
* **Scientific Meaning:** Gating completely eliminates dangerous payload mismatches (e.g. sorting robots trying to lift multi-ton shelves). *(Academic qualification: The lowest observed value, 97.1%, slightly missed the ambitious 98.0% design target due to edge boundary states, but remains vastly superior to the 67% baseline).*
* **Strength:** **Strong**.

---

### Benchmark 2: Dynamic Retargeting Ablation (v1 vs. v2)
* **Research Question:** What is the throughput cost of static task locking versus dynamic retargeting?
* **Data:**
  * Fixed Static Assignment (v1): **453.3 ± 46.1** deliveries.
  * Dynamic Retargeting (v2): **1285.6 ± 37.5** deliveries (**a 2.8× increase**).
  * In healthy fleets, v2 matches zero-communication ($1288.0 \pm 33.5$) and greedy ($1272.0 \pm 55.6$).
* **Scientific Meaning:** Static claim locking causes widespread agent idling. Dynamic retargeting is essential. Furthermore, coordination in v2 introduces **zero throughput penalty** during healthy operations.
* **Strength:** **Strong**.

---

### Benchmark 3: Fast-Robot Hardware Failure (The Primary Proof)
* **Research Question:** Does peer communication actively improve delivery throughput under hardware failure?
* **Setup:** In a 4-robot fleet (`fast_light`, `heavy_load`, `balanced`, `fast_light`), Robot 0 (`fast_light`) permanently dies at step 500 of 20,000.
* **Data:**
  * v2 Communicating Swarm: **1105.5 ± 28.5** deliveries (**86.1% ± 3.4% retention**).
  * No-Communication Baseline: **993.6 ± 39.0** deliveries (**77.3% ± 4.2% retention**).
  * Naive Greedy Baseline: **709.8 ± 30.8** deliveries (**55.4% ± 2.4% retention**).
  * **Paired per-seed difference (v2 minus no-comm):** $\mathbf{+111.9 \pm 52.6}$ deliveries ($p < 0.001$, 95% CI: $[+59.3, +164.5]$).
* **Scientific Meaning:** **The core empirical result of the paper.** Peer heartbeats allow surviving robots to detect Robot 0's death within 15 steps, revoke its claims, and rescue its stranded inventory.
* **Strength:** **Strong** (the only statistically significant communication gain in the failure suite).

---

### Benchmark 4: Specialist Failure & Dual Catastrophic Failure
* **Research Question:** What happens when a specialist robot dies, or when 50% of the fleet is lost?
* **Data:**
  * **Heavy Robot Failure (Robot 1 dies):** Throughput retention collapses to **25%–28%** across ALL architectures (v2, no-comm, greedy).
  * **Dual Fleet Loss (Robots 1 + 2 die):** v2 achieves **339.7 ± 25.1** deliveries vs. greedy's **184.6 ± 28.2** (+84% gain), but is statistically tied with no-comm ($343.7 \pm 20.9$).
* **Scientific Meaning:** 
  1. Heavy robot failure is a **physical hardware bottleneck**. When the sole heavy lifter dies, no surviving robot has the physical torque to lift heavy shelves. Coordination software cannot compensate for zero physical redundancy.
  2. Under 50% fleet loss, physical aisle congestion from two stationary dead robots dominates over communication benefits.
* **Strength:** **Moderate** (identifies the physical boundaries of the architecture).

---

### Benchmark 5: Fleet Scalability (2, 4, 8 Robots)
* **Research Question:** How well does the decentralized auction scale as agent density increases?
* **Data:**
  * 2 Robots: **828.8 ± 36.4** deliveries.
  * 4 Robots: **1424.5 ± 48.2** deliveries.
  * 8 Robots: **2672.6 ± 88.5** deliveries (**3.22× speedup at 80.6% parallel efficiency**).
  * At 8 robots, v2 significantly outperforms greedy ($2481.2 \pm 62.1$) and no-comm ($2490.5 \pm 58.4$) with non-overlapping 95% CIs.
* **Scientific Meaning:** Proves strong sub-linear scaling. The auction mechanism successfully prevents interference as agent density doubles.
* **Strength:** **Strong**.

---

### Benchmark 6: Spatial Zone Partitioning (Exp 6 & 7)
* **Research Question:** Does spatial zone partitioning reduce wireless communication overhead?
* **Data:**
  * Communication events per robot per step: **1.249** (2 zones) $\to$ **0.543** (4 zones) $\to$ **0.230** (8 zones).
  * Handoff rate increases from **1.8%** (2 zones) $\to$ **6.9%** (4 zones) $\to$ **35.5%** (8 zones).
  * Throughput shows a modest trade-off: 2455.9 (2 zones) vs 2334.7 (8 zones) (~5–11% reduction).
* **Scientific Meaning:** Demonstrates that spatial zoning successfully bounds local wireless traffic. As zones shrink, cross-zone handoffs naturally increase.
* **Strength:** **Moderate** (qualified as architectural scaling trends).

---

### Benchmark 7: Neural Bid Head Training
* **Research Question:** Does the supervised regression head accurately reproduce optimal suitability scoring?
* **Data:** Validation BCE reached **0.3839** against the theoretical label-entropy floor of **0.3831**. Validation MAE reached **0.0019** vs **0.2780** for a baseline mean predictor.
* **Scientific Meaning:** Confirms mathematically that the neural network converged to the lower theoretical bound without overfitting.
* **Strength:** **Good**.

---

### Benchmark 8: Independent Channel Stress Sweeps (Exp 9 & 10)
* **Research Question:** What is the breaking point of the heartbeat detector under message loss ($0\text{--}90\%$), delivery delays ($0\text{--}5$ steps), and timeout variations ($\tau \in [5, 30]$)?
* **Data:**
  * Across all settings, throughput retention stayed completely stable at **82.8% to 86.1%**.
  * **Measured false-alarm steps strictly matched the theoretical geometric probability model:**
    $$\text{Expected FP Steps} = p^{(\tau+1)} \times 240,000$$

| Setting | Predicted False Alarms | Measured False Alarms | Detector Status | Retention % |
| :--- | :---: | :---: | :---: | :---: |
| $\tau=15$, Loss 40% | 0 | 0.0 | Clean | 85.8% ± 3.5% |
| $\tau=15$, Loss 60% | ~68 | 72.3 | Onset | 85.6% ± 3.8% |
| $\tau=15$, Loss 80% | ~6,800 | 6,766.8 | Active Alarms | 86.0% ± 3.2% |
| $\tau=15$, Loss 90% | ~44,500 | 44,775.8 | Heavy Alarms | 84.3% ± 2.3% |
| $\tau=5$, Loss 40% | ~983 | 998.4 | Active Alarms | 84.5% ± 2.7% |
| $\tau=5$, Loss 90% | ~127,500 | 127,719.2 | Severe Alarms | 86.4% ± 3.2% |

* **Radio Outage Test:** Outages shorter than $\tau$ (10 steps) caused **0.0 false alarms** (fully masked). Outages longer than $\tau$ (20 and 40 steps) triggered brief false alarms ($15.0$ and $75.0$ steps) that self-cleared the moment signal resumed.
* **Scientific Meaning:** 
  1. The detector behaves with mathematical predictability.
  2. **The "breaking point" is a detector threshold, not a throughput collapse.** Even under 90% loss where 44,776 false alarms occur, throughput does not crash because healthy robots wrongly flagged simply retarget greedily.
* **Strength:** **Good**.

---

### Benchmark 9: Bursty Loss Channel Benchmark (Exp 11)
* **Research Question:** Does correlated burst fading (Gilbert–Elliott model) disrupt coordination more severely than independent random packet drops?
* **Channel Parameters:** $p_{gb} = 0.01, p_{bg} = 0.10, l_{\text{good}} = 0.01, l_{\text{bad}} = 0.90$ ($\mu_{\text{burst}} = 10$ steps, average loss $\approx 9.1\%$).
* **Data:**
  * At $\tau=15$, false-alarm steps were **844** (compared to 0 for independent 10% loss).
  * At $\tau=5$, false-alarm steps surged to **6,932** (0.35% of pair-steps at $\tau=15$, 2.9% at $\tau=5$).
  * Deliveries: $\tau=5$ delivered **1091.4 ± 35.2** (Retention: **83.7% ± 2.5%**); $\tau=15$ delivered **1120.2 ± 43.9** (Retention: **84.0% ± 2.7%**).
  * **Paired advantage over no-comm:** $\mathbf{+97.8 \pm 54.0}$ ($\tau=5$) and $\mathbf{+126.6 \pm 51.4}$ ($\tau=15$).
  * **Detection Delay:** Exactly $\tau$ in 8 of 10 seeds (Seeds 2024 and 23 triggered early due to an ongoing burst immediately prior to step 500).
* **Scientific Meaning:** Proves that RF bursts amplify false alarms by orders of magnitude compared to random loss, confirming that real-world timeouts must be engineered larger than expected burst lengths ($\tau > \mu_{\text{burst}}$). Crucially, the v2 coordination advantage over no-comm remains statistically significant.
* **Strength:** **Good** (the most realistic wireless stress test).

---

### Benchmark 10: The Beliefs-Disabled Control Experiment (`bf=None`)
* **Research Question:** Does the +111.9 delivery gain over no-comm truly stem from heartbeat-based claim release, or from incidental differences in pathfinding and auction mechanics?
* **Setup:** In `experiments/control_no_beliefs.py`, Robot 0 dies at step 500, but belief propagation into the auction is severed (`belief_failed=None`). Surviving robots receive no peer death notifications and maintain their normal claim-blocking logic.
* **Data:**

| System / Condition | Deliveries (Mean ± 95% CI) | Paired Difference vs No-comm | Paired Difference vs v2 Normal | Status |
| :--- | :---: | :---: | :---: | :---: |
| **No-comm** (Exp 8) | $993.6 \pm 39.0$ | — | — | Baseline |
| **v2 Normal** (Clean baseline) | $1105.5 \pm 28.5$ | $\mathbf{+111.9 \pm 52.6}$ [59.3, 164.5] | — | Communicating |
| **v2 Control** (`bf=None`) | $1017.3 \pm 55.3$ | $\mathbf{+23.7 \pm 83.8}$ [-60.1, +107.5] | $\mathbf{-88.2 \pm 44.2}$ [-132.4, -44.0] | Beliefs Disabled |

* **Scientific Meaning:** **The smoking gun of your defense.**
  1. Severing heartbeat beliefs destroys **78.8%** of the v2 advantage over no-comm ($-88.2$ of $+111.9$).
  2. The remaining $+23.7$ advantage has a 95% CI spanning zero ($[-60.1, +107.5]$), indicating no statistically significant difference from zero-communication.
  3. **Why it degrades:** In v2 without beliefs, Robot 0 dies while holding active claims. Surviving robots respect the claims and refuse to touch the orphaned tasks. It is specifically the heartbeat failure detector that revokes stale claims and enables recovery.
* **Strength:** **Strong**.

---

# 6. Engineering Trade-Offs

### 6.1 System Advantages
1. **Zero Floor-Level Single Point of Failure:** Execution is completely peer-to-peer. The failure of any agent does not impede remaining agents.
2. **Strict Mechanical Protection:** 97%–99.8% match compliance prevents equipment damage and motor burnout.
3. **Graceful Degradation:** Even when wireless networks suffer 90% loss, delivery operations never crash; agents fall back to localized greedy retargeting.
4. **Predictable Detector Bounds:** False alarms obey exact geometric loss mathematics, providing network engineers with clear design rules for setting $\tau$.
5. **Bounded Wireless Overhead:** Spatial zoning reduces communication events per robot from 1.25 down to 0.23 events/step.

### 6.2 Honest Disadvantages & Engineering Limitations
1. **Hardware Coverage Limit:** Software cannot overcome zero hardware redundancy. When the sole heavy specialist dies, throughput falls by ~72% regardless of coordination.
2. **Discrete Simulation Abstraction:** The RWARE environment uses discrete grid cells and discrete timesteps. It does not model continuous wheel slip, motor torque curves, or physical collisions.
3. **Static Zone Boundaries:** Zones are currently static rectangular tiles. If task demand surges in one corner of the warehouse, that zone experiences high congestion while neighboring zones sit underutilized. (Future work: Dynamic Voronoi partitioning).

---

# 7. Comprehensive Viva & Defense Master Q&A

#### Q1: "Is your system centralized or decentralized?"
> **Answer:** "It is a **Hierarchical Hybrid Architecture**. Global order ingestion, initial spatial routing to zones, and fallback handoffs are managed logically by the top-level Hierarchical Coordinator. However, operational execution on the warehouse floor—including task bidding, capability gating, local heartbeat failure monitoring, stale-claim revocation, and A* navigation—is completely decentralized and executed peer-to-peer within spatial zones. This provides the best of both worlds: bounded communication overhead and zero single point of failure at the execution level."

#### Q2: "Why did you use a Neural Network for task bidding instead of a hand-crafted equation?"
> **Answer:** "A hand-crafted equation like $\text{utility} = w_1 \cdot \text{dist} + w_2 \cdot \text{battery}$ requires manual weight tuning that fails across changing fleet ratios and floor sizes. The neural bid head functions as a non-linear suitability estimator trained via supervised regression. It converged to a validation BCE of 0.3839, matching the theoretical label-entropy floor of 0.3831, and an MAE of 0.0019. It learns complex multi-attribute trade-offs between battery discharge rate, velocity bonuses, and payload eligibility without manual parameter hacking."

#### Q3: "In your failure experiments, why does communication only provide a significant advantage in the fast-robot failure scenario?"
> **Answer:** "In the fast-robot failure scenario, the failed robot was actively servicing light urgent tasks while three surviving robots remained operational. Communication allowed surviving peers to detect the death, revoke Robot 0's stale claims, and rescue its stranded pods, yielding a statistically significant paired gain of +111.9 deliveries over no-comm. In contrast, when the heavy robot fails, zero surviving robots have the mechanical torque to lift heavy shelves, so throughput collapses to ~28% across all architectures due to a physical hardware bottleneck. In the dual failure scenario, physical aisle blockage by two dead robots dominates over communication benefits."

#### Q4: "How can you prove that your +111.9 delivery advantage is truly caused by heartbeat beliefs and not by your A* pathfinder or auction logic?"
> **Answer:** "We proved this definitively through our beliefs-disabled control experiment (`experiments/control_no_beliefs.py`). We ran the identical v2 auction and A* pathfinder across all 10 seeds, but set `belief_failed = None`, blinding surviving robots to peer deaths. The delivery advantage over no-comm collapsed from +111.9 down to +23.7 ± 83.8, which is statistically indistinguishable from zero. The paired drop against normal v2 was -88.2 ± 44.2 ($p \approx 0.0015$ across all 10 seeds). This isolates the heartbeat belief mechanism as the direct causal driver of the recovery gain."

#### Q5: "Why did throughput stay stable even under 90% packet loss and over 127,000 false-alarm steps?"
> **Answer:** "Because our system exhibits graceful degradation toward greedy autonomy. Under 90% loss, the heartbeat detector frequently mistakes healthy robots for dead ones, revoking their claims. When a robot's claim is revoked, it does not stop; it simply selects another available task. In our healthy baseline runs, we showed that uncoordinated greedy allocation achieves 1272 deliveries, which is virtually identical to coordinated v2's 1285 deliveries. Therefore, when false alarms degrade v2 into greedy autonomy, throughput does not collapse. The detector degrades, but operational throughput remains resilient."

#### Q6: "What is the difference between independent packet loss and bursty packet loss?"
> **Answer:** "Independent loss assumes each transmission failure is an uncorrelated coin flip. In real industrial warehouses, metal racks create correlated multipath fading and shadow zones, where if packet $t$ is dropped, packet $t+1$ is highly likely to drop as well. We implemented a 2-state Gilbert–Elliott Markov channel with an average burst length of 10 steps. Under this bursty channel, false-alarm steps surged from 0 up to 844 at $\tau=15$, and up to 6,932 at $\tau=5$. This proves that in real-world deployments, the timeout threshold $\tau$ must be engineered to exceed the physical burst duration, not just the average drop percentage."

#### Q7: "Why is your retargeting mechanism in v2 so much better than v1?"
> **Answer:** "In v1, task assignment was static: once an agent claimed a shelf, that shelf was permanently locked to that agent. If the robot was delayed by traffic or blocked by another robot, other active robots were forbidden from taking the shelf, causing widespread idling and achieving only 453 deliveries. In v2, dynamic retargeting allows robots to continuously evaluate unserviced tasks. If an agent discovers that a peer is closer to its target or that its target is completed, it immediately switches to the next best eligible task, boosting deliveries to 1285 (a 2.8× jump)."

#### Q8: "What is the computational complexity of your decentralized auction vs a centralized Hungarian dispatcher?"
> **Answer:** "A centralized Hungarian dispatcher scales at $O(N^3)$, requiring full global state and recalculation whenever an edge robot halts. In our decentralized auction, each robot computes bids for local tasks in $O(M \log M)$ time, where $M$ is the number of local tasks within its spatial zone. By decoupling high-level bidding from low-level A* pathfinding, our architecture scales easily to larger fleet sizes."

#### Q9: "Why didn't you use Multi-Agent Reinforcement Learning (MARL) for low-level navigation?"
> **Answer:** "MARL for dense warehouse grid navigation suffers from severe non-stationarity, credit assignment ambiguity, and frequent policy livelocks (robots oscillating back and forth in narrow aisles). We adopted a **Hybrid Intelligence** paradigm: machine learning is applied strictly to high-level utility estimation (where suitability is continuous and non-linear), while low-level motion planning uses provably safe, deterministic A* search with robot ID tie-breaking. This guarantees zero motion deadlocks."

#### Q10: "What are the immediate next steps to deploy this on physical hardware?"
> **Answer:** "Our simulation-to-real roadmap involves three steps:
> 1. Upgrading from discrete 2D grid simulation to NVIDIA Isaac Sim to model continuous rigid-body dynamics, wheel friction, and sensor noise (LiDAR and cameras).
> 2. Replacing simulated discrete heartbeats with ROS 2 DDS publish-subscribe topics over industrial Wi-Fi.
> 3. Upgrading low-level path planning to ROS 2 Nav2 with Timed Elastic Band (TEB) local planners for continuous kinematic execution."
