# Warehouse-Swarm: Fault-Tolerant Decentralized Coordination of Heterogeneous Robot Swarms

**Academic Capstone / Research Project — Team 24**  
**Authors:** Avikshith Aravind, Agraj K, Keziah Suzaine George  
**Project Guide:** Prathibha Prakash  
**Repository:** `c:\Users\Kezia\warehouse-swarm`  
**Target Domain:** Multi-Agent Robotic Fulfillment Centers & Smart Intralogistics  

---

## Table of Contents
1. [Executive Summary & Project Overview](#1-executive-summary--project-overview)
2. [Industry Context: Real-World Comparison & Amazon Fulfillment Centers](#2-industry-context-real-world-comparison--amazon-fulfillment-centers)
3. [System Architecture & Core Methodology](#3-system-architecture--core-methodology)
4. [Mathematical & Algorithmic Foundations](#4-mathematical--algorithmic-foundations)
5. [Complete Codebase Architecture & File Walkthrough](#5-complete-codebase-architecture--file-walkthrough)
6. [Comprehensive Experimental Results & Empirical Benchmarks](#6-comprehensive-experimental-results--empirical-benchmarks)
7. [Engineering Challenges, Debugging Log & Key Fixes](#7-engineering-challenges-debugging-log--key-fixes)
8. [Execution Guide: How to Run Experiments & Interactive GUI](#8-execution-guide-how-to-run-experiments--interactive-gui)
9. [Future Work & Hardware Deployment Roadmap](#9-future-work--hardware-deployment-roadmap)
10. [Complete Verbatim Source Code of All Project Files](#10-complete-verbatim-source-code-of-all-project-files)

---

## 1. Executive Summary & Project Overview

### 1.1 Problem Statement
Modern e-commerce warehouses (e.g., Amazon, Alibaba, Ocado) operate hundreds or thousands of Autonomous Mobile Robots (AMRs) to transport heavy inventory pods, sorting bins, and parcels across vast fulfillment floors. Traditional warehouse management architectures rely on **centralized orchestration**, where a central server computes all task dispatches, schedules, and global collision-free paths. 

While centralized systems achieve high theoretical optimality, they introduce critical vulnerabilities:
1. **Single Point of Failure (SPOF):** A central server crash, network latency spike, or database lock halts the entire warehouse floor.
2. **Quadratic Scaling Bottleneck:** As fleet size $N$ expands, central Multi-Agent Path Finding (MAPF) becomes NP-hard, and wireless communication channels congest rapidly.
3. **Hardware Fragility:** In real warehouses, robots suffer mechanical failures (wheel slippage, motor stalls, battery depletion, camera occlusion). Centralized systems struggle to detect unannounced edge failures without heavy polling.

This project presents a **decentralized, zone-based coordination architecture for heterogeneous robot swarms**. Robots bid for tasks through capability-aware peer-to-peer auctions, detect failures through peer heartbeats, and automatically recover orphaned tasks. Spatial zones maintain bounded, local communication per robot as the fleet grows. The system is evaluated in simulation on the Multi-Robot Warehouse (RWARE) environment, extended with robot and task types.

### 1.2 Proposed Solution: Decentralized Hierarchical Zone Coordination
This project designs, implements, and evaluates an end-to-end fault-tolerant, decentralized, zone-based coordination architecture for heterogeneous robot swarms. The challenge is to allocate tasks across robots of different capabilities without per-task central dispatch, while surviving robot failures and keeping communication overhead bounded as the fleet grows. Experimental evaluations demonstrate that the architecture sustains ~86% throughput retention across communication delays and message drop rates, tolerating temporary radio outages and channel packet loss up to 40% under default timeouts without false failure triggers.

### 1.3 Project Objectives
1. **Capability Matching:** Match tasks to robots strictly by mechanical capability, targeting at least 98% capability match rate across configurations (empirically 97.1% to 99.8% vs 67.0% for naive greedy).
2. **Decentralized Allocation:** Allocate tasks through localized peer auctions guided by a learned neural bid function and capability gates.
3. **Autonomous Self-Healing:** Detect and recover from sudden hardware failures automatically via local heartbeat monitoring, retaining operational throughput.
4. **Bounded Communication Scaling:** Maintain bounded, local communication overhead per robot across the tested fleet sizes through spatial zone partitioning.
5. **Rigorous Empirical Benchmarking:** Benchmark against greedy allocation baselines and zero-communication ablations under verified seed-matched conditions.

### 1.4 Scope & Evaluated Scenarios
- **Included:** Extended RWARE simulation environment, heterogeneous fleet configurations of 2 to 16 robots, multi-class payloads, supervised neural suitability bid scoring, localized zone-based auctions, heartbeat fault detection, stranded cargo recovery, and decentralized $A^*$ navigation.
- **Evaluated Scenarios:**
  - **S1 (Healthy Operations):** Multi-agent fulfillment efficiency across 2, 4, 8, and 16 robots.
  - **S2 (Robot 0 Fast Failure):** Single non-critical AMR hardware failure testing coordination reallocation.
  - **S3 (Robot 1 Heavy Failure):** Bottleneck single-specialist AMR failure testing capability coverage.
  - **S4 (Dual Failure):** Catastrophic 50% fleet loss (Robots 1+2) evaluating degradation resilience.
  - **S5 (No-Communication Ablation):** Isolating the precise empirical value of peer communication.
  - **S6 (Multi-Zone Scaling):** Spatial partitioning bounding wireless traffic on larger floors.
  - **S7 (Communication Fault Robustness - Exp 9, 10, 11 & Control):** Systematic evaluation of heartbeat transmission under message drop rates (\text{--}90\%$), delivery delays (\text{--}5$ steps), timeout variations ($\tau \in [5, 30]$), temporal radio muting (10–40 steps), lossy auction claim broadcasts (\text{--}20\%$), Gilbert–Elliott bursty channel fading (Exp 11), and a beliefs-disabled ablation control verifying the heartbeat mechanism.
- **Excluded:** Physical hardware AMR deployment and continuous-physics robotic simulation (designated as future roadmap work targeting NVIDIA Isaac Sim and ROS 2).

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
|   - Decentralized 4-directional A* grid search                                    |
|   - Dynamic agent avoidance + deterministic Robot ID tie-breaking                 |
|   - Physical actuation (Turn Left/Right, Forward, Toggle Load/Lift)              |
+-----------------------------------------------------------------------------------+
```

Key characteristics of this system:
- **No Central Coordinator for Execution:** Robots bid on tasks locally via range-limited peer-to-peer communication.
- **Heterogeneous Specialization:** Fleets are composed of distinct AMR classes (`fast_light`, `heavy_load`, `balanced`) matched against varied task payloads (`urgent_light`, `heavy_shelf`, `standard_delivery`).
- **Hybrid Intelligence:** Neural learning (Supervised Regression on capability, battery, and proximity) is applied strictly to **task evaluation/bidding**, while motion planning remains **provably safe, deterministic A\***.
- **Heartbeat-Driven Self-Healing:** Failed robots are detected within 15 timesteps by local neighbors via missing heartbeats; their claimed tasks are automatically freed and reassigned to active robots.
- **Hierarchical Spatial Partitioning:** The warehouse is divided into spatial zones, restricting message passing to local spatial clusters and maintaining bounded, local communication per robot.

---

## 2. Industry Context: Real-World Comparison & Amazon Fulfillment Centers

### 2.1 How Amazon Robotics (Kiva Systems) Operates Today
Following the acquisition of Kiva Systems, commercial automated fulfillment centers (e.g., Amazon Robotics) have scaled Autonomous Mobile Robots across fulfillment centers worldwide (such as sortable and non-sortable facilities).

```
+---------------------------------------------------------------------------------+
|                        AMAZON FULFILLMENT CENTER LAYOUT                         |
+---------------------------------------------------------------------------------+
|                                                                                 |
|  [PICK / PACK STATIONS]  ===================>  [HUMAN WORKERS / ROBIN ROBOTS]  |
|         ^                                                                       |
|         |  High-density transit highway (Unidirectional drive aisles)           |
|         v                                                                       |
|  +--------------+  +--------------+  +--------------+  +--------------+         |
|  | Pod (Shelf)  |  | Pod (Shelf)  |  | Pod (Shelf)  |  | Pod (Shelf)  |         |
|  | High Velocity|  | Heavy Items  |  | Standard SKU |  | Electronics  |         |
|  +--------------+  +--------------+  +--------------+  +--------------+         |
|         ^                                                                       |
|         | Drive units drive UNDER pods, lift them with corkscrew/ball-screw     |
|         v                                                                       |
|  [CHARGING PADS]   ===================>   [MAINTENANCE & SAFETY CORDON (FID)]   |
|                                                                                 |
+---------------------------------------------------------------------------------+
```

#### The Real-World Pipeline at Amazon:
1. **Drive Units:** Autonomous drive units (such as the *Pegasus*, *Hercules*, and *Proteus* models) navigate over a concrete floor marked with 2D DataMatrix (fiducial) QR code stickers spaced approximately 1 meter apart.
2. **Central Dispatch ("Pegasus / Central Resource Allocation Server"):** A large-scale distributed cloud/edge server assigns picking tasks from customer orders (manifests) to specific pods, and reserves time-space trajectories for each robot to avoid collisions.
3. **Goods-to-Person (G2P):** Drive units tunnel beneath movable inventory pods, actuate a rotating lift mechanism to raise the entire pod off the floor, and navigate to ergonomic workstations where human pickers or robotic arms (e.g., *Sparrow*, *Robin*) pick individual items.
4. **Safety & Fault Handling:** If a drive unit breaks down, drops an item, or experiences wheel slip, it halts. Human operators must wear safety transmitter vests (Safety Vests / FID) to enter the cordoned active robotics floor to inspect or tow the vehicle.

### 2.2 Detailed Comparison: Amazon Robotics vs. Our Warehouse-Swarm Architecture

| Feature / Dimension | Amazon Robotics (Commercial Production) | Flat Decentralized Swarm (Academic Baseline) | Our Proposed System (`warehouse-swarm`) |
| :--- | :--- | :--- | :--- |
| **Control Architecture** | **Centralized / Cloud Orchestrated** (Central Dispatch Server + Floor Controller). | **Flat Decentralized** (Every robot broadcasts to all peers across the whole warehouse). | **Hierarchical Zone-Based Decentralized** (Deterministic zone routing + intra-zone peer auctions). |
| **Single Point of Failure** | **High:** Central server or wireless access point failure paralyzes the entire floor. | **Zero:** No central server; pure peer-to-peer broadcast. | **Low / Graceful Degradation:** Execution auctions and heartbeats run purely decentralized within zones; the top-level coordinator functions as a logical task ingress/router. |
| **Task Allocation** | Central Hungarian algorithm or global auction solver. | Flat broadcast auction across all robots; bids evaluated locally. | Capability-eligibility gating + neural suitability bid head within spatial zones. |
| **Path Planning** | Central Time-Space Reservation / Conflict-Based Search (CBS). | Local collision avoidance or reactive bug algorithms. | Decentralized A* grid search with deterministic ID priority tie-breaking. |
| **Fleet Composition** | Heterogeneous hardware: Pegasus (tote sortation), Hercules (heavy pods), Proteus (free-roaming). | Homogeneous (all robots have identical speeds, payloads, and battery). | **Heterogeneous Swarm:** 3 distinct robot classes (`fast_light`, `heavy_load`, `balanced`) matched to 3 task types. |
| **Communication Scaling** | $O(N)$ backhaul to central server; requires dedicated industrial enterprise Wi-Fi. | $O(N^2)$ peer-to-peer broadcasts; causes severe RF channel saturation as $N$ grows. | **Bounded Local Traffic:** Communication is restricted to intra-zone peers; counted message events fall progressively as zone partitioning granularity increases ($1.25 \to 0.23$ events/step). |
| **Fault Detection & Recovery** | Central heartbeat timeout $\to$ manual human technician dispatch with safety vest. | Naive task drop; stranded tasks remain locked indefinitely. | **Automated Liveness Monitoring:** Peer heartbeat matrix with timeout $\tau=15$; realistic in-place cargo rescue by healthy AMR & delayed perimeter towing. |
| **Throughput under 25% Failure** | Requires floor clearance procedure; entire grid cell often blocked. | Severe degradation or complete stall on locked tasks. | **86.1% Retention (v2)** under fast-robot failure (+111.9 deliveries over no-comm). Under heavy failure, retention drops to ~28% due to capability coverage. |

### 2.3 Real-World Problem Alignment

1. **Overcoming the Centralized Bottleneck:**  
   In large-scale facilities, computing global conflict-free paths introduces computational latency and centralized scheduling overhead, which can induce stop-and-go stuttering. Our architecture executes pathfinding locally on each robot using decentralized A* search.
2. **Preventing Wi-Fi Congestion in Industrial Environments:**  
   Warehouses are harsh RF environments filled with metal shelving. When hundreds of robots broadcast status packets simultaneously, packet drop rates surge. By enforcing **Zone Partitioning**, radio packets never leave local spatial clusters, preventing RF spectrum collapse.
3. **Hardware Specialization & Battery Dynamics:**  
   Real fulfillment centers deploy specialized robots: lightweight sorting robots move fast with lower battery capacity, while heavy pod-lifters consume massive current. Our system actively models continuous battery degradation and penalizes mismatched assignments.

---

## 3. System Architecture & Core Methodology

The project operates as a layered 4-tier pipeline combining multi-agent environment wrapping, neural decision heads, spatial partitioning, and deterministic safety mechanisms.

```
+---------------------------------------------------------------------------------------------------------+
|                                    RWARE GYMNASIUM SIMULATOR BASE                                       |
|  Grid Environment (e.g. 11x10, 20x10, 20x16) | Shelves, Drop-off Goals, Physical Collision Engine       |
+---------------------------------------------------------------------------------------------------------+
                                                     |
                                                     v
+---------------------------------------------------------------------------------------------------------+
|                                HeterogeneousWarehouseWrapper (Gym.Wrapper)                              |
|  - Injects Robot Types: fast_light (0), heavy_load (1), balanced (2)                                    |
|  - Injects Task Types: urgent_light (0), heavy_shelf (1), standard_delivery (2)                         |
|  - Dynamic Battery Drain (0.8x, 1.3x, 1.0x) & Probabilistic Velocity Slip (Bonus / Drop step)           |
|  - Extended 80-Dimensional Observation Vector per Robot                                                 |
|  - Reward Shaping: Potential-Based Manhattan Proximity Delta + Mismatch Delivery Penalty                         |
+---------------------------------------------------------------------------------------------------------+
                                                     |
                                                     v
+---------------------------------------------------------------------------------------------------------+
|                                 MultiRobotVecEnv (Vectorized Wrapper)                                   |
|  - Coordinates step/reset for N agents                                                                  |
|  - Converts list rewards to structured numpy arrays; suppresses Gymnasium passive warnings              |
+---------------------------------------------------------------------------------------------------------+
                                                     |
                  +----------------------------------+----------------------------------+
                  |                                                                     |
                  v                                                                     v
+---------------------------------------------------+ +---------------------------------------------------+
|      FLAT BASELINE PIPELINE (Phases 1 - 7)        | |     HIERARCHICAL ZONE PIPELINE (Phase 8)          |
|                                                   | |                                                   |
| 1. DecentralizedAuctionLayer                      | | 1. ZonePartitioner                                |
|    - Eligibility Gating                           | |    - Spatial tiling & Centroid computation        |
|    - Neural Bid Head Evaluation                   | | 2. HierarchicalCoordinator                        |
|    - Global Belief & Claim Tables                 | |    - Ingests new tasks; routes to nearest zone    |
| 2. FaultLayer                                     | |    - Tracks bid timeouts -> Fallback handoffs     |
|    - Peer Heartbeat Matrix                        | | 3. ZoneAuctionLayer (1 per zone)                  |
|    - Detects dead robots in tau=15 steps           | |    - Localized bidding & range-limited claims     |
|    - Frees claims of failed robots                | | 4. ZoneFaultLayer (1 per zone)                    |
| 3. GridPathfinder                                 | |    - Intra-zone heartbeat tracking                |
|    - Deterministic A* on static shelves           | | 5. GridPathfinder                                 |
|    - Dynamic collision tie-breaking by Robot ID   | |    - Local path planning & priority execution     |
+---------------------------------------------------+ +---------------------------------------------------+
```

---

## 4. Mathematical & Algorithmic Foundations

### 4.1 Robot and Task Heterogeneity Model

Each robot $i \in \{0, \dots, N-1\}$ is assigned a capability class $T_i \in \{0, 1, 2\}$:
- **Class 0: `fast_light`**  
  - Battery drain rate: $\Delta B = 0.8$ per environment step.  
  - Velocity bonus probability: $P(\text{bonus}) = 0.5$ (50% chance of an extra step forward per tick).  
  - Velocity drop probability: $P(\text{drop}) = 0.0$.  
  - Acceptable tasks: $\{0, 2\}$ (`urgent_light`, `standard_delivery`).
- **Class 1: `heavy_load`**  
  - Battery drain rate: $\Delta B = 1.3$ per environment step.  
  - Velocity bonus probability: $P(\text{bonus}) = 0.0$.  
  - Velocity drop probability: $P(\text{drop}) = 0.3$ (30% chance of a motor lag / dropped move per tick).  
  - Acceptable tasks: $\{1, 2\}$ (`heavy_shelf`, `standard_delivery`). Exclusive carrier for heavy items!
- **Class 2: `balanced`**  
  - Battery drain rate: $\Delta B = 1.0$ per environment step.  
  - Velocity bonus probability: $P(\text{bonus}) = 0.0$.  
  - Velocity drop probability: $P(\text{drop}) = 0.0$.  
  - Acceptable tasks: $\{0, 2\}$ (`urgent_light`, `standard_delivery`).

#### Capability Compatibility Matrix $\mathcal{C}(T_{\text{robot}}, \tau_{\text{task}})$:
$$\mathcal{C}(T, \tau) = \begin{cases} 
1, & \text{if } T \in \text{ACCEPTABLE}[ \tau ] \\
0, & \text{otherwise}
\end{cases}$$

Where:
- $\text{ACCEPTABLE}[\tau = 0] = \{0, 2\}$ (`urgent_light` handled by `fast_light` or `balanced`)
- $\text{ACCEPTABLE}[\tau = 1] = \{1\}$ (`heavy_shelf` strictly handled by `heavy_load`)
- $\text{ACCEPTABLE}[\tau = 2] = \{0, 1, 2\}$ (`standard_delivery` handled by any robot)

### 4.2 Observation Space Formulation (80 Dimensions)
RWARE provides a base spatial observation vector per agent (71 dimensions for `rware-tiny`). The `HeterogeneousWarehouseWrapper` appends specialized heterogeneous state information to construct the exact 80-dimensional input vector:

$$\mathbf{o}_i = \big[ \mathbf{o}_{\text{rware}} \;(71) \;\Vert\; \mathbf{e}_{\text{robot}} \;(3) \;\Vert\; B_i/100 \;(1) \;\Vert\; \mathbf{s}_{\text{task}} \;(5) \big] \in \mathbb{R}^{80}$$

Where the sub-vectors are formulated exactly as implemented in `HeterogeneousWarehouseWrapper._augment_obs()`:
- $\mathbf{o}_{\text{rware}} \in \mathbb{R}^{71}$: Standard RWARE local agent observation (surrounding grid cells, nearby shelves, agent orientation).
- $\mathbf{e}_{\text{robot}} \in \mathbb{R}^3$: One-hot vector of the robot's mechanical capability class ($T_i \in \{0, 1, 2\}$).
- $B_i / 100 \in [0, 1]$: Continuous normalized battery level scalar (1 dimension).
- $\mathbf{s}_{\text{task}} \in \mathbb{R}^5$: Nearest requested shelf descriptor, consisting of:
  - $\mathbf{e}_{\text{task}} \in \mathbb{R}^3$: One-hot vector of task type (urgent_light, heavy_shelf, standard_delivery).
  - $w_{\text{norm}} \in [0, 1]$: Normalized task weight $\text{TASK\_WEIGHT}[\tau] / 3.0$ (1 dimension).
  - $d_{\text{norm}} \in [0, 1]$: Normalized Manhattan distance $\min(d_M(i, \text{shelf}) / 20.0, 1.0)$ (1 dimension).

### 4.3 Neural Bid Head Architecture & Supervised Training
Task suitability is predicted using `CapabilityConditionedPolicy`:

```
Input Vector (80-dim)
         |
    Linear(80 -> 128) -> ReLU
         |
    Linear(128 -> 64) -> ReLU
         |
   +-----+-------------------------+
   |                               |
Linear(64 -> 1)             Linear(64 -> 1)
   |                               |
Sigmoid                         Sigmoid
   v                               v
Bid Value [0, 1]           Comm Gate [0, 1]
(Task Suitability)         (Broadcast Probability)
```

#### Ground-Truth Training Label Formulation:
During offline experience collection (`collect_bid_data.py`), 400,000 observations (200,000 steps across 2 robots) are logged under uniform random action sampling. Each transition is assigned a continuous target suitability label $y \in [0, 1]$:

$$y = 0.6 \cdot \text{match} + 0.4 \cdot \text{batt\_ok}$$

Where:
- $\text{match} = 1.0$ if $T_i \in \text{ACCEPTABLE}[\tau]$, else $0.0$.
- $\text{batt\_ok} = 1.0$ if $B_i \ge (d_M(i, \text{shelf}) \cdot \Delta B_{T_i} \cdot 1.2)$, else $0.0$ (safety margin = 1.2).
- **Ground-Truth Delivery Outcome Override:** When an active shelf is successfully delivered, the labels for the preceding 5 timesteps (`BUFFER_LEN = 5`) for that robot are retroactively overwritten with the actual delivery outcome ($1.0$ if robot type was acceptable, $0.0$ if mismatch).

*Note:* The training label contains no distance penalty term (distance is handled by nearest-task selection and A* routing); ties are broken by robot ID.

The network is trained using Binary Cross-Entropy Loss:
$$\mathcal{L}_{\text{BCE}} = -\frac{1}{B} \sum_{k=1}^B \left[ y_k \log(\hat{y}_k) + (1 - y_k) \log(1 - \hat{y}_k) \right]$$

### 4.4 Task Auction Architecture (v2 Final with Retargeting)

The coordination layer implements an emulated range-limited peer auction (`auction_layer_v2.py` / `ZoneAuctionLayer`). Robots coordinate within communication range $R_{\text{comm}} = 8$:

```
Step 1: Eligibility Gating & Candidate Evaluation
   Each robot inspects the request queue and filters tasks by mechanical eligibility:
   Eligible(i) = { s in request_queue | C(T_i, task_type_s) == 1 }
   Robots sort eligible tasks by Manhattan distance.

Step 2: Neural Suitability Bidding
   Each robot evaluates the neural bid head on its 80-dim observation:
   b_i = BidHead(o_i) in [0, 1].

Step 3: Descending Bid Processing with Receiver-Side Filtering
   Robots are processed in descending order of bid value (highest bidder first).
   Peers actively filter out and ignore any bids or claims originating from robots
   currently marked as failed in their local heartbeat belief table (receiver-side filtering).

Step 4: Claim Assignment with Immediate Retargeting
   For robot i in descending bid order:
       Iterate through candidate eligible tasks s in distance order:
           - In flat v2 (auction_layer_v2.py): Check if task s was already claimed by an earlier
             higher-bidding robot within communication range (||pos_i - pos_j||_1 <= R_comm = 8).
           - In hierarchical zones (ZoneAuctionLayer.py): Exclude tasks claimed anywhere within the
             same zone (zone-wide exclusion set `selected_tasks`).
           If task s is NOT blocked by an earlier claim:
               Robot i claims task s.
               Mark task s as assigned; BREAK.
           Else:
               RETARGET: Continue to robot i's next-closest eligible task.
       If all candidate eligible tasks are claimed, robot i remains idle (NOOP) without blocking aisles.
```

**Key Architectural Significance of Retargeting:**  
In earlier auction iterations (`fixed` ablation), a robot whose preferred shelf was claimed by a peer became idle (`pursue = False`) for that entire step. Adding immediate sequential **retargeting** to the next-closest eligible task raised healthy 4-robot throughput from **453.3 to 1285.6 deliveries (2.8× increase)**, completely eliminating the idle-robot penalty.

### 4.5 Heartbeat Fault Detection & Stale-Claim Recovery
AMRs fail unexpectedly due to deadlocks, motor failures, or depleted batteries. The system detects and recovers from failures autonomously:

```
Algorithm: Heartbeat Monitoring & Task Reallocation
---------------------------------------------------
State per robot i:
   steps_since_heard[j] : int (ticks elapsed since last heartbeat received from robot j)
   belief_failed[j]     : bool (whether robot i believes robot j is dead)

At each timestep t:
   1. Active (non-failed) robots broadcast Heartbeat(i) to neighbors within range R_heartbeat.
   2. For each peer j:
      If Heartbeat(j) received:
          steps_since_heard[j] = 0
          belief_failed[j] = False
      Else:
          steps_since_heard[j] += 1
          If steps_since_heard[j] >= TIMEOUT_THRESHOLD (tau = 15):
              belief_failed[j] = True

   3. Reallocation Trigger:
      If belief_failed[j] == True:
          For each task s claimed by robot j in known_claims[i]:
              Delete known_claims[i][s]
              Reset belief_table[i][s] = 0.0
              -> Task s is instantly unlocked and returned to the open auction!
```

### 4.6 Spatial Zone Partitioning Mathematics
For a warehouse grid of dimensions $W \times H$ partitioned into $Z$ zones:
- Column partitions: $C = \lceil \sqrt{Z} \rceil$
- Row partitions: $R = \lceil Z / C \rceil$
- Zone dimensions: $w_z = W / C, \quad h_z = H / R$

Each cell $(x, y)$ belongs to zone $z(x, y)$:
$$c = \min\left(C - 1, \; \lfloor x / w_z \rfloor\right), \quad r = \min\left(R - 1, \; \lfloor y / h_z \rfloor\right)$$
$$z(x, y) = c \cdot R + r \quad \text{(Column-Major Indexing)}$$

Zone centroid $\mathbf{\mu}_z$:
$$\mathbf{\mu}_z = \left( \left(c + 0.5\right) w_z, \; \left(r + 0.5\right) h_z \right)$$

When a new task arrives at $(x_t, y_t)$, it is routed to zone $z^*$:
$$z^* = \arg\min_{z \in \{0, \dots, Z-1\}} \left( |x_t - \mu_{z, x}| + |y_t - \mu_{z, y}| \right)$$

---

## 5. Complete Codebase Architecture & File Walkthrough

The repository is organized into distinct modular directories:

```
c:\Users\Kezia\warehouse-swarm\
│
├── core\                             # Core Coordination & Engine Modules
│   ├── __init__.py                   # sys.path initialization and package exports
│   ├── hetero_wrapper.py             # Heterogeneous Gym environment wrapper
│   ├── multi_robot_vecenv.py         # Vectorized multi-agent environment runner
│   ├── policy_network.py             # PyTorch CapabilityConditionedPolicy neural model
│   ├── pathfinding.py                # Deterministic A* grid pathfinder & collision handler
│   ├── auction_layer.py              # Flat decentralized auction and belief engine
│   ├── fault_layer.py                # Flat heartbeat-based failure monitoring engine
│   ├── zone_partitioner.py           # Spatial grid partitioner and centroid calculator
│   ├── zone_auction_layer.py         # Zone-scoped auction engine with task/comm filters
│   ├── zone_fault_layer.py           # Zone-scoped heartbeat liveness monitoring
│   ├── hierarchical_coordinator.py   # Top-level coordinator: zone routing & handoff fallback
│   └── bid_head_trained.pt           # Trained PyTorch neural bid model weights
│
├── experiments\                      # Empirical Test & Benchmark Scripts
│   ├── collect_bid_data.py           # Generates 200k steps of labeled experience data
│   ├── train_bid_head.py             # Supervised training loop for CapabilityConditionedPolicy
│   ├── greedy_baseline.py            # Naive nearest-robot allocation benchmark
│   ├── auction_layer_nocomm.py       # Zero-communication ablation auction layer
│   ├── phase6_degradation_experiment.py # Fault degradation curve benchmark (0, 1, 2 failures)
│   ├── phase7_nocomm_experiment.py   # Ablation study isolating communication value
│   ├── phase7_scalability_experiment.py # Fleet scalability test (2, 4, 8 robots)
│   └── phase8_zone_experiment.py     # Hierarchical zone vs flat scalability experiment
│
├── models_and_data\                  # Saved Artifacts & Pretrained Models
│   ├── bid_head_trained.pt           # Canonical trained weights for neural bid head
│   ├── bid_training_data.npz         # 400,000-sample experience replay dataset (200k steps x 2 robots, 129 MB)
│   ├── ppo_warehouse_1M.zip          # 1,000,000-step PPO baseline policy model
│   └── ppo_warehouse_4robot_500k_*.zip# 500,000-step 4-robot PPO checkpoint
│
├── visualization\                    # Interactive User Interfaces
│   └── run_simulation_gui.py         # Live animated Tkinter GUI visualizer
│
├── docs\                             # Project Documentation
│   └── Project Status Summary.docx   # Historical research milestone document
│
└── PROJECT_MASTER_DOCUMENTATION.md   # Complete unified project documentation (This File)
```

---

### 5.1 Deep File-by-File Breakdown

#### 1. `core/__init__.py`
- **Purpose:** Ensures both the `core/` package and the project root directory are prepended to `sys.path`.
- **Key Logic:** Dynamically resolves `Path(__file__).resolve().parent` and prevents duplicate path insertions, ensuring modular cross-imports run cleanly across any script.

#### 2. `core/hetero_wrapper.py` (`HeterogeneousWarehouseWrapper`)
- **Purpose:** The environment wrapper that extends standard RWARE into a heterogeneous multi-robot warehouse.
- **Key Variables & Dictionaries:**
  - `TYPE_NAMES = ["fast_light", "heavy_load", "balanced"]`
  - `BATTERY_DRAIN = {0: 0.8, 1: 1.3, 2: 1.0}`
  - `BONUS_STEP_PROB = {0: 0.5, 1: 0.0, 2: 0.0}`: Models high-speed AMRs.
  - `DROP_STEP_PROB = {0: 0.0, 1: 0.3, 2: 0.0}`: Models motor lag under heavy loads.
  - `ACCEPTABLE_ROBOTS = {0: {0, 2}, 1: {1}, 2: {0, 1, 2}}`: Defines strict capability requirements.
  - `MISMATCH_PENALTY = 0.5`: Penalty subtracted from step reward if a delivery violates capability constraints.
- **Methods:**
  - `step(actions)`: Intercepts actions; applies probabilistic velocity slips; updates battery levels; checks RWARE delivery events; verifies capability matching; computes potential-based distance shaping rewards ($0.05 \cdot \Delta d_{\text{Manhattan}}$).
  - `_augment_obs(obs_tuple)`: Constructs the 80-dimensional observation tensor per robot by appending one-hot capability embedding (3), normalized battery (1), and nearest task descriptor (5) to base RWARE observations (71).

#### 3. `core/multi_robot_vecenv.py` (`MultiRobotVecEnv`)
- **Purpose:** Vectorized environment runner compatible with multi-agent stepping.
- **Key Logic:**
  - Instantiates `HeterogeneousWarehouseWrapper(gym.make(env_id, disable_env_checker=True))`.
  - Disables Gymnasium's passive environment checker to prevent warning spam caused by list-valued multi-agent rewards.
  - Converts reward lists to `np.float32` arrays in `step_wait()`.

#### 4. `core/policy_network.py` (`CapabilityConditionedPolicy`)
- **Purpose:** Neural network architecture implementing the shared bid and communication heads.
- **Architecture:**
  - Shared Encoder: 80 inputs $\to$ Linear(128) $\to$ ReLU $\to$ Linear(64) $\to$ ReLU.
  - `bid_head`: Linear(64 $\to$ 1) + Sigmoid $\to$ Output suitability score in $[0, 1]$.
  - `comm_head`: Linear(64 $\to$ 1) + Sigmoid $\to$ Output broadcast probability in $[0, 1]$.

#### 5. `core/pathfinding.py` (`GridPathfinder`)
- **Purpose:** Deterministic decentralized local pathfinder and collision resolver.
- **Key Logic:**
  - `astar(start, goal, blocked)`: Computes shortest obstacle-free path across the warehouse grid.
  - `_turn_action(current_dir, desired_dir)`: Maps 4 compass orientations (`UP=0`, `DOWN=1`, `LEFT=2`, `RIGHT=3`) to minimal rotational commands (`LEFT_TURN=2`, `RIGHT_TURN=3`, `FORWARD=1`).
  - `get_actions_for_pursuit(...)`: Translates assigned tasks into concrete RWARE actions. If an agent arrives at a shelf or drop-off zone, automatically triggers `TOGGLE_LOAD=4` to lift or drop the pod.
  - **Decentralized Collision Avoidance:** If two robots attempt to step into the same grid cell on the next tick, the robot with the lower numerical index (`robot_i < robot_j`) is granted right-of-way; the contesting robot executes `NOOP=0`.

#### 6. `core/auction_layer.py` (`DecentralizedAuctionLayer`)
- **Purpose:** Flat decentralized peer-to-peer auction coordinator.
- **Key Logic:**
  - `compute_bids(obs_batch)`: Evaluates `bid_head_trained.pt` across all agents without gradients.
  - `_nearest_eligible_task(agent, robot_type)`: Scans the request queue and enforces strict capability eligibility filtering.
  - `step(obs_batch, belief_failed)`: Executes peer-to-peer claim broadcasts within range $R=8$. Resolves conflicting bids by taking the maximum bid value. Automatically clears stale claims belonging to robots flagged as failed.

#### 7. `core/fault_layer.py` (`FaultLayer`)
- **Purpose:** Simulates hardware failures and manages heartbeat-based liveness tables.
- **Key Logic:**
  - `inject_failure(robot_id)`: Marks a robot as physically incapacitated.
  - `step()`: Updates heartbeat timeout counters `steps_since_heard`. Flags a robot as failed if no heartbeat is received within `HEARTBEAT_TIMEOUT = 15` ticks.
  - `get_actions_override(actions)`: Forces `NOOP=0` on any physically incapacitated robot, preventing further movement.

#### 8. `core/zone_partitioner.py` (`ZonePartitioner`)
- **Purpose:** Performs 2D spatial grid partitioning into $Z$ rectangular zones.
- **Key Logic:**
  - Computes column-major zone grids ($C \times R$) and bounding boxes.
  - Precomputes spatial centroids $\mathbf{\mu}_z$ for each zone.
  - `get_zone(x, y)`: Determines the zone index for any coordinate in $O(1)$ time.
  - `assign_robots_to_zones(agents)`: Maps each robot to a primary home zone based on initial spawn locations.
  - `get_nearest_zones_for_task(x, y)`: Returns a ranked list of zones ordered by centroid distance (used for fallback routing).

#### 9. `core/zone_auction_layer.py` (`ZoneAuctionLayer`)
- **Purpose:** Specializes `DecentralizedAuctionLayer` to operate within a specific spatial zone.
- **Key Logic:**
  - **Task Scoping:** `_nearest_eligible_task` ignores tasks located outside the managed zone.
  - **Communication Scoping:** Message broadcast loops filter out peer robots belonging to external zones, constraining communication complexity.
  - Tracks detailed communication metrics: `comm_events_per_step`.

#### 10. `core/zone_fault_layer.py` (`ZoneFaultLayer`)
- **Purpose:** Extends `FaultLayer` to bound heartbeat monitoring to intra-zone neighbors.
- **Key Logic:** Heartbeat verification only checks peers assigned to the same zone, eliminating long-distance wireless chatter across the warehouse.

#### 11. `core/hierarchical_coordinator.py` (`HierarchicalCoordinator`)
- **Purpose:** Top-level coordinator unifying zone partitioners, zone auctions, and fault layers.
- **Key Logic:**
  - Manages an array of `ZoneAuctionLayer` and `ZoneFaultLayer` instances (one per zone).
  - Routes newly requested shelves from the RWARE queue to the geographically nearest zone.
  - **Fallback Task Handoff:** If a task in Zone $A$ cannot be serviced because no live capable robot exists in that zone (or after `BID_TIMEOUT = 10` unserviced steps), the coordinator hands off the task to the next-closest neighboring zone.
  - `get_fault_action_overrides(actions)`: Single authoritative point overriding actions of failed robots to `NOOP`.

#### 12. `visualization/run_simulation_gui.py` (`WarehouseGUI`)
- **Purpose:** Interactive live graphical interface built in Tkinter.
- **Key Features:**
  - Renders 4 color-coded zonal quadrants with dynamic borders.
  - Displays grid cells, stationary inventory shelves, and active requested task pods.
  - Renders AMRs color-coded by class: Blue (`fast_light`), Red (`heavy_load`), Green (`balanced`), Dark Grey (Failed).
  - Indicates carrying state (golden ring around robot when lifting a shelf).
  - Interactive sidebar: Displays real-time step counter, total deliveries, capability match rate (~99%), and handoff counts.
  - **Interactive Fault Injection Button:** Allows the user to select any robot from a dropdown and click "Fail Robot" in real time to observe live heartbeat failure detection and task recovery!

---


## 6. Comprehensive Experimental Results & Empirical Benchmarks

All experiments were executed with rigorous statistical validation across **10 random seeds** (`1, 42, 100, 2024, 7, 11, 23, 99, 314, 777`) for **20,000 steps per run** (70–90 seconds per run on CPU, totaling 200,000 steps per data point). The environment reset forwards the seed deterministically; bitwise determinism checks confirmed identical runs across identical seeds.

```
====================================================================================================
               MASTER EXPERIMENTAL RESULTS OVERVIEW (Exp 8 Seeded Suite, 10 Seeds)
====================================================================================================
Benchmark / Study        Conditions Evaluated              Key Finding / Result
----------------------------------------------------------------------------------------------------
Phase 3 Neural Training  Bid Head (400k samples, 60 ep)   val_bce: 0.3815 | val_mae: 0.0019
                         Adam lr=3e-3, StepLR decay        bce floor: 0.3831 (near-theoretical floor)
----------------------------------------------------------------------------------------------------
Healthy Baseline         4 Robots (11x10 floor)           v2: 1285.6 ± 37.5 deliveries (99.0% match)
Comparison (Table 6.3)   Ablation: fixed (no retargeting) 453.3 ± 44.8 deliveries (Retargeting = 2.8x!)
                         Ablation: nocomm                 1288.0 ± 33.5 deliveries
                         Baseline: greedy (eligible)      1272.0 ± 55.6 deliveries
----------------------------------------------------------------------------------------------------
Phase 6 Fault Tolerance  Healthy Baseline (v2 system)     1285.6 ± 37.5 deliveries | 100% retention
(4 robots, 11x10 floor,  Robot 0 (fast_light) fails        1105.5 ± 28.5 (86.1% ± 3.4% retention)
 seed-matched retention) Robot 1 (heavy_load) fails        356.4 ± 21.7  (27.8% ± 2.0% retention)
                         Robots 1+2 fail                   339.7 ± 19.4  (26.4% ± 1.3% retention)
----------------------------------------------------------------------------------------------------
Phase 7 Ablation Study   Fast-robot failure:
(seed-matched vs nocomm)   v2 Comm: 86.1% ± 3.4% retention vs No-Comm: 77.3% ± 4.2% (Greedy: 55.4%)
                           v2 minus No-Comm advantage: +111.9 ± 52.6 deliveries (95% CI excludes 0)
                         Heavy-robot failure: all variants retain ~25-28% (capability coverage bottleneck)
                         Two-robot failure: v2 (339.7) delivers 84% more than greedy (184.6)
----------------------------------------------------------------------------------------------------
Phase 7 Fleet Scaling    2 Robots  (11x10 grid)           828.8 ± 16.1 deliveries (414.4 / robot)
(Healthy v2 system)      4 Robots  (11x10 grid)           1285.6 ± 37.5 (1.55x speed-up, 78% efficiency)
                         8 Robots  (11x10 grid)           2672.6 ± 29.5 (3.22x speed-up, 81% efficiency)
                         16 Robots (20x10 small, flat)    2403.2 ± 12.4 deliveries
                         16 Robots (20x16 med, flat)      2634.7 ± 12.8 deliveries
----------------------------------------------------------------------------------------------------
Phase 8 Zone Scaling     Flat Baseline (16 robots, 20x16) 2634.7 ± 12.8 deliveries (no zone boundaries)
(Hierarchical Multi-Zone)Hierarchical Medium (4 zones)    2333.1 ± 121.6 deliveries | comm: 0.543 ev/step
                         Hierarchical Medium (8 zones)    2334.7 ± 85.0 deliveries  | comm: 0.230 ev/step
                         Handoff rate rises 1.8% -> 35.5% as zones increase from 2 to 8
====================================================================================================
```

---

### 6.1 Phase 3: Neural Bid Head Training Results (Table 6.2, Figure 4.1)

- **Dataset:** 400,000 state-action pairs collected via `collect_bid_data.py` (200,000 steps across 2 robots: `fast_light` and `heavy_load` with uniform random actions, seed 123). Split: 85% train (340,000) / 15% validation (60,000).
- **Optimization:** Adam optimizer ($\eta = 3\times 10^{-3}$, StepLR decay $\gamma=0.3$ every 20 epochs), batch size 256, 60 epochs. Standard defaults, not tuned by hyperparameter search.
- **Reference Benchmarks:**
  - Binary Cross-Entropy (BCE) label entropy floor: $0.3831$
  - Mean Absolute Error (MAE) of predicting the sample mean: $0.2780$

| Epoch | Train BCE | Validation BCE | Validation MAE | Learning Rate |
| :---: | :---: | :---: | :---: | :---: |
| 1 | 0.4074 | 0.3916 | 0.0212 | 0.00300 |
| 10 | 0.3831 | 0.3848 | 0.0059 | 0.00300 |
| 20 | 0.3823 | 0.3843 | 0.0036 | 0.00300 |
| 21 | 0.3818 | 0.3840 | 0.0026 | 0.00090 |
| 40 | 0.3817 | 0.3840 | 0.0032 | 0.00090 |
| 41 | 0.3814 | 0.3839 | 0.0014 | 0.00027 |
| 60 | **0.3815** | **0.3839** | **0.0019** | 0.00027 |

- **Training Insights:**
  - Validation BCE converges within $0.0008$ of the theoretical soft-label entropy floor ($0.3831$).
  - Validation MAE reaches $0.0019$, which is **99.3% lower** than the predict-the-mean baseline ($0.2780$), with the majority of the gain achieved within the first 10 epochs.
  - *Honest Framing:* The bid head functions as a differentiable suitability scorer distilling capability and battery rules. It is trained on random-policy data from 2 robot classes (no `balanced` robot was present in training data).

---

### 6.2 Healthy System Comparison & Retargeting Ablation (Table 6.3)

Evaluated on 4 robots on the $11 \times 10$ floor over 10 random seeds (20,000 steps each):

| System Architecture | Deliveries (mean ± 95% CI) | Match Rate | Notes |
| :--- | :---: | :---: | :--- |
| **v2 (Final System)** | **1285.6 ± 37.5** | **99.0%** | Full system with retargeting and receiver-side filtering |
| **No-Communication (`nocomm`)** | **1288.0 ± 33.5** | 99.1% | Range = 0; robots act on local suitability only |
| **Greedy (Capability-Eligible)** | **1272.0 ± 55.6** | 99.2% | Nearest eligible robot takes nearest task; no bidding |
| **Fixed (No Retargeting Ablation)** | **453.3 ± 44.8** | 99.5% | Original auction: robots losing a bid idle for the step |

**Key Findings:**
1. **Auction Overheads are Negligible:** v2 healthy throughput ($1285.6$) matches `nocomm` ($1288.0$) and greedy ($1272.0$) within 95% confidence intervals. The coordination mechanism adds zero healthy-throughput penalty.
2. **Retargeting is Decisive:** Adding retargeting to the auction raises throughput from **$453.3$ to $1285.6$ deliveries (2.8× increase)**. When losing bidders immediately target their next-closest task rather than idling, fleet utilization reaches baseline maximum.

---

### 6.3 Phase 6: Fault Tolerance & Seed-Matched Degradation (Table 6.4, Figures 4.2–4.4)

Failures injected at step 500 on 4 robots ($11 \times 10$ grid). Retention is calculated **seed-matched** (deliveries under failure divided by the same system's healthy deliveries for the same seed):

| System | Injected Failure Condition | Deliveries (mean ± 95% CI) | Match Rate | Seed-Matched Retention |
| :--- | :--- | :---: | :---: | :---: |
| **greedy (eligible)** | None (Healthy) | 1272.0 ± 55.6 | 99.2% | 100.0% |
| greedy | Robot 0 (`fast_light`) | 703.1 ± 15.7 | 99.4% | 55.4% ± 2.4% |
| greedy | Robot 1 (`heavy_load`) | 313.9 ± 21.1 | 99.3% | 24.7% ± 1.6% |
| greedy | Robots 1 + 2 (`heavy` + `balanced`) | 184.6 ± 16.1 | 99.5% | 14.6% ± 1.4% |
| **no-comm** | None (Healthy) | 1288.0 ± 33.5 | 99.1% | 100.0% |
| no-comm | Robot 0 (`fast_light`) | 993.6 ± 39.0 | 99.3% | 77.3% ± 4.2% |
| no-comm | Robot 1 (`heavy_load`) | 353.7 ± 16.3 | 99.5% | 27.5% ± 1.4% |
| no-comm | Robots 1 + 2 (`heavy` + `balanced`) | 343.7 ± 20.9 | 99.6% | 26.7% ± 1.9% |
| **v2 (final)** | None (Healthy) | 1285.6 ± 37.5 | 99.0% | 100.0% |
| v2 (final) | Robot 0 (`fast_light`) | **1105.5 ± 28.5** | **99.4%** | **86.1% ± 3.4%** |
| v2 (final) | Robot 1 (`heavy_load`) | 356.4 ± 21.7 | 99.3% | 27.8% ± 2.0% |
| v2 (final) | Robots 1 + 2 (`heavy` + `balanced`) | **339.7 ± 19.4** | **99.7%** | **26.4% ± 1.3%** |
| *fixed (ablation)* | None (Healthy) | 453.3 ± 44.8 | 99.5% | 100.0% |
| *fixed (ablation)* | Robot 0 (`fast_light`) | 523.8 ± 53.7 | 99.2% | 118.4% ± 19.1%* |
| *fixed (ablation)* | Robot 1 (`heavy_load`) | 184.4 ± 16.8 | 99.7% | 41.6% ± 6.7% |
| *fixed (ablation)* | Robots 1 + 2 | 198.6 ± 24.5 | 99.6% | 44.3% ± 5.7% |

*\*Note on Fixed Ablation: Its apparent >100% retention under Robot 0 failure is an artifact of its severely depressed healthy baseline ($453.3$ deliveries due to idle losing robots). It must not be cited as authentic retention.*

#### Seed-Matched Differences (v2 minus No-Communication):
- **Healthy (0 failures):** $-2.4 \pm 50.8$ deliveries (statistically identical)
- **Robot 0 (`fast_light`) fails:** **$+111.9 \pm 52.6$ deliveries** (95% CI excludes zero; measurable communication advantage)
- **Robot 1 (`heavy_load`) fails:** $+2.7 \pm 29.9$ deliveries (no significant communication difference)
- **Robots 1 + 2 fail:** $-4.0 \pm 29.7$ deliveries (no significant communication difference)

#### Analytical Conclusions:
1. **Communication Value is Context-Specific:** Peer-to-peer communication produces a measurable, statistically significant throughput advantage **specifically when a fast robot fails** ($+111.9 \pm 52.6$ deliveries, 95% CI excludes zero; $86.1\%$ retention vs $77.3\%$ for no-comm and $55.4\%$ for greedy). It should *not* be claimed that communication improves throughput across all failure modes.
2. **Capability Coverage Dominates Heavy Failures:** When Robot 1 (`heavy_load`) fails, all systems collapse to $\sim 25\text{--}28\%$ retention (greedy $24.7\%$, no-comm $27.5\%$, v2 $27.8\%$). Robot 1 is the sole carrier capable of transporting the $\sim 41\%$ of tasks requiring heavy lifting. Coordination protocols cannot substitute for absent physical hardware capabilities.
3. **Multi-Failure Resilience Over Greedy:** Under 50% fleet loss (Robots 1+2), v2 matches no-comm ($339.7$ vs $343.7$) and delivers **84% more throughput than greedy** ($339.7$ vs $184.6$ deliveries).

---

### 6.4 Fleet Scalability & Multi-Zone Coordination (Tables 6.5 & 6.6, Figure 4.5)

#### Healthy Swarm Fleet Scaling (Table 6.5)

| Swarm Size (Floor) | greedy | no-comm | v2 (final) | v2 Speed-up vs 2-Robot | Parallel Efficiency |
| :---: | :---: | :---: | :---: | :---: | :---: |
| **2 Robots** ($11 \times 10$) | 820.9 ± 15.6 | 824.1 ± 12.2 | **828.8 ± 16.1** | 1.00× (baseline) | 100.0% |
| **4 Robots** ($11 \times 10$) | 1272.0 ± 55.6 | 1288.0 ± 33.5 | **1285.6 ± 37.5** | **1.55×** | 77.6% |
| **8 Robots** ($11 \times 10$) | 2555.1 ± 54.0 | 2561.7 ± 17.5 | **2672.6 ± 29.5** | **3.22×** | 80.6% |

- **Scaling Dynamics:** From 4 to 8 robots, throughput doubles (+108%), sustaining $\sim 81\%$ parallel efficiency. At 8 robots, v2 achieves $2672.6$ deliveries, outperforming greedy ($2555.1$) and no-comm ($2561.7$) with non-overlapping confidence intervals.
- **Per-Robot Deliveries:** 414.4 (2 robots) $\to$ 321.4 (4 robots) $\to$ 334.1 (8 robots).
- **Match Rates:** Maintained at $\ge 97.1\%$ across all fleet sizes.

#### 16-Robot Large Fleet Controls (Table 6.6)

| Floor Layout (16 Robots) | greedy | no-comm | v2 (flat control) |
| :--- | :---: | :---: | :---: |
| **Small Floor ($20 \times 10$)** | 2412.1 ± 47.7 (98.4%) | 2384.4 ± 13.9 (98.2%) | **2403.2 ± 12.4** (98.2%) |
| **Medium Floor ($20 \times 16$)** | 2570.3 ± 44.2 (99.1%) | 2510.1 ± 21.3 (99.2%) | **2634.7 ± 12.8** (99.0%) |

On the small 16-robot grid, all three systems overlap due to corridor congestion. On the medium floor, flat v2 delivers $2634.7$, which is $2.5\%$ above greedy and $5.0\%$ above no-comm.

#### Multi-Zone Hierarchical Coordination Trends (Table 6.7)

> **Status Note:** The multi-zone hierarchical configurations below were recorded over 10 independent evaluation runs. A formal seeded re-run matching Exp 8 flat controls is designated for final verification.

| Configuration | Deliveries | Match Rate | Comm Events / Robot / Step | Cross-Zone Handoff Rate |
| :--- | :---: | :---: | :---: | :---: |
| **Hierarchical Small ($20 \times 10$, 4 zones)** | 2099.0 ± 200.8 | 98.7% | 0.656 | 14.1% |
| **Hierarchical Medium ($20 \times 16$, 2 zones)** | 2455.9 ± 68.8 | 99.2% | 1.249 | 1.8% |
| **Hierarchical Medium ($20 \times 16$, 4 zones)** | 2333.1 ± 121.6 | 99.2% | 0.543 | 6.9% |
| **Hierarchical Medium ($20 \times 16$, 8 zones)** | 2334.7 ± 85.0 | 99.4% | 0.230 | 35.5% |
| **Hierarchical Medium (4 zones, zone failure)** | 1968.7 ± 221.4 | 99.4% | 0.424 | 28.1% |

**Scientific Interpretation of Zone Coordination:**
- **Communication Containment:** As zone count increases from 2 to 8, communication events per robot drop from **$1.249 \to 0.543 \to 0.230$**, providing bounded wireless traffic.
- **Handoff Trade-off:** More granular zones increase cross-zone handoffs ($1.8\% \to 6.9\% \to 35.5\%$) as tasks near boundaries require neighboring zone assistance.
- **Throughput:** In these tests, zone partitioning maintains stable deliveries ($2333\text{--}2456$) but does not increase raw throughput over the flat retargeting v2 control ($2634.7$). The primary objective of zoning is bounded wireless communication overhead and local fault containment.

---

### 6.5 Capability Matching Compliance (Summary)

- **With Eligibility Gate:** All tested systems (greedy-eligible, no-comm, v2) achieve **$97.1\%\text{--}99.8\%$** capability match compliance across all configurations.
- **Without Eligibility Gate (Naive Greedy):** In 10 independent baseline runs, naive greedy without an eligibility gate achieved only **$67.0\%$ match compliance** ($1835.9$ total deliveries, but only $1230.1$ valid deliveries; $\sim 33\%$ mismatch penalty rate).
- **Residual Mismatches ($0.2\text{--}2.9\%$):** Detailed traces revealed that residual mismatches occur when unassigned robots carrying shelves over drop-off goal cells trigger incidental RWARE deliveries outside active pursuit intent.

### 6.6 Phase 9 & 10: Communication Fault Robustness & Stress Analysis (440 Seeded Runs)

In real-world industrial fulfillment centers, wireless environments suffer from radio frequency attenuation, channel fading from metallic shelving racks, and access point queuing latency. To evaluate system resilience and discover true empirical breaking points, LossyFaultLayer and AuctionLayerV2 were subjected to 440 full seeded evaluations (,000$ steps each across 10 matched seeds).

#### 1. Exp 9 Initial Sweeps: Moderate Loss, Delay, and Timeout Sensitivity

##### Sweep 1: Heartbeat Message Loss ({\\text{loss}} \\in [0.0, 0.20]$, $\\text{delay}=0$, $\\tau=15$)

| Message Loss | Deliveries (Healthy) | Deliveries (Robot 0 Fails) | Match Rate | Detection Delay (steps) | False Positive Steps | Seed-Matched Retention |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **0% (Baseline)** | 1285.6 ± 37.5 | 1105.5 ± 28.5 | 99.4% | 15.0 ± 0.0 | 0.0 ± 0.0 | **86.1% ± 3.4%** |
| **5% Loss**       | 1285.6 ± 37.5 | 1105.5 ± 28.5 | 99.4% | 14.8 ± 0.3 | 0.0 ± 0.0 | **86.1% ± 3.4%** |
| **10% Loss**      | 1285.6 ± 37.5 | 1103.6 ± 28.3 | 99.4% | 14.4 ± 0.8 | 0.0 ± 0.0 | **86.0% ± 3.3%** |
| **20% Loss**      | 1285.6 ± 37.5 | 1103.6 ± 28.3 | 99.4% | 14.0 ± 0.8 | 0.0 ± 0.0 | **86.0% ± 3.3%** |

*Finding:* Because $\\tau=15$, the probability of 15 consecutive dropped packets on a link is .20^{15} \\approx 3.28 \\times 10^{-11}$, resulting in **0 false positive steps** and perfectly stable throughput.

##### Sweep 2: Heartbeat Transmission Delay ( \\in [0, 5]$ steps, {\\text{loss}}=0$, $\\tau=15$)

| Transmission Delay | Deliveries (Healthy) | Deliveries (Robot 0 Fails) | Match Rate | Detection Delay (steps) | False Positive Steps | Seed-Matched Retention |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **0 steps (Baseline)** | 1285.6 ± 37.5 | 1105.5 ± 28.5 | 99.4% | 15.0 ± 0.0 | 0.0 ± 0.0 | **86.1% ± 3.4%** |
| **1 step delay**       | 1285.6 ± 37.5 | 1103.4 ± 28.0 | 99.4% | 16.0 ± 0.0 | 0.0 ± 0.0 | **86.0% ± 3.4%** |
| **3 steps delay**      | 1285.6 ± 37.5 | 1091.3 ± 38.5 | 99.4% | 18.0 ± 0.0 | 0.0 ± 0.0 | **85.0% ± 3.7%** |
| **5 steps delay**      | 1285.6 ± 37.5 | 1103.5 ± 28.2 | 99.4% | 20.0 ± 0.0 | 0.0 ± 0.0 | **86.0% ± 3.3%** |

*Finding:* Network propagation latency cleanly shifts failure detection by exactly $ steps ($\\text{detection delay} = \\tau + d = 15 + d$). Retention remains unaffected at **.0\\% \\pm 3.3\\%$**, demonstrating jitter tolerance.

##### Sweep 3: Timeout Threshold Sensitivity ($\\tau \\in [5, 30]$ steps, {\\text{loss}}=0$, =0$)

| Timeout Threshold $\\tau$ | Deliveries (Healthy) | Deliveries (Robot 0 Fails) | Match Rate | Detection Delay (steps) | False Positive Steps | Seed-Matched Retention |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **$\\tau = 5$ steps**   | 1285.6 ± 37.5 | 1103.2 ± 27.4 | 99.4% | 5.0 ± 0.0 | 0.0 ± 0.0 | **85.9% ± 3.4%** |
| **$\\tau = 10$ steps**  | 1285.6 ± 37.5 | 1098.4 ± 30.5 | 99.4% | 10.0 ± 0.0 | 0.0 ± 0.0 | **85.6% ± 3.7%** |
| **$\\tau = 15$ steps (Default)** | 1285.6 ± 37.5 | 1105.5 ± 28.5 | 99.4% | 15.0 ± 0.0 | 0.0 ± 0.0 | **86.1% ± 3.4%** |
| **$\\tau = 30$ steps**  | 1285.6 ± 37.5 | 1086.9 ± 30.5 | 99.5% | 30.0 ± 0.0 | 0.0 ± 0.0 | **84.7% ± 3.7%** |

---

#### 2. Exp 10 Stress Benchmarks: Detector Boundaries and Coordination Persistence (220 Runs)

To identify where the heartbeat detector generates false alarms and whether the coordination advantage over no-comm persists, Exp 10 runs 220 seeded evaluations under severe conditions. All throughput gaps are computed as paired per-seed differences with 95% Student-t CIs (df=9, t=2.262). No setting produced a statistically significant throughput drop (CI half-width is ~3-5 points with 10 seeds, so drops smaller than that cannot be detected with this sample size).

> **What "breaking point" means here:** The thresholds below are **detector breaking points** (where false alarms begin), not throughput breaking points. Throughput never collapses across any tested setting.

##### Stress Grid: Severe Packet Loss under Aggressive vs. Default Timeouts (160 Runs)

**False alarm counts and theoretical prediction** (predicted = $p^{(\tau+1)} \times 240000$ link-steps):

| Setting | Predicted FP Steps | Measured FP Steps | Detector Status |
| :--- | :---: | :---: | :---: |
| $\tau=15$, Loss 40% | 0 | 0.0 | Clean |
| $\tau=15$, Loss 60% | ~68 | 72.3 | Small alarm onset |
| $\tau=15$, Loss 80% | ~6,800 | 6,766.8 | Active false alarms |
| $\tau=15$, Loss 90% | ~44,500 | 44,775.8 | Heavy false alarms |
| $\tau=5$, Loss 40% | ~983 | 998.4 | Active false alarms |
| $\tau=5$, Loss 90% | ~127,500 | 127,719.2 | Very heavy false alarms |

*The measured counts match the geometric model, confirming `LossyFaultLayer` behaves correctly. This also gives a design rule: choose $\tau$ so that $p^{(\tau+1)} \times N_\text{pairs} \times T$ is below your tolerated false alarm budget.*

**Throughput and coordination advantage under stress** (all settings paired against no-comm, 993.6 ± 39.0 deliveries):

| Setting | v2 Deliveries (Robot 0 Fails) | Paired Advantage over No-comm (95% CI) | Advantage Confirmed (P<0.05)? |
| :--- | :---: | :---: | :---: |
| Baseline ($\tau=15$, Loss 0%) | 1105.5 ± 28.5 | +111.9 ± 52.6 [59.3, 164.5] | YES |
| $\tau=5$, Loss 40% | 1086.2 ± 35.0 | +92.6 ± 45.6 [47.0, 138.2] | YES |
| $\tau=5$, Loss 60% | 1094.4 ± 26.1 | +100.8 ± 55.0 [45.8, 155.8] | YES |
| $\tau=5$, Loss 80% | 1091.1 ± 38.0 | +97.5 ± 65.4 [32.1, 162.9] | YES |
| $\tau=5$, Loss 90% | 1110.3 ± 41.5 | +116.7 ± 58.8 [57.9, 175.5] | YES |
| $\tau=15$, Loss 40% | 1103.6 ± 28.3 | +110.0 ± 51.1 [58.9, 161.1] | YES |
| $\tau=15$, Loss 60% | 1101.1 ± 30.6 | +107.5 ± 50.4 [57.1, 157.9] | YES |
| $\tau=15$, Loss 80% | 1105.1 ± 41.5 | +111.5 ± 50.9 [60.6, 162.4] | YES |
| $\tau=15$, Loss 90% | 1084.1 ± 30.0 | +90.5 ± 48.0 [42.5, 138.5] | YES |

*Key finding (all 8 stress-grid settings): The v2 coordination advantage over no-comm (claim release on failure detection) is preserved at P<0.05 across all tested loss rates and timeout settings, including 90% loss with 44,776 false alarm steps. The advantage narrows slightly at 90% loss (tau=15: +90.5 vs baseline +111.9) but the CI excludes zero at every setting. This is a performance result, not an overclaim: the heartbeat mechanism continues to release true failure claims even under heavy false-alarm churn. Note that with 10 seeds the CI is ~3-5 points wide, so drops smaller than that are undetectable with this sample size.*

> **Why throughput does not collapse under heavy false alarms:** When v2 misidentifies a healthy robot as failed, it releases that robot's current claim. The healthy robot immediately retargets to any available eligible task. Because the fleet continues to be fully operational, throughput continues. The false alarms add wasted retargeting steps, not stalls.

##### Radio Outage Benchmark: Temporary RF Blackouts (30 Runs)

| Setting | Healthy Deliveries | False Positive Steps | Recovery Behavior |
| :--- | :---: | :---: | :--- |
| Mute 10 steps ($< \tau=15$) | 1285.6 ± 37.5 | 0.0 | **Fully masked**; zero false alarms. |
| Mute 20 steps ($> \tau=15$) | 1278.7 ± 38.5 | 15.0 | $(20-15)\times 3 = 15$ observer-steps of false alarm; self-clears on signal restoration. |
| Mute 40 steps ($> \tau=15$) | 1288.8 ± 37.9 | 75.0 | $(40-15)\times 3 = 75$ observer-steps of false alarm; self-clears on signal restoration. |

*Outage threshold: outages shorter than $\tau$ are invisible to the detector. Longer outages trigger transient task preemption but self-recover the step communication resumes. All delivery counts remain within noise of the healthy baseline (1285.6 ± 37.5).*

##### Lossy Task Claims Sweep: Unreliable Conflict Broadcast (30 Runs)

**Dual-claim context:** At claim_loss=0%, 5,361.3 ± 265.6 dual claims occur per episode. This is the normal rate of out-of-comm-range concurrent targeting. Dropping claim announcements adds additional races on top of this baseline.

| Setting | Healthy Deliveries | Dual Claims | Match Rate |
| :--- | :---: | :---: | :---: |
| 0% claim loss (reference) | 1285.6 ± 37.5 | 5,361.3 ± 265.6 | 99.4% |
| 5% claim loss | 1316.3 ± 23.2 | 6,805.5 ± 164.0 (+27%) | 99.04% |
| 10% claim loss | 1287.7 ± 23.8 | 7,933.0 ± 141.6 (+48%) | 99.20% |
| 20% claim loss | 1311.2 ± 51.6 | 10,141.1 ± 371.5 (+89%) | 98.92% |

*Dropping claim broadcasts increases dual-targeting races non-proportionally relative to the drop rate. Per percentage-point of loss, the increases are approximately 289, 226, and 221 conflicts (slightly flattening). The mechanism behind this pattern was not tested in these runs. Despite this, capability match compliance stays above 98.9% and all delivery paired differences include zero (no detectable throughput change with 10 seeds). The claim-loss runs do not test failure recovery, so this only measures healthy-condition coordination overhead. A plausible explanation is that physical shelf-pickup serialization prevents dual delivery corruption, but this was not directly observed in these runs.*


---

#### 3. Exp 11 Bursty Channel Benchmark: Gilbert–Elliott Fading (40 Seeded Runs)

Independent drop models assume each packet loss is uncorrelated. In reality, industrial warehouse RF channels experience temporal fading bursts (e.g., passing behind metallic racking). Exp 11 implements a two-state Markov Gilbert–Elliott (GE) model:
* **Transition Probabilities:** (\text{Good} \to \text{Bad}) = 0.01$, (\text{Bad} \to \text{Good}) = 0.10$.
* **State Loss Rates:** \text{good} = 0.01$ (1%), \text{bad} = 0.90$ (90%).
* **Channel Properties:** Steady-state bad probability $\pi_\text{bad} \approx 9.09\%$; average packet loss rate $\bar{L} \approx 9.09\%$; mean burst length $\mu_\text{burst} = 1 / 0.10 = 10\text{ steps}$; mean inter-burst length $\mu_\text{good} = 100\text{ steps}$.

Evaluated across $\tau \in [5, 15]$ under healthy operations and Robot 0 failure (step 500) over 10 seeds:

| Setting | Healthy Deliveries | Failure Deliveries | Retention (%) | Measured FP Steps | Paired vs No-comm (95% CI) | Det Delay (Steps) |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: |
| **GE Bursty $\tau=5$** | 1304.5 ± 34.0 | 1091.4 ± 35.2 | 83.7% ± 2.5% | 6,932 | **+97.8 ± 54.0** [43.8, 151.8] | 5 in 8/10 seeds* |
| **GE Bursty $\tau=15$** | 1333.2 ± 33.9 | 1120.2 ± 43.9 | 84.0% ± 2.7% | 844 | **+126.6 ± 51.4** [75.2, 178.0] | 15 in 8/10 seeds* |
| *Indep 10% (Exp 9)* | *1285.6 ± 37.5* | *1103.6 ± 31.8* | *85.8% ± 3.5%* | *0.0* | *+110.0 ± 51.1* | *15 in 10/10 seeds* |

*\*Detection Delay Exceptions: In Seeds 2024 and 23, an active packet burst happened to silence Robot 0 on an observer\'s link immediately prior to step 500, pre-accumulating timeout counters so detection triggered earlier (0 and 4 steps at $\tau=5$; 2 and 14 steps at $\tau=15$). In the remaining 8 of 10 seeds, detection delay was exactly $\tau$.*

**Key Takeaways:**
1. **Bursts drastically amplify false alarms:** Because the average burst length (10 steps) exceeds $\tau=5$ and approaches $\tau=15$, bursty loss produces orders of magnitude more false-alarm steps than independent loss (844 steps at $\tau=15$ vs 0 steps at independent 10% loss and 72 at independent 60% loss; 6,932 steps at $\tau=5$). Over 240,000 pair-steps, this is 0.35% of pair-steps at $\tau=15$ and 2.9% at $\tau=5$.
2. **Throughput resilience persists:** Throughput retention remains virtually identical to clean conditions (.7\%\text{--}84.0\%$ vs .1\%$ baseline). The paired advantage of v2 over no-comm remains statistically significant ($+97.8$ at $\tau=5$ and $+126.6$ at $\tau=15$), confirming that the coordination mechanism survives bursty RF environments.
3. **Engineering design rule:** In real warehouse deployments, $\tau$ must be sized relative to the physical burst duration rather than independent drop probability.

---

#### 4. Mechanism Isolation: Beliefs-Disabled Control Experiment (10 Seeded Runs)

A crucial control tests whether the +111.9 delivery advantage of v2 over no-comm under failure truly stems from heartbeat-based claim release, or from secondary differences in auction and pathfinding logic.

In experiments/control_no_beliefs.py, Robot 0 is physically disabled at step 500, but belief propagation into the auction is suppressed (elief_failed=None). Surviving robots receive no peer failure notifications and maintain their normal claim-blocking logic:

| System / Condition | Deliveries (Mean ± 95% CI) | Paired Difference vs No-comm | Paired Difference vs v2 Normal | Status |
| :--- | :---: | :---: | :---: | :---: |
| **No-comm** (Exp 8) | 993.6 ± 39.0 | — | — | Baseline |
| **v2 Normal** (Clean baseline) | 1105.5 ± 28.5 | **+111.9 ± 52.6** [59.3, 164.5] | — | Communicating |
| **v2 Control** (f=None) | 1017.3 ± 55.3 | **+23.7 ± 83.8** [-60.1, +107.5] | **-88.2 ± 44.2** [-132.4, -44.0] | Beliefs Disabled |

**Per-Seed Deliveries Comparison:**
* Across all 10 seeds, disabling beliefs produces a consistent drop vs normal v2: Seed 1 (-81), Seed 42 (-122), Seed 100 (-27), Seed 2024 (-207), Seed 7 (-105), Seed 11 (-7), Seed 23 (-105), Seed 99 (-21), Seed 314 (-143), Seed 777 (-64).
* Paired drop: $-88.2 \pm 44.2$ ( \approx 0.0015$, two-tailed Student-t).

**Scientific Conclusion:**
* **Heartbeat beliefs are the necessary driver:** Suppressing heartbeat failure beliefs removes **78.8%** of the v2 delivery advantage over no-comm.
* **No residual advantage:** The remaining $+23.7 \pm 83.8$ difference from no-comm has a 95% CI that broadly spans zero ($[-60.1, +107.5]$), showing that without heartbeat failure detection, v2 behaves at statistical parity with zero-communication.
* **Why the control degrades:** When Robot 0 fails without belief notification, its active task claims remain locked in peer claim tables indefinitely. Active robots respect the claims and refuse to rescue the stranded tasks, confirming that heartbeat-triggered claim revocation is the sole mechanism responsible for the resilience gain.
---

## 7. Engineering Challenges, Debugging Log & Key Fixes

During the research and implementation phases, several subtle multi-agent bugs were identified and resolved:

### 1. The Missing Shelf Pickup / Drop Action Bug (Phase 5)
- **Symptom:** Robots navigated correctly to shelf coordinates via A*, but lingered indefinitely without picking up items.
- **Root Cause:** RWARE requires an explicit discrete action (`TOGGLE_LOAD = 4`) while facing a shelf to lift it, and another `TOGGLE_LOAD` at the goal zone to complete the drop-off.
- **Fix:** Enhanced `GridPathfinder` with arrival state detection: when $(x, y) == \text{target}$, it issues `TOGGLE_LOAD` before transitioning back to navigational moves.

### 2. False-Positive Heartbeat Timeouts (Phase 6)
- **Symptom:** Healthy robots were falsely marked as dead, causing their active tasks to be preemptively reassigned.
- **Root Cause:** `COMM_RANGE` for heartbeats was initially set to 8 cells. In larger grids, healthy robots operating at opposite ends of the warehouse moved out of radio range, causing heartbeat counters to exceed $\tau=15$.
- **Fix:** Decoupled task bidding range ($R_{\text{bid}} = 8$) from heartbeat liveness range ($R_{\text{heartbeat}} = 20$). In Phase 8, this was refined so that heartbeats are strictly evaluated among **intra-zone peers**.

### 3. NumPy `int64` vs Python `int` Coordinate Comparison (GUI / Core)
- **Symptom:** Robots occasionally stalled near goals, and GUI counters failed to increment.
- **Root Cause:** RWARE returns shelf locations as NumPy `(np.int64, np.int64)` tuples. Comparing these against native Python integer coordinates in dictionary lookups caused subtle hash misses.
- **Fix:** Normalized all coordinates using explicit `(int(x), int(y))` casting across `pathfinding.py` and `auction_layer.py`.

### 4. Quadruple NOOP Action Override in Zone Coordinator
- **Symptom:** Injecting a failure printed warning logs 4 times, and step execution slowed down.
- **Root Cause:** `HierarchicalCoordinator` maintained 4 `ZoneFaultLayer` instances, each independently checking `actual_failed` and applying `NOOP` overrides to all robots.
- **Fix:** Consolidated failure tracking in `HierarchicalCoordinator.get_fault_action_overrides()` to use `fault_layers[0]` as the single authoritative failure registry.

### 5. Gymnasium Passive Environment Checker Warning
- **Symptom:** Terminal spam reporting `UserWarning: The reward returned by step() is a list, but it should be a float`.
- **Root Cause:** Standard Gymnasium expects single-agent scalar rewards. In a multi-agent setting, `HeterogeneousWarehouseWrapper` returns a reward list (one per agent).
- **Fix:** Added `disable_env_checker=True` in `MultiRobotVecEnv` and ensured `step_wait()` converts reward lists to structured `np.float32` arrays.

### 6. Narrow Vertical Corridor Deadlock & Idle Robot Aisle Blocking
- **Symptom:** In narrow vertical aisles (e.g., Columns 0, 3, 6, 9 with width 1 between double-row shelf blocks), robots frequently queued up and froze behind an unmoving robot.
- **Root Cause:**
  1. *Idle Robot Parking:* Robots with no assigned task (`pursue = False`) executed `NOOP` and parked directly in single-lane transit aisles, creating impassable bottlenecks.
  2. *Static Reservation Blocking:* Trajectory following treated a moving peer's current cell as permanently blocked, preventing trailing robots from pipelining into cells being vacated.
  3. *Head-on Encounters:* When a loaded robot and an unloaded robot met in a 1-wide aisle, both evaluated the next cell as blocked and stopped indefinitely.
- **Fix:**
  - *Anti-Deadlock Aisle Yielding:* Unloaded idle robots standing in transit corridors yield into adjacent empty shelf cells (which empty AMRs are physically allowed to enter) to clear the aisle for loaded carriers.
  - *Dynamic Pipelining:* Trajectory checks permit robots to follow directly behind peers vacating their cells.
  - *Distinct Task Distribution:* `ZoneAuctionLayer` distributes available tasks across all active robots in a zone rather than collapsing multiple robots onto the same single shelf, eliminating redundant idle robots.

### 7. Realistic Physical Fault Recovery: In-Place Cargo Rescue & Delayed Perimeter Towing
- **Symptom:** In early prototypes, when a robot failed while carrying an inventory pod, the shelf instantly teleported back to its original rack coordinate, and the broken robot instantly teleported to the maintenance bay. In industrial fulfillment operations (e.g., Amazon Robotics), such instantaneous teleportation is physically impossible and unrepresentative of real automated facilities.
- **Real-World Operational Reality:**
  - An Automated Mobile Robot (AMR) carrying a 500 kg inventory pod cannot magically send the pod back to its home rack upon motor or battery failure.
  - The broken robot remains stationary on the floor at $(x_f, y_f)$, holding the pod, blocking that localized segment.
  - A healthy, capability-compatible peer AMR must be routed specifically to the exact breakdown location to physically relieve/collect the stranded pod.
  - **Only after** the active peer arrives and secures the cargo can the incapacitated robot be safely cleared/towed away to the perimeter maintenance bay (Row 0).
- **Algorithmic Protocol Implemented in `core/hierarchical_coordinator.py`:**
  1. *Stationary Breakdown with Stranded Task Registration:* When `inject_failure(robot_id)` fires, if the robot is carrying a shelf (`agent.carrying_shelf is not None`), the shelf is **not** moved and the robot is **not** towed. The incident is registered in `self.stranded_tasks[robot_id] = {'shelf': shelf, 'pos': (x_f, y_f), 'assigned_robot': None}`. The broken robot halts via `NOOP` overrides at $(x_f, y_f)$.
  2. *Autonomous Capability-Matched Rescue Dispatch:* In `_process_stranded_tasks()`, the coordinator evaluates healthy, unladen AMRs whose robot class matches the task capability requirement (`ACCEPTABLE_ROBOTS[task_type]`) and computes Manhattan distances to $(x_f, y_f)$. The optimal recovery AMR $R$ is assigned and dynamically routed (`global_pursue[R] = True`, `global_nearest_tasks[R] = pos`).
  3. *In-Place Cargo Handoff:* When recovery robot $R$ arrives adjacent to the broken AMR ($\text{dist} \le 1$), the physical load is transferred in-place (`rec_agent.carrying_shelf = shelf`, `failed_agent.carrying_shelf = None`, `shelf.x, shelf.y = rec_agent.x, rec_agent.y`). Robot $R$ immediately transitions into active delivery transit toward the pack station.
  4. *Delayed Towing to Perimeter Bay:* **Only after** the cargo has been successfully transferred is the incapacitated AMR towed to the perimeter maintenance bay `(robot_id % grid_width, 0)`, followed immediately by `self.env.unwrapped._recalc_grid()` to synchronize RWARE's spatial collision matrix and fully clear the transit aisle.
  *(Note: If an AMR fails while completely empty with no cargo, it is towed immediately to Row 0 as no task is stranded).*

---


### 8. Auction Incomplete Retargeting & Idle Loser Flaw (Phase 7 -> v2)
- **Symptom:** The flat auction layer produced only $453.3$ deliveries in 4-robot healthy baseline runs, lagging far behind naive greedy heuristics.
- **Root Cause:** In the initial auction implementation (`auction_layer_fixed.py`), a robot that bid on a task but lost to a higher-bidding peer set `pursue = False` and idled for that timestep, even if multiple other eligible tasks were waiting unserviced in the queue.
- **Fix:** Implemented sequential retargeting in `auction_layer_v2.py`: if a robot's highest-priority task is already claimed by a higher-bidding peer within communication range, the robot immediately retargets to its next-closest eligible task. This single change increased healthy throughput **2.8× (from $453.3$ to $1285.6$ deliveries)**.

### 9. Failed Robots Re-broadcasting Ghost Bids & Stale Claim Freezing
- **Symptom:** After injecting a failure, healthy robots frequently froze and refused to service tasks originally claimed by the failed robot.
- **Root Cause:** Incapacitated robots stopped updating motion but continued executing local belief updates in memory, and healthy robots continued evaluating historical claims from failed IDs.
- **Fix:** Added receiver-side filtering: surviving robots actively check their local heartbeat belief matrix and strictly ignore all bids and claims originating from robots marked as failed.

### 10. Non-Deterministic Environment Reset Seed Forwarding
- **Symptom:** Multi-seed evaluation runs produced non-reproducible trajectories, preventing valid seed-matched paired comparisons.
- **Root Cause:** `MultiRobotVecEnv.reset()` called `self.hetero_env.reset()` without forwarding the `seed` argument to base RWARE. While the wrapper maintained its own local RNG for task types, RWARE's underlying environment generator never received the seed, leaving initial robot spawn positions and warehouse shelf layouts unseeded and non-deterministic.
- **Fix:** Stored `self.seed = seed` in `MultiRobotVecEnv.__init__()` and explicitly forwarded `reset(seed=self.seed)` through the wrapper stack into RWARE's base environment. Bitwise determinism checks (`determinism_check.py`) confirmed identical step-for-step state trajectories across runs with the same seed.

### 11. Pathfinder Turn Logic & Heading Alignment
- **Symptom:** Robots occasionally performed unnecessary 360-degree rotations or oscillating turns when facing targets.
- **Root Cause:** Orientation changes did not account for modular clockwise turn distance arithmetic (`UP=0, RIGHT=1, DOWN=2, LEFT=3`).
- **Fix:** Implemented minimal angular rotation selection in `GridPathfinder._turn_action()`: calculating `(desired - current) % 4` to select the minimal single-step turn command (`LEFT` or `RIGHT`).

---

## 8. Execution Guide: How to Run Experiments & Interactive GUI

### 8.1 Prerequisites & Virtual Environment Setup
Open PowerShell or Command Prompt in the project root:

```bash
cd C:\Users\Kezia\warehouse-swarm
.\venv\Scripts\activate
```

Verify package dependencies:
```bash
python -c "import gymnasium, rware, torch; print('Environment operational!')"
```

---

### 8.2 Running the Interactive Visualizer GUI

Launch the live graphical interface:
```bash
python visualization/run_simulation_gui.py
```

```
+---------------------------------------------------------------------------------+
|                   WAREHOUSE SWARM INTERACTIVE CONTROL PANEL                     |
|                                                                                 |
|  [ Play / Pause ]  [ Step Forward ]  [ Reset Simulation ]  [ Speed Slider ]     |
|                                                                                 |
|  Fail Robot: [ Robot 1 (heavy_load) v ]  [ INJECT FAILURE NOW ]                 |
+---------------------------------------------------------------------------------+
```
- **Observe Zones:** The 4 quadrants represent spatial zones with color-coded boundaries.
- **Observe Specialization:** Watch Blue robots (`fast_light`) sprint toward cyan tasks, while Red robots (`heavy_load`) haul magenta pallets.
- **Interactive Failure Test:** Select any robot from the dropdown and click **"Fail Robot"**. Watch the robot turn dark grey, observe neighbor heartbeats time out within 15 ticks, and watch an active peer take over its abandoned task!

---

### 8.3 Executing Experimental Benchmarks

#### 1. Exp 8 Canonical Final Benchmark Suite (All 280 Seeded Runs)
Executes the full seeded benchmark suite across 10 seeds (`1, 42, 100, 2024, 7, 11, 23, 99, 314, 777`) for greedy, no-comm, fixed, and v2 configurations:
```bash
python experiments/exp8_final_suite.py
```
Output files: `experiments/results/exp8_summary.csv` and `experiments/results/exp8_raw.json`.

#### 2. Supervised Neural Bid Head Training
Generate 400,000 training observations:
```bash
python experiments/collect_bid_data.py
```
Train the `CapabilityConditionedPolicy` network:
```bash
python experiments/exp1_train_bid_head_logged.py
```
Output files: `experiments/results/exp1_history.csv` and `models_and_data/bid_head_trained.pt`.

#### 3. Diagnostic & Determinism Verification Tools
Verify bitwise deterministic seed execution:
```bash
python experiments/determinism_check.py
```
Inspect capability mismatch traces:
```bash
python experiments/diag_mismatch.py
```

#### 4. Hierarchical Multi-Zone Scaling Benchmark
Run the multi-zone hierarchical coordination suite across 2, 4, and 8 zone configurations:
```bash
python experiments/exp6_zone_scaling.py
```

#### 5. Exp 9 & 10 Communication Fault & Stress Suites (440 Runs Total)
Executes moderate channel imperfections (Exp 9) and extreme breaking point sweeps (Exp 10: severe drop up to 90%, temporary radio outages, and lossy auction claims):
`ash
python experiments/exp9_robustness.py --workers 6
python experiments/exp10_stress.py --workers 6
python experiments/analyze_stress.py
`
Output files: experiments/results/exp9_summary.csv, experiments/results/exp10_summary.csv, and associated JSON stores.

---

## 8.4 Architectural Assumptions & System Limitations

To maintain strict scientific integrity, the explicit limitations and design boundaries of this work are catalogued below:

1. **Static Zone Boundaries vs. Local Congestion:**  
   Warehouse zones are partitioned as fixed rectangular quadrants based on spatial geometry. They do not dynamically deform or balance boundaries if customer pick/pack order demand concentrates heavily within one specific aisle or station.
2. **Hybrid Decentralization Boundary (Single Logical Router):**  
   The system is decentralized and zone-based during operational execution: robots bid, exchange heartbeats, deconflict claims, and plan motion purely locally. However, task admission ingests from RWARE's shared order queue, and the top-level `HierarchicalCoordinator` operates as a single logical router for initial centroid assignment and cross-zone timeout handoffs.
3. **Discrete Grid Abstraction & Simulation Scope:**  
   The evaluation is conducted in simulation on discrete 2D grid environments in RWARE. Continuous differential-drive physics, non-holonomic kinematics, motor slippage, sensor SLAM noise, and RF multipath wireless fading are not modeled.
4. **Hardware Specialization Asymmetry:**  
   The default 4-robot fleet contains exactly one `heavy_load` robot. While representative of specialized capital equipment in fulfillment centers, losing this single robot leaves heavy tasks permanently unserviceable.
5. **Bid-Ordered Auction Emulation & Communication Boundaries (Exp 9 & 10, 440 runs):**  
   The v2 auction resolves bids in descending suitability order in Python memory. The peer heartbeat and claim layers were stress-tested across 440 seeded evaluations. Empirical boundaries:
   - **Detector (not throughput) breaking points:** At tau=15, false alarms begin above 40% loss (0 false alarms at 40%, 72 at 60%, 44,776 at 90%). At tau=5, false alarms appear at 40% loss (998 steps). Throughput never collapses: the v2 coordination advantage over no-comm (claim release on failure) is confirmed at P<0.05 across all settings including 90% loss.
   - **Outage threshold:** Outages shorter than tau are invisible. Longer outages cause transient false alarms that self-clear when signal resumes.
   - **Claim loss:** Dropping 20% of claim announcements raises dual-targeting conflicts by 89% (5,361 to 10,141 per episode) but match compliance stays above 98.9%.
   - **Detection limit:** With 10 seeds, the 95% CI is ~3-5 points wide; throughput drops smaller than that cannot be detected with this sample size.
6. **Untrained Communication Gating Head:**  
   The secondary `comm_head` output of `CapabilityConditionedPolicy` remains untrained; wireless range filtering ($R_{\text{comm}}=8$) is currently enforced spatially rather than through learned gating.
7. **Clean Stop Failure Model with Robustness Validation:**  
   Hardware breakdown is modeled as an unannounced motor/power freeze at step 500 where the robot executes `NOOP` and ceases heartbeats. While partial brownouts and sensor drift are not explicitly simulated, the liveness detection mechanism has been rigorously tested across timeout thresholds $\tau \in [5, 30]$ and shown to maintain $\ge 84.7\%$ retention.
8. **Residual Capability Mismatches ($0.2\text{--}2.9\%$):**  
   Detailed traces established that residual mismatches occur due to passive shelf transport over goal cells during transit, which RWARE's engine evaluates as an immediate delivery event regardless of pursuit state.

---

## 8.5 Experimental Setup, Hardware & Reproducibility Metadata

- **Host Processor (CPU):** Intel(R) Core(TM) Ultra 7 155H (16 physical cores: 6 Performance, 8 Efficient, 2 Low-Power Efficient; 22 logical threads; up to 4.8 GHz).
- **Host Memory (RAM):** 16.0 GB system RAM.
- **Operating System:** Microsoft Windows 11 Home (64-bit).
- **Python Environment:** Python 3.11.9 running in a dedicated virtual environment.
- **Key Package Versions:** `torch` 2.13.0+cpu, `numpy` 2.4.6, `gymnasium` 1.3.0, `rware` 2.0.0, `stable-baselines3` 2.9.0, `matplotlib` 3.11.2, `scipy` 1.17.1.
- **Standardized Seed Suite (10 Seeds):** `[1, 42, 100, 2024, 7, 11, 23, 99, 314, 777]`.
- **Environment Floor Configurations:**
  - `rware-tiny`: $11 \times 10$ grid, 32 storage shelves, 2 drop-off delivery stations (evaluated with 2, 4, and 8 robots).
  - `rware-small-16ag`: $20 \times 10$ grid, evaluated with 16 robots.
  - `rware-medium-16ag`: $20 \times 16$ grid, evaluated with 16 robots.
- **Fleet Capability Compositions:**
  - 4-robot fleet: `[fast_light, heavy_load, balanced, fast_light]`.
  - 16-robot fleet: `[fast_light, heavy_load, balanced] * 5 + [fast_light]`.
  - RWARE request queue capacity: 4 active concurrent shelf requests.

---

## 8.6 Sim-to-Real Applicability & Industrial Deployment Mapping

To bridge the gap between simulation abstractions and physical deployment in real-world automated logistics facilities, the table below maps core architectural components to their commercial robotic counterparts and empirical validation status:

| Architectural Component | Simulation Assumption | Real-World Commercial Counterpart | Empirical Validation Status (Exp 8 & 9) |
| :--- | :--- | :--- | :--- |
| **Hardware Specialization** | 3 discrete AMR classes with differential battery drain and probabilistic slip | Specialized industrial AMR fleets (e.g., lightweight tote sorters vs heavy pallet/pod lifters) | Validated: $99.0\%\text{--}99.8\%$ capability compliance; mismatch penalties successfully steer assignments |
| **Peer Coordination** | Descending bid-ordered auction emulation with $R_{\text{comm}}=8$ radius | Range-limited Wi-Fi / Ultra-Wideband (UWB) mesh networking among peer AMRs | Validated: Instant retargeting eliminates idle penalty ($2.8\times$ throughput boost) |
| **Network Loss & Jitter** | Synthetic packet drop rate $p_{\text{loss}} \in [0, 0.20]$ and queue delay $d \le 5$ | Industrial Wi-Fi multipath fading, shadow fading from steel racks, AP roaming latency | Validated (Exp 9): Zero false-positive task retractions; $\ge 86.0\%$ throughput retention sustained up to $20\%$ loss |
| **Fault Detection Layer** | Discrete step timeout counter $\tau=15$ steps ($\sim 15\text{ s}$ wall-clock) | Asynchronous UDP heartbeat pinging with timeout detection window | Validated: Failsafe detection in $\tau+d$ steps; stable retention across $\tau \in [5, 30]$ |
| **Stranded Cargo Recovery** | Stationary breakdown followed by peer rescue and delayed maintenance towing | Automated floor clearance protocol: unladen AMR dispatched to recover pod before tug vehicle clears chassis | Implemented in `HierarchicalCoordinator`: prevents aisle deadlocks and preserves cargo flow |
| **Spatial Scaling** | Fixed rectangular zone partitioning ($2, 4, 8$ spatial zones) | Dynamic or static fulfillment sectoring (e.g., mezzanine picking zones vs outbound dock staging) | Validated: Wireless communication events bounded ($1.25 \to 0.23$ events/step) as zones increase |

---

## 9. Future Work & Hardware Deployment Roadmap

1. **Partial Hardware Faults & Intermittent Actuation:**  
   Extend the failure model beyond clean stops to include intermittent motor stalls, degraded velocity, and sensor brownouts, evaluating whether auction belief tables can dynamically downgrade suitability scores rather than applying binary failure flags.
2. **Distributed Router & Leader Election:**  
   Decentralize the top-level task admission router by replacing the logical hierarchical coordinator with a distributed leader-election protocol or edge-gateway consensus across zones.
2. **Dynamic Zone Rebalancing (Voronoi Partitioning):**  
   Implement dynamic Voronoi or load-adaptive zone rebalancing to dynamically expand or contract zone boundaries in real time as order fulfillment hotspots shift across the floor.
3. **Learned Communication Gating (`comm_head`):**  
   Train the secondary output head of `CapabilityConditionedPolicy` via reinforcement learning to dynamically suppress broadcast packets when local certainty is high, further optimizing wireless bandwidth.
4. **Sim-to-Real Transfer on ROS 2 / Isaac Sim:**  
   Bridge the discrete RWARE grid abstractions to continuous physics in NVIDIA Isaac Sim, targeting differential-drive TurtleBot 4 or Clearpath Ridgeback AMR platforms with Nav2 SLAM integration.

---


---

---

---

## 9.1 Academic Bibliography & Key References

1. **RWARE Multi-Agent Benchmark:**  
   F. Christianos, L. Schaefer, and S. V. Albrecht, *"Shared experience actor-critic for multi-agent reinforcement learning,"* in Advances in Neural Information Processing Systems (NeurIPS), vol. 33, pp. 10707–10717, 2020.
2. **Decentralized Multi-Robot Task Allocation (DC-MRTA):**  
   A. Agrawal, S. Hariharan, A. S. Bedi, and D. Manocha, *"DC-MRTA: Decentralized multi-robot task allocation and navigation in complex environments,"* arXiv preprint arXiv:2209.02865, 2022.
3. **Decentralized Dynamic Zoning (DDZ):**  
   R. Keith and H. La, *"Dynamic zoning of industrial environments with autonomous mobile robots,"* arXiv preprint arXiv:2411.07382, 2024.
4. **Multi-Agent Coordination in Logistics:**  
   A. D'Urso et al., *"Decentralized Multi-Agent Path Finding and Task Assignment in Automated Warehouses,"* in IEEE International Conference on Automation Science and Engineering (CASE), 2023. [Note: Verify exact author roster and final proceeding volume title against IEEE Xplore before camera-ready submission].

---


## 10. Complete Verbatim Source Code of All Project Files

This section contains the unabridged source code for the core system modules, the final canonical Exp 8 evaluation suite, and key diagnostic tools.

> **Architecture Note for Appendix B:**  
> The final deployed system uses `experiments/auction_layer_v2.py` as its canonical auction coordinator (with sequential retargeting and receiver-side filtering). The legacy `core/auction_layer.py` is preserved below for architectural continuity as the baseline non-retargeting ablation.

### 10.1 `core/__init__.py`

- **Purpose:** Path resolution and package initialization
- **Relative Path:** `[core/__init__.py](file:///c:/Users/Kezia/warehouse-swarm/core/__init__.py)`
- **System Location:** `c:\Users\Kezia\warehouse-swarm\core\__init__.py`

```python
import os
import sys
from pathlib import Path

# Ensure core and project root are always available on sys.path
_CORE_DIR = str(Path(__file__).resolve().parent)
_ROOT_DIR = str(Path(__file__).resolve().parent.parent)

for _p in [_CORE_DIR, _ROOT_DIR]:
    if _p not in sys.path:
        sys.path.insert(0, _p)
```

### 10.2 `core/hetero_wrapper.py`

- **Purpose:** Heterogeneous environment wrapper with capabilities, battery drain, and shaping rewards
- **Relative Path:** `[core/hetero_wrapper.py](file:///c:/Users/Kezia/warehouse-swarm/core/hetero_wrapper.py)`
- **System Location:** `c:\Users\Kezia\warehouse-swarm\core\hetero_wrapper.py`

```python
import gymnasium as gym
import numpy as np


class HeterogeneousWarehouseWrapper(gym.Wrapper):
    """
    Wraps RWARE to add:
      - robot capability types (fast_light, heavy_load, balanced)
      - battery draining per type
      - speed effects per type (bonus/dropped moves)
      - task types on requested shelves (urgent_light, heavy_shelf, standard_delivery)
      - nearest-task info appended to each robot's observation
      - capability mismatch penalty on delivery
      - distance-based shaping reward (addresses sparse-reward problem)
      - match/mismatch counters for evaluation
    """

    TYPE_NAMES = ["fast_light", "heavy_load", "balanced"]
    TASK_NAMES = ["urgent_light", "heavy_shelf", "standard_delivery"]
    TASK_WEIGHT = {0: 1, 1: 3, 2: 2}

    BATTERY_DRAIN = {0: 0.8, 1: 1.3, 2: 1.0}
    BONUS_STEP_PROB = {0: 0.5, 1: 0.0, 2: 0.0}
    DROP_STEP_PROB = {0: 0.0, 1: 0.3, 2: 0.0}

    ACCEPTABLE_ROBOTS = {
        0: {0, 2},      # urgent_light: fast_light or balanced
        1: {1},         # heavy_shelf: heavy_load only
        2: {0, 1, 2},   # standard_delivery: anyone
    }
    MISMATCH_PENALTY = 0.5

    MAX_BATTERY = 100.0
    FORWARD_ACTION = 1
    TOGGLE_LOAD_ACTION = 4
    MAX_SEARCH_DISTANCE = 20.0

    SHAPING_SCALE = 0.05

    def __init__(self, env, robot_types=None, seed=None, verbose=False):
        super().__init__(env)
        n_agents = self.unwrapped.n_agents

        if robot_types is None:
            robot_types = [i % 3 for i in range(n_agents)]
        assert len(robot_types) == n_agents

        self.robot_types = robot_types
        self.type_embeddings = np.eye(3)
        self.task_embeddings = np.eye(3)
        self.battery = np.full(n_agents, self.MAX_BATTERY, dtype=np.float32)
        self.verbose = verbose
        self.rng = np.random.default_rng(seed)

        self.shelf_task_type = {}
        self._prev_request_queue = []
        self._prev_objective_distance = [None] * n_agents

        # --- delivery tracking counters, per robot ---
        self.match_count = [0] * n_agents
        self.mismatch_count = [0] * n_agents
        # also track, per robot, how many deliveries of EACH task type they did
        self.deliveries_by_task_type = [{0: 0, 1: 0, 2: 0} for _ in range(n_agents)]

        print("Robot types assigned:")
        for i, t in enumerate(self.robot_types):
            print(f"  Robot {i}: {self.TYPE_NAMES[t]} "
                  f"(battery drain: {self.BATTERY_DRAIN[t]}, "
                  f"bonus prob: {self.BONUS_STEP_PROB[t]}, "
                  f"drop prob: {self.DROP_STEP_PROB[t]})")

    def _assign_task_types_to_new_requests(self):
        current_queue = list(self.unwrapped.request_queue)
        for shelf in current_queue:
            if shelf not in self.shelf_task_type:
                task_type = int(self.rng.integers(0, 3))
                self.shelf_task_type[shelf] = task_type
                if self.verbose:
                    print(f"  New task requested: shelf at ({shelf.x},{shelf.y}) "
                          f"-> type={self.TASK_NAMES[task_type]}")
        for shelf in list(self.shelf_task_type.keys()):
            if shelf not in current_queue:
                del self.shelf_task_type[shelf]
        self._prev_request_queue = current_queue

    def _get_nearest_task_info(self, agent):
        queue = self.unwrapped.request_queue
        if not queue:
            return np.zeros(5, dtype=np.float32)

        best_dist = None
        best_shelf = None
        for shelf in queue:
            dist = abs(shelf.x - agent.x) + abs(shelf.y - agent.y)
            if best_dist is None or dist < best_dist:
                best_dist = dist
                best_shelf = shelf

        task_type = self.shelf_task_type.get(best_shelf, 2)
        type_onehot = self.task_embeddings[task_type]
        weight_norm = np.array([self.TASK_WEIGHT[task_type] / 3.0], dtype=np.float32)
        dist_norm = np.array([min(best_dist / self.MAX_SEARCH_DISTANCE, 1.0)], dtype=np.float32)

        return np.concatenate([type_onehot, weight_norm, dist_norm]).astype(np.float32)

    def _get_objective_distance(self, agent):
        if agent.carrying_shelf is not None:
            goals = self.unwrapped.goals
            if not goals:
                return None
            dists = [abs(gx - agent.x) + abs(gy - agent.y) for gx, gy in goals]
            return min(dists)
        else:
            queue = self.unwrapped.request_queue
            if not queue:
                return None
            dists = [abs(s.x - agent.x) + abs(s.y - agent.y) for s in queue]
            return min(dists)

    def _augment_obs(self, obs_tuple):
        new_obs = []
        for i, obs in enumerate(obs_tuple):
            type_vec = self.type_embeddings[self.robot_types[i]]
            battery_norm = np.array([self.battery[i] / self.MAX_BATTERY], dtype=np.float32)
            task_info = self._get_nearest_task_info(self.unwrapped.agents[i])
            augmented = np.concatenate([obs, type_vec, battery_norm, task_info]).astype(np.float32)
            new_obs.append(augmented)
        return tuple(new_obs)

    def reset(self, **kwargs):
        obs, info = self.env.reset(**kwargs)
        self.battery[:] = self.MAX_BATTERY
        self.shelf_task_type = {}
        self._assign_task_types_to_new_requests()

        self._prev_objective_distance = [
            self._get_objective_distance(agent) for agent in self.unwrapped.agents
        ]

        return self._augment_obs(obs), info

    def _apply_speed_effects(self, actions):
        actions = list(actions)
        bonus_flags = [False] * len(actions)
        for i, t in enumerate(self.robot_types):
            agent_action = actions[i][0] if isinstance(actions[i], tuple) else actions[i]
            if agent_action == self.FORWARD_ACTION:
                if self.DROP_STEP_PROB[t] > 0 and self.rng.random() < self.DROP_STEP_PROB[t]:
                    if isinstance(actions[i], tuple):
                        actions[i] = (0,) + actions[i][1:]
                    else:
                        actions[i] = 0
                elif self.BONUS_STEP_PROB[t] > 0 and self.rng.random() < self.BONUS_STEP_PROB[t]:
                    bonus_flags[i] = True
        return actions, bonus_flags

    def step(self, actions):
        modified_actions, bonus_flags = self._apply_speed_effects(actions)

        queue_before = list(self.unwrapped.request_queue)
        shelf_task_before = dict(self.shelf_task_type)

        obs, rewards, terminated, truncated, info = self.env.step(modified_actions)

        if any(bonus_flags):
            bonus_actions = []
            for i in range(len(modified_actions)):
                if bonus_flags[i]:
                    if isinstance(modified_actions[i], tuple):
                        bonus_actions.append((self.FORWARD_ACTION,) + modified_actions[i][1:])
                    else:
                        bonus_actions.append(self.FORWARD_ACTION)
                else:
                    if isinstance(modified_actions[i], tuple):
                        bonus_actions.append((0,) + modified_actions[i][1:])
                    else:
                        bonus_actions.append(0)
            obs, bonus_rewards, terminated, truncated, info = self.env.step(bonus_actions)
            rewards = [r1 + r2 for r1, r2 in zip(rewards, bonus_rewards)]

        rewards = list(rewards)

        queue_after = set(self.unwrapped.request_queue)
        delivered_shelves = [s for s in queue_before if s not in queue_after]

        for shelf in delivered_shelves:
            task_type = shelf_task_before.get(shelf, 2)
            for i, agent in enumerate(self.unwrapped.agents):
                if agent.x == shelf.x and agent.y == shelf.y:
                    self.deliveries_by_task_type[i][task_type] += 1
                    if self.robot_types[i] not in self.ACCEPTABLE_ROBOTS[task_type]:
                        rewards[i] -= self.MISMATCH_PENALTY
                        self.mismatch_count[i] += 1
                        if self.verbose:
                            print(f"  MISMATCH: Robot {i} ({self.TYPE_NAMES[self.robot_types[i]]}) "
                                  f"delivered a {self.TASK_NAMES[task_type]} task -> penalty applied")
                    else:
                        self.match_count[i] += 1
                        if self.verbose:
                            print(f"  OK MATCH: Robot {i} ({self.TYPE_NAMES[self.robot_types[i]]}) "
                                  f"delivered a {self.TASK_NAMES[task_type]} task -> no penalty")

        for i, agent in enumerate(self.unwrapped.agents):
            new_dist = self._get_objective_distance(agent)
            old_dist = self._prev_objective_distance[i]

            if new_dist is not None and old_dist is not None:
                delta = old_dist - new_dist
                rewards[i] += self.SHAPING_SCALE * delta

            self._prev_objective_distance[i] = new_dist

        for i, t in enumerate(self.robot_types):
            self.battery[i] = max(0.0, self.battery[i] - self.BATTERY_DRAIN[t])

        self._assign_task_types_to_new_requests()

        return self._augment_obs(obs), rewards, terminated, truncated, info
```

### 10.3 `core/multi_robot_vecenv.py`

- **Purpose:** Vectorized multi-agent environment runner
- **Relative Path:** `[core/multi_robot_vecenv.py](file:///c:/Users/Kezia/warehouse-swarm/core/multi_robot_vecenv.py)`
- **System Location:** `c:\Users\Kezia\warehouse-swarm\core\multi_robot_vecenv.py`

```python
import numpy as np
from stable_baselines3.common.vec_env import VecEnv
from stable_baselines3.common.vec_env.base_vec_env import VecEnvIndices
import gymnasium as gym
import rware
from hetero_wrapper import HeterogeneousWarehouseWrapper


class MultiRobotVecEnv(VecEnv):
    """
    Presents a single HeterogeneousWarehouseWrapper's N robots as N parallel
    'environments' to SB3. Internally, all robots are stepped together in the
    same real environment every call -- this correctly implements IPPO with
    parameter sharing (one SB3 model, N agents, one shared world).
    """

    def __init__(self, env_id="rware-tiny-2ag-v2", robot_types=None, seed=None):
        # disable_env_checker suppresses the Gymnasium passive env checker warning.
        # HeterogeneousWarehouseWrapper returns a per-robot reward list; step_wait()
        # converts it to np.float32 correctly.
        base_env = gym.make(env_id, disable_env_checker=True)
        self.hetero_env = HeterogeneousWarehouseWrapper(base_env, robot_types=robot_types, seed=seed)
        self.seed = seed  # stored for deterministic reset() across all seeds
        self.n_robots = self.hetero_env.unwrapped.n_agents

        obs_dim = 80  # from our confirmed observation size
        observation_space = gym.spaces.Box(low=-np.inf, high=np.inf, shape=(obs_dim,), dtype=np.float32)
        action_space = gym.spaces.Discrete(5)  # single robot's action space

        super().__init__(self.n_robots, observation_space, action_space)

        self._actions = None
        self._last_obs = None

    def reset(self):
        obs_tuple, info = self.hetero_env.reset(seed=self.seed)
        self._last_obs = np.stack(obs_tuple).astype(np.float32)
        return self._last_obs

    def step_async(self, actions):
        self._actions = actions

    def step_wait(self):
        # SB3 gives us N actions (one per robot); RWARE expects a tuple
        actions_tuple = tuple(int(a) for a in self._actions)

        obs_tuple, rewards, terminated, truncated, info = self.hetero_env.step(actions_tuple)

        obs = np.stack(obs_tuple).astype(np.float32)
        rewards = np.array(rewards, dtype=np.float32)
        dones = np.array([terminated or truncated] * self.n_robots, dtype=bool)
        infos = [{} for _ in range(self.n_robots)]

        # if the episode ended, SB3 expects an auto-reset, matching normal VecEnv behaviour
        if terminated or truncated:
            reset_obs, _ = self.hetero_env.reset()
            obs = np.stack(reset_obs).astype(np.float32)

        self._last_obs = obs
        return obs, rewards, dones, infos

    def close(self):
        self.hetero_env.close()

    # --- required abstract methods from VecEnv, minimal implementations ---
    def get_attr(self, attr_name, indices=None):
        return [getattr(self.hetero_env, attr_name)] * self.n_robots

    def set_attr(self, attr_name, value, indices=None):
        setattr(self.hetero_env, attr_name, value)

    def env_method(self, method_name, *args, indices=None, **kwargs):
        method = getattr(self.hetero_env, method_name)
        return [method(*args, **kwargs)] * self.n_robots

    def env_is_wrapped(self, wrapper_class, indices=None):
        return [False] * self.n_robots

    def seed(self, seed=None):
        return [seed] * self.n_robots
```

### 10.4 `core/policy_network.py`

- **Purpose:** CapabilityConditionedPolicy neural network architecture
- **Relative Path:** `[core/policy_network.py](file:///c:/Users/Kezia/warehouse-swarm/core/policy_network.py)`
- **System Location:** `c:\Users\Kezia\warehouse-swarm\core\policy_network.py`

```python
import torch
import torch.nn as nn

class CapabilityConditionedPolicy(nn.Module):
    """
    Shared policy network used by all robots (IPPO-style).
    Takes the 80-dim heterogeneous observation and outputs:
      - bid_value: scalar in [0, 1], how suitable this robot is for the nearest task
      - comm_gate: scalar in [0, 1] (probability of broadcasting this step)

    Because the observation includes the robot's own type embedding,
    the same shared weights naturally produce different outputs for
    different robot types.
    """

    def __init__(self, obs_dim=80, hidden1=128, hidden2=64):
        super().__init__()

        self.encoder = nn.Sequential(
            nn.Linear(obs_dim, hidden1),
            nn.ReLU(),
            nn.Linear(hidden1, hidden2),
            nn.ReLU(),
        )

        self.bid_head = nn.Linear(hidden2, 1)
        self.comm_head = nn.Linear(hidden2, 1)

    def forward(self, obs):
        """
        obs: tensor of shape (batch_size, 80)
        returns: bid_value (batch_size, 1), comm_gate (batch_size, 1)
                 both squashed into [0, 1] via sigmoid
        """
        features = self.encoder(obs)
        bid_value = torch.sigmoid(self.bid_head(features))
        comm_gate = torch.sigmoid(self.comm_head(features))
        return bid_value, comm_gate
```

### 10.5 `core/pathfinding.py`

- **Purpose:** Deterministic A* grid pathfinder, orientation tracker, and collision avoidance
- **Relative Path:** `[core/pathfinding.py](file:///c:/Users/Kezia/warehouse-swarm/core/pathfinding.py)`
- **System Location:** `c:\Users\Kezia\warehouse-swarm\core\pathfinding.py`

```python
"""
Phase 5 (patched): A* pathfinding + RWARE action conversion with
decentralized collision avoidance.

Fixes over the previous version:
  1. Robots carrying a shelf treat other shelf cells as obstacles
     (RWARE refuses to move a loaded robot onto a shelf cell).
  2. After a delivery the robot RETURNS the shelf to its home cell and
     drops it there (TOGGLE_LOAD). Previously robots stayed loaded forever.
  3. A robot that is carrying a shelf always acts on it, regardless of
     the auction's pursue flag (otherwise it froze mid-delivery).
  4. Other robots' cells are treated as obstacles via a replan; if no
     route exists the robot waits. A stuck-breaker sidesteps after
     STUCK_LIMIT steps without progress.
  5. Same-cell conflicts: lower robot_id has priority (unchanged).
"""

import heapq


class GridPathfinder:
    NOOP = 0
    FORWARD = 1
    LEFT = 2
    RIGHT = 3
    TOGGLE_LOAD = 4

    UP, DOWN, LEFT_DIR, RIGHT_DIR = 0, 1, 2, 3
    DELTA = {UP: (0, -1), DOWN: (0, 1), LEFT_DIR: (-1, 0), RIGHT_DIR: (1, 0)}

    # clockwise ordering used to compute shortest turn direction
    CLOCKWISE_ORDER = [UP, RIGHT_DIR, DOWN, LEFT_DIR]

    STUCK_LIMIT = 12  # steps of wanting to move but not moving before sidestepping

    def __init__(self, hetero_env):
        self.env = hetero_env
        grid_size = hetero_env.unwrapped.grid_size
        self.width, self.height = int(grid_size[1]), int(grid_size[0])

        self.shelf_home = {}   # shelf key -> (x, y) home cell
        self._ensure_homes()
        self.reset_episode()

    # ------------------------------------------------------------------
    # Bookkeeping
    # ------------------------------------------------------------------

    def reset_episode(self):
        n = self.env.unwrapped.n_agents
        self._last_pos = [None] * n
        self._stuck = [0] * n
        self._wanted_move = [False] * n
        self._side_target = [None] * n
        self._t = 0

    @staticmethod
    def _shelf_key(shelf):
        sid = getattr(shelf, "id", None)
        return int(sid) if sid is not None else id(shelf)

    def _ensure_homes(self):
        """Remember where every shelf lives when it is not being carried."""
        ua = self.env.unwrapped
        carried = {a.carrying_shelf for a in ua.agents if a.carrying_shelf is not None}
        for s in ua.shelfs:
            key = self._shelf_key(s)
            if key not in self.shelf_home and s not in carried:
                self.shelf_home[key] = (int(s.x), int(s.y))

    # ------------------------------------------------------------------
    # A*
    # ------------------------------------------------------------------

    def _neighbors(self, cell, blocked):
        x, y = cell
        for dx, dy in [(0, -1), (0, 1), (-1, 0), (1, 0)]:
            nx, ny = x + dx, y + dy
            if 0 <= nx < self.width and 0 <= ny < self.height and (nx, ny) not in blocked:
                yield (nx, ny)

    def astar(self, start, goal, blocked):
        if start == goal:
            return []

        open_set = [(0, start)]
        came_from = {}
        g_score = {start: 0}

        while open_set:
            _, current = heapq.heappop(open_set)
            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.reverse()
                return path

            for neighbor in self._neighbors(current, blocked):
                tentative_g = g_score[current] + 1
                if tentative_g < g_score.get(neighbor, float("inf")):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + abs(neighbor[0] - goal[0]) + abs(neighbor[1] - goal[1])
                    heapq.heappush(open_set, (f_score, neighbor))

        return None

    # ------------------------------------------------------------------
    # Action conversion
    # ------------------------------------------------------------------

    def _turn_action(self, current_dir, desired_dir):
        """Shortest rotation from current_dir to desired_dir using the
        clockwise ordering UP -> RIGHT -> DOWN -> LEFT -> UP."""
        cur_idx = self.CLOCKWISE_ORDER.index(current_dir)
        des_idx = self.CLOCKWISE_ORDER.index(desired_dir)
        diff = (des_idx - cur_idx) % 4

        if diff == 1:
            return self.RIGHT
        elif diff == 3:
            return self.LEFT
        elif diff == 2:
            return self.RIGHT  # 180 degree turn, arbitrarily go right first
        else:
            return self.FORWARD

    def next_cell_to_action(self, agent, next_cell):
        dx = next_cell[0] - int(agent.x)
        dy = next_cell[1] - int(agent.y)

        if dx == 0 and dy == 0:
            return self.NOOP

        if dx == 1:
            desired_dir = self.RIGHT_DIR
        elif dx == -1:
            desired_dir = self.LEFT_DIR
        elif dy == 1:
            desired_dir = self.DOWN
        else:
            desired_dir = self.UP

        current_dir = agent.dir.value if hasattr(agent.dir, "value") else agent.dir

        if current_dir == desired_dir:
            return self.FORWARD

        return self._turn_action(current_dir, desired_dir)

    # ------------------------------------------------------------------
    # Main entry point
    # ------------------------------------------------------------------

    def get_actions_for_pursuit(self, obs_batch, pursue_flags, nearest_task_per_robot):
        ua = self.env.unwrapped
        agents = ua.agents
        n_robots = len(agents)
        self._t += 1
        self._ensure_homes()

        actions = [self.NOOP] * n_robots
        positions = [(int(a.x), int(a.y)) for a in agents]
        shelf_cells = {(int(s.x), int(s.y)) for s in ua.shelfs}
        request_queue = list(ua.request_queue)
        goals = [(int(g[0]), int(g[1])) for g in ua.goals]

        # ---- update stuck counters from last step's intent ----
        for i in range(n_robots):
            moved = self._last_pos[i] != positions[i]
            if moved:
                self._side_target[i] = None
            if self._wanted_move[i] and not moved:
                self._stuck[i] += 1
            else:
                self._stuck[i] = 0
            self._last_pos[i] = positions[i]
            self._wanted_move[i] = False

        goals_per_robot = [None] * n_robots
        toggle_per_robot = [False] * n_robots

        for i, agent in enumerate(agents):
            carrying = agent.carrying_shelf
            pos = positions[i]

            if carrying is not None:
                if carrying in request_queue:
                    if goals:
                        goals_per_robot[i] = min(goals, key=lambda g: abs(g[0] - pos[0]) + abs(g[1] - pos[1]))
                        toggle_per_robot[i] = False
                else:
                    home = self.shelf_home.get(self._shelf_key(carrying))
                    if home is not None:
                        goals_per_robot[i] = home
                        toggle_per_robot[i] = True
            else:
                if pursue_flags[i] and nearest_task_per_robot[i] is not None:
                    task = nearest_task_per_robot[i]
                    goals_per_robot[i] = (int(task[0]), int(task[1]))
                    toggle_per_robot[i] = True

        # Priority order: (0) delivering, (1) returning shelf, (2) pursuing task, (3) idle
        def robot_priority(i):
            agent = agents[i]
            if agent.carrying_shelf is not None:
                if agent.carrying_shelf in request_queue:
                    return (0, i)
                return (1, i)
            if pursue_flags[i]:
                return (2, i)
            return (3, i)

        order = sorted(range(n_robots), key=robot_priority)
        proposed_next_cell = {}

        for i in order:
            agent = agents[i]
            pos = positions[i]
            goal = goals_per_robot[i]
            carrying = agent.carrying_shelf

            if goal is not None:
                if pos == goal:
                    if toggle_per_robot[i]:
                        actions[i] = self.TOGGLE_LOAD
                    continue

                self._wanted_move[i] = True
                static_blocked = (shelf_cells - {pos, goal}) if carrying is not None else set()

                dynamic_blocked = set()
                for j in range(n_robots):
                    if j == i:
                        continue
                    if j in proposed_next_cell:
                        dynamic_blocked.add(proposed_next_cell[j])
                    else:
                        dynamic_blocked.add(positions[j])

                # Stuck-breaker sidestep
                if self._stuck[i] >= self.STUCK_LIMIT:
                    free_neighbors = list(self._neighbors(pos, static_blocked | dynamic_blocked))
                    if free_neighbors:
                        tgt = self._side_target[i]
                        if tgt not in free_neighbors:
                            tgt = free_neighbors[(i + self._t) % len(free_neighbors)]
                            self._side_target[i] = tgt
                        proposed_next_cell[i] = tgt
                        continue

                path = self.astar(pos, goal, static_blocked | dynamic_blocked)
                if not path:
                    # Next cell is occupied: if blocked for multiple steps, sidestep into free neighbor
                    raw_path = self.astar(pos, goal, static_blocked)
                    if raw_path:
                        free_n = list(self._neighbors(pos, static_blocked | dynamic_blocked))
                        if free_n and self._stuck[i] >= 2:
                            proposed_next_cell[i] = free_n[0]
                else:
                    proposed_next_cell[i] = path[0]

            else:
                # Idle robot yielding: if in transit corridor (col 0, 3, 6, 9) and blocking someone
                is_blocking = any(target == pos for target in proposed_next_cell.values())
                in_corridor = pos[0] in [0, 3, 6, 9]

                if is_blocking or (in_corridor and self._t % 3 == 0):
                    free_n = list(self._neighbors(pos, set(positions) | set(proposed_next_cell.values())))
                    if free_n:
                        # Unloaded idle robots can step into shelf cells to clear the aisle!
                        shelf_neighbors = [c for c in free_n if c in shelf_cells]
                        if shelf_neighbors:
                            proposed_next_cell[i] = shelf_neighbors[0]
                        else:
                            proposed_next_cell[i] = free_n[0]

        # Same-cell conflicts: lower robot_id wins
        cell_claims = {}
        for i, cell in proposed_next_cell.items():
            if cell not in cell_claims or i < cell_claims[cell]:
                cell_claims[cell] = i

        for i, cell in proposed_next_cell.items():
            if cell_claims[cell] == i:
                actions[i] = self.next_cell_to_action(agents[i], cell)
            else:
                actions[i] = self.NOOP

        return actions
```

### 10.6 `core/auction_layer.py` (Baseline Auction - Non-Retargeting Ablation)

- **Purpose:** Flat decentralized auction (legacy baseline; losing robots do not retarget)
- **Relative Path:** `[core/auction_layer.py](file:///c:/Users/Kezia/warehouse-swarm/core/auction_layer.py)`
- **System Location:** `c:\Users\Kezia\warehouse-swarm\core\auction_layer.py`

### 10.6.1 `experiments/auction_layer_v2.py` (Canonical v2 Retargeting Auction)

- **Purpose:** Canonical final auction layer: descending bid-ordered assignment, receiver-side filtering, and instant sequential retargeting (2.8× throughput boost)
- **Relative Path:** `[experiments/auction_layer_v2.py](file:///c:/Users/Kezia/warehouse-swarm/experiments/auction_layer_v2.py)`
- **System Location:** `c:\Users\Kezia\warehouse-swarm\experiments\auction_layer_v2.py`

```python
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

    def step(self, obs_batch, belief_failed=None (bf=None)):
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
                    near = abs(agents[i].x - agents[j].x) + abs(agents[i].y - agents[j].y) <= self.COMM_RANGE
                    believed_failed = belief_failed is not None and bool(belief_failed[i][j])
                    if near and not believed_failed:
                        blocked = True
                        break
                if not blocked:
                    chosen[i] = task
                    claims.setdefault(task, []).append(i)
                    break

        pursue = [c is not None for c in chosen]
        tasks = [c if c is not None else (ranked[i][0][1] if ranked[i] else None)
                 for i, c in enumerate(chosen)]
        return pursue, bids, tasks

```

---

### 10.6.2 `core/auction_layer.py` (Legacy Non-Retargeting Source)


- **Purpose:** Flat decentralized peer-to-peer auction and claim manager
- **Relative Path:** `[core/auction_layer.py](file:///c:/Users/Kezia/warehouse-swarm/core/auction_layer.py)`
- **System Location:** `c:\Users\Kezia\warehouse-swarm\core\auction_layer.py`

```python
"""
Stage 2 of Phase 4 (bidding + claims) + eligibility fix + Phase 6 Stage 2
integration: stale claims from robots believed to have failed are cleared,
freeing up their claimed tasks for other robots to bid on again.
"""

import os
from pathlib import Path
import numpy as np
import torch
from policy_network import CapabilityConditionedPolicy


def _resolve_model_path(path: str) -> str:
    if os.path.exists(path):
        return path
    # Check core/ folder
    core_candidate = Path(__file__).resolve().parent / path
    if core_candidate.exists():
        return str(core_candidate)
    # Check models_and_data/ folder
    models_candidate = Path(__file__).resolve().parent.parent / "models_and_data" / path
    if models_candidate.exists():
        return str(models_candidate)
    return path


class DecentralizedAuctionLayer:
    COMM_RANGE = 8

    def __init__(self, hetero_env, bid_model_path="bid_head_trained.pt"):
        self.env = hetero_env
        self.n_robots = hetero_env.unwrapped.n_agents

        self.bid_model = CapabilityConditionedPolicy(obs_dim=80)
        resolved_path = _resolve_model_path(bid_model_path)
        self.bid_model.load_state_dict(torch.load(resolved_path, map_location="cpu"))
        self.bid_model.eval()

        self.belief = [dict() for _ in range(self.n_robots)]
        self.known_claims = [dict() for _ in range(self.n_robots)]

        self.bid_broadcasts_made = 0
        self.bid_broadcasts_useful = 0
        self.claim_broadcasts_made = 0
        self.claim_broadcasts_useful = 0

        # Phase 6 additions
        self.stale_claims_cleared = 0
        self.stale_beliefs_cleared = 0

    def _task_id(self, shelf):
        return (shelf.x, shelf.y)

    def _distance(self, a, b):
        return abs(a.x - b.x) + abs(a.y - b.y)

    def compute_bids(self, obs_batch):
        bids = []
        with torch.no_grad():
            for i in range(self.n_robots):
                obs_tensor = torch.tensor(obs_batch[i], dtype=torch.float32).unsqueeze(0)
                bid_value, _ = self.bid_model(obs_tensor)
                bids.append(bid_value.item())
        return bids

    def _nearest_eligible_task(self, agent, robot_type):
        queue = self.env.unwrapped.request_queue
        if not queue:
            return None

        best_dist, best_shelf = None, None
        for shelf in queue:
            task_type = self.env.shelf_task_type.get(shelf, 2)
            if robot_type not in self.env.ACCEPTABLE_ROBOTS[task_type]:
                continue
            d = self._distance(agent, shelf)
            if best_dist is None or d < best_dist:
                best_dist, best_shelf = d, shelf

        if best_shelf is None:
            return None
        return self._task_id(best_shelf)

    def clear_stale_claims_from_failed_robots(self, belief_failed):
        """
        Phase 6 Stage 2: for every robot j, drop any belief/claim entry that
        was made by a robot j currently believes has failed. This frees up
        tasks previously claimed/bid-won by a now-failed robot, so others
        can bid on them again. belief_failed is the (n_robots x n_robots)
        matrix from FaultLayer.step(): belief_failed[j][k] == True means
        robot j believes robot k has failed.
        """
        if belief_failed is None:
            return

        for j in range(self.n_robots):
            # clear stale entries in belief table (bid winners)
            stale_tasks_belief = [
                task for task, (bid_val, winner) in self.belief[j].items()
                if winner != j and belief_failed[j][winner]
            ]
            for task in stale_tasks_belief:
                del self.belief[j][task]
                self.stale_beliefs_cleared += 1

            # clear stale entries in known_claims (claimed tasks)
            stale_tasks_claims = [
                task for task, claimer in self.known_claims[j].items()
                if claimer != j and belief_failed[j][claimer]
            ]
            for task in stale_tasks_claims:
                del self.known_claims[j][task]
                self.stale_claims_cleared += 1

    def step(self, obs_batch, belief_failed=None (bf=None)):
        """
        belief_failed: optional (n_robots x n_robots) matrix from FaultLayer.
        If provided, stale claims/beliefs from failed robots are cleared
        before this step's auction logic runs.
        """
        if belief_failed is not None:
            self.clear_stale_claims_from_failed_robots(belief_failed)

        agents = self.env.unwrapped.agents
        bids = self.compute_bids(obs_batch)

        nearest_task_per_robot = [
            self._nearest_eligible_task(agents[i], self.env.robot_types[i])
            for i in range(self.n_robots)
        ]

        # skip bidding/pursuing entirely for robots believed to have failed
        # by themselves is meaningless (a failed robot doesn't act), but we
        # also should not let a failed robot's OWN bid count if it's still
        # being computed -- handled naturally since failed robots get NOOP
        # actions from FaultLayer regardless of what's decided here.

        for i, agent_i in enumerate(agents):
            task_i = nearest_task_per_robot[i]
            if task_i is None:
                continue

            self.bid_broadcasts_made += 1
            changed_someone = False

            for j, agent_j in enumerate(agents):
                if i == j or self._distance(agent_i, agent_j) > self.COMM_RANGE:
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

        for i, agent_i in enumerate(agents):
            task_i = nearest_task_per_robot[i]
            if task_i is None or not provisional_pursue[i]:
                continue

            self.claim_broadcasts_made += 1
            changed_someone = False

            for j, agent_j in enumerate(agents):
                if i == j or self._distance(agent_i, agent_j) > self.COMM_RANGE:
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

    def clear_stale_beliefs(self):
        active_tasks = set(self._task_id(s) for s in self.env.unwrapped.request_queue)
        for i in range(self.n_robots):
            self.belief[i] = {t: v for t, v in self.belief[i].items() if t in active_tasks}
            self.known_claims[i] = {t: v for t, v in self.known_claims[i].items() if t in active_tasks}

    def report_stats(self):
        bid_rate = (self.bid_broadcasts_useful / self.bid_broadcasts_made * 100) if self.bid_broadcasts_made else 0
        claim_rate = (self.claim_broadcasts_useful / self.claim_broadcasts_made * 100) if self.claim_broadcasts_made else 0
        print(f"Bid broadcasts made: {self.bid_broadcasts_made}, "
              f"changed a receiver's belief: {self.bid_broadcasts_useful} ({bid_rate:.1f}%)")
        print(f"Claim broadcasts made: {self.claim_broadcasts_made}, "
              f"prevented a would-be conflict: {self.claim_broadcasts_useful} ({claim_rate:.1f}%)")
        print(f"Stale beliefs cleared (from failed robots): {self.stale_beliefs_cleared}")
        print(f"Stale claims cleared (from failed robots): {self.stale_claims_cleared}")
```

### 10.7 `core/fault_layer.py`

- **Purpose:** Flat heartbeat-based liveness monitoring and failure injector
- **Relative Path:** `[core/fault_layer.py](file:///c:/Users/Kezia/warehouse-swarm/core/fault_layer.py)`
- **System Location:** `c:\Users\Kezia\warehouse-swarm\core\fault_layer.py`

```python
"""
Phase 6, Stage 1: heartbeat-based failure detection.

Each active robot 'broadcasts' a heartbeat every step to robots within
COMM_RANGE (reusing the same range concept as the auction layer). If a
robot hasn't been heard from in HEARTBEAT_TIMEOUT steps, other robots
mark it as failed in their local belief -- no central registry, purely
local/decentralized, consistent with the rest of the system.

Stage 1 only detects failures; it does not yet reallocate stale claims
(that's Stage 2).
"""

import numpy as np


class FaultLayer:
    COMM_RANGE = 20
    HEARTBEAT_TIMEOUT = 15  # steps without a heartbeat before considered failed

    def __init__(self, hetero_env):
        self.env = hetero_env
        self.n_robots = hetero_env.unwrapped.n_agents

        # ground truth: which robots are actually failed (for evaluation only,
        # not something other robots have direct access to)
        self.actual_failed = [False] * self.n_robots

        # last_heard[i][j] = steps since robot i last heard a heartbeat from j
        self.steps_since_heard = [[0] * self.n_robots for _ in range(self.n_robots)]

        # belief_failed[i][j] = True if robot i currently believes robot j has failed
        self.belief_failed = [[False] * self.n_robots for _ in range(self.n_robots)]

    def _distance(self, a, b):
        return abs(a.x - b.x) + abs(a.y - b.y)

    def inject_failure(self, robot_id):
        """Mark a robot as failed. It will stop broadcasting heartbeats from
        this point on. Call this once, at whatever step you want the failure
        to occur."""
        self.actual_failed[robot_id] = True
        print(f"[FaultLayer] Robot {robot_id} has FAILED at this step.")

    def step(self):
        """
        Call once per environment step. Active (non-failed) robots broadcast
        heartbeats to robots within range. Updates each robot's belief about
        which other robots have failed, based on heartbeat timeout.

        Returns: belief_failed (n_robots x n_robots bool matrix) -- what each
        robot currently believes about every other robot's status.
        """
        agents = self.env.unwrapped.agents

        # every robot's "steps since heard" ticks up by 1 by default
        for i in range(self.n_robots):
            for j in range(self.n_robots):
                if i != j:
                    self.steps_since_heard[i][j] += 1

        # active robots broadcast heartbeat to everyone in range -> resets their counter
        for i, agent_i in enumerate(agents):
            if self.actual_failed[i]:
                continue  # failed robots don't broadcast

            for j, agent_j in enumerate(agents):
                if i == j:
                    continue
                if self._distance(agent_i, agent_j) <= self.COMM_RANGE:
                    self.steps_since_heard[j][i] = 0

        # update belief based on timeout
        for i in range(self.n_robots):
            for j in range(self.n_robots):
                if i == j:
                    continue
                self.belief_failed[i][j] = self.steps_since_heard[i][j] > self.HEARTBEAT_TIMEOUT

        return self.belief_failed

    def get_actions_override(self, actions):
        """Force NOOP for any robot that has actually failed, regardless of
        what the rest of the pipeline decided. Call this right before
        vec_env.step()."""
        actions = list(actions)
        for i in range(self.n_robots):
            if self.actual_failed[i]:
                actions[i] = 0  # NOOP
        return actions

    def detection_accuracy_report(self):
        """Compare belief_failed against actual_failed for a quick sanity check."""
        print("\n=== Fault Detection Accuracy ===")
        for i in range(self.n_robots):
            beliefs_about_i = [self.belief_failed[j][i] for j in range(self.n_robots) if j != i]
            correctly_detected = all(b == self.actual_failed[i] for b in beliefs_about_i) if beliefs_about_i else None
            print(f"Robot {i}: actually_failed={self.actual_failed[i]}, "
                  f"detected as failed by others: {beliefs_about_i}")
```

### 10.8 `core/zone_partitioner.py`

- **Purpose:** 2D spatial grid partitioner and centroid calculator
- **Relative Path:** `[core/zone_partitioner.py](file:///c:/Users/Kezia/warehouse-swarm/core/zone_partitioner.py)`
- **System Location:** `c:\Users\Kezia\warehouse-swarm\core\zone_partitioner.py`

```python
"""
Phase 8: Zone Partitioner

Divides the warehouse grid into Z spatial zones and provides:
  - Zone assignment for any (x, y) cell
  - Zone centroids (used by HierarchicalCoordinator for task routing)
  - Robot-to-zone assignment at startup (fixed for the run)
  - Sorted nearest-zone list for a given task location (for handoff fallback)

Partitioning strategy: simple quadrant/rectangular split — no external
dependencies (no scipy/sklearn needed). For Z=4 this gives a 2x2 grid of
zones; for Z=2 it splits horizontally. Scales to any Z by tiling in column-
major order (fill column by column).

This module is intentionally stateless and lightweight — just spatial math.
Nothing in this file touches the RWARE env, the auction layer, or the
pathfinder. It is imported by zone_auction_layer, zone_fault_layer, and
hierarchical_coordinator.
"""

import math
from typing import List, Tuple, Dict


class ZonePartitioner:
    """
    Rectangular spatial partitioner for a warehouse grid.

    Parameters
    ----------
    grid_width  : int  — number of columns in the grid (x-axis)
    grid_height : int  — number of rows in the grid (y-axis)
    n_zones     : int  — number of zones to create (default 4)

    Zone layout (column-major, left-to-right then top-to-bottom):
        For n_zones=4, grid split into 2 columns × 2 rows:
            Zone 0 | Zone 2
            -------+-------
            Zone 1 | Zone 3
    """

    def __init__(self, grid_width: int, grid_height: int, n_zones: int = 4):
        self.grid_width = grid_width
        self.grid_height = grid_height
        self.n_zones = n_zones

        # Determine grid of zone columns and rows
        # e.g. n_zones=4 -> n_cols=2, n_rows=2
        #      n_zones=5 -> n_cols=3, n_rows=2 (one col has fewer rows)
        self.n_cols = math.ceil(math.sqrt(n_zones))
        self.n_rows = math.ceil(n_zones / self.n_cols)

        # Width/height of each zone cell (last zone in each axis may be smaller)
        self.zone_col_width = grid_width / self.n_cols
        self.zone_row_height = grid_height / self.n_rows

        # Precompute centroids for all zones
        self._centroids: List[Tuple[float, float]] = []
        for z in range(n_zones):
            col = z // self.n_rows
            row = z % self.n_rows
            cx = (col + 0.5) * self.zone_col_width
            cy = (row + 0.5) * self.zone_row_height
            self._centroids.append((cx, cy))

    # ------------------------------------------------------------------
    # Core spatial queries
    # ------------------------------------------------------------------

    def get_zone(self, x: int, y: int) -> int:
        """
        Return the zone ID that cell (x, y) belongs to.
        Clamps to valid range so boundary cells always get a valid zone.
        """
        col = min(int(x / self.zone_col_width), self.n_cols - 1)
        row = min(int(y / self.zone_row_height), self.n_rows - 1)
        zone_id = col * self.n_rows + row
        # Clamp in case n_zones is not a perfect rectangle
        return min(zone_id, self.n_zones - 1)

    def get_centroid(self, zone_id: int) -> Tuple[float, float]:
        """Return the (x, y) centroid of the given zone."""
        return self._centroids[zone_id]

    def nearest_zones_to_task(self, task_x: float, task_y: float) -> List[int]:
        """
        Return all zone IDs sorted by Manhattan distance from (task_x, task_y)
        to each zone's centroid. Used by HierarchicalCoordinator to route a
        task (and find fallback zones for cross-zone handoff).
        """
        dists = []
        for z in range(self.n_zones):
            cx, cy = self._centroids[z]
            d = abs(task_x - cx) + abs(task_y - cy)
            dists.append((d, z))
        dists.sort()
        return [z for _, z in dists]

    # ------------------------------------------------------------------
    # Robot assignment
    # ------------------------------------------------------------------

    def assign_robots_to_zones(self, agents) -> Dict[int, int]:
        """
        Assign each robot (by its current position) to the zone it physically
        starts in. Assignment is fixed at call-time — robots do not migrate
        between zones during a run.

        Parameters
        ----------
        agents : list of RWARE agent objects (must have .x and .y attributes)

        Returns
        -------
        dict mapping robot_index -> zone_id
        """
        assignment = {}
        for i, agent in enumerate(agents):
            assignment[i] = self.get_zone(agent.x, agent.y)
        return assignment

    # ------------------------------------------------------------------
    # Diagnostics
    # ------------------------------------------------------------------

    def print_zone_map(self):
        """Print an ASCII map of zone assignments for visual inspection."""
        print(f"Zone map ({self.grid_width}w x {self.grid_height}h, "
              f"{self.n_zones} zones, {self.n_cols}col x {self.n_rows}row):")
        for y in range(self.grid_height):
            row_str = ""
            for x in range(self.grid_width):
                row_str += str(self.get_zone(x, y))
            print(f"  {row_str}")
        print("Centroids:")
        for z, (cx, cy) in enumerate(self._centroids):
            print(f"  Zone {z}: centroid=({cx:.1f}, {cy:.1f})")

    def report_robot_assignment(self, assignment: Dict[int, int]):
        """Print the robot-to-zone assignment."""
        from collections import defaultdict
        by_zone = defaultdict(list)
        for robot_id, zone_id in assignment.items():
            by_zone[zone_id].append(robot_id)
        print("Robot-to-zone assignment:")
        for z in range(self.n_zones):
            print(f"  Zone {z}: robots {by_zone[z]}")

```

### 10.9 `core/zone_auction_layer.py`

- **Purpose:** Zone-scoped decentralized auction engine with task and peer filtering
- **Relative Path:** `[core/zone_auction_layer.py](file:///c:/Users/Kezia/warehouse-swarm/core/zone_auction_layer.py)`
- **System Location:** `c:\Users\Kezia\warehouse-swarm\core\zone_auction_layer.py`

```python
"""
Phase 8 (patched): Zone-Aware Auction Layer

Changes over the previous version:
  1. Task ownership comes from the coordinator's routing map
     (task_zone_map), so cross-zone handoffs really change who may bid.
     If a task is not in the map, the spatial zone is used.
  2. Robots that are carrying a shelf do not bid (they are busy), and
     shelves being carried are not open tasks for anyone else.
  3. Failed robots are SILENT: they send no bids or claims. (Before, a
     dead robot kept re-asserting its claims every step, undoing the
     stale-claim clearing.) Shelves under a dead robot are skipped.
  4. Bids are computed in one batched forward pass; the coordinator can
     pass pre-computed bids so this is done once per step, not per zone.
  5. Bid timeout now fires when NO LIVE ROBOT IN THE ZONE IS CAPABLE of a
     task (e.g. an H task in a zone without a heavy_load robot), instead
     of when no robot happened to currently bid on it.
  6. reset_episode() clears all tables when RWARE resets the world.
"""

import os
from pathlib import Path
import numpy as np
import torch
from policy_network import CapabilityConditionedPolicy


def _resolve_model_path(path: str) -> str:
    if os.path.exists(path):
        return path
    core_candidate = Path(__file__).resolve().parent / path
    if core_candidate.exists():
        return str(core_candidate)
    models_candidate = Path(__file__).resolve().parent.parent / "models_and_data" / path
    if models_candidate.exists():
        return str(models_candidate)
    return path


class ZoneAuctionLayer:
    """
    Zone-scoped decentralized auction layer.

    Parameters
    ----------
    hetero_env       : HeterogeneousWarehouseWrapper
    zone_id          : int   - which zone this auction instance manages
    zone_assignment  : dict  - {robot_id -> zone_id} from ZonePartitioner
    zone_partitioner : ZonePartitioner
    bid_model_path   : str   - path to trained bid head weights
    task_zone_map    : dict  - {task_id -> owning zone_id}, shared with and
                               mutated in place by HierarchicalCoordinator
    """

    COMM_RANGE = 8
    BID_TIMEOUT = 10  # steps a task may be uncovered before it is handed off

    def __init__(self, hetero_env, zone_id: int, zone_assignment: dict,
                 zone_partitioner, bid_model_path="bid_head_trained.pt",
                 task_zone_map=None):
        self.env = hetero_env
        self.zone_id = zone_id
        self.zone_assignment = zone_assignment
        self.partitioner = zone_partitioner
        self.task_zone_map = task_zone_map if task_zone_map is not None else {}

        self.n_robots = hetero_env.unwrapped.n_agents
        self.zone_robots = [i for i, z in zone_assignment.items() if z == zone_id]

        self.bid_model = CapabilityConditionedPolicy(obs_dim=80)
        resolved_path = _resolve_model_path(bid_model_path)
        self.bid_model.load_state_dict(torch.load(resolved_path, map_location="cpu"))
        self.bid_model.eval()

        self.belief = [dict() for _ in range(self.n_robots)]
        self.known_claims = [dict() for _ in range(self.n_robots)]

        # --- stats ---
        self.bid_broadcasts_made = 0
        self.bid_broadcasts_useful = 0
        self.claim_broadcasts_made = 0
        self.claim_broadcasts_useful = 0
        self.stale_claims_cleared = 0
        self.stale_beliefs_cleared = 0
        self.comm_events_per_step: list = []

        # task_id -> consecutive steps the task has had no capable robot here
        self._no_bid_counter: dict = {}
        self.timed_out_tasks: list = []

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _task_id(self, shelf):
        return (int(shelf.x), int(shelf.y))

    def _distance(self, a, b):
        return abs(int(a.x) - int(b.x)) + abs(int(a.y) - int(b.y))

    def _task_in_zone(self, shelf) -> bool:
        """True if this zone currently OWNS the task (routing map first,
        spatial zone as fallback)."""
        owner = self.task_zone_map.get(self._task_id(shelf))
        if owner is None:
            owner = self.partitioner.get_zone(int(shelf.x), int(shelf.y))
        return owner == self.zone_id

    def _carried_shelves(self):
        return {a.carrying_shelf for a in self.env.unwrapped.agents
                if a.carrying_shelf is not None}

    def _nearest_eligible_task(self, agent, robot_type, carried, dead_cells, excluded=None):
        """
        Nearest open task that:
          - this zone owns,
          - is not currently being carried by someone,
          - does not sit under a dead robot,
          - this robot type is eligible for,
          - is not already in the excluded set (e.g. chosen by a higher-priority peer).
        Busy (carrying) robots do not bid at all.
        """
        if agent.carrying_shelf is not None:
            return None

        queue = self.env.unwrapped.request_queue
        best_dist, best_shelf = None, None
        for shelf in queue:
            tid = self._task_id(shelf)
            if excluded and tid in excluded:
                continue
            if shelf in carried:
                continue
            if (int(shelf.x), int(shelf.y)) in dead_cells:
                continue
            if not self._task_in_zone(shelf):
                continue
            task_type = self.env.shelf_task_type.get(shelf, 2)
            if robot_type not in self.env.ACCEPTABLE_ROBOTS[task_type]:
                continue
            d = self._distance(agent, shelf)
            if best_dist is None or d < best_dist:
                best_dist, best_shelf = d, shelf

        if best_shelf is None:
            return None
        return self._task_id(best_shelf)

    def compute_bids(self, obs_batch):
        """One batched forward pass for all robots."""
        with torch.no_grad():
            x = torch.as_tensor(np.asarray(obs_batch, dtype=np.float32))
            bid_value, _ = self.bid_model(x)
        return bid_value.squeeze(1).tolist()

    # ------------------------------------------------------------------
    # Stale-claim clearing
    # ------------------------------------------------------------------

    def clear_stale_claims_from_failed_robots(self, belief_failed):
        if belief_failed is None:
            return
        for j in self.zone_robots:
            stale_belief = [
                task for task, (bid_val, winner) in self.belief[j].items()
                if winner != j and belief_failed[j][winner]
            ]
            for task in stale_belief:
                del self.belief[j][task]
                self.stale_beliefs_cleared += 1

            stale_claims = [
                task for task, claimer in self.known_claims[j].items()
                if claimer != j and belief_failed[j][claimer]
            ]
            for task in stale_claims:
                del self.known_claims[j][task]
                self.stale_claims_cleared += 1

    def clear_stale_beliefs(self):
        """Remove belief/claim entries for tasks no longer open."""
        carried = self._carried_shelves()
        active_tasks = {
            self._task_id(s) for s in self.env.unwrapped.request_queue
            if s not in carried
        }
        for i in self.zone_robots:
            self.belief[i] = {
                t: v for t, v in self.belief[i].items() if t in active_tasks
            }
            self.known_claims[i] = {
                t: v for t, v in self.known_claims[i].items() if t in active_tasks
            }

    def reset_episode(self):
        """Call when RWARE resets the world."""
        self.belief = [dict() for _ in range(self.n_robots)]
        self.known_claims = [dict() for _ in range(self.n_robots)]
        self._no_bid_counter = {}
        self.timed_out_tasks = []

    # ------------------------------------------------------------------
    # Bid-timeout tracking for cross-zone handoff
    # ------------------------------------------------------------------

    def _update_bid_timeout(self, zone_task_ids: set, covered_task_ids: set):
        """
        zone_task_ids    : open tasks currently owned by this zone
        covered_task_ids : subset that at least one LIVE zone robot is
                           capable of serving
        A task that stays uncovered for BID_TIMEOUT steps is emitted into
        self.timed_out_tasks for the coordinator to hand off.
        """
        self.timed_out_tasks = []

        for tid in zone_task_ids:
            if tid in covered_task_ids:
                self._no_bid_counter[tid] = 0
            else:
                self._no_bid_counter[tid] = self._no_bid_counter.get(tid, 0) + 1

        for tid, count in list(self._no_bid_counter.items()):
            if count >= self.BID_TIMEOUT:
                self.timed_out_tasks.append(tid)
                del self._no_bid_counter[tid]

        for tid in list(self._no_bid_counter.keys()):
            if tid not in zone_task_ids:
                del self._no_bid_counter[tid]

    # ------------------------------------------------------------------
    # Main step
    # ------------------------------------------------------------------

    def step(self, obs_batch, belief_failed=None (bf=None), bids=None, failed=None):
        """
        Run one auction step for this zone's robots.

        Parameters
        ----------
        obs_batch    : np.ndarray (n_robots, obs_dim)
        belief_failed: optional (n_robots x n_robots) bool matrix
        bids         : optional pre-computed bids (list of n_robots floats)
        failed       : optional list[bool] of ACTUALLY failed robots; they
                       send no messages and do not bid

        Returns
        -------
        pursue, bids, nearest_tasks  (all length n_robots)
        """
        n = self.n_robots
        failed = list(failed) if failed is not None else [False] * n

        if belief_failed is not None:
            self.clear_stale_claims_from_failed_robots(belief_failed)

        ua = self.env.unwrapped
        agents = ua.agents
        if bids is None:
            bids = self.compute_bids(obs_batch)

        active = [i for i in self.zone_robots if not failed[i]]
        carried = self._carried_shelves()
        dead_cells = {(int(agents[i].x), int(agents[i].y))
                      for i in range(n) if failed[i]}

        nearest_task_per_robot = [None] * n
        selected_tasks = set()
        # Sort by bid priority so highest bidder gets first choice of distinct tasks
        sorted_active = sorted(active, key=lambda idx: bids[idx], reverse=True)
        for i in sorted_active:
            task = self._nearest_eligible_task(
                agents[i], self.env.robot_types[i], carried, dead_cells,
                excluded=selected_tasks
            )
            nearest_task_per_robot[i] = task
            if task is not None:
                selected_tasks.add(task)

        # ---- Phase 1: bid broadcasting (zone-scoped, live robots only) ----
        step_comm_events = 0

        for i in active:
            task_i = nearest_task_per_robot[i]
            if task_i is None:
                continue

            self.bid_broadcasts_made += 1
            changed_someone = False

            for j in active:
                if i == j or self._distance(agents[i], agents[j]) > self.COMM_RANGE:
                    continue

                step_comm_events += 1
                current_best = self.belief[j].get(task_i)
                if current_best is None or bids[i] > current_best[0]:
                    old_winner = current_best[1] if current_best else None
                    self.belief[j][task_i] = (bids[i], i)
                    if old_winner != i:
                        changed_someone = True

            if changed_someone:
                self.bid_broadcasts_useful += 1

        for i in active:
            task_i = nearest_task_per_robot[i]
            if task_i is None:
                continue
            current_best = self.belief[i].get(task_i)
            if current_best is None or bids[i] > current_best[0]:
                self.belief[i][task_i] = (bids[i], i)

        # ---- Phase 2: provisional pursue ----
        provisional_pursue = [False] * n
        for i in active:
            task_i = nearest_task_per_robot[i]
            if task_i is None:
                continue
            best_bid, best_robot = self.belief[i][task_i]
            provisional_pursue[i] = (best_robot == i)

        # ---- Phase 3: claim broadcasting ----
        for i in active:
            task_i = nearest_task_per_robot[i]
            if task_i is None or not provisional_pursue[i]:
                continue

            self.claim_broadcasts_made += 1
            changed_someone = False

            for j in active:
                if i == j or self._distance(agents[i], agents[j]) > self.COMM_RANGE:
                    continue

                step_comm_events += 1
                would_have_pursued = (
                    nearest_task_per_robot[j] == task_i and provisional_pursue[j]
                )
                self.known_claims[j][task_i] = i
                if would_have_pursued and j != i:
                    changed_someone = True

            if changed_someone:
                self.claim_broadcasts_useful += 1

        for i in active:
            if provisional_pursue[i] and nearest_task_per_robot[i] is not None:
                self.known_claims[i][nearest_task_per_robot[i]] = i

        # ---- Phase 4: final pursue (claim-resolved) ----
        final_pursue = [False] * n
        for i in active:
            task_i = nearest_task_per_robot[i]
            if task_i is None:
                continue
            claimer = self.known_claims[i].get(task_i)
            if claimer is not None and claimer != i:
                final_pursue[i] = False
            else:
                final_pursue[i] = provisional_pursue[i]

        self.comm_events_per_step.append(step_comm_events)

        # ---- Handoff bookkeeping: which owned tasks have a capable live robot? ----
        zone_tasks = {}
        for s in ua.request_queue:
            if s in carried or not self._task_in_zone(s):
                continue
            zone_tasks[self._task_id(s)] = self.env.shelf_task_type.get(s, 2)

        covered = {
            tid for tid, tt in zone_tasks.items()
            if any(self.env.robot_types[i] in self.env.ACCEPTABLE_ROBOTS[tt]
                   for i in active)
        }
        self._update_bid_timeout(set(zone_tasks.keys()), covered)

        return final_pursue, bids, nearest_task_per_robot

    # ------------------------------------------------------------------
    # Reporting
    # ------------------------------------------------------------------

    def avg_comm_per_step(self) -> float:
        """Mean intra-zone comm events per step (whole zone)."""
        if not self.comm_events_per_step:
            return 0.0
        return sum(self.comm_events_per_step) / len(self.comm_events_per_step)

    def avg_comm_per_robot_per_step(self) -> float:
        n = len(self.zone_robots)
        if n == 0:
            return 0.0
        return self.avg_comm_per_step() / n

    def report_stats(self):
        bid_rate = (
            self.bid_broadcasts_useful / self.bid_broadcasts_made * 100
            if self.bid_broadcasts_made else 0
        )
        claim_rate = (
            self.claim_broadcasts_useful / self.claim_broadcasts_made * 100
            if self.claim_broadcasts_made else 0
        )
        print(f"[Zone {self.zone_id}] Robots: {self.zone_robots}")
        print(f"  Bid broadcasts: {self.bid_broadcasts_made} "
              f"({self.bid_broadcasts_useful} useful, {bid_rate:.1f}%)")
        print(f"  Claim broadcasts: {self.claim_broadcasts_made} "
              f"({self.claim_broadcasts_useful} useful, {claim_rate:.1f}%)")
        print(f"  Stale beliefs cleared: {self.stale_beliefs_cleared}, "
              f"stale claims cleared: {self.stale_claims_cleared}")
        print(f"  Avg comm events/step: {self.avg_comm_per_step():.3f} "
              f"({self.avg_comm_per_robot_per_step():.3f} per robot)")
```

### 10.10 `core/zone_fault_layer.py`

- **Purpose:** Zone-scoped heartbeat liveness monitoring
- **Relative Path:** `[core/zone_fault_layer.py](file:///c:/Users/Kezia/warehouse-swarm/core/zone_fault_layer.py)`
- **System Location:** `c:\Users\Kezia\warehouse-swarm\core\zone_fault_layer.py`

```python
"""
Phase 8: Zone-Scoped Fault Layer

A thin wrapper around FaultLayer that restricts heartbeat monitoring to
robots within the same zone. The only change from the base FaultLayer is
that the inner loop in step() skips any robot that lives in a different zone
-- consistent with the hierarchical design where liveness-checking is a
local, zone-internal concern.

All other behaviour is identical: HEARTBEAT_TIMEOUT, belief_failed matrix
structure, inject_failure(), get_actions_override(), and
detection_accuracy_report() are all unchanged. This means the rest of the
pipeline (auction layer, pathfinder) can consume the output of this class
interchangeably with FaultLayer.
"""

from fault_layer import FaultLayer


class ZoneFaultLayer(FaultLayer):
    """
    Heartbeat-based failure detection scoped to intra-zone robots only.

    Parameters
    ----------
    hetero_env      : HeterogeneousWarehouseWrapper
    zone_assignment : dict {robot_id -> zone_id}  (from ZonePartitioner)
    """

    # Use a tighter comm range for heartbeats within a zone — since we no
    # longer need to span the whole warehouse, use the same range as the
    # auction layer (8 cells). Robots in the same zone will always be within
    # this range given the grid sizes we test (tiny=11x10, small=20x10).
    # Override to 20 if you observe false positives (same logic that fixed
    # the original FaultLayer).
    COMM_RANGE = 20  # kept generous; zone filter is the primary restriction

    def __init__(self, hetero_env, zone_assignment: dict):
        super().__init__(hetero_env)
        self.zone_assignment = zone_assignment  # robot_id -> zone_id

    def step(self):
        """
        Identical to FaultLayer.step() except the heartbeat broadcast loop
        additionally skips robot pairs that belong to different zones.

        Returns: belief_failed (n_robots x n_robots bool matrix)
        """
        agents = self.env.unwrapped.agents

        # Every robot's "steps since heard" ticks up by 1
        for i in range(self.n_robots):
            for j in range(self.n_robots):
                if i != j:
                    self.steps_since_heard[i][j] += 1

        # Active robots broadcast heartbeat — only to same-zone robots within range
        for i, agent_i in enumerate(agents):
            if self.actual_failed[i]:
                continue  # failed robots don't broadcast

            for j, agent_j in enumerate(agents):
                if i == j:
                    continue

                # === Zone filter: only monitor robots in the same zone ===
                if self.zone_assignment.get(i) != self.zone_assignment.get(j):
                    # Different zone — never update the counter; these robots
                    # should not be tracking each other's liveness at all.
                    # Reset to 0 so they never trip the timeout for each other.
                    self.steps_since_heard[j][i] = 0
                    continue

                if self._distance(agent_i, agent_j) <= self.COMM_RANGE:
                    self.steps_since_heard[j][i] = 0

        # Update belief based on timeout
        for i in range(self.n_robots):
            for j in range(self.n_robots):
                if i == j:
                    continue
                # Cross-zone robots are never considered "failed" by each other
                if self.zone_assignment.get(i) != self.zone_assignment.get(j):
                    self.belief_failed[i][j] = False
                    continue
                self.belief_failed[i][j] = (
                    self.steps_since_heard[i][j] > self.HEARTBEAT_TIMEOUT
                )

        return self.belief_failed

```

### 10.11 `core/hierarchical_coordinator.py`

- **Purpose:** Top-level hierarchical coordinator, task routing, and cross-zone handoffs
- **Relative Path:** `[core/hierarchical_coordinator.py](file:///c:/Users/Kezia/warehouse-swarm/core/hierarchical_coordinator.py)`
- **System Location:** `c:\Users\Kezia\warehouse-swarm\core\hierarchical_coordinator.py`

```python
"""
Phase 8 (patched): Hierarchical Coordinator

Changes over the previous version:
  1. The routing map (_task_zone) is shared with every ZoneAuctionLayer, so
     cross-zone handoffs actually move a task to another zone's auction.
  2. Carried shelves are not treated as new tasks (their position changes
     every step, which used to inflate total_tasks_seen).
  3. Bids are computed once per step and shared by all zones; actual
     failures are passed to the auctions so dead robots stay silent.
  4. Episode resets are handled: reset_episode() clears routing, claims and
     pathfinder state. step() also auto-detects a RWARE reset.
  5. avg_comm_per_robot_per_step() fixed: total events per step across all
     zones divided by fleet size (it was multiplied by zone size before).
  6. inject_failure() prints once.
"""

from collections import defaultdict
from zone_auction_layer import ZoneAuctionLayer
from zone_fault_layer import ZoneFaultLayer
from pathfinding import GridPathfinder
from zone_partitioner import ZonePartitioner


class HierarchicalCoordinator:
    def __init__(self, hetero_env, partitioner: ZonePartitioner,
                 zone_assignment: dict, bid_model_path="bid_head_trained.pt"):
        self.env = hetero_env
        self.partitioner = partitioner
        self.zone_assignment = zone_assignment
        self.n_zones = partitioner.n_zones
        self.n_robots = hetero_env.unwrapped.n_agents

        # --- Task routing state (must exist BEFORE the auctions are built) ---
        # task_id -> zone_id currently responsible; shared, mutated in place only
        self._task_zone: dict = {}
        self._task_tried_zones: dict = defaultdict(set)

        self.auctions = [
            ZoneAuctionLayer(
                hetero_env, zone_id=z,
                zone_assignment=zone_assignment,
                zone_partitioner=partitioner,
                bid_model_path=bid_model_path,
                task_zone_map=self._task_zone,
            )
            for z in range(self.n_zones)
        ]

        self.fault_layers = [
            ZoneFaultLayer(hetero_env, zone_assignment=zone_assignment)
            for _ in range(self.n_zones)
        ]

        self.pathfinder = GridPathfinder(hetero_env)
        self.stranded_tasks: dict = {}

        # --- Stats ---
        self.cross_zone_handoffs = 0
        self.tasks_routed_by_zone = defaultdict(int)
        self.total_tasks_seen = 0

        self._last_cur_steps = None

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _carried_shelves(self):
        return {a.carrying_shelf for a in self.env.unwrapped.agents
                if a.carrying_shelf is not None}

    def _open_tasks(self):
        """Requested shelves that nobody is carrying yet."""
        carried = self._carried_shelves()
        return [s for s in self.env.unwrapped.request_queue if s not in carried]

    # ------------------------------------------------------------------
    # Task routing
    # ------------------------------------------------------------------

    def _assign_new_tasks(self):
        for shelf in self._open_tasks():
            tid = (int(shelf.x), int(shelf.y))
            if tid not in self._task_zone:
                sorted_zones = self.partitioner.nearest_zones_to_task(shelf.x, shelf.y)
                nearest = sorted_zones[0]
                self._task_zone[tid] = nearest
                self._task_tried_zones[tid].add(nearest)
                self.tasks_routed_by_zone[nearest] += 1
                self.total_tasks_seen += 1

    def _handle_bid_timeouts(self):
        """Tasks no live robot of their zone can serve move to the next zone."""
        for z in range(self.n_zones):
            for tid in self.auctions[z].timed_out_tasks:
                if tid not in self._task_zone:
                    continue
                sorted_zones = self.partitioner.nearest_zones_to_task(tid[0], tid[1])
                already_tried = self._task_tried_zones[tid]
                next_zone = None
                for candidate in sorted_zones:
                    if candidate not in already_tried:
                        next_zone = candidate
                        break

                if next_zone is None:
                    # every zone tried once: start another lap from the current owner
                    self._task_tried_zones[tid] = {self._task_zone[tid]}
                    continue

                self._task_zone[tid] = next_zone
                self._task_tried_zones[tid].add(next_zone)
                self.cross_zone_handoffs += 1

    def _clean_completed_tasks(self):
        active = {(int(s.x), int(s.y)) for s in self._open_tasks()}
        for tid in list(self._task_zone.keys()):
            if tid not in active:
                del self._task_zone[tid]
                self._task_tried_zones.pop(tid, None)

    # ------------------------------------------------------------------
    # Episode handling
    # ------------------------------------------------------------------

    def reset_episode(self):
        """Call after RWARE resets the world (also auto-detected in step)."""
        self._task_zone.clear()
        self._task_tried_zones.clear()
        self.stranded_tasks.clear()
        for auction in self.auctions:
            auction.reset_episode()
        self.pathfinder.reset_episode()

    # ------------------------------------------------------------------
    # Main step
    # ------------------------------------------------------------------

    def _process_stranded_tasks(self, global_pursue, global_nearest_tasks):
        """Dispatches active AMR to physically collect load from broken AMR before towing."""
        ua = self.env.unwrapped
        agents = ua.agents
        failed_flags = self.fault_layers[0].actual_failed

        for f_id, data in list(self.stranded_tasks.items()):
            shelf = data['shelf']
            pos = data['pos']
            rec_id = data['assigned_robot']

            # 1. Check if assigned recovery robot has arrived adjacent to failed robot
            if rec_id is not None and not failed_flags[rec_id]:
                rec_agent = agents[rec_id]
                dist = abs(int(rec_agent.x) - pos[0]) + abs(int(rec_agent.y) - pos[1])
                if dist <= 1 and rec_agent.carrying_shelf is None:
                    # Physically transfer load to recovery robot
                    rec_agent.carrying_shelf = shelf
                    agents[f_id].carrying_shelf = None
                    shelf.x, shelf.y = rec_agent.x, rec_agent.y
                    print(f"\n>>> [FaultRecovery] HANDOFF COMPLETE: Robot {rec_id} arrived at {pos} and COLLECTED shelf {shelf.id} from failed Robot {f_id}! <<<")

                    # ONLY NOW can the incapacitated robot be towed away to perimeter maintenance bay
                    agents[f_id].x = f_id % ua.grid_size[1]
                    agents[f_id].y = 0
                    ua._recalc_grid()
                    print(f">>> [FaultRecovery] Failed Robot {f_id} is NOW towed to maintenance bay ({agents[f_id].x}, {agents[f_id].y})! <<<\n")

                    del self.stranded_tasks[f_id]
                    continue

            # 2. Assign best available eligible robot if not currently assigned
            if rec_id is None or failed_flags[rec_id] or agents[rec_id].carrying_shelf is not None:
                task_type = getattr(self.env, "shelf_task_type", {}).get(shelf, 2)
                acceptable = getattr(self.env, "ACCEPTABLE_ROBOTS", {task_type: [0, 1, 2]}).get(task_type, [0, 1, 2])
                best_robot = None
                best_d = float('inf')
                for r in range(self.n_robots):
                    if not failed_flags[r] and agents[r].carrying_shelf is None:
                        robot_type = self.env.robot_types[r] if hasattr(self.env, "robot_types") else 2
                        if robot_type in acceptable:
                            d = abs(int(agents[r].x) - pos[0]) + abs(int(agents[r].y) - pos[1])
                            if d < best_d:
                                best_d = d
                                best_robot = r
                if best_robot is not None:
                    data['assigned_robot'] = best_robot
                    print(f"[FaultRecovery] Recovery Robot {best_robot} dispatched to collect stranded shelf {shelf.id} from Robot {f_id} at {pos}")

            # 3. Direct assigned recovery robot to failed robot's location
            if data['assigned_robot'] is not None:
                r = data['assigned_robot']
                global_pursue[r] = True
                global_nearest_tasks[r] = pos

    def step(self, obs_batch):
        # auto-detect a RWARE reset (step counter went backwards)
        cur = getattr(self.env.unwrapped, "_cur_steps", None)
        if cur is not None:
            if self._last_cur_steps is not None and cur < self._last_cur_steps:
                self.reset_episode()
            self._last_cur_steps = cur

        self._assign_new_tasks()

        all_belief_failed = [self.fault_layers[z].step() for z in range(self.n_zones)]

        failed = list(self.fault_layers[0].actual_failed)
        bids = self.auctions[0].compute_bids(obs_batch)   # once per step

        global_pursue = [False] * self.n_robots
        global_bids = list(bids)
        global_nearest_tasks = [None] * self.n_robots

        for z in range(self.n_zones):
            pursue_z, _, nearest_z = self.auctions[z].step(
                obs_batch,
                belief_failed=all_belief_failed[z],
                bids=bids,
                failed=failed,
            )
            for i in self.auctions[z].zone_robots:
                global_pursue[i] = pursue_z[i]
                global_nearest_tasks[i] = nearest_z[i]

        self._process_stranded_tasks(global_pursue, global_nearest_tasks)

        self._handle_bid_timeouts()
        self._clean_completed_tasks()

        return global_pursue, global_bids, global_nearest_tasks

    # ------------------------------------------------------------------
    # Fault handling
    # ------------------------------------------------------------------

    def get_fault_action_overrides(self, actions):
        """Force NOOP on any robot that has actually failed (single authority)."""
        return self.fault_layers[0].get_actions_override(list(actions))

    def inject_failure(self, robot_id: int):
        ua = self.env.unwrapped
        agent = ua.agents[robot_id]

        # 1. Mark failure in all fault layers
        self.fault_layers[0].inject_failure(robot_id)          # prints once
        for z in range(1, self.n_zones):
            self.fault_layers[z].actual_failed[robot_id] = True  # silent

        # 2. Check if robot is carrying a shelf (Realistic Physical Fault Recovery)
        shelf = agent.carrying_shelf
        if shelf is not None:
            # Robot stays at its current location holding the load.
            # It will NOT teleport to the perimeter bay, and shelf is NOT teleported home.
            # Another AMR must be dispatched to collect the load in-place.
            pos = (int(agent.x), int(agent.y))
            self.stranded_tasks[robot_id] = {
                'shelf': shelf,
                'pos': pos,
                'assigned_robot': None,
            }
            print(f"[FaultRecovery] Robot {robot_id} FAILED at {pos} holding shelf {shelf.id}! "
                  f"Incapacitated robot remains in place holding load until recovery robot arrives.")
        else:
            # Unladen robot: safely towed immediately to perimeter maintenance bay
            orig_pos = (int(agent.x), int(agent.y))
            agent.x = robot_id % ua.grid_size[1]
            agent.y = 0
            ua._recalc_grid()
            print(f"[FaultRecovery] Empty Robot {robot_id} failed at {orig_pos} -> Towed to maintenance bay ({agent.x}, {agent.y}) to clear traffic aisle")

        # 3. Clear all beliefs and claims held by robot_id
        for auction in self.auctions:
            auction.known_claims[robot_id].clear()
            for r in range(self.n_robots):
                auction.known_claims[r] = {
                    tid: c for tid, c in auction.known_claims[r].items() if c != robot_id
                }
                auction.belief[r] = {
                    tid: b for tid, b in auction.belief[r].items() if b[1] != robot_id
                }

    def clear_stale_beliefs(self):
        for auction in self.auctions:
            auction.clear_stale_beliefs()

    # ------------------------------------------------------------------
    # Stats / reporting
    # ------------------------------------------------------------------

    def total_match_count(self) -> int:
        return sum(self.env.match_count)

    def total_mismatch_count(self) -> int:
        return sum(self.env.mismatch_count)

    def avg_comm_per_robot_per_step(self) -> float:
        """Total comm events per step over ALL zones, divided by fleet size."""
        steps = max((len(a.comm_events_per_step) for a in self.auctions), default=0)
        if self.n_robots == 0 or steps == 0:
            return 0.0
        total_per_step = sum(a.avg_comm_per_step() for a in self.auctions)
        return total_per_step / self.n_robots

    def report_stats(self):
        total_deliveries = self.total_match_count() + self.total_mismatch_count()
        match_rate = (
            self.total_match_count() / total_deliveries * 100
            if total_deliveries > 0 else 0.0
        )
        print(f"\n=== HierarchicalCoordinator Stats ===")
        print(f"Total deliveries: {total_deliveries} (match rate: {match_rate:.1f}%)")
        print(f"Total tasks seen: {self.total_tasks_seen}")
        print(f"Cross-zone handoffs: {self.cross_zone_handoffs} "
              f"({self.cross_zone_handoffs / max(self.total_tasks_seen, 1) * 100:.1f}% of tasks)")
        print(f"Tasks routed by zone: {dict(self.tasks_routed_by_zone)}")
        print(f"Avg comm events per robot per step: "
              f"{self.avg_comm_per_robot_per_step():.4f}")
        print()
        for auction in self.auctions:
            auction.report_stats()
```

---

### 10.11.1 `experiments/exp8_final_suite.py` (Canonical Seeded Benchmark Suite - 280 Runs)

- **Purpose:** Master benchmark executing all 280 seeded evaluation runs across 10 random seeds for greedy, no-comm, fixed, and v2 systems
- **Relative Path:** `[experiments/exp8_final_suite.py](file:///c:/Users/Kezia/warehouse-swarm/experiments/exp8_final_suite.py)`
- **System Location:** `c:\Users\Kezia\warehouse-swarm\experiments\exp8_final_suite.py`

```python
"""
exp8_final_suite.py - final comparison, SEEDED environment, parallel.

Systems : greedy (eligible), nocomm, fixed (original auction + failed-sender
          filter), v2 (retargeting + filter)
Configs : tiny4 (4 robots, 4 failure conditions), tiny2, tiny8,
          small16, medium16 (healthy)
10 seeds x 20,000 steps. Resumable. Run:
    python exp8_final_suite.py --workers 4
Place in warehouse-swarm/experiments/ next to the other scripts.
"""
import argparse
import csv
import sys
from multiprocessing import Pool
from pathlib import Path

_ROOT_DIR = Path(__file__).resolve().parent.parent
for _p in [str(_ROOT_DIR / "core"), str(_ROOT_DIR / "experiments"), str(_ROOT_DIR)]:
    if _p not in sys.path:
        sys.path.insert(0, _p)

import numpy as np
from multi_robot_vecenv import MultiRobotVecEnv
from pathfinding import GridPathfinder
from fault_layer import FaultLayer
from auction_layer_fixed import FixedAuctionLayer
from auction_layer_v2 import AuctionLayerV2
from auction_layer_nocomm import NoCommAuctionLayer
from exp3_greedy_baselines import greedy_assign
from exp_utils import ResultStore, RESULTS_DIR, mean_ci95

SEEDS = [1, 42, 100, 2024, 7, 11, 23, 99, 314, 777]
N_STEPS, FAILURE_STEP = 20000, 500
BID = "bid_head_trained.pt"
T16 = [0, 1, 2] * 5 + [0]
T8 = [0, 1, 2, 0, 1, 2, 0, 1]

CONFIGS = {
    "tiny4":    ("rware-tiny-4ag-v2",    [0, 1, 2, 0]),
    "tiny2":    ("rware-tiny-2ag-v2",    [0, 1]),
    "tiny8":    ("rware-tiny-8ag-v2",    T8),
    "small16":  ("rware-small-16ag-v2",  T16),
    "medium16": ("rware-medium-16ag-v2", T16),
}
FAIL = {"none": (), "robot0": (0,), "robot1": (1,), "robot1+2": (1, 2)}
ALL_SYSTEMS = ["greedy", "nocomm", "fixed", "v2"]


def build_specs():
    specs = []
    for cfg in CONFIGS:
        systems = ALL_SYSTEMS if cfg == "tiny4" else ["greedy", "nocomm", "v2"]
        fails = FAIL if cfg == "tiny4" else {"none": ()}
        for system in systems:
            for fname, failed in fails.items():
                for seed in SEEDS:
                    specs.append((system, cfg, fname, failed, seed))
    return specs


def key_of(spec):
    system, cfg, fname, _, seed = spec
    return f"{system}|{cfg}|{fname}|{seed}"


def _init():
    import torch
    torch.set_num_threads(1)


def run_one(spec):
    system, cfg, fname, failed, seed = spec
    env_id, types = CONFIGS[cfg]
    vec = MultiRobotVecEnv(env_id=env_id, robot_types=list(types), seed=seed)
    obs = vec.reset()
    env = vec.hetero_env
    path = GridPathfinder(env)
    fault = FaultLayer(env) if failed else None
    auction = None
    if system == "nocomm":
        auction = NoCommAuctionLayer(env, bid_model_path=BID)
    elif system == "fixed":
        auction = FixedAuctionLayer(env, bid_model_path=BID)
    elif system == "v2":
        auction = AuctionLayerV2(env, bid_model_path=BID)

    for step in range(N_STEPS):
        bf = None
        if fault is not None:
            if step == FAILURE_STEP:
                for r in failed:
                    fault.inject_failure(r)
            bf = fault.step()
        if system == "greedy":
            pursue, tasks = greedy_assign(env, True)
        elif system == "nocomm":
            pursue, _, tasks = auction.step(obs)
        else:
            pursue, _, tasks = auction.step(obs, belief_failed=bf)
        actions = path.get_actions_for_pursuit(obs, pursue, tasks)
        if fault is not None:
            actions = fault.get_actions_override(actions)
        obs, _, _, _ = vec.step(actions)
        if auction is not None:
            auction.clear_stale_beliefs()

    m, mm = sum(env.match_count), sum(env.mismatch_count)
    vec.close()
    return key_of(spec), dict(deliveries=int(m + mm),
                              match_rate=(m / (m + mm) * 100) if (m + mm) else 0.0)


def summarize(store):
    rows = []
    for system in ALL_SYSTEMS:
        for cfg in CONFIGS:
            for fname in FAIL:
                keys = [f"{system}|{cfg}|{fname}|{s}" for s in SEEDS]
                if not all(store.has(k) for k in keys):
                    continue
                runs = [store.get(k) for k in keys]
                d = [r["deliveries"] for r in runs]
                dm, dsd, dci = mean_ci95(d)
                row = dict(system=system, config=cfg, failed=fname, n=len(SEEDS),
                           deliveries=dm, sd=dsd, ci95=dci,
                           match=float(np.mean([r["match_rate"] for r in runs])))
                base_keys = [f"{system}|{cfg}|none|{s}" for s in SEEDS]
                if cfg == "tiny4" and all(store.has(k) for k in base_keys):
                    ret = [100 * store.get(k)["deliveries"] / store.get(b)["deliveries"]
                           for k, b in zip(keys, base_keys)]
                    rm, _, rci = mean_ci95(ret)
                    row.update(retention=rm, retention_ci95=rci)
                rows.append(row)
    if not rows:
        return
    names = sorted({k for r in rows for k in r}, key=lambda k: list(rows[0]).index(k) if k in rows[0] else 99)
    with open(RESULTS_DIR / "exp8_summary.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=names)
        w.writeheader(); w.writerows(rows)
    print("\n=== EXP 8 FINAL SUMMARY (mean +/- 95% CI, seeded) ===")
    for r in rows:
        ret = f" | retention {r['retention']:.1f}% +/- {r['retention_ci95']:.1f}" if "retention" in r else ""
        print(f"{r['system']:7s} {r['config']:9s} {r['failed']:9s} deliveries {r['deliveries']:7.1f} +/- {r['ci95']:5.1f} "
              f"| match {r['match']:.1f}%{ret}")
    # paired (seed-matched) v2 minus nocomm on tiny4
    print("\nSeed-matched v2 minus nocomm (tiny4):")
    for fname in FAIL:
        try:
            diff = [store.get(f"v2|tiny4|{fname}|{s}")["deliveries"] - store.get(f"nocomm|tiny4|{fname}|{s}")["deliveries"]
                    for s in SEEDS]
            m, _, ci = mean_ci95(diff)
            print(f"  {fname:9s} {m:7.1f} +/- {ci:5.1f}")
        except KeyError:
            pass


def main(workers):
    store = ResultStore("exp8_raw")
    todo = [s for s in build_specs() if not store.has(key_of(s))]
    print(f"{len(todo)} runs to do ({len(build_specs()) - len(todo)} cached)")
    if todo:
        with Pool(workers, initializer=_init) as pool:
            for i, (k, res) in enumerate(pool.imap_unordered(run_one, todo), 1):
                store.set(k, res)
                print(f"[{i}/{len(todo)}] {k}: {res['deliveries']} deliveries, {res['match_rate']:.1f}% match", flush=True)
    summarize(store)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--workers", type=int, default=4)
    main(ap.parse_args().workers)

```

---

### 10.11.2 `experiments/exp1_train_bid_head_logged.py` (Logged Neural Bid Head Training)

- **Purpose:** Supervised training script for `CapabilityConditionedPolicy` with epoch-by-epoch loss logging, LR decay, and validation metrics
- **Relative Path:** `[experiments/exp1_train_bid_head_logged.py](file:///c:/Users/Kezia/warehouse-swarm/experiments/exp1_train_bid_head_logged.py)`
- **System Location:** `c:\Users\Kezia\warehouse-swarm\experiments\exp1_train_bid_head_logged.py`

```python
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

```

---

### 10.11.3 `experiments/determinism_check.py` (Bitwise Seed Reproducibility Verification)

- **Purpose:** Validates that identical random seeds produce bitwise-identical trajectories across environment steps
- **Relative Path:** `[experiments/determinism_check.py](file:///c:/Users/Kezia/warehouse-swarm/experiments/determinism_check.py)`
- **System Location:** `c:\Users\Kezia\warehouse-swarm\experiments\determinism_check.py`

```python
"""
determinism_check.py - Verify whether simulation execution is bitwise deterministic.
Runs two identical trials for a given seed and verifies that observations,
agent positions, actions, and delivery counts match exactly at every step.
"""
import argparse
import sys
from pathlib import Path
import numpy as np

_ROOT_DIR = Path(__file__).resolve().parent.parent
for _p in [str(_ROOT_DIR / "core"), str(_ROOT_DIR / "experiments"), str(_ROOT_DIR)]:
    if _p not in sys.path:
        sys.path.insert(0, _p)

from multi_robot_vecenv import MultiRobotVecEnv
from auction_layer import DecentralizedAuctionLayer
from pathfinding import GridPathfinder
from fault_layer import FaultLayer


def run_trajectory(seed, n_steps=2000, inject_fault_at=500):
    vec_env = MultiRobotVecEnv(env_id="rware-tiny-4ag-v2", robot_types=[0, 1, 2, 0], seed=seed)
    obs = vec_env.reset()
    env = vec_env.hetero_env
    auction = DecentralizedAuctionLayer(env, bid_model_path="bid_head_trained.pt")
    pathfinder = GridPathfinder(env)
    fault = FaultLayer(env)

    step_log = []
    for step in range(n_steps):
        if step == inject_fault_at:
            fault.inject_failure(1)
        bf = fault.step()
        pursue, bids, tasks = auction.step(obs, belief_failed=bf)
        actions = pathfinder.get_actions_for_pursuit(obs, pursue, tasks)
        actions = fault.get_actions_override(actions)
        obs, rewards, dones, info = vec_env.step(actions)
        auction.clear_stale_beliefs()

        m, mm = sum(env.match_count), sum(env.mismatch_count)
        step_log.append({
            "step": step,
            "actions": tuple(actions),
            "deliveries": m + mm,
            "match": m,
            "agent_positions": tuple((a.x, a.y) for a in env.unwrapped.agents)
        })

    vec_env.close()
    return step_log


def main(seeds=[1, 42], n_steps=2000):
    print(f"=== DETERMINISM VERIFICATION CHECK ({n_steps} steps per trial) ===")
    all_passed = True
    for s in seeds:
        print(f"\nTesting Seed {s}:")
        t1 = run_trajectory(s, n_steps)
        t2 = run_trajectory(s, n_steps)

        assert len(t1) == len(t2)
        mismatches = 0
        for step_idx, (s1, s2) in enumerate(zip(t1, t2)):
            if s1 != s2:
                mismatches += 1
                if mismatches <= 3:
                    print(f"  Step {step_idx} mismatch: Run 1={s1} vs Run 2={s2}")

        if mismatches == 0:
            print(f"  [PASSED] Run 1 and Run 2 are BITWISE IDENTICAL over all {n_steps} steps.")
            print(f"  Final deliveries: {t1[-1]['deliveries']} (Matches: {t1[-1]['match']})")
        else:
            print(f"  [FAILED] Found {mismatches} mismatching steps out of {n_steps}!")
            all_passed = False

    print("\n" + ("=" * 50))
    if all_passed:
        print("OVERALL VERDICT: Simulation environment is 100% DETERMINISTIC.")
    else:
        print("OVERALL VERDICT: Non-determinism detected.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--steps", type=int, default=2000)
    args = parser.parse_args()
    main(n_steps=args.steps)

```

---

### 10.11.4 `experiments/diag_mismatch.py` (Capability Mismatch Diagnostic Tracer)

- **Purpose:** Step-by-step diagnostic tracer logging coordinates, pursuit states, and shelf types during mismatch delivery events
- **Relative Path:** `[experiments/diag_mismatch.py](file:///c:/Users/Kezia/warehouse-swarm/experiments/diag_mismatch.py)`
- **System Location:** `c:\Users\Kezia\warehouse-swarm\experiments\diag_mismatch.py`

```python
"""
Diagnose the 0.1-1.3% capability mismatches.

Wraps env.step and, whenever a robot's mismatch_count goes up, records:
  step, robot id/type, the delivered task type,
  whether the robot was pursuing (pursue flag) and its target,
  whether the shelf it carried was ALREADY in the request queue when it
  picked it up ("pickup_requested").
pickup_requested == False means the robot was carrying a shelf for another
reason (for example returning it to storage) and that shelf was requested
while it carried it - a mismatch the eligibility gate cannot see.

Run:  python diag_mismatch.py [--seed 1] [--steps 20000]
Writes results/diag_mismatch_seed<seed>.csv and prints a summary.
"""
import argparse
import csv
import sys
from collections import Counter
from pathlib import Path

_ROOT_DIR = Path(__file__).resolve().parent.parent
_CORE_DIR = _ROOT_DIR / "core"
_EXP_DIR = _ROOT_DIR / "experiments"
for _p in [str(_CORE_DIR), str(_EXP_DIR), str(_ROOT_DIR)]:
    if _p not in sys.path:
        sys.path.insert(0, _p)

from multi_robot_vecenv import MultiRobotVecEnv
from auction_layer import DecentralizedAuctionLayer
from pathfinding import GridPathfinder
from fault_layer import FaultLayer
from exp_utils import RESULTS_DIR

TYPE_NAMES = {0: "fast_light", 1: "heavy_load", 2: "balanced"}
TASK_NAMES = {0: "urgent_light", 1: "heavy_shelf", 2: "standard_delivery"}


def main(seed, steps, robot_types, env_id):
    vec_env = MultiRobotVecEnv(env_id=env_id, robot_types=robot_types, seed=seed)
    obs = vec_env.reset()
    env = vec_env.hetero_env
    auction = DecentralizedAuctionLayer(env, bid_model_path="bid_head_trained.pt")
    pathfinder = GridPathfinder(env)
    fault = FaultLayer(env)

    n = len(robot_types)
    prev_mis = [0] * n
    carrying_prev = [None] * n
    pickup_requested = [None] * n
    events = []

    for step in range(steps):
        belief_failed = fault.step()
        pursue, bids, tasks = auction.step(obs, belief_failed=belief_failed)
        actions = pathfinder.get_actions_for_pursuit(obs, pursue, tasks)
        actions = fault.get_actions_override(actions)

        queue_now = set(env.unwrapped.request_queue)
        obs, _, _, _ = vec_env.step(actions)
        auction.clear_stale_beliefs()

        for i, agent in enumerate(env.unwrapped.agents):
            carrying = getattr(agent, "carrying_shelf", None)
            if carrying is not None and carrying_prev[i] is None:
                pickup_requested[i] = carrying in queue_now
            carrying_prev[i] = carrying
            if env.mismatch_count[i] > prev_mis[i]:
                events.append(dict(step=step, robot=i, robot_type=TYPE_NAMES[robot_types[i]],
                                   pursuing=bool(pursue[i]), target=str(tasks[i]),
                                   pickup_requested=pickup_requested[i]))
                prev_mis[i] = env.mismatch_count[i]

    total = sum(env.match_count) + sum(env.mismatch_count)
    path = RESULTS_DIR / f"diag_mismatch_seed{seed}.csv"
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["step", "robot", "robot_type", "pursuing", "target", "pickup_requested"])
        w.writeheader(); w.writerows(events)

    print(f"\nSeed {seed}: {total} deliveries, {len(events)} mismatches ({len(events)/max(total,1)*100:.2f}%)")
    print("By robot type:", dict(Counter(e['robot_type'] for e in events)))
    print("Pickup was already requested:", dict(Counter(e['pickup_requested'] for e in events)))
    print("Robot was pursuing at delivery:", dict(Counter(e['pursuing'] for e in events)))
    print(f"Saved {path}")
    vec_env.close()


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--steps", type=int, default=20000)
    a = ap.parse_args()
    main(a.seed, a.steps, [0, 1, 2, 0], "rware-tiny-4ag-v2")

```

---

### 10.11.5 `experiments/exp6_zone_scaling.py` (Hierarchical Multi-Zone Scaling Benchmark)

- **Purpose:** Benchmark evaluating 2, 4, and 8 zone spatial partitioning configurations on 16-robot large warehouse floors
- **Relative Path:** `[experiments/exp6_zone_scaling.py](file:///c:/Users/Kezia/warehouse-swarm/experiments/exp6_zone_scaling.py)`
- **System Location:** `c:\Users\Kezia\warehouse-swarm\experiments\exp6_zone_scaling.py`

```python
"""
Exp 6 / 7 - Zone-based coordination, 20,000-step protocol, 10 seeds, 95% CIs.

Configurations (all healthy unless stated):
  flat_8_tiny        8 robots, flat, 11x10            (same as Exp 5, 8 robots)
  flat_16_small      16 robots, flat, 20x10           <- like-for-like control
  hier_16_small_4z   16 robots, 4 zones, 20x10
  flat_16_medium     16 robots, flat, 20x16           <- like-for-like control
  hier_16_medium_2z  16 robots, 2 zones, 20x16        (zone-size sensitivity)
  hier_16_medium_4z  16 robots, 4 zones, 20x16
  hier_16_medium_8z  16 robots, 8 zones, 20x16        (zone-size sensitivity)
  hier_16_medium_4z_zonefail
        4 zones, 20x16, all heavy_load robots in robot-1's zone fail at step 500
        (tests the handoff when a zone loses its only eligible robots)

IMPORTANT - metric definitions (state them in the report):
  * flat  : comm events = number of robot pairs within COMM_RANGE per step
            (geometry-based), divided by robots.
  * zoned : comm events as counted by HierarchicalCoordinator
            (broadcasts that reach a peer inside the same zone).
  These two counters are not identical, so compare flat vs zoned mainly on
  deliveries and match rate, and use the communication numbers as indicative.

Run:  python exp6_zone_scaling.py [--n-seeds 10] [--only hier_16_medium_4z]
Resumable; writes results/exp6_summary.csv
"""
import argparse
import csv
import sys
from pathlib import Path

_ROOT_DIR = Path(__file__).resolve().parent.parent
_CORE_DIR = _ROOT_DIR / "core"
_EXP_DIR = _ROOT_DIR / "experiments"
for _p in [str(_CORE_DIR), str(_EXP_DIR), str(_ROOT_DIR)]:
    if _p not in sys.path:
        sys.path.insert(0, _p)

import numpy as np
from multi_robot_vecenv import MultiRobotVecEnv
from zone_partitioner import ZonePartitioner
from hierarchical_coordinator import HierarchicalCoordinator
from auction_layer import DecentralizedAuctionLayer
from pathfinding import GridPathfinder
from exp_utils import ResultStore, RESULTS_DIR, mean_ci95

ALL_SEEDS = [1, 42, 100, 2024, 7, 11, 23, 99, 314, 777]
N_STEPS = 20000
FAILURE_STEP = 500
T16 = [0, 1, 2] * 5 + [0]
T8 = [0, 1, 2, 0, 1, 2, 0, 1]

CONFIGS = {
    "flat_8_tiny":       dict(env="rware-tiny-8ag-v2",    types=T8,  mode="flat"),
    "flat_16_small":     dict(env="rware-small-16ag-v2",  types=T16, mode="flat"),
    "hier_16_small_4z":  dict(env="rware-small-16ag-v2",  types=T16, mode="hier", zones=4),
    "flat_16_medium":    dict(env="rware-medium-16ag-v2", types=T16, mode="flat"),
    "hier_16_medium_2z": dict(env="rware-medium-16ag-v2", types=T16, mode="hier", zones=2),
    "hier_16_medium_4z": dict(env="rware-medium-16ag-v2", types=T16, mode="hier", zones=4),
    "hier_16_medium_8z": dict(env="rware-medium-16ag-v2", types=T16, mode="hier", zones=8),
    "hier_16_medium_4z_zonefail": dict(env="rware-medium-16ag-v2", types=T16, mode="hier",
                                       zones=4, zonefail=True),
}


def run_flat(cfg, seed):
    vec_env = MultiRobotVecEnv(env_id=cfg["env"], robot_types=cfg["types"], seed=seed)
    obs = vec_env.reset()
    env = vec_env.hetero_env
    auction = DecentralizedAuctionLayer(env, bid_model_path="bid_head_trained.pt")
    pathfinder = GridPathfinder(env)
    n = vec_env.n_robots
    events = []
    for step in range(N_STEPS):
        pursue, bids, tasks = auction.step(obs)
        actions = pathfinder.get_actions_for_pursuit(obs, pursue, tasks)
        obs, _, _, _ = vec_env.step(actions)
        auction.clear_stale_beliefs()
        ag = env.unwrapped.agents
        pairs = sum(1 for i in range(n) for j in range(i + 1, n)
                    if abs(ag[i].x - ag[j].x) + abs(ag[i].y - ag[j].y) <= DecentralizedAuctionLayer.COMM_RANGE)
        events.append(pairs)
    m, mm = sum(env.match_count), sum(env.mismatch_count)
    vec_env.close()
    return dict(deliveries=int(m + mm), match_rate=(m / (m + mm) * 100) if (m + mm) else 0.0,
                comm_per_robot_per_step=float(np.mean(events) / n), handoff_rate=0.0, n_robots=n)


def run_hier(cfg, seed):
    vec_env = MultiRobotVecEnv(env_id=cfg["env"], robot_types=cfg["types"], seed=seed)
    obs = vec_env.reset()
    env = vec_env.hetero_env
    ua = env.unwrapped
    grid_w, grid_h = ua.grid_size[1], ua.grid_size[0]
    partitioner = ZonePartitioner(grid_w, grid_h, n_zones=cfg["zones"])
    zone_assignment = partitioner.assign_robots_to_zones(ua.agents)
    coordinator = HierarchicalCoordinator(env, partitioner, zone_assignment,
                                          bid_model_path="bid_head_trained.pt")

    to_fail = []
    if cfg.get("zonefail"):
        z = zone_assignment[1]
        to_fail = [r for r, zz in zone_assignment.items() if zz == z and cfg["types"][r] == 1]

    for step in range(N_STEPS):
        if step == FAILURE_STEP:
            for r in to_fail:
                coordinator.inject_failure(r)
        pursue, bids, tasks = coordinator.step(obs)
        actions = coordinator.pathfinder.get_actions_for_pursuit(obs, pursue, tasks)
        actions = coordinator.get_fault_action_overrides(actions)
        obs, _, _, _ = vec_env.step(actions)
        coordinator.clear_stale_beliefs()

    m, mm = coordinator.total_match_count(), coordinator.total_mismatch_count()
    seen = coordinator.total_tasks_seen
    out = dict(deliveries=int(m + mm), match_rate=(m / (m + mm) * 100) if (m + mm) else 0.0,
               comm_per_robot_per_step=float(coordinator.avg_comm_per_robot_per_step()),
               handoff_rate=(coordinator.cross_zone_handoffs / seen * 100) if seen else 0.0,
               n_robots=vec_env.n_robots, failed_robots=to_fail)
    vec_env.close()
    return out


def main(n_seeds, only):
    seeds = ALL_SEEDS[:n_seeds]
    store = ResultStore("exp6_raw")
    names = [only] if only else list(CONFIGS)
    for name in names:
        cfg = CONFIGS[name]
        for seed in seeds:
            key = f"{name}|{seed}"
            if store.has(key):
                continue
            try:
                r = run_flat(cfg, seed) if cfg["mode"] == "flat" else run_hier(cfg, seed)
            except Exception as e:
                print(f"[{name}] seed={seed}: FAILED {type(e).__name__}: {e}")
                continue
            store.set(key, r)
            print(f"[{name}] seed={seed}: {r['deliveries']} deliveries, {r['match_rate']:.1f}% match, "
                  f"comm/robot/step {r['comm_per_robot_per_step']:.3f}, handoff {r['handoff_rate']:.1f}%")

    rows = []
    for name in CONFIGS:
        runs = [store.get(f"{name}|{s}") for s in seeds if store.has(f"{name}|{s}")]
        if not runs:
            continue
        dm, dsd, dci = mean_ci95([r["deliveries"] for r in runs])
        cm, _, cci = mean_ci95([r["comm_per_robot_per_step"] for r in runs])
        hm, _, hci = mean_ci95([r["handoff_rate"] for r in runs])
        rows.append(dict(config=name, n_robots=runs[0]["n_robots"], n_seeds=len(runs),
                         deliveries_mean=dm, deliveries_sd=dsd, deliveries_ci95=dci,
                         match_mean=float(np.mean([r["match_rate"] for r in runs])),
                         comm_mean=cm, comm_ci95=cci, handoff_mean=hm, handoff_ci95=hci))
    path = RESULTS_DIR / "exp6_summary.csv"
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader(); w.writerows(rows)
    print("\n=== EXP 6 SUMMARY (mean +/- 95% CI) ===")
    for r in rows:
        print(f"{r['config']:28s} {r['n_robots']:2d} robots | deliveries {r['deliveries_mean']:.1f} +/- {r['deliveries_ci95']:.1f} | "
              f"match {r['match_mean']:.1f}% | comm/robot/step {r['comm_mean']:.3f} +/- {r['comm_ci95']:.3f} | "
              f"handoff {r['handoff_mean']:.1f}% +/- {r['handoff_ci95']:.1f}")
    print(f"\nSaved {path}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--n-seeds", type=int, default=10)
    ap.add_argument("--only", choices=list(CONFIGS), default=None)
    a = ap.parse_args()
    main(a.n_seeds, a.only)

```

---

---

### 10.7.1 `core/lossy_fault_layer.py` (Fault Layer with Packet Loss, Delay, and Configurable Timeout)

- **Purpose:** Subclass of `FaultLayer` implementing synthetic heartbeat packet drop ($p_{\text{loss}}$), transmission queue delay ($d$), configurable timeout threshold ($\tau$), and false-positive step tracking
- **Relative Path:** `[core/lossy_fault_layer.py](file:///c:/Users/Kezia/warehouse-swarm/core/lossy_fault_layer.py)`
- **System Location:** `c:\Users\Kezia\warehouse-swarm\core\lossy_fault_layer.py`

```python
import numpy as np
from fault_layer import FaultLayer

class LossyFaultLayer(FaultLayer):
    """FaultLayer with heartbeat loss, delay and a configurable timeout."""
    def __init__(self, hetero_env, loss=0.0, delay=0, timeout=15, seed=0):
        super().__init__(hetero_env)
        self.loss, self.delay = loss, delay
        self.HEARTBEAT_TIMEOUT = timeout
        self.rng = np.random.default_rng(seed)
        self.pending = []          # (deliver_at_step, receiver j, sender i)
        self.t = 0
        self.false_positive_steps = 0   # healthy robot believed failed
        self.detect_step = {}           # failed robot -> first step anyone flags it

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
            for j, aj in enumerate(agents):
                if i != j and self._distance(ai, aj) <= self.COMM_RANGE:
                    if self.loss > 0 and self.rng.random() < self.loss:
                        continue                      # packet lost
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


```

---

### 10.11.6 `experiments/exp9_robustness.py` (Communication Robustness Sweep Suite)

- **Purpose:** Benchmark suite executing 220 seeded evaluation runs across message loss, delay, and timeout sweeps
- **Relative Path:** `[experiments/exp9_robustness.py](file:///c:/Users/Kezia/warehouse-swarm/experiments/exp9_robustness.py)`
- **System Location:** `c:\Users\Kezia\warehouse-swarm\experiments\exp9_robustness.py`

```python
"""
experiments/exp9_robustness.py - Robustness sweeps for heartbeat loss, delay, and timeout.

Evaluations on tiny4 (4 robots) across 10 seeds:
  1. Base Baseline:
     - v2: loss=0, delay=0, timeout=15 (healthy, robot0) -> 20 runs
     - greedy: baseline (healthy, robot0) -> 20 runs (greedy ignores heartbeat beliefs)
  2. Message Loss Sweep (v2 only):
     - loss in [0.05, 0.10, 0.20], delay=0, timeout=15 (healthy, robot0) -> 60 runs
  3. Transmission Delay Sweep (v2 only):
     - delay in [1, 3, 5], loss=0.0, timeout=15 (healthy, robot0) -> 60 runs
  4. Timeout Sweep (v2 only):
     - timeout in [5, 10, 30], loss=0.0, delay=0 (healthy, robot0) -> 60 runs

Total runs: 20 + 20 + 60 + 60 + 60 = 220 runs.
Resumable with ResultStore('exp9_raw'). Run:
    python experiments/exp9_robustness.py --workers 6
"""
import argparse
import csv
import sys
from multiprocessing import Pool
from pathlib import Path

_ROOT_DIR = Path(__file__).resolve().parent.parent
for _p in [str(_ROOT_DIR / "core"), str(_ROOT_DIR / "experiments"), str(_ROOT_DIR)]:
    if _p not in sys.path:
        sys.path.insert(0, _p)

import numpy as np
from multi_robot_vecenv import MultiRobotVecEnv
from pathfinding import GridPathfinder
from lossy_fault_layer import LossyFaultLayer
from auction_layer_v2 import AuctionLayerV2
from exp3_greedy_baselines import greedy_assign
from exp_utils import ResultStore, RESULTS_DIR, mean_ci95

SEEDS = [1, 42, 100, 2024, 7, 11, 23, 99, 314, 777]
N_STEPS = 20000
FAILURE_STEP = 500
BID = "bid_head_trained.pt"
ENV_ID = "rware-tiny-4ag-v2"
ROBOT_TYPES = [0, 1, 2, 0]

CONDITIONS = {"none": (), "robot0": (0,)}


def build_specs():
    specs = []

    # 1. Base setting (loss=0, delay=0, timeout=15): v2 and greedy
    for fname, failed in CONDITIONS.items():
        for seed in SEEDS:
            specs.append(("v2", fname, failed, seed, "base", "baseline", 0.0, 0, 15))
            specs.append(("greedy", fname, failed, seed, "base", "baseline", 0.0, 0, 15))

    # 2. Loss sweep (v2 only: 0.05, 0.10, 0.20)
    for loss in [0.05, 0.10, 0.20]:
        for fname, failed in CONDITIONS.items():
            for seed in SEEDS:
                specs.append(("v2", fname, failed, seed, "loss", loss, loss, 0, 15))

    # 3. Delay sweep (v2 only: 1, 3, 5)
    for delay in [1, 3, 5]:
        for fname, failed in CONDITIONS.items():
            for seed in SEEDS:
                specs.append(("v2", fname, failed, seed, "delay", delay, 0.0, delay, 15))

    # 4. Timeout sweep (v2 only: 5, 10, 30)
    for timeout in [5, 10, 30]:
        for fname, failed in CONDITIONS.items():
            for seed in SEEDS:
                specs.append(("v2", fname, failed, seed, "timeout", timeout, 0.0, 0, timeout))

    return specs


def key_of(spec):
    system, fname, _, seed, sweep_name, sweep_val, _, _, _ = spec
    return f"{system}|{fname}|{sweep_name}_{sweep_val}|{seed}"


def _init():
    import torch
    torch.set_num_threads(1)


def run_one(spec):
    system, fname, failed, seed, sweep_name, sweep_val, loss, delay, timeout = spec

    vec = MultiRobotVecEnv(env_id=ENV_ID, robot_types=list(ROBOT_TYPES), seed=seed)
    obs = vec.reset()
    env = vec.hetero_env
    path = GridPathfinder(env)

    # Always create LossyFaultLayer, even for healthy runs, to test spurious failure detection!
    fault = LossyFaultLayer(env, loss=loss, delay=delay, timeout=timeout, seed=seed)

    auction = None
    if system == "v2":
        auction = AuctionLayerV2(env, bid_model_path=BID)

    for step in range(N_STEPS):
        if step == FAILURE_STEP and failed:
            for r in failed:
                fault.inject_failure(r)

        bf = fault.step()

        if system == "greedy":
            pursue, tasks = greedy_assign(env, True)
        elif system == "v2":
            pursue, _, tasks = auction.step(obs, belief_failed=bf)

        actions = path.get_actions_for_pursuit(obs, pursue, tasks)
        actions = fault.get_actions_override(actions)

        obs, _, _, _ = vec.step(actions)
        if auction is not None:
            auction.clear_stale_beliefs()

    m, mm = sum(env.match_count), sum(env.mismatch_count)
    vec.close()

    det_delay = fault.detect_step.get(0, None)
    if det_delay is not None:
        det_delay -= FAILURE_STEP

    res = dict(
        deliveries=int(m + mm),
        match_rate=(m / (m + mm) * 100) if (m + mm) else 0.0,
        false_positive_steps=fault.false_positive_steps,
        det_delay=det_delay,
    )
    return key_of(spec), res


def summarize(store):
    print("\n" + "=" * 90)
    print("EXP 9 ROBUSTNESS EVALUATION SUMMARY (10 Seeds, mean +/- 95% CI)")
    print("=" * 90)

    rows = []

    # Define the sweep presentation order
    eval_groups = [
        ("BASE BASELINE", [("base", "baseline", 0.0, 0, 15)], ["v2", "greedy"]),
        ("LOSS SWEEP", [( "loss", 0.05, 0.05, 0, 15), ("loss", 0.10, 0.10, 0, 15), ("loss", 0.20, 0.20, 0, 15)], ["v2"]),
        ("DELAY SWEEP", [("delay", 1, 0.0, 1, 15), ("delay", 3, 0.0, 3, 15), ("delay", 5, 0.0, 5, 15)], ["v2"]),
        ("TIMEOUT SWEEP", [("timeout", 5, 0.0, 0, 5), ("timeout", 10, 0.0, 0, 10), ("timeout", 30, 0.0, 0, 30)], ["v2"]),
    ]

    for group_name, param_list, systems in eval_groups:
        print(f"\n--- {group_name} ---")
        for sweep_name, sweep_val, loss, delay, timeout in param_list:
            for system in systems:
                for fname in ["none", "robot0"]:
                    keys = [f"{system}|{fname}|{sweep_name}_{sweep_val}|{s}" for s in SEEDS]
                    if not all(store.has(k) for k in keys):
                        continue
                    runs = [store.get(k) for k in keys]

                    delivs = [r["deliveries"] for r in runs]
                    dm, dsd, dci = mean_ci95(delivs)

                    matches = [r["match_rate"] for r in runs]
                    mm = float(np.mean(matches))

                    fps = [r["false_positive_steps"] for r in runs]
                    fpm, _, fpci = mean_ci95(fps)

                    delays = [r["det_delay"] for r in runs if r["det_delay"] is not None]
                    delay_m, _, delay_ci = mean_ci95(delays) if delays else (None, None, None)

                    # Compute seed-matched retention against the system's baseline healthy
                    retention_str = ""
                    rm, rci = None, None
                    if fname == "robot0":
                        base_keys = [f"{system}|none|base_baseline|{s}" for s in SEEDS]
                        if all(store.has(b) for b in base_keys):
                            ret = [100.0 * store.get(k)["deliveries"] / store.get(b)["deliveries"]
                                   for k, b in zip(keys, base_keys)]
                            rm, _, rci = mean_ci95(ret)
                            retention_str = f" | ret {rm:5.1f}% +/- {rci:4.1f}%"

                    delay_str = f" | det_delay {delay_m:4.1f} +/- {delay_ci:3.1f}" if delay_m is not None else ""
                    print(f"[{sweep_name:7s} = {str(sweep_val):8s}] {system:6s} {fname:6s} | "
                          f"deliv {dm:6.1f} +/- {dci:4.1f} | match {mm:5.1f}% | "
                          f"FP_steps {fpm:5.1f} +/- {fpci:4.1f}{delay_str}{retention_str}")

                    rows.append(dict(
                        sweep=sweep_name,
                        val=sweep_val,
                        loss=loss,
                        delay=delay,
                        timeout=timeout,
                        system=system,
                        condition=fname,
                        deliveries=dm,
                        deliv_ci95=dci,
                        match_rate=mm,
                        fp_steps=fpm,
                        fp_ci95=fpci,
                        det_delay=delay_m,
                        det_delay_ci95=delay_ci,
                        retention=rm,
                        retention_ci95=rci
                    ))

    if rows:
        summary_csv = RESULTS_DIR / "exp9_summary.csv"
        names = list(rows[0].keys())
        with open(summary_csv, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=names)
            w.writeheader()
            w.writerows(rows)
        print(f"\nSummary saved to: {summary_csv}")


def main(workers):
    store = ResultStore("exp9_raw")
    specs = build_specs()
    todo = [s for s in specs if not store.has(key_of(s))]
    print(f"Total specs: {len(specs)} | To execute: {len(todo)} | Already cached: {len(specs) - len(todo)}")

    if todo:
        print(f"Launching worker pool with {workers} parallel processes...")
        with Pool(workers, initializer=_init) as pool:
            for i, (k, res) in enumerate(pool.imap_unordered(run_one, todo), 1):
                store.set(k, res)
                det_info = f"det_delay={res['det_delay']}" if res['det_delay'] is not None else ""
                print(f"[{i:3d}/{len(todo):3d}] {k}: {res['deliveries']} deliv, {res['match_rate']:.1f}% match, "
                      f"FP={res['false_positive_steps']} {det_info}", flush=True)

    summarize(store)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--workers", type=int, default=6)
    main(ap.parse_args().workers)

```

---

### 10.12 `experiments/collect_bid_data.py`

- **Purpose:** Generates 200,000 steps of experience replay data with continuous suitability labels
- **Relative Path:** `[experiments/collect_bid_data.py](file:///c:/Users/Kezia/warehouse-swarm/experiments/collect_bid_data.py)`
- **System Location:** `c:\Users\Kezia\warehouse-swarm\experiments\collect_bid_data.py`

```python
"""
Collects (observation, label) pairs for training the bid_head.
Label combines:
  - capability match (type vs nearest task's acceptable set)
  - battery sufficiency (enough charge to reach the task, given drain rate)
  - actual delivery outcome (overrides heuristic for the few steps right
    before a real delivery, since real outcome > heuristic estimate)
"""

import os
import sys
from pathlib import Path

# Ensure core and project root are on sys.path
_ROOT_DIR = Path(__file__).resolve().parent.parent
_CORE_DIR = _ROOT_DIR / "core"
_MODELS_DIR = _ROOT_DIR / "models_and_data"
for _p in [str(_CORE_DIR), str(_ROOT_DIR)]:
    if _p not in sys.path:
        sys.path.insert(0, _p)

import numpy as np
from collections import deque
from multi_robot_vecenv import MultiRobotVecEnv

ENV_ID = "rware-tiny-2ag-v2"
ROBOT_TYPES = None
N_STEPS = 200000
OUTPUT_FILE = str(_MODELS_DIR / "bid_training_data.npz")

BUFFER_LEN = 5
BATTERY_SAFETY_MARGIN = 1.2


def collect_data():
    vec_env = MultiRobotVecEnv(env_id=ENV_ID, robot_types=ROBOT_TYPES, seed=123)
    obs = vec_env.reset()
    env = vec_env.hetero_env
    n_robots = vec_env.n_robots

    all_obs = []
    all_labels = []

    # per-robot buffer: stores INDICES into all_labels for the last BUFFER_LEN
    # steps, so we can go back and overwrite them if a real delivery follows
    pending_indices = [deque(maxlen=BUFFER_LEN) for _ in range(n_robots)]

    print(f"Collecting bid-training data over {N_STEPS} steps...")

    for step in range(N_STEPS):
        actions = np.array([vec_env.action_space.sample() for _ in range(n_robots)])

        queue_before = list(env.unwrapped.request_queue)
        shelf_task_before = dict(env.shelf_task_type)

        for i, agent in enumerate(env.unwrapped.agents):
            queue = env.unwrapped.request_queue
            if queue:
                best_dist = None
                best_shelf = None
                for shelf in queue:
                    dist = abs(shelf.x - agent.x) + abs(shelf.y - agent.y)
                    if best_dist is None or dist < best_dist:
                        best_dist = dist
                        best_shelf = shelf

                nearest_task_type = env.shelf_task_type.get(best_shelf, 2)
                robot_type = env.robot_types[i]

                capability_match = 1.0 if robot_type in env.ACCEPTABLE_ROBOTS[nearest_task_type] else 0.0

                drain_rate = env.BATTERY_DRAIN[robot_type]
                battery_needed = best_dist * drain_rate * BATTERY_SAFETY_MARGIN
                battery_sufficient = 1.0 if env.battery[i] >= battery_needed else 0.0

                heuristic_label = 0.6 * capability_match + 0.4 * battery_sufficient

                # save immediately, every step
                all_obs.append(obs[i].copy())
                all_labels.append(heuristic_label)
                pending_indices[i].append(len(all_labels) - 1)

        obs, rewards, dones, infos = vec_env.step(actions)

        queue_after = set(env.unwrapped.request_queue)
        delivered_shelves = [s for s in queue_before if s not in queue_after]

        for shelf in delivered_shelves:
            task_type = shelf_task_before.get(shelf, 2)
            for i, agent in enumerate(env.unwrapped.agents):
                if agent.x == shelf.x and agent.y == shelf.y:
                    real_outcome = 1.0 if env.robot_types[i] in env.ACCEPTABLE_ROBOTS[task_type] else 0.0
                    # overwrite the labels of the last few steps for this robot
                    for idx in pending_indices[i]:
                        all_labels[idx] = real_outcome
                    pending_indices[i].clear()

        if step % 20000 == 0:
            print(f"  step {step}, samples so far: {len(all_obs)}")

    all_obs = np.array(all_obs, dtype=np.float32)
    all_labels = np.array(all_labels, dtype=np.float32)

    print(f"\nDone. Total samples: {len(all_obs)}")
    print(f"Mean label value: {all_labels.mean():.3f}")
    print(f"Label std dev: {all_labels.std():.3f}")

    np.savez(OUTPUT_FILE, obs=all_obs, labels=all_labels)
    print(f"Saved to {OUTPUT_FILE}")

    vec_env.close()


if __name__ == "__main__":
    collect_data()
```

### 10.13 `experiments/train_bid_head.py`

- **Purpose:** Supervised training loop for neural bid head with BCE loss and learning rate decay
- **Relative Path:** `[experiments/train_bid_head.py](file:///c:/Users/Kezia/warehouse-swarm/experiments/train_bid_head.py)`
- **System Location:** `c:\Users\Kezia\warehouse-swarm\experiments\train_bid_head.py`

```python
"""
Trains only the bid_head of CapabilityConditionedPolicy using supervised
regression against a continuous suitability label (capability + battery +
real delivery outcome blend). Reports MAE instead of thresholded accuracy,
since labels are continuous, not binary.
"""

import os
import sys
from pathlib import Path

# Ensure core and project root are on sys.path
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

DATA_FILE = str(_MODELS_DIR / "bid_training_data.npz")
MODEL_OUT = str(_MODELS_DIR / "bid_head_trained.pt")
CORE_MODEL_OUT = str(_CORE_DIR / "bid_head_trained.pt")

EPOCHS = 60
BATCH_SIZE = 256
LEARNING_RATE = 3e-3
VAL_SPLIT = 0.15


def train():
    if not os.path.exists(DATA_FILE):
        raise FileNotFoundError(f"Training data not found at {DATA_FILE}")

    data = np.load(DATA_FILE)
    obs = torch.tensor(data["obs"], dtype=torch.float32)
    labels = torch.tensor(data["labels"], dtype=torch.float32).unsqueeze(1)

    n_total = len(obs)
    n_val = int(n_total * VAL_SPLIT)
    n_train = n_total - n_val

    perm = torch.randperm(n_total)
    train_idx, val_idx = perm[:n_train], perm[n_train:]

    train_ds = TensorDataset(obs[train_idx], labels[train_idx])
    val_ds = TensorDataset(obs[val_idx], labels[val_idx])

    train_loader = DataLoader(train_ds, batch_size=BATCH_SIZE, shuffle=True)
    val_loader = DataLoader(val_ds, batch_size=BATCH_SIZE, shuffle=False)

    model = CapabilityConditionedPolicy(obs_dim=80)
    optimizer = torch.optim.Adam(model.parameters(), lr=LEARNING_RATE)
    scheduler = torch.optim.lr_scheduler.StepLR(optimizer, step_size=20, gamma=0.3)
    bce_loss_fn = nn.BCELoss()
    mae_loss_fn = nn.L1Loss()

    print(f"Training on {n_train} samples, validating on {n_val} samples")

    for epoch in range(EPOCHS):
        model.train()
        train_losses = []
        for batch_obs, batch_labels in train_loader:
            bid_value, _ = model(batch_obs)
            loss = bce_loss_fn(bid_value, batch_labels)

            optimizer.zero_grad()
            loss.backward()
            optimizer.step()

            train_losses.append(loss.item())

        scheduler.step()

        model.eval()
        val_bce_losses = []
        val_mae_losses = []
        with torch.no_grad():
            for batch_obs, batch_labels in val_loader:
                bid_value, _ = model(batch_obs)
                val_bce_losses.append(bce_loss_fn(bid_value, batch_labels).item())
                val_mae_losses.append(mae_loss_fn(bid_value, batch_labels).item())

        train_loss = np.mean(train_losses)
        val_bce = np.mean(val_bce_losses)
        val_mae = np.mean(val_mae_losses)

        print(f"Epoch {epoch+1}/{EPOCHS} | train_bce: {train_loss:.4f} | "
              f"val_bce: {val_bce:.4f} | val_mae: {val_mae:.4f} | lr: {scheduler.get_last_lr()[0]:.5f}")

    torch.save(model.state_dict(), MODEL_OUT)
    torch.save(model.state_dict(), CORE_MODEL_OUT)
    print(f"\nTraining complete. Saved bid_head weights to {MODEL_OUT} and {CORE_MODEL_OUT}")
    print(f"Final val_mae: {val_mae:.4f} (lower is better; 0 = perfect, 0.25 ≈ predicting the mean)")


if __name__ == "__main__":
    train()
```

### 10.14 `experiments/greedy_baseline.py`

- **Purpose:** Nearest-robot uncoordinated baseline benchmark
- **Relative Path:** `[experiments/greedy_baseline.py](file:///c:/Users/Kezia/warehouse-swarm/experiments/greedy_baseline.py)`
- **System Location:** `c:\Users\Kezia\warehouse-swarm\experiments\greedy_baseline.py`

```python
"""
Phase 7 baseline: greedy nearest-robot allocation.
No bidding, no eligibility check, no communication -- whichever robot is
physically closest to a task pursues it. This is the naive baseline used
to show why capability-aware bidding (Phase 3-5) actually matters.

Reuses the same A* pathfinding from Phase 5 for movement, so the only
difference from your main pipeline is the task-assignment logic itself.
"""

import os
import sys
from pathlib import Path

# Ensure core and project root are on sys.path
_ROOT_DIR = Path(__file__).resolve().parent.parent
_CORE_DIR = _ROOT_DIR / "core"
for _p in [str(_CORE_DIR), str(_ROOT_DIR)]:
    if _p not in sys.path:
        sys.path.insert(0, _p)

import numpy as np
from multi_robot_vecenv import MultiRobotVecEnv
from pathfinding import GridPathfinder

SEEDS = [1, 42, 100, 2024]
N_STEPS = 20000
ROBOT_TYPES = [0, 1, 2, 0]


def distance(a, b):
    return abs(a.x - b.x) + abs(a.y - b.y)


def greedy_assign(env):
    """
    For every task in the request queue, assign it to whichever robot is
    physically closest -- no capability check, no communication, no bidding.
    Multiple tasks can be claimed by different robots in the same step;
    if a robot is closest to more than one task, it only pursues its single
    nearest one (ties broken by lowest robot_id, consistent with the rest
    of your project).
    """
    agents = env.unwrapped.agents
    queue = env.unwrapped.request_queue
    n_robots = len(agents)

    nearest_task_per_robot = [None] * n_robots
    pursue = [False] * n_robots

    if not queue:
        return pursue, nearest_task_per_robot

    # each robot's own nearest task, regardless of whether it can match it
    for i, agent in enumerate(agents):
        best_dist, best_shelf = None, None
        for shelf in queue:
            d = distance(agent, shelf)
            if best_dist is None or d < best_dist:
                best_dist, best_shelf = d, shelf
        nearest_task_per_robot[i] = (best_shelf.x, best_shelf.y)

    # for each task, find the single closest robot among those targeting it
    task_best_robot = {}
    for i, agent in enumerate(agents):
        task = nearest_task_per_robot[i]
        d = distance(agent, next(s for s in queue if (s.x, s.y) == task))
        if task not in task_best_robot or d < task_best_robot[task][0] or \
           (d == task_best_robot[task][0] and i < task_best_robot[task][1]):
            task_best_robot[task] = (d, i)

    for task, (d, winner) in task_best_robot.items():
        pursue[winner] = True

    return pursue, nearest_task_per_robot


def run_seed(seed):
    vec_env = MultiRobotVecEnv(env_id="rware-tiny-4ag-v2", robot_types=ROBOT_TYPES, seed=seed)
    obs = vec_env.reset()
    env = vec_env.hetero_env
    pathfinder = GridPathfinder(env)

    for step in range(N_STEPS):
        pursue, nearest_tasks = greedy_assign(env)
        actions = pathfinder.get_actions_for_pursuit(obs, pursue, nearest_tasks)
        obs, rewards, dones, infos = vec_env.step(actions)

    total_matches = sum(env.match_count)
    total_mismatches = sum(env.mismatch_count)
    total_deliveries = total_matches + total_mismatches
    match_rate = (total_matches / total_deliveries * 100) if total_deliveries > 0 else 0.0

    vec_env.close()
    return total_deliveries, match_rate


print(f"Running greedy nearest-robot baseline: {len(SEEDS)} seeds, {N_STEPS} steps each.\n")

deliveries_list = []
match_rate_list = []

for seed in SEEDS:
    deliveries, match_rate = run_seed(seed)
    deliveries_list.append(deliveries)
    match_rate_list.append(match_rate)
    print(f"seed={seed}: {deliveries} deliveries, {match_rate:.1f}% match rate")

avg_deliveries = np.mean(deliveries_list)
avg_match_rate = np.mean(match_rate_list)

print(f"\n=== Greedy Baseline Summary ===")
print(f"Average deliveries: {avg_deliveries:.1f} (raw: {deliveries_list})")
print(f"Average match rate: {avg_match_rate:.1f}% (raw: {match_rate_list})")
print(f"\nCompare against your eligibility-gated bidding system: 100.0% match rate, ~14.2 avg deliveries (0-failure baseline)")
```

### 10.15 `experiments/auction_layer_nocomm.py`

- **Purpose:** Zero-communication auction layer for ablation benchmarking
- **Relative Path:** `[experiments/auction_layer_nocomm.py](file:///c:/Users/Kezia/warehouse-swarm/experiments/auction_layer_nocomm.py)`
- **System Location:** `c:\Users\Kezia\warehouse-swarm\experiments\auction_layer_nocomm.py`

```python
"""
Phase 7 ablation: no-communication auction layer.
Identical to DecentralizedAuctionLayer, except COMM_RANGE = 0, meaning no
robot ever hears another robot's bid or claim broadcast. Each robot only
ever "hears" itself, so every robot pursues its own nearest eligible task
independently with no conflict resolution and no fault-driven reallocation
possible (since a robot can never learn about another robot's failure
either, this file also skips fault-layer integration entirely -- there is
no mechanism BY WHICH it could reallocate, which is exactly the point of
this ablation).

Used to isolate how much of Phase 6's fault-tolerance result depends on
the communication/reallocation mechanism, versus the system just doing
fine on its own regardless.
"""

import os
import sys
from pathlib import Path

# Ensure core and project root are in sys.path
_ROOT_DIR = Path(__file__).resolve().parent.parent
_CORE_DIR = _ROOT_DIR / "core"
for _p in [str(_CORE_DIR), str(_ROOT_DIR)]:
    if _p not in sys.path:
        sys.path.insert(0, _p)

import torch
from policy_network import CapabilityConditionedPolicy


def _resolve_model_path(path: str) -> str:
    if os.path.exists(path):
        return path
    core_candidate = _CORE_DIR / path
    if core_candidate.exists():
        return str(core_candidate)
    models_candidate = _ROOT_DIR / "models_and_data" / path
    if models_candidate.exists():
        return str(models_candidate)
    return path


class NoCommAuctionLayer:
    COMM_RANGE = 0  # nobody ever hears anybody

    def __init__(self, hetero_env, bid_model_path="bid_head_trained.pt"):
        self.env = hetero_env
        self.n_robots = hetero_env.unwrapped.n_agents

        self.bid_model = CapabilityConditionedPolicy(obs_dim=80)
        resolved_path = _resolve_model_path(bid_model_path)
        self.bid_model.load_state_dict(torch.load(resolved_path, map_location="cpu"))
        self.bid_model.eval()

    def _task_id(self, shelf):
        return (shelf.x, shelf.y)

    def _distance(self, a, b):
        return abs(a.x - b.x) + abs(a.y - b.y)

    def compute_bids(self, obs_batch):
        bids = []
        with torch.no_grad():
            for i in range(self.n_robots):
                obs_tensor = torch.tensor(obs_batch[i], dtype=torch.float32).unsqueeze(0)
                bid_value, _ = self.bid_model(obs_tensor)
                bids.append(bid_value.item())
        return bids

    def _nearest_eligible_task(self, agent, robot_type):
        queue = self.env.unwrapped.request_queue
        if not queue:
            return None

        best_dist, best_shelf = None, None
        for shelf in queue:
            task_type = self.env.shelf_task_type.get(shelf, 2)
            if robot_type not in self.env.ACCEPTABLE_ROBOTS[task_type]:
                continue
            d = self._distance(agent, shelf)
            if best_dist is None or d < best_dist:
                best_dist, best_shelf = d, shelf

        if best_shelf is None:
            return None
        return self._task_id(best_shelf)

    def step(self, obs_batch, belief_failed=None (bf=None)):
        """
        No communication: every robot independently decides to pursue its
        own nearest eligible task, with no awareness of what any other
        robot is doing, no conflict resolution, and no ability to detect
        or react to failures (belief_failed is accepted for interface
        compatibility with the test harness but intentionally ignored --
        a robot with COMM_RANGE=0 cannot hear heartbeats either).
        """
        agents = self.env.unwrapped.agents
        bids = self.compute_bids(obs_batch)  # computed but unused for coordination

        nearest_task_per_robot = [
            self._nearest_eligible_task(agents[i], self.env.robot_types[i])
            for i in range(self.n_robots)
        ]

        # every robot with an eligible task simply pursues it -- no
        # awareness of collisions or duplicate pursuit of the same shelf
        pursue = [task is not None for task in nearest_task_per_robot]

        return pursue, bids, nearest_task_per_robot

    def clear_stale_beliefs(self):
        pass  # no belief tables exist in this ablation

    def report_stats(self):
        print("No-comm ablation: no broadcast/belief statistics to report (by design).")
```

### 10.16 `experiments/phase6_degradation_experiment.py`

- **Purpose:** Fault degradation curve benchmark (0, 1, 2 failures)
- **Relative Path:** `[experiments/phase6_degradation_experiment.py](file:///c:/Users/Kezia/warehouse-swarm/experiments/phase6_degradation_experiment.py)`
- **System Location:** `c:\Users\Kezia\warehouse-swarm\experiments\phase6_degradation_experiment.py`

```python
"""
Phase 6 headline experiment: fault-tolerance degradation curve.
Runs the full pipeline (bidding + eligibility-gated auction + pathfinding +
heartbeat fault detection + stale-claim reallocation) across different
numbers of simultaneous robot failures (0, 1, 2 out of 4), each repeated
over multiple seeds, with long enough runs to get trustworthy delivery
counts (matching the scale where prior tests showed real signal).

Failures are injected early (step 500) so most of the run happens
post-failure, giving a fair read on sustained degraded throughput.
"""

import os
import sys
from pathlib import Path

# Ensure core and project root are on sys.path
_ROOT_DIR = Path(__file__).resolve().parent.parent
_CORE_DIR = _ROOT_DIR / "core"
for _p in [str(_CORE_DIR), str(_ROOT_DIR)]:
    if _p not in sys.path:
        sys.path.insert(0, _p)

import numpy as np
from multi_robot_vecenv import MultiRobotVecEnv
from auction_layer import DecentralizedAuctionLayer
from pathfinding import GridPathfinder
from fault_layer import FaultLayer

SEEDS = [1, 42, 100, 2024]
N_STEPS = 20000
FAILURE_STEP = 500
ROBOT_TYPES = [0, 1, 2, 0]

# which robots fail under each condition (indices into the 4-robot list)
FAILURE_CONDITIONS = {
    "0_failures": [],
    "1_failure": [1],       # heavy_load fails
    "2_failures": [1, 2],   # heavy_load + balanced fail
}


def run_condition(failed_robots, seed):
    vec_env = MultiRobotVecEnv(env_id="rware-tiny-4ag-v2", robot_types=ROBOT_TYPES, seed=seed)
    obs = vec_env.reset()
    env = vec_env.hetero_env

    auction = DecentralizedAuctionLayer(env, bid_model_path="bid_head_trained.pt")
    pathfinder = GridPathfinder(env)
    fault_layer = FaultLayer(env)

    injected = False

    for step in range(N_STEPS):
        if not injected and step == FAILURE_STEP and failed_robots:
            for r in failed_robots:
                fault_layer.inject_failure(r)
            injected = True

        belief_failed = fault_layer.step()
        pursue, bids, nearest_tasks = auction.step(obs, belief_failed=belief_failed)
        actions = pathfinder.get_actions_for_pursuit(obs, pursue, nearest_tasks)
        actions = fault_layer.get_actions_override(actions)

        obs, rewards, dones, infos = vec_env.step(actions)
        auction.clear_stale_beliefs()

    total_matches = sum(env.match_count)
    total_mismatches = sum(env.mismatch_count)
    total_deliveries = total_matches + total_mismatches
    match_rate = (total_matches / total_deliveries * 100) if total_deliveries > 0 else 0.0

    vec_env.close()
    return total_deliveries, match_rate


print(f"Running degradation experiment: {len(FAILURE_CONDITIONS)} conditions x {len(SEEDS)} seeds, "
      f"{N_STEPS} steps each. This will take a while.\n")

results = {}

for condition_name, failed_robots in FAILURE_CONDITIONS.items():
    deliveries_list = []
    match_rate_list = []

    for seed in SEEDS:
        deliveries, match_rate = run_condition(failed_robots, seed)
        deliveries_list.append(deliveries)
        match_rate_list.append(match_rate)
        print(f"  [{condition_name}] seed={seed}: {deliveries} deliveries, {match_rate:.1f}% match rate")

    avg_deliveries = np.mean(deliveries_list)
    avg_match_rate = np.mean(match_rate_list)
    results[condition_name] = (avg_deliveries, avg_match_rate, deliveries_list)
    print(f"  --> {condition_name} AVERAGE: {avg_deliveries:.1f} deliveries, {avg_match_rate:.1f}% match rate\n")

print("\n=== DEGRADATION CURVE SUMMARY ===")
baseline_deliveries = results["0_failures"][0]
for condition_name, (avg_deliveries, avg_match_rate, raw_list) in results.items():
    pct_of_baseline = (avg_deliveries / baseline_deliveries * 100) if baseline_deliveries > 0 else 0.0
    print(f"{condition_name}: avg {avg_deliveries:.1f} deliveries "
          f"({pct_of_baseline:.1f}% of no-failure baseline), "
          f"avg match rate {avg_match_rate:.1f}%, raw={raw_list}")
```

### 10.17 `experiments/phase7_nocomm_experiment.py`

- **Purpose:** Ablation study evaluating communication contribution under failure
- **Relative Path:** `[experiments/phase7_nocomm_experiment.py](file:///c:/Users/Kezia/warehouse-swarm/experiments/phase7_nocomm_experiment.py)`
- **System Location:** `c:\Users\Kezia\warehouse-swarm\experiments\phase7_nocomm_experiment.py`

```python
"""
Phase 7: no-communication ablation experiment. Same failure conditions and
seeds as the Phase 6 degradation curve, but using NoCommAuctionLayer instead
of DecentralizedAuctionLayer -- isolates how much of the fault-tolerance
result depends on communication/reallocation.

Note: failed robots still get forced to NOOP via FaultLayer's action
override (that part is a physical fact of the robot being broken, not a
communication feature) -- what's missing here is other robots' ABILITY
to detect the failure and reallocate the failed robot's claimed tasks.
"""

import os
import sys
from pathlib import Path

# Ensure core, experiments, and project root are on sys.path
_ROOT_DIR = Path(__file__).resolve().parent.parent
_CORE_DIR = _ROOT_DIR / "core"
_EXP_DIR = _ROOT_DIR / "experiments"
for _p in [str(_CORE_DIR), str(_EXP_DIR), str(_ROOT_DIR)]:
    if _p not in sys.path:
        sys.path.insert(0, _p)

import numpy as np
from multi_robot_vecenv import MultiRobotVecEnv
from auction_layer_nocomm import NoCommAuctionLayer
from pathfinding import GridPathfinder
from fault_layer import FaultLayer

SEEDS = [1, 42, 100, 2024]
N_STEPS = 20000
FAILURE_STEP = 500
ROBOT_TYPES = [0, 1, 2, 0]

FAILURE_CONDITIONS = {
    "0_failures": [],
    "1_failure": [1],
    "2_failures": [1, 2],
}


def run_condition(failed_robots, seed):
    vec_env = MultiRobotVecEnv(env_id="rware-tiny-4ag-v2", robot_types=ROBOT_TYPES, seed=seed)
    obs = vec_env.reset()
    env = vec_env.hetero_env

    auction = NoCommAuctionLayer(env, bid_model_path="bid_head_trained.pt")
    pathfinder = GridPathfinder(env)
    fault_layer = FaultLayer(env)  # still used to force NOOP on failed robots physically

    injected = False

    for step in range(N_STEPS):
        if not injected and step == FAILURE_STEP and failed_robots:
            for r in failed_robots:
                fault_layer.inject_failure(r)
            injected = True

        fault_layer.step()  # updates internal state but beliefs are unused here

        pursue, bids, nearest_tasks = auction.step(obs)  # no belief_failed passed in
        actions = pathfinder.get_actions_for_pursuit(obs, pursue, nearest_tasks)
        actions = fault_layer.get_actions_override(actions)  # failed robots physically can't move

        obs, rewards, dones, infos = vec_env.step(actions)

    total_matches = sum(env.match_count)
    total_mismatches = sum(env.mismatch_count)
    total_deliveries = total_matches + total_mismatches
    match_rate = (total_matches / total_deliveries * 100) if total_deliveries > 0 else 0.0

    vec_env.close()
    return total_deliveries, match_rate


print(f"Running no-communication ablation: {len(FAILURE_CONDITIONS)} conditions x {len(SEEDS)} seeds, "
      f"{N_STEPS} steps each.\n")

results = {}

for condition_name, failed_robots in FAILURE_CONDITIONS.items():
    deliveries_list = []
    match_rate_list = []

    for seed in SEEDS:
        deliveries, match_rate = run_condition(failed_robots, seed)
        deliveries_list.append(deliveries)
        match_rate_list.append(match_rate)
        print(f"  [{condition_name}] seed={seed}: {deliveries} deliveries, {match_rate:.1f}% match rate")

    avg_deliveries = np.mean(deliveries_list)
    avg_match_rate = np.mean(match_rate_list)
    results[condition_name] = (avg_deliveries, avg_match_rate, deliveries_list)
    print(f"  --> {condition_name} AVERAGE: {avg_deliveries:.1f} deliveries, {avg_match_rate:.1f}% match rate\n")

print("\n=== NO-COMMUNICATION ABLATION SUMMARY ===")
baseline_deliveries = results["0_failures"][0]
for condition_name, (avg_deliveries, avg_match_rate, raw_list) in results.items():
    pct_of_baseline = (avg_deliveries / baseline_deliveries * 100) if baseline_deliveries > 0 else 0.0
    print(f"{condition_name}: avg {avg_deliveries:.1f} deliveries "
          f"({pct_of_baseline:.1f}% of no-failure baseline), "
          f"avg match rate {avg_match_rate:.1f}%, raw={raw_list}")

print("\nCompare against your WITH-communication system:")
print("0_failures: 14.2 avg deliveries (100.0%), 100.0% match rate")
print("1_failure: 13.8 avg deliveries (96.5%), 100.0% match rate")
print("2_failures: 10.8 avg deliveries (75.4%), 100.0% match rate")
    
```

### 10.18 `experiments/phase7_scalability_experiment.py`

- **Purpose:** Multi-robot fleet scalability benchmark (2, 4, 8 robots)
- **Relative Path:** `[experiments/phase7_scalability_experiment.py](file:///c:/Users/Kezia/warehouse-swarm/experiments/phase7_scalability_experiment.py)`
- **System Location:** `c:\Users\Kezia\warehouse-swarm\experiments\phase7_scalability_experiment.py`

```python
"""
Phase 7: scalability experiment. Runs the full pipeline (bidding +
eligibility-gated auction + A* pathfinding, no failures) at 2, 4, and 8
robots, to see how per-robot and total throughput change as swarm size
increases. Uses RWARE's built-in 8-agent config.
"""

import os
import sys
from pathlib import Path

# Ensure core and project root are on sys.path
_ROOT_DIR = Path(__file__).resolve().parent.parent
_CORE_DIR = _ROOT_DIR / "core"
for _p in [str(_CORE_DIR), str(_ROOT_DIR)]:
    if _p not in sys.path:
        sys.path.insert(0, _p)

import numpy as np
from multi_robot_vecenv import MultiRobotVecEnv
from auction_layer import DecentralizedAuctionLayer
from pathfinding import GridPathfinder

SEEDS = [1, 42, 100, 2024]
N_STEPS = 20000

# robot type assignments per swarm size (cycling fast_light/heavy_load/balanced)
SWARM_CONFIGS = {
    "2_robots": ("rware-tiny-2ag-v2", [0, 1]),
    "4_robots": ("rware-tiny-4ag-v2", [0, 1, 2, 0]),
    "8_robots": ("rware-tiny-8ag-v2", [0, 1, 2, 0, 1, 2, 0, 1]),
}


def run_seed(env_id, robot_types, seed):
    vec_env = MultiRobotVecEnv(env_id=env_id, robot_types=robot_types, seed=seed)
    obs = vec_env.reset()
    env = vec_env.hetero_env

    auction = DecentralizedAuctionLayer(env, bid_model_path="bid_head_trained.pt")
    pathfinder = GridPathfinder(env)

    for step in range(N_STEPS):
        pursue, bids, nearest_tasks = auction.step(obs)
        actions = pathfinder.get_actions_for_pursuit(obs, pursue, nearest_tasks)
        obs, rewards, dones, infos = vec_env.step(actions)
        auction.clear_stale_beliefs()

    total_matches = sum(env.match_count)
    total_mismatches = sum(env.mismatch_count)
    total_deliveries = total_matches + total_mismatches
    match_rate = (total_matches / total_deliveries * 100) if total_deliveries > 0 else 0.0
    n_robots = vec_env.n_robots

    vec_env.close()
    return total_deliveries, match_rate, n_robots


print(f"Running scalability experiment: {len(SWARM_CONFIGS)} swarm sizes x {len(SEEDS)} seeds, "
      f"{N_STEPS} steps each. This will take a while.\n")

results = {}

for config_name, (env_id, robot_types) in SWARM_CONFIGS.items():
    deliveries_list = []
    match_rate_list = []
    n_robots = None

    for seed in SEEDS:
        try:
            deliveries, match_rate, n_robots = run_seed(env_id, robot_types, seed)
        except Exception as e:
            print(f"  [{config_name}] seed={seed}: FAILED with error: {e}")
            continue

        deliveries_list.append(deliveries)
        match_rate_list.append(match_rate)
        print(f"  [{config_name}] seed={seed}: {deliveries} deliveries, {match_rate:.1f}% match rate")

    if not deliveries_list:
        print(f"  --> {config_name}: no successful runs, skipping\n")
        continue

    avg_deliveries = np.mean(deliveries_list)
    avg_match_rate = np.mean(match_rate_list)
    per_robot_deliveries = avg_deliveries / n_robots
    results[config_name] = (avg_deliveries, avg_match_rate, per_robot_deliveries, n_robots)
    print(f"  --> {config_name} AVERAGE: {avg_deliveries:.1f} deliveries "
          f"({per_robot_deliveries:.2f} per robot), {avg_match_rate:.1f}% match rate\n")

print("\n=== SCALABILITY SUMMARY ===")
for config_name, (avg_deliveries, avg_match_rate, per_robot, n_robots) in results.items():
    print(f"{config_name} ({n_robots} robots): {avg_deliveries:.1f} total deliveries, "
          f"{per_robot:.2f} per-robot deliveries, {avg_match_rate:.1f}% match rate")
```

### 10.19 `experiments/phase8_zone_experiment.py`

- **Purpose:** Hierarchical zone-based coordination scalability benchmark
- **Relative Path:** `[experiments/phase8_zone_experiment.py](file:///c:/Users/Kezia/warehouse-swarm/experiments/phase8_zone_experiment.py)`
- **System Location:** `c:\Users\Kezia\warehouse-swarm\experiments\phase8_zone_experiment.py`

```python
"""
Phase 8: Hierarchical Zone-Based Coordination Experiment

Compares three configurations to demonstrate that per-robot communication
overhead scales with ZONE SIZE (roughly constant) rather than TOTAL FLEET
SIZE (which grows) -- the central scalability claim of the hierarchical
zone extension.

Configurations
--------------
  baseline_flat_8    : 8 robots, 1 zone (flat), rware-tiny-8ag-v2
                       (same as Phase 7 scalability result -- sanity check)
  hierarchical_16_4z : 16 robots, 4 zones, rware-small-16ag-v2 (20x10 grid)
  hierarchical_16_4z_med : 16 robots, 4 zones, rware-medium-16ag-v2 (20x16 grid)

For the baseline (flat, 1 zone), we reuse DecentralizedAuctionLayer directly
with a wrapper that exposes the same comm-per-robot-per-step metric, so the
comparison is apples-to-apples.

Metrics collected per run
-------------------------
  - Total deliveries and match rate (same as Phase 6/7 experiments)
  - Avg comm events per robot per step (KEY scaling metric)
  - Cross-zone handoff rate (sanity check: should be low ~<5%)
  - Per-zone robot count and per-zone delivery breakdown

Seeds and steps: same as prior phases (SEEDS=[1,42,100,2024], N_STEPS=20000)
so results are directly comparable.
"""

import os
import sys
from pathlib import Path

# Ensure core, experiments, and project root are on sys.path
_ROOT_DIR = Path(__file__).resolve().parent.parent
_CORE_DIR = _ROOT_DIR / "core"
_EXP_DIR = _ROOT_DIR / "experiments"
for _p in [str(_CORE_DIR), str(_EXP_DIR), str(_ROOT_DIR)]:
    if _p not in sys.path:
        sys.path.insert(0, _p)

import numpy as np
from collections import defaultdict

from multi_robot_vecenv import MultiRobotVecEnv
from hetero_wrapper import HeterogeneousWarehouseWrapper
from zone_partitioner import ZonePartitioner
from hierarchical_coordinator import HierarchicalCoordinator

# For flat baseline — reuse existing auction + pathfinder
from auction_layer import DecentralizedAuctionLayer
from pathfinding import GridPathfinder

SEEDS = [1, 42, 100, 2024]
N_STEPS = 20_000

# ---------------------------------------------------------------------------
# Configuration table
# Confirmed env IDs (verified against installed RWARE before running):
#   rware-tiny-8ag-v2   -> grid (11,10), 8 agents
#   rware-small-16ag-v2 -> grid (20,10), 16 agents
#   rware-medium-16ag-v2-> grid (20,16), 16 agents
# ---------------------------------------------------------------------------

CONFIGS = {
    "baseline_flat_8": {
        "env_id": "rware-tiny-8ag-v2",
        "robot_types": [0, 1, 2, 0, 1, 2, 0, 1],
        "n_zones": 1,   # flat = single zone = existing system
        "mode": "flat",
        "description": "8 robots, flat (Phase 7 baseline replication)",
    },
    "hierarchical_16_small": {
        "env_id": "rware-small-16ag-v2",
        "robot_types": [0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0],
        "n_zones": 4,
        "mode": "hierarchical",
        "description": "16 robots, 4 zones, small grid (20x10)",
    },
    "hierarchical_16_medium": {
        "env_id": "rware-medium-16ag-v2",
        "robot_types": [0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0],
        "n_zones": 4,
        "mode": "hierarchical",
        "description": "16 robots, 4 zones, medium grid (20x16)",
    },
}


# ===========================================================================
# Flat baseline runner (reuses existing DecentralizedAuctionLayer verbatim)
# Adds comm-per-robot-per-step measurement for fair comparison.
# ===========================================================================

class _FlatAuctionWithMetrics:
    """
    Thin wrapper around DecentralizedAuctionLayer that counts per-step
    intra-robot communication events (same definition as ZoneAuctionLayer:
    one event = one bid or claim broadcast that reaches a peer within range).
    """

    def __init__(self, hetero_env, bid_model_path="bid_head_trained.pt"):
        self._auction = DecentralizedAuctionLayer(hetero_env, bid_model_path)
        self.comm_events_per_step = []
        self.env = hetero_env
        self.n_robots = hetero_env.unwrapped.n_agents

    def step(self, obs_batch, belief_failed=None (bf=None)):
        pursue, bids, nearest_tasks = self._auction.step(obs_batch, belief_failed)
        # Count how many same-range broadcasts happened this step by
        # re-examining distances (mirrors ZoneAuctionLayer's counter)
        agents = self.env.unwrapped.agents
        events = 0
        for i, agent_i in enumerate(agents):
            for j, agent_j in enumerate(agents):
                if i == j:
                    continue
                dist = abs(agent_i.x - agent_j.x) + abs(agent_i.y - agent_j.y)
                if dist <= DecentralizedAuctionLayer.COMM_RANGE:
                    events += 1
        # events double-counts (i->j and j->i both counted); halve for consistency
        self.comm_events_per_step.append(events // 2)
        return pursue, bids, nearest_tasks

    def clear_stale_beliefs(self):
        self._auction.clear_stale_beliefs()

    def avg_comm_per_robot_per_step(self) -> float:
        if not self.comm_events_per_step:
            return 0.0
        return np.mean(self.comm_events_per_step) / self.n_robots


# ===========================================================================
# Run helpers
# ===========================================================================

def run_flat(env_id, robot_types, seed):
    """Run the flat (existing) system with comm metric instrumentation."""
    vec_env = MultiRobotVecEnv(env_id=env_id, robot_types=robot_types, seed=seed)
    obs = vec_env.reset()
    env = vec_env.hetero_env

    auction = _FlatAuctionWithMetrics(env)
    pathfinder = GridPathfinder(env)

    for step in range(N_STEPS):
        pursue, bids, nearest_tasks = auction.step(obs)
        actions = pathfinder.get_actions_for_pursuit(obs, pursue, nearest_tasks)
        obs, rewards, dones, infos = vec_env.step(actions)
        auction.clear_stale_beliefs()

    total_matches = sum(env.match_count)
    total_mismatches = sum(env.mismatch_count)
    total_deliveries = total_matches + total_mismatches
    match_rate = (total_matches / total_deliveries * 100) if total_deliveries > 0 else 0.0
    comm_per_robot = auction.avg_comm_per_robot_per_step()
    n_robots = vec_env.n_robots

    vec_env.close()
    return {
        "deliveries": total_deliveries,
        "match_rate": match_rate,
        "comm_per_robot_per_step": comm_per_robot,
        "n_robots": n_robots,
        "cross_zone_handoffs": 0,
        "cross_zone_handoff_rate": 0.0,
    }


def run_hierarchical(env_id, robot_types, n_zones, seed):
    """Run the hierarchical zone system."""
    vec_env = MultiRobotVecEnv(env_id=env_id, robot_types=robot_types, seed=seed)
    obs = vec_env.reset()
    env = vec_env.hetero_env

    # Build zone partitioner
    ua = env.unwrapped
    grid_w, grid_h = ua.grid_size[1], ua.grid_size[0]
    partitioner = ZonePartitioner(grid_w, grid_h, n_zones=n_zones)

    # Assign robots to zones based on start positions
    zone_assignment = partitioner.assign_robots_to_zones(ua.agents)

    # Print zone setup on first seed
    if seed == SEEDS[0]:
        partitioner.print_zone_map()
        partitioner.report_robot_assignment(zone_assignment)

    # Build coordinator
    coordinator = HierarchicalCoordinator(
        env, partitioner, zone_assignment, bid_model_path="bid_head_trained.pt"
    )

    for step in range(N_STEPS):
        pursue, bids, nearest_tasks = coordinator.step(obs)
        actions = coordinator.pathfinder.get_actions_for_pursuit(
            obs, pursue, nearest_tasks
        )
        actions = coordinator.get_fault_action_overrides(actions)
        obs, rewards, dones, infos = vec_env.step(actions)
        coordinator.clear_stale_beliefs()

    total_matches = coordinator.total_match_count()
    total_mismatches = coordinator.total_mismatch_count()
    total_deliveries = total_matches + total_mismatches
    match_rate = (total_matches / total_deliveries * 100) if total_deliveries > 0 else 0.0
    comm_per_robot = coordinator.avg_comm_per_robot_per_step()
    handoffs = coordinator.cross_zone_handoffs
    tasks_seen = coordinator.total_tasks_seen
    handoff_rate = (handoffs / tasks_seen * 100) if tasks_seen > 0 else 0.0
    n_robots = vec_env.n_robots

    vec_env.close()
    return {
        "deliveries": total_deliveries,
        "match_rate": match_rate,
        "comm_per_robot_per_step": comm_per_robot,
        "n_robots": n_robots,
        "cross_zone_handoffs": handoffs,
        "cross_zone_handoff_rate": handoff_rate,
    }


# ===========================================================================
# Main experiment loop
# ===========================================================================

print("=" * 70)
print("Phase 8: Hierarchical Zone-Based Coordination Experiment")
print(f"Configs: {len(CONFIGS)}  |  Seeds: {SEEDS}  |  Steps: {N_STEPS:,}")
print("=" * 70)
print()

all_results = {}

for config_name, cfg in CONFIGS.items():
    print(f"\n--- {config_name} ---")
    print(f"    {cfg['description']}")
    print(f"    env={cfg['env_id']}, n_zones={cfg['n_zones']}")
    print()

    seed_results = []
    for seed in SEEDS:
        try:
            if cfg["mode"] == "flat":
                r = run_flat(cfg["env_id"], cfg["robot_types"], seed)
            else:
                r = run_hierarchical(
                    cfg["env_id"], cfg["robot_types"], cfg["n_zones"], seed
                )
            seed_results.append(r)
            print(f"  seed={seed}: {r['deliveries']} deliveries, "
                  f"{r['match_rate']:.1f}% match, "
                  f"comm/robot/step={r['comm_per_robot_per_step']:.4f}, "
                  f"handoffs={r['cross_zone_handoffs']} "
                  f"({r['cross_zone_handoff_rate']:.1f}%)")
        except Exception as e:
            print(f"  seed={seed}: FAILED -> {type(e).__name__}: {e}")
            import traceback; traceback.print_exc()

    if not seed_results:
        print(f"  --> {config_name}: no successful runs, skipping")
        continue

    # Aggregate
    avg = {
        key: np.mean([r[key] for r in seed_results])
        for key in ["deliveries", "match_rate", "comm_per_robot_per_step",
                    "cross_zone_handoffs", "cross_zone_handoff_rate"]
    }
    avg["n_robots"] = seed_results[0]["n_robots"]
    avg["per_robot_deliveries"] = avg["deliveries"] / avg["n_robots"]

    all_results[config_name] = avg
    print(f"\n  --> AVERAGE: {avg['deliveries']:.1f} deliveries "
          f"({avg['per_robot_deliveries']:.2f}/robot), "
          f"{avg['match_rate']:.1f}% match, "
          f"comm/robot/step={avg['comm_per_robot_per_step']:.4f}, "
          f"handoff rate={avg['cross_zone_handoff_rate']:.1f}%\n")


# ===========================================================================
# Summary table
# ===========================================================================

print("\n" + "=" * 70)
print("PHASE 8 RESULTS SUMMARY")
print("=" * 70)

header = (f"{'Config':<28} {'Robots':>6} {'Zones':>5} "
          f"{'Deliveries':>10} {'Match%':>7} "
          f"{'Comm/robot/step':>16} {'Handoff%':>9}")
print(header)
print("-" * len(header))

for config_name, r in all_results.items():
    cfg = CONFIGS[config_name]
    print(f"{config_name:<28} {r['n_robots']:>6} {cfg['n_zones']:>5} "
          f"{r['deliveries']:>10.1f} {r['match_rate']:>7.1f} "
          f"{r['comm_per_robot_per_step']:>16.4f} "
          f"{r['cross_zone_handoff_rate']:>9.1f}")

print()
print("KEY SCALING CLAIM:")
if "baseline_flat_8" in all_results and "hierarchical_16_small" in all_results:
    flat_comm = all_results["baseline_flat_8"]["comm_per_robot_per_step"]
    hier_comm = all_results["hierarchical_16_small"]["comm_per_robot_per_step"]
    ratio = hier_comm / flat_comm if flat_comm > 0 else float("nan")
    print(f"  Flat (8 robots)   comm/robot/step: {flat_comm:.4f}")
    print(f"  Zoned (16 robots) comm/robot/step: {hier_comm:.4f}")
    print(f"  Ratio (zoned/flat): {ratio:.2f}x")
    if ratio < 1.5:
        print("  -> Per-robot comm overhead stays roughly CONSTANT "
              "as fleet doubles (zoning works as intended).")
    else:
        print("  -> Per-robot comm overhead increased — review zone sizing.")
else:
    print("  (not enough configs completed to compute scaling ratio)")

print()
print("Note: Match rate should be 100% for all configs (eligibility gate unchanged).")
print("Note: Handoff rate should be low (<10%) if zone sizing is adequate.")

```

### 10.20 `visualization/run_simulation_gui.py`

- **Purpose:** Interactive animated Tkinter graphical interface with real-time fault injection
- **Relative Path:** `[visualization/run_simulation_gui.py](file:///c:/Users/Kezia/warehouse-swarm/visualization/run_simulation_gui.py)`
- **System Location:** `c:\Users\Kezia\warehouse-swarm\visualization\run_simulation_gui.py`

```python
"""
Warehouse Swarm Simulation Visualizer (Live Interactive Tkinter GUI)

Patched:
  - Episode-aware: RWARE resets the world every few hundred steps. The GUI
    now shows the episode number / step-in-episode and resets coordinator
    state on every reset.
  - Diagnostics: shows how many live robots are idle (NOOP) and how many
    are carrying a shelf, so you can tell "stuck" from "working".
"""

import sys
import os
from pathlib import Path
import time
import tkinter as tk
from tkinter import ttk

# Ensure core and project root are on sys.path
_ROOT_DIR = Path(__file__).resolve().parent.parent
_CORE_DIR = _ROOT_DIR / "core"
_EXP_DIR = _ROOT_DIR / "experiments"
for _p in [str(_CORE_DIR), str(_EXP_DIR), str(_ROOT_DIR)]:
    if _p not in sys.path:
        sys.path.insert(0, _p)

import numpy as np
from multi_robot_vecenv import MultiRobotVecEnv
from zone_partitioner import ZonePartitioner
from hierarchical_coordinator import HierarchicalCoordinator


class WarehouseGUI:
    # Color palette
    ZONE_COLORS = ["#f8f9fa", "#f1f3f5", "#e9ecef", "#dee2e6"]
    ZONE_BORDER = "#adb5bd"

    ROBOT_COLORS = {
        0: "#228be6",  # fast_light -> Blue
        1: "#fa5252",  # heavy_load -> Red
        2: "#40c057",  # balanced   -> Green
    }
    FAILED_ROBOT_COLOR = "#495057"

    TASK_COLORS = {
        0: "#15aabf",
        1: "#e64980",
        2: "#fab005",
    }

    CELL_SIZE = 36  # pixels per grid cell

    def __init__(self, root, env_id="rware-small-16ag-v2", n_zones=4, seed=42):
        self.root = root
        self.root.title("Warehouse Swarm — Hierarchical Zone-Based Multi-Robot Coordination")
        self.root.configure(bg="#212529")

        self.running = False
        self.step_delay = 80  # ms between steps
        self.step_count = 0
        self.episode = 1
        self.episode_steps = 0

        self.n_robots = 16
        self.robot_types = [0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0]
        self.last_actions = [0] * self.n_robots

        self.vec_env = MultiRobotVecEnv(
            env_id=env_id, robot_types=self.robot_types, seed=seed
        )
        self.obs = self.vec_env.reset()
        self.env = self.vec_env.hetero_env
        self.ua = self.env.unwrapped

        # RWARE grid_size is (rows, cols) -> (height, width)
        self.grid_h, self.grid_w = self.ua.grid_size[0], self.ua.grid_size[1]

        self.partitioner = ZonePartitioner(self.grid_w, self.grid_h, n_zones=n_zones)
        self.zone_assignment = self.partitioner.assign_robots_to_zones(self.ua.agents)
        self.partitioner.report_robot_assignment(self.zone_assignment)
        self.coordinator = HierarchicalCoordinator(
            self.env, self.partitioner, self.zone_assignment, bid_model_path="bid_head_trained.pt"
        )

        self._build_ui()
        self._draw_grid_base()
        self._update_display()

        self.root.update_idletasks()
        self.root.geometry("")

    def _build_ui(self):
        header = tk.Frame(self.root, bg="#343a40", padx=16, pady=10)
        header.pack(fill=tk.X)

        title = tk.Label(
            header,
            text="Autonomous Warehouse Swarm — Hierarchical Coordination Simulation",
            font=("Segoe UI", 15, "bold"),
            fg="#f8f9fa",
            bg="#343a40",
        )
        title.pack(side=tk.LEFT)

        subtitle = tk.Label(
            header,
            text="16 Robots | 4 Spatial Zones | Decentralized Auctions + A*",
            font=("Segoe UI", 10),
            fg="#adb5bd",
            bg="#343a40",
        )
        subtitle.pack(side=tk.RIGHT)

        body = tk.Frame(self.root, bg="#212529", padx=12, pady=12)
        body.pack(fill=tk.BOTH, expand=True)

        canvas_frame = tk.Frame(body, bg="#111", relief=tk.RIDGE, bd=2)
        canvas_frame.pack(side=tk.LEFT, padx=(0, 12), fill=tk.BOTH, expand=True)

        c_w = self.grid_w * self.CELL_SIZE
        c_h = self.grid_h * self.CELL_SIZE
        self.canvas = tk.Canvas(
            canvas_frame, width=c_w, height=c_h, bg="#ffffff", highlightthickness=0
        )
        self.canvas.pack(padx=8, pady=8)

        sidebar = tk.Frame(body, bg="#2b3035", width=340, padx=14, pady=10)
        sidebar.pack(side=tk.RIGHT, fill=tk.Y)

        # Controls
        ctrl_frame = tk.LabelFrame(
            sidebar, text=" Controls ", font=("Segoe UI", 10, "bold"),
            fg="#ced4da", bg="#2b3035", padx=10, pady=8
        )
        ctrl_frame.pack(fill=tk.X, pady=(0, 10))

        self.btn_run = tk.Button(
            ctrl_frame, text="▶ Start Live Sim", bg="#2b8a3e", fg="white",
            font=("Segoe UI", 10, "bold"), relief=tk.FLAT, padx=10, pady=6,
            command=self.toggle_run
        )
        self.btn_run.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)

        self.btn_step = tk.Button(
            ctrl_frame, text="Step ❯", bg="#495057", fg="white",
            font=("Segoe UI", 10), relief=tk.FLAT, padx=8, pady=6,
            command=self.step_simulation
        )
        self.btn_step.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)

        speed_frame = tk.Frame(sidebar, bg="#2b3035")
        speed_frame.pack(fill=tk.X, pady=(0, 10))
        tk.Label(speed_frame, text="Speed:", bg="#2b3035", fg="#ced4da", font=("Segoe UI", 9)).pack(side=tk.LEFT)
        self.speed_slider = tk.Scale(
            speed_frame, from_=20, to=250, orient=tk.HORIZONTAL,
            bg="#2b3035", fg="#ced4da", highlightthickness=0,
            command=lambda v: setattr(self, 'step_delay', int(v))
        )
        self.speed_slider.set(self.step_delay)
        self.speed_slider.pack(side=tk.RIGHT, fill=tk.X, expand=True)

        # Metrics
        stats_box = tk.LabelFrame(
            sidebar, text=" Swarm Metrics ", font=("Segoe UI", 10, "bold"),
            fg="#ced4da", bg="#2b3035", padx=10, pady=8
        )
        stats_box.pack(fill=tk.X, pady=(0, 10))

        self.lbl_step = tk.Label(stats_box, text="Step: 0", font=("Segoe UI", 10), bg="#2b3035", fg="#f8f9fa", anchor="w")
        self.lbl_step.pack(fill=tk.X)

        self.lbl_episode = tk.Label(stats_box, text="Episode: 1 (step 0)", font=("Segoe UI", 10), bg="#2b3035", fg="#f8f9fa", anchor="w")
        self.lbl_episode.pack(fill=tk.X)

        self.lbl_deliveries = tk.Label(stats_box, text="Deliveries: 0", font=("Segoe UI", 10, "bold"), bg="#2b3035", fg="#69db7c", anchor="w")
        self.lbl_deliveries.pack(fill=tk.X)

        self.lbl_match = tk.Label(stats_box, text="Capability Match Rate: 100.0%", font=("Segoe UI", 10), bg="#2b3035", fg="#74c0fc", anchor="w")
        self.lbl_match.pack(fill=tk.X)

        self.lbl_handoffs = tk.Label(stats_box, text="Cross-Zone Handoffs: 0", font=("Segoe UI", 10), bg="#2b3035", fg="#ffd43b", anchor="w")
        self.lbl_handoffs.pack(fill=tk.X)

        self.lbl_comm = tk.Label(stats_box, text="Comm Events/Robot/Step: 0.00", font=("Segoe UI", 10), bg="#2b3035", fg="#ced4da", anchor="w")
        self.lbl_comm.pack(fill=tk.X)

        self.lbl_idle = tk.Label(stats_box, text="Idle robots (NOOP): 0/16", font=("Segoe UI", 10), bg="#2b3035", fg="#ced4da", anchor="w")
        self.lbl_idle.pack(fill=tk.X)

        self.lbl_carry = tk.Label(stats_box, text="Carrying a shelf: 0", font=("Segoe UI", 10), bg="#2b3035", fg="#ced4da", anchor="w")
        self.lbl_carry.pack(fill=tk.X)

        self.lbl_stranded = tk.Label(stats_box, text="Stranded Tasks (Awaiting Rescue): 0", font=("Segoe UI", 10), bg="#2b3035", fg="#ff8787", anchor="w")
        self.lbl_stranded.pack(fill=tk.X)

        # Fault injection
        fault_box = tk.LabelFrame(
            sidebar, text=" Fault Injection (Test Resilience) ",
            font=("Segoe UI", 10, "bold"), fg="#ff8787", bg="#2b3035", padx=10, pady=8
        )
        fault_box.pack(fill=tk.X, pady=(0, 10))

        fault_select_frame = tk.Frame(fault_box, bg="#2b3035")
        fault_select_frame.pack(fill=tk.X, pady=4)
        tk.Label(fault_select_frame, text="Select Robot:", bg="#2b3035", fg="#ced4da", font=("Segoe UI", 9)).pack(side=tk.LEFT)

        self.fault_robot_var = tk.StringVar(value="Robot 1")
        options = [f"Robot {i} ({self.env.TYPE_NAMES[self.robot_types[i]]})" for i in range(self.n_robots)]
        self.combo_fault = ttk.Combobox(
            fault_select_frame, textvariable=self.fault_robot_var, values=options, state="readonly", width=18
        )
        self.combo_fault.pack(side=tk.RIGHT)

        self.btn_fail = tk.Button(
            fault_box, text="⚡ Inject Failure", bg="#e03131", fg="white",
            font=("Segoe UI", 9, "bold"), relief=tk.FLAT, pady=4,
            command=self.inject_selected_failure
        )
        self.btn_fail.pack(fill=tk.X, pady=(4, 0))

        # Legend
        legend_box = tk.LabelFrame(
            sidebar, text=" Legend ", font=("Segoe UI", 10, "bold"),
            fg="#ced4da", bg="#2b3035", padx=10, pady=8
        )
        legend_box.pack(fill=tk.X)

        self._add_legend_item(legend_box, "#228be6", "Robot: fast_light")
        self._add_legend_item(legend_box, "#fa5252", "Robot: heavy_load")
        self._add_legend_item(legend_box, "#40c057", "Robot: balanced")
        self._add_legend_item(legend_box, "#495057", "Robot: Failed / Deactivated")
        self._add_legend_item(legend_box, "#fab005", "Gold: Requested Task Shelf (U/H/S)")
        self._add_legend_item(legend_box, "#20c997", "Green Base: Delivery Goal")

    def _add_legend_item(self, parent, color, text):
        row = tk.Frame(parent, bg="#2b3035")
        row.pack(fill=tk.X, pady=2)
        box = tk.Canvas(row, width=14, height=14, bg=color, highlightthickness=0)
        box.pack(side=tk.LEFT, padx=(0, 6))
        lbl = tk.Label(row, text=text, font=("Segoe UI", 8), bg="#2b3035", fg="#ced4da")
        lbl.pack(side=tk.LEFT)

    def _draw_grid_base(self):
        self.canvas.delete("base")
        c = self.CELL_SIZE

        for z in range(self.partitioner.n_zones):
            col = z // self.partitioner.n_rows
            row = z % self.partitioner.n_rows
            x1 = col * self.partitioner.zone_col_width * c
            y1 = row * self.partitioner.zone_row_height * c
            x2 = (col + 1) * self.partitioner.zone_col_width * c
            y2 = (row + 1) * self.partitioner.zone_row_height * c

            bg_color = self.ZONE_COLORS[z % len(self.ZONE_COLORS)]
            self.canvas.create_rectangle(
                x1, y1, x2, y2, fill=bg_color, outline=self.ZONE_BORDER, width=2, tags="base"
            )
            self.canvas.create_text(
                x1 + 45, y1 + 18, text=f"ZONE {z}", fill="#ced4da",
                font=("Segoe UI", 11, "bold"), tags="base"
            )

        for gx, gy in self.ua.goals:
            self.canvas.create_rectangle(
                gx * c, gy * c, (gx + 1) * c, (gy + 1) * c,
                fill="#8ce99a", outline="#37b24d", width=2, tags="base"
            )

    def _update_display(self):
        self.canvas.delete("dynamic")
        c = self.CELL_SIZE

        # 1. Shelves
        active_shelves = set(self.ua.request_queue)
        for shelf in self.ua.shelfs:
            sx, sy = shelf.x, shelf.y
            is_requested = shelf in active_shelves
            fill_c = "#ffd43b" if is_requested else "#adb5bd"
            outline_c = "#e67700" if is_requested else "#868e96"

            self.canvas.create_rectangle(
                sx * c + 5, sy * c + 5, (sx + 1) * c - 5, (sy + 1) * c - 5,
                fill=fill_c, outline=outline_c, width=1.5, tags="dynamic"
            )
            if is_requested:
                task_t = self.env.shelf_task_type.get(shelf, 2)
                task_char = ["U", "H", "S"][task_t]
                self.canvas.create_text(
                    sx * c + c / 2, sy * c + c / 2, text=task_char,
                    fill="#343a40", font=("Segoe UI", 8, "bold"), tags="dynamic"
                )

        # 2. Robots
        failed_flags = self.coordinator.fault_layers[0].actual_failed
        n_carrying = 0
        for i, agent in enumerate(self.ua.agents):
            rx, ry = agent.x, agent.y
            r_type = self.robot_types[i]
            is_failed = failed_flags[i]

            robot_col = self.FAILED_ROBOT_COLOR if is_failed else self.ROBOT_COLORS[r_type]
            outline_col = "#ff6b6b" if (is_failed and agent.carrying_shelf is not None) else "#212529"
            outline_w = 3 if (is_failed and agent.carrying_shelf is not None) else 2

            self.canvas.create_oval(
                rx * c + 3, ry * c + 3, (rx + 1) * c - 3, (ry + 1) * c - 3,
                fill=robot_col, outline=outline_col, width=outline_w, tags="dynamic"
            )

            if agent.carrying_shelf is not None:
                n_carrying += 1
                self.canvas.create_oval(
                    rx * c + 8, ry * c + 8, (rx + 1) * c - 8, (ry + 1) * c - 8,
                    fill="#ffe066", outline="#f08c00", width=1.5, tags="dynamic"
                )

            self.canvas.create_text(
                rx * c + c / 2, ry * c + c / 2, text=str(i),
                fill="white" if not agent.carrying_shelf else "#212529",
                font=("Segoe UI", 9, "bold"), tags="dynamic"
            )

        # 3. Sidebar
        tot_d = self.coordinator.total_match_count() + self.coordinator.total_mismatch_count()
        match_rate = (
            (self.coordinator.total_match_count() / tot_d * 100) if tot_d > 0 else 100.0
        )
        n_idle = sum(
            1 for i, a in enumerate(self.last_actions)
            if a == 0 and not failed_flags[i]
        )
        n_alive = sum(1 for f in failed_flags if not f)
        n_stranded = len(self.coordinator.stranded_tasks)

        self.lbl_step.config(text=f"Step: {self.step_count:,}")
        self.lbl_episode.config(text=f"Episode: {self.episode} (step {self.episode_steps})")
        self.lbl_deliveries.config(text=f"Deliveries: {tot_d}")
        self.lbl_match.config(text=f"Capability Match Rate: {match_rate:.1f}%")
        self.lbl_handoffs.config(text=f"Cross-Zone Handoffs: {self.coordinator.cross_zone_handoffs}")
        self.lbl_comm.config(
            text=f"Comm Events/Robot/Step: {self.coordinator.avg_comm_per_robot_per_step():.3f}"
        )
        self.lbl_idle.config(text=f"Idle robots (NOOP): {n_idle}/{n_alive}")
        self.lbl_carry.config(text=f"Carrying a shelf: {n_carrying}")
        self.lbl_stranded.config(text=f"Stranded Tasks (Awaiting Rescue): {n_stranded}")

    def step_simulation(self):
        try:
            pursue, bids, nearest_tasks = self.coordinator.step(self.obs)
            actions = self.coordinator.pathfinder.get_actions_for_pursuit(
                self.obs, pursue, nearest_tasks
            )
            actions = self.coordinator.get_fault_action_overrides(actions)
            self.last_actions = list(actions)

            self.obs, rewards, dones, infos = self.vec_env.step(actions)
            self.step_count += 1
            self.episode_steps += 1

            if np.any(dones):
                # RWARE ended the episode and auto-reset the world
                self.episode += 1
                self.episode_steps = 0
                self.coordinator.reset_episode()

            self.coordinator.clear_stale_beliefs()
        except Exception as e:
            print(f"[GUI] Step error at step {self.step_count}: {e}")
            import traceback; traceback.print_exc()
            self.running = False
            self.btn_run.config(text="▶ Start Live Sim", bg="#2b8a3e")
            return

        self._update_display()

    def _loop(self):
        if self.running:
            self.step_simulation()
            self.root.after(self.step_delay, self._loop)

    def toggle_run(self):
        self.running = not self.running
        if self.running:
            self.btn_run.config(text="⏸ Pause", bg="#f59f00")
            self._loop()
        else:
            self.btn_run.config(text="▶ Start Live Sim", bg="#2b8a3e")

    def inject_selected_failure(self):
        selection = self.combo_fault.get()
        if selection:
            robot_id = int(selection.split()[1])
            self.coordinator.inject_failure(robot_id)
            print(f"[UI] Injected physical failure into Robot {robot_id}")
            self._update_display()


def main():
    root = tk.Tk()
    app = WarehouseGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
```
