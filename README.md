
# Mobile Robotics Project: Reactive Navigation via Hybrid Reciprocal Velocity Obstacles (HRVO)

**Course:** Mobile Robotics (MR)  
**Status:** ✅ All Phases Complete (VO/RVO/HRVO, Multi-Agent, Maze Demo)  
**Language:** MATLAB (Simulation)

## 📌 Project Overview
This project explores the transition from static path planning to dynamic, reactive navigation in multi-agent environments. While traditional algorithms like A* or Dijkstra provide optimal paths in immutable environments, they fail in chaotic environments where obstacles (pedestrians, other robots) are in constant motion.

Our objective is to implement and simulate **Velocity Obstacles (VO)** and its advanced variants, **Reciprocal Velocity Obstacles (RVO)** and **Hybrid Reciprocal Velocity Obstacles (HRVO)**. These algorithms shift the planning perspective from the Cartesian configuration space ($x, y$) to the **velocity space** ($v_x, v_y$), allowing agents to anticipate collisions before they happen.

**Architectural Pivot:** Following recent advisement, we have moved away from a ROS/Gazebo visualization bridge. Instead, we are focusing on a **Layered Planning Architecture**, demonstrating how our reactive planners (Local Layer) can be integrated with established global path planners (Global Layer) to navigate large, complex maps.

## 🚀 Current Status
| Feature | Status | Description |
| :--- | :--- | :--- |
| **Framework** | ✅ Complete | Modular simulation engine (`main_simulation.m`) with unicycle kinematics. |
| **Phase 1: VO** | ✅ Complete | Dynamic Velocity Obstacles with multi-speed sampling in `plan_VO.m`. |
| **Phase 2: RVO** | ✅ Complete | Reciprocal Velocity Obstacles solving oscillation problem in `plan_RVO_new.m`. |
| **Phase 3: HRVO** | ✅ Complete | Hybrid RVO with true geometric apex intersection in `plan_HRVO_new.m`. |
| **Phase 4: Multi-Agent** | ✅ Complete | Full N-robot system in `multi_agent_simulation.m` with parallel planning. |
| **Phase 5: Scenarios** | ✅ Complete | Extended scenarios: `crossing_4`, `swarm_8`, `dense_crowd`, `wall_corridor`, `interactive`. |
| **Phase 6: Maze Demo** | ✅ Complete | A* global planner + local VO navigation in `maze_demo.m`. |

## 📂 Repository Structure
The repository is organized to separate the physical simulation from the algorithmic "brain".

```text
MR-Project-VOs/
│
├── main_simulation.m          # ENTRY POINT — Original 2-robot simulation
├── multi_agent_simulation.m   # NEW — N-robot multi-agent simulation (Phase 4)
├── maze_demo.m                # NEW — A* + VO hierarchical navigation (Phase 6)
├── benchmark_algorithms.m     # NEW — Compare VO/RVO/HRVO across all scenarios
├── demo_presentation.m        # NEW — Quick demo script for presentations
├── README.md                  # Project documentation
├── DEVELOPMENT_PLAN.md        # Technical roadmap for Phase 4-6
│
├── classes/                   # Physical object definitions
│   ├── Robot.m                # Robot state (pos, vel, status) + Unicycle Kinematics
│   ├── Obstacle.m             # Standardized object for Static Walls, Dynamic Blocks
│   └── Simulator.m            # (Future use)
│
├── algorithms/                # The "Brains" - Path Planning Logic
│   ├── plan_VO.m              # PHASE 1 — Dynamic Velocity Obstacles (✅)
│   ├── plan_RVO_new.m         # PHASE 2 — Reciprocal VOs (✅)
│   └── plan_HRVO_new.m        # PHASE 3 — Hybrid RVOs (✅)
│
├── global_planner/            # NEW — Global Path Planning (Phase 6)
│   ├── astar_grid.m           # A* pathfinding on occupancy grid
│   ├── generate_maze.m        # Maze/environment generator
│   ├── waypoint_follower.m    # Hierarchical planner integration
│   ├── path_to_world.m        # Grid-to-world coordinate conversion
│   └── world_to_cell.m        # World-to-grid coordinate conversion
│
├── scenarios/
│   ├── VOs/                   # Original 2-Robot Scenarios
│   │   ├── basic.m            # Random Static Blocks
│   │   ├── u_trap.m           # Local Minimum Stress Test
│   │   ├── setup_hallway.m    # Two Robots Head-On
│   │   ├── somewhat_busy.m    # Robot vs Moving Obstacles
│   │   └── very_busy.m        # Complex Dynamic Environment
│   │
│   └── multi_agent/           # NEW — Multi-Agent Scenarios (Phase 5)
│       ├── crossing_4.m       # 4-way intersection (4 robots)
│       ├── swarm_8.m          # Circle swap (8 robots)
│       ├── dense_crowd.m      # High-density dynamic (2 robots + 20+ agents)
│       ├── wall_corridor.m    # Constrained space navigation
│       └── interactive.m      # User-placed robots via mouse clicks
│
├── utils/                     # Math & Helper Functions
│   ├── check_angles.m         # Angular interval checking
│   ├── check_collision.m      # Collision verification
│   ├── get_tangents.m         # Tangent line calculations
│   ├── intersect_rays.m       # Ray intersection for HRVO
│   └── plot_cone.m            # Cone visualization helper
│
└── output/                    # Generated video recordings
````

## 🛠️ Usage & Installation

### Prerequisites

  * MATLAB (R2020b or newer recommended).
  * `Image Processing Toolbox` (used for `viscircles` function).

### Running the Simulation

1.  **Clone the repository.**
2.  Open `main_simulation.m` in MATLAB.
3.  **Select Algorithm & Scenario:**
    Edit the configuration section at the top of the file:
    ```matlab
    % Algorithm Selector: 1 = VO, 2 = RVO, 3 = HRVO
    ALGORITHM = 1; 

    % Scenario Selector: 1 = Basic, 2 = U-Trap, 3 = Hallway, 4 = Busy, 5 = Very Busy
    SCENARIO_ID = 3; 
    ```
4.  **Run:** Click "Run" or press F5.
5.  **Output:** Videos saved to `output/` folder.

### Running Multi-Agent Simulations (NEW!)

1. Open `multi_agent_simulation.m` in MATLAB.
2. Configure:
   ```matlab
   ALGORITHM = 1;    % 1=VO, 2=RVO, 3=HRVO
   SCENARIO_ID = 1;  % 1=crossing_4, 2=swarm_8, 3=dense_crowd, 4=wall_corridor, 5=interactive
   ```
3. Run and watch N robots coordinate!

### Maze Demo with A* + Local Avoidance (Phase 6)

1. Open `maze_demo.m` in MATLAB.
2. Configure at top of file:
   ```matlab
   config.local_planner = 'VO';    % 'VO', 'RVO', or 'HRVO'
   config.maze_type = 'simple';    % Maze layout
   config.record_video = true;     % Auto-save video to output/
   ```
3. Run to see hierarchical planning:
   - **Global layer:** A* computes waypoints through the maze
   - **Local layer:** VO/RVO/HRVO avoids walls and other robots in real-time
4. Features:
   - Dual-view visualization (global path + local avoidance)
   - Wall obstacles visible in local view
   - Avoidance indicator (title turns red when deviating from path)
   - Multi-robot support (main robot + 2 other agents)
   - Dark theme for presentation visibility

### Quick Demos for Presentations

```matlab
% Open demo_presentation.m
DEMO_TYPE = 2;   % 2=crossing, 3=swarm, 4=dense, 5=corridor
ALGORITHM = 1;   % 1=VO, 2=RVO, 3=HRVO
% Run!
```

### Benchmark All Algorithms

```matlab
% Run benchmark_algorithms.m
% Compares VO/RVO/HRVO on all scenarios automatically
% Outputs a summary table with success rates and times
```

## 🧠 Architectural Highlights

### 1\. "Hot-Swappable" Brains

We avoid hardcoding navigation logic into the robot. Instead, `main_simulation.m` acts as the game engine. It passes the current state to a specific planner function (e.g., `plan_VO`), which returns an optimal velocity vector ($v_{opt}$). This allows us to test VO, RVO, and HRVO side-by-side on identical scenarios.

### 2\. Differential Drive Controller

The planners output a **holonomic** velocity vector ($v_x, v_y$). Since our robot works on a Unicycle Model (non-holonomic), we implement a low-level P-controller in `classes/Robot.m` to track this vector:

  * **Target Heading:** $\theta_{target} = \text{atan2}(v_y, v_x)$
  * **Angular Velocity:** $\omega = K_p \times \text{angdiff}(\theta_{current}, \theta_{target})$
  * **Linear Velocity:** $v = \|v_{opt}\| \times \cos(\text{error})$.

### 3\. Layered Planning (New\!)

Instead of relying solely on reactive logic, which can get stuck in large U-shaped traps (Local Minima), we are implementing a layered approach:

  * **Global Layer:** Calculates a high-level path (waypoints) from Start to Goal using a standard algorithm (e.g., A\*).
  * **Local Layer (VO/HRVO):** Follows the global waypoints while actively dodging dynamic obstacles in real-time.

## 📚 Theoretical Background

### Velocity Obstacles (VO) - *Implemented* [Fiorini & Shiller, 1998]

VO answers the question: *"If I choose velocity $v$, will I collide with obstacle $B$ at any future time?"*

**Mathematical Definition:**
$$VO_{A|B} = \{ v_A \mid \exists t > 0 : p_A + v_A \cdot t \in D(p_B + v_B \cdot t, r_A + r_B) \}$$

This is equivalent to translating the **Collision Cone** by $v_B$:
$$VO_{A|B} = v_B + CC_{A|B}$$

Where the cone half-angle is: $\alpha = \arcsin\left(\frac{r_A + r_B}{\|p_B - p_A\|}\right)$

**Limitation:** Assumes obstacles maintain constant velocity. Causes oscillation when two reactive agents meet (the "Mirror Effect").

### Reciprocal Velocity Obstacles (RVO) - *Implemented* [Van den Berg et al., 2008]

RVO addresses oscillation by introducing **Reciprocity** — both agents share collision avoidance responsibility.

**Mathematical Definition:**
$$RVO_{A|B} = \{ v_A \mid 2v_A - v_A^{cur} \in VO_{A|B} \}$$

Geometrically, the cone apex shifts from $v_B$ to $\frac{v_A^{cur} + v_B^{cur}}{2}$.

**Key Insight:** If both agents use RVO, they will each move halfway to avoid collision, resulting in smooth passing without oscillation.

### Hybrid Reciprocal Velocity Obstacles (HRVO) - *Implemented* [Snape et al., 2011]

RVO can lead to "Reciprocal Dances" where agents are unsure whether to pass left or right. HRVO resolves this by creating an **asymmetric** forbidden region.

**Construction:** HRVO combines one leg from VO with one leg from RVO:
- **Pass Right:** Use VO's right leg + RVO's left leg
- **Pass Left:** Use VO's left leg + RVO's right leg

The passing side is determined by: $\text{cross}(p_{rel}, v_{rel}) \gtrless 0$

**Geometric Implementation (Snape et al., 2011 - Section III):**

The key insight is that the two legs originate from **different apexes**:
- VO leg originates from $v_B$ (obstacle velocity)
- RVO leg originates from $\frac{v_A + v_B}{2}$ (reciprocal apex)

These legs intersect at a **hybrid apex** $p_{\text{inter}}$, computed via ray intersection:

$$p_{\text{inter}} = \text{intersect}(\text{origin}_{\text{VO}}, \theta_{\text{VO}}, \text{origin}_{\text{RVO}}, \theta_{\text{RVO}})$$

Our implementation uses Cramer's rule to solve the 2D line intersection:
```matlab
% Solve: origin1 + t1*dir1 = origin2 + t2*dir2
det_A = dir1(1)*dir2(2) - dir1(2)*dir2(1);
t1 = (d(1)*dir2(2) - d(2)*dir2(1)) / det_A;
p_inter = origin1 + t1 * dir1;
```

This creates the correct asymmetric forbidden region that implicitly encodes a **right-hand traffic** convention, ensuring consistent passing behavior without reciprocal dance oscillations.

**Static vs Dynamic Handling:**
- **Dynamic obstacles:** Full HRVO with hybrid apex intersection
- **Static obstacles:** Fall back to standard VO (apex at $v_B = [0,0]$)

## 🔬 Implementation Details

### Algorithm Comparison

| Property | VO | RVO | HRVO |
|:---------|:---|:----|:-----|
| Apex Location | $v_B$ | $\frac{v_A + v_B}{2}$ | Intersection of VO/RVO legs |
| Cone Geometry | Symmetric | Symmetric | **Asymmetric** |
| Oscillation | ❌ Causes jitter | ✅ Eliminated | ✅ Eliminated |
| Reciprocal Dance | N/A | ❌ Can occur | ✅ Eliminated |
| Static Obstacles | Works | Falls back to VO | Falls back to VO |

### Key Implementation Choices

1. **Dynamic Threshold:** Obstacles with $\|v_{obs}\| < 0.05$ m/s are treated as static
2. **Sensor Range:** 6.0m finite time horizon for computational efficiency
3. **Safety Buffer:** 0.25m added to Minkowski sum radius for uncertainty
4. **Velocity Sampling:** Multi-resolution search with 5 speed fractions and 37 angles
5. **Emergency Escape:** Immediate reverse if already in collision configuration

## 🧪 Simulation Scenarios

We validate our algorithms against 5 distinct scenarios:

1.  **Basic:** Single robot navigating static blocks. Validates baseline avoidance.
2.  **U-Trap:** A local minimum test. VO should prevent the robot from entering the trap if the velocity vector points into it.
3.  **Hallway:** Two robots moving head-on. This is the "Gold Standard" test for RVO to demonstrate smooth passing without jitter.
4.  **Somewhat Busy:** Robot crossing paths with dynamic agents moving at constant velocities.
5.  **Very Busy Plaza:** A "stress test" with multiple crossing paths and static obstacles to ensure no deadlocks occur.

## 👥 Team & Plan

This project is executed by a team of 4, adhering to a 6-week timeline.

  * **Member 1 (Theoretician):** Derivations, Technical Handout, RVO Logic.
  * **Member 2 (Algorithm Engineer):** HRVO Logic, Optimization Routine, Visualization.
  * **Member 3 (Architect):** MATLAB Framework, Class Structure, Kinematics.
  * **Member 4 (Global Integration):** Implementation of Global Planner (e.g., A\*) and integration layer with VO/HRVO local planners.

## 🔗 References

### Primary Sources (Velocity Obstacles Family)

1. **Fiorini, P., & Shiller, Z. (1998).** *Motion Planning in Dynamic Environments Using Velocity Obstacles.* International Journal of Robotics Research (IJRR), 17(7), 760–772.
   - **Foundational paper** introducing Velocity Obstacles (VO)
   - Defines collision cones, Minkowski sums, and velocity-space planning

2. **Van den Berg, J., Lin, M., & Manocha, D. (2008).** *Reciprocal Velocity Obstacles for Real-Time Multi-Agent Navigation.* IEEE International Conference on Robotics and Automation (ICRA).
   - Introduces **Reciprocal Velocity Obstacles (RVO)**
   - Solves oscillation problem by sharing avoidance responsibility

3. **Snape, J., Van Den Berg, J., Guy, S.J., & Manocha, D. (2011).** *The Hybrid Reciprocal Velocity Obstacle.* IEEE Transactions on Robotics.
   - Introduces **Hybrid RVO (HRVO)**
   - Solves "reciprocal dance" with asymmetric passing preference

### Supporting Textbooks

4. **Choset, H., et al. (2005).** *Principles of Robot Motion: Theory, Algorithms, and Implementations.* MIT Press.
   - Comprehensive coverage of motion planning fundamentals

5. **Siegwart, R., Nourbakhsh, I.R., & Scaramuzza, D. (2011).** *Introduction to Autonomous Mobile Robots* (2nd ed.). MIT Press.
   - Mobile robot kinematics and navigation strategies

6. **LaValle, S.M. (2006).** *Planning Algorithms.* Cambridge University Press.
   - Theoretical foundations of sampling-based planning