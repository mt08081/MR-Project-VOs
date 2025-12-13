
# Mobile Robotics Project: Reactive Navigation via Hybrid Reciprocal Velocity Obstacles (HRVO)

**Course:** Mobile Robotics (MR)  
**Status:** Phase 1 (Velocity Obstacles) Complete | Moving to Phase 2 (RVO) & 3 (HRVO)  
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
| **Phase 2: RVO** | ✅ Complete | Reciprocal Velocity Obstacles solving oscillation problem in `plan_RVO.m`. |
| **Phase 3: HRVO** | ✅ Complete | Hybrid RVO with right-hand passing preference in `plan_HRVO.m`. |
| **Phase 4: Integration** | ⏳ Planned | Integration of VO/HRVO as a local controller alongside a Global Planner (e.g., A* or RRT). |

## 📂 Repository Structure
The repository is organized to separate the physical simulation from the algorithmic "brain".

```text
MR-Project-VOs/
│
├── main_simulation.m          # ENTRY POINT — Controls the loop, switching, and visualization
├── README.md                  # Project documentation
│
├── classes/                   # Physical object definitions
│   ├── Robot.m                # Unicycle Kinematics, P-Controller, State (pos, vel, theta)
│   └── Obstacle.m             # Standardized object for Static Walls and Dynamic Agents
│
├── algorithms/                # The "Brains" - Path Planning Logic
│   ├── plan_VO.m              # PHASE 1: Dynamic Velocity Obstacles (Implemented)
│   ├── plan_RVO.m             # PHASE 2: Reciprocal VOs (Implemented)
│   └── plan_HRVO.m            # PHASE 3: Hybrid RVOs (Implemented)
│
├── scenarios/                 # Modular Scenario Definitions
│   └── VOs/                   
│       ├── basic.m            # 1. Random Static Blocks (VO Baseline)
│       ├── u_trap.m           # 2. Local Minimum Stress Test
│       ├── setup_hallway.m    # 3. Head-On Collision (RVO Motivation)
│       ├── somewhat_busy.m    # 4. Robot vs Dynamic Obstacles
│       └── very_busy.m        # 5. The Plaza (Complex Multi-Agent Chaos)
│
├── output/                    # Video recordings of simulations
│
└── utils/                     # Math & Helper Functions
    ├── check_angles.m         # Angular interval checking for collision cones
    ├── check_collision.m      # Collision verification logic
    ├── get_tangents.m         # Tangent line calculations
    └── plot_cone.m            # Visualization helper for transparent cones
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
    % Algorithm Selector
    % 1 = VO (Ready), 2 = RVO (Planned), 3 = HRVO (Planned)
    ALGORITHM = 1; 

    % Scenario Selector
    % 1 = Basic, 2 = U-Trap, 3 = Hallway, 4 = Busy, 5 = Very Busy
    SCENARIO_ID = 3; 
    ```
4.  **Run:** Click "Run" or press F5.
5.  **Output:** \* A visualization window will open showing the robot (Blue), goal (Green X), and obstacles (Red).
      * Video recordings are automatically saved to the `output/` folder.

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

This implicitly encodes a **right-hand traffic** convention, ensuring consistent passing behavior.

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