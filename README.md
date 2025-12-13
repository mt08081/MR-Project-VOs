
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
| **Phase 1: VO** | ✅ Complete | Baseline Dynamic Velocity Obstacles implemented in `plan_VO.m`. |
| **Phase 2: RVO** | 🚧 Planned | Reciprocal Velocity Obstacles to solve the oscillation ("jitter") problem. |
| **Phase 3: HRVO** | 🚧 Planned | Hybrid RVO to solve the "Reciprocal Dance" (passing side confusion). |
| **Phase 4: Integration** | ⏳ In Progress | Integration of VO/HRVO as a local controller alongside a Global Planner (e.g., A* or RRT). |

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
│   ├── plan_RVO.m             # PHASE 2: Reciprocal VOs (To Be Implemented)
│   └── plan_HRVO.m            # PHASE 3: Hybrid RVOs (To Be Implemented)
│
├── scenarios/                 # Modular Scenario Definitions
│   └── VOs/                   
│       ├── basic.m            # 1. Random Static Blocks (VO Baseline)
│       ├── u_trap.m           # 2. Local Minimum Stress Test
│       ├── setup_hallway.m    # 3. Head-On Collision (RVO Motivation)
│       ├── somewhat_busy.m    # 4. Robot vs Dynamic Obstacles
│       └── very_busy.m        # 5. The Plaza (Complex Multi-Agent Chaos)
│
└── utils/                     # Math & Helper Functions
    ├── check_angles.m         # Angular interval checking for collision cones
    ├── plot_cone.m            # Visualization helper for transparent cones
    └── check_collision.m      # Collision verification logic
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

### Velocity Obstacles (VO) - *Implemented*

VO answers the question: *"If I choose velocity $v$, will I collide with obstacle $B$ at any future time?"*
It assumes obstacles maintain their current velocity. While effective for static or non-reactive dynamic obstacles, it causes oscillation when two agents use it simultaneously (the "Mirror Effect").

### Reciprocal Velocity Obstacles (RVO) - *Next Step*

RVO addresses oscillation by introducing **Reciprocity**.

  * **Concept:** "I will move only halfway to avoid the collision, assuming the other agent will do the same."
  * **Math:** The apex of the collision cone is shifted from $v_{obs}$ to $\frac{v_{rob} + v_{obs}}{2}$.

### Hybrid Reciprocal Velocity Obstacles (HRVO) - *Final Goal*

RVO can lead to "Reciprocal Dances" where robots are unsure whether to pass left or right. HRVO adds a geometric preference (Right-of-Way) by stitching one leg of the RVO cone with one leg of the VO cone, forcing the robot to choose a specific side for passing.

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

1.  *The Hybrid Reciprocal Velocity Obstacle*, Snape et al.
2.  *Reciprocal Velocity Obstacles for Real-Time Multi-Agent Navigation*, van den Berg et al.
3.  Google Gemini Deep Research (Project Guide & Architecture Plans).