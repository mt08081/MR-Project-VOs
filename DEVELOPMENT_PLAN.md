# Development Plan: Multi-Agent System & Global Planning Integration

**Document Version:** 1.0  
**Last Updated:** December 2024  
**Status:** ✅ COMPLETE

This document outlines the technical specifications and implementation plan for the remaining three major features of the MR-Project.

### Implementation Status
- ✅ **Phase 4:** Multi-Agent System - `multi_agent_simulation.m`, `scenarios/multi_agent/`
- ✅ **Phase 5:** Extended Scenarios - 5 scenarios in `scenarios/multi_agent/`
- ✅ **Phase 6:** Maze Demo with Global Planner - `maze_demo.m`, `global_planner/`

---

## Table of Contents
1. [Phase 4: Multi-Agent System](#phase-4-multi-agent-system)
2. [Phase 5: Extended Scenarios](#phase-5-extended-scenarios)
3. [Phase 6: Maze Demo with Global Planner](#phase-6-maze-demo-with-global-planner)

---

## Phase 4: Multi-Agent System

### 4.1 Overview
Transform the current 2-robot system into a fully scalable N-robot simulation where each robot independently plans using VO/RVO/HRVO while perceiving all other robots as dynamic obstacles.

### 4.2 Technical Requirements

#### 4.2.1 Data Structures

**Robots Array:**
```matlab
% Replace individual robot1, robot2 with array
robots = Robot.empty(0, N);  % Preallocate N robots

% Each robot stores:
%   - pos:      [2x1] current position
%   - vel:      [2x1] current velocity  
%   - goal:     [2x1] target position
%   - radius:   scalar collision radius
%   - v_max:    scalar max speed
%   - theta:    scalar heading angle
%   - color:    [1x3] RGB for visualization
%   - status:   string {'navigating', 'arrived', 'waiting', 'crashed'}
%   - id:       integer unique identifier
```

**Robot Status Enumeration:**
```matlab
% New property in Robot.m class
properties
    status = 'navigating'  % 'navigating' | 'arrived' | 'waiting' | 'crashed'
    arrival_time = NaN     % Time when robot reached goal
end
```

#### 4.2.2 Core Loop Modifications

**File:** `multi_agent_simulation.m` (new file based on `main_simulation.m`)

```matlab
% Main simulation loop pseudocode
for t = 0:dt:T_max
    % 1. Update non-reactive obstacles (walls, dumb agents)
    obstacles = update_obstacles(obstacles, dt);
    
    % 2. For each active robot, plan and move
    for i = 1:N
        if strcmp(robots(i).status, 'arrived') || strcmp(robots(i).status, 'crashed')
            continue;  % Skip finished robots
        end
        
        % Build perception: static obstacles + ALL other robots
        obs_for_robot_i = obstacles;
        for j = 1:N
            if i ~= j
                % Add robot j as dynamic obstacle with its CURRENT velocity
                obs_for_robot_i = [obs_for_robot_i, ...
                    Obstacle(robots(j).pos, robots(j).radius, robots(j).vel)];
            end
        end
        
        % Plan using selected algorithm
        [v_opt, ~] = run_planner(ALGORITHM, robots(i), obs_for_robot_i);
        
        % Execute movement
        robots(i) = robots(i).move(v_opt, dt);
        
        % Update status
        robots(i) = update_robot_status(robots(i), t);
    end
    
    % 3. Check for collisions (robot-robot and robot-obstacle)
    collision_pairs = check_all_collisions(robots, obstacles);
    
    % 4. Visualization
    draw_multi_agent_scene(robots, obstacles, t);
    
    % 5. Termination: Only when ALL robots finished or T_max reached
    if all_robots_finished(robots)
        break;
    end
end
```

#### 4.2.3 Termination Logic

**Key Change:** Simulation continues until ALL robots reach their goals or crash.

```matlab
function finished = all_robots_finished(robots)
    finished = true;
    for i = 1:length(robots)
        if strcmp(robots(i).status, 'navigating') || strcmp(robots(i).status, 'waiting')
            finished = false;
            return;
        end
    end
end

function robot = update_robot_status(robot, t)
    dist_to_goal = norm(robot.goal - robot.pos);
    
    if dist_to_goal < 0.5
        robot.status = 'arrived';
        if isnan(robot.arrival_time)
            robot.arrival_time = t;
        end
    elseif norm(robot.vel) < 0.01
        robot.status = 'waiting';
    else
        robot.status = 'navigating';
    end
end
```

#### 4.2.4 UI/Visualization Updates

**HUD Panel (per-robot statistics):**
```matlab
% Multi-robot HUD showing all robot statuses
function update_multi_hud(h_hud, robots, t, algo_name)
    hud_str = sprintf('Algorithm: %s | Time: %.2fs\n', algo_name, t);
    hud_str = [hud_str, '─────────────────────────\n'];
    
    for i = 1:length(robots)
        r = robots(i);
        dist = norm(r.goal - r.pos);
        speed = norm(r.vel);
        
        % Status indicator
        switch r.status
            case 'navigating', icon = '🔵';
            case 'arrived',    icon = '✅';
            case 'waiting',    icon = '⏸️';
            case 'crashed',    icon = '💥';
        end
        
        hud_str = [hud_str, sprintf('Robot %d %s: %.1fm | %.2f m/s\n', ...
            i, icon, dist, speed)];
    end
    
    h_hud.String = hud_str;
end
```

**Robot Visualization:**
```matlab
function h = draw_robot_with_status(robot)
    % Color based on status
    switch robot.status
        case 'navigating', edge_color = robot.color;
        case 'arrived',    edge_color = [0 1 0];  % Green
        case 'waiting',    edge_color = [1 0.6 0]; % Orange
        case 'crashed',    edge_color = [1 0 0];  % Red
    end
    
    % Draw robot body
    h = viscircles(robot.pos', robot.radius, 'Color', edge_color, 'LineWidth', 2);
    
    % Draw ID label
    text(robot.pos(1), robot.pos(2)+robot.radius+0.2, sprintf('R%d', robot.id), ...
        'Color', 'w', 'HorizontalAlignment', 'center', 'FontSize', 8);
    
    % Draw velocity vector
    quiver(robot.pos(1), robot.pos(2), robot.vel(1), robot.vel(2), 0, ...
        'Color', edge_color, 'LineWidth', 1.5);
end
```

### 4.3 File Structure

```
MR_Project/
├── multi_agent_simulation.m    # NEW: Main entry point for N-robot simulation
├── main_simulation.m           # KEEP: Original 2-robot demo
├── classes/
│   └── Robot.m                 # MODIFY: Add status, arrival_time, id properties
├── scenarios/
│   └── multi_agent/            # NEW: Multi-agent specific scenarios
│       ├── crossing_4.m        # 4 robots crossing paths
│       ├── swarm_8.m           # 8 robots in formation
│       └── interactive.m       # User-placed robots
```

### 4.4 Implementation Checklist

- [x] Modify `Robot.m` class to add `status`, `arrival_time`, `id` properties
- [x] Create `multi_agent_simulation.m` with robots array
- [x] Implement `update_robot_status()` function
- [x] Implement `all_robots_finished()` termination check
- [x] Implement `check_all_collisions()` for N robots
- [x] Create multi-robot HUD visualization
- [x] Update `draw_robot_with_status()` for status-based coloring
- [x] Test with 4, 8, 16 robots

---

## Phase 5: Extended Scenarios

### 5.1 Overview
Create a comprehensive suite of scenarios to test different aspects of the VO/RVO/HRVO algorithms, including an interactive mode where users can place robots manually.

### 5.2 Scenario Specifications

#### 5.2.1 Scenario: Crossing Intersection (`crossing_4.m`)
**Purpose:** Test 4-way intersection handling with HRVO right-hand preference.

```matlab
function [robots, obstacles, map_bounds, scn_name] = crossing_4()
    scn_name = 'Crossing_4Way';
    map_bounds = [-10 10 -10 10];
    
    % 4 robots starting at cardinal directions, crossing to opposite side
    robots = [
        Robot([8; 0], [-8; 0], 0.4, 1.0, 'b'),   % East → West
        Robot([-8; 0], [8; 0], 0.4, 1.0, 'r'),   % West → East
        Robot([0; 8], [0; -8], 0.4, 1.0, 'g'),   % North → South
        Robot([0; -8], [0; 8], 0.4, 1.0, 'm')    % South → North
    ];
    
    % No static obstacles - pure multi-agent coordination test
    obstacles = [];
end
```

#### 5.2.2 Scenario: Swarm Formation (`swarm_8.m`)
**Purpose:** Test scalability with 8 robots navigating simultaneously.

```matlab
function [robots, obstacles, map_bounds, scn_name] = swarm_8()
    scn_name = 'Swarm_8';
    map_bounds = [-15 15 -15 15];
    
    % 8 robots in circle, each going to opposite point
    N = 8;
    radius = 10;
    robots = Robot.empty(0, N);
    colors = lines(N);  % Distinct colors
    
    for i = 1:N
        angle_start = (i-1) * 2*pi/N;
        angle_goal = angle_start + pi;  % Opposite side
        
        start_pos = radius * [cos(angle_start); sin(angle_start)];
        goal_pos = radius * [cos(angle_goal); sin(angle_goal)];
        
        robots(i) = Robot(start_pos, goal_pos, 0.4, 1.0, colors(i,:));
        robots(i).id = i;
    end
    
    % Add some static obstacles in center
    obstacles = [
        Obstacle([0; 0], 1.5, [0; 0])  % Central pillar
    ];
end
```

#### 5.2.3 Scenario: Wall Corridor (`wall_corridor.m`)
**Purpose:** Test navigation in constrained spaces with wall obstacles.

```matlab
function [robots, obstacles, map_bounds, scn_name] = wall_corridor()
    scn_name = 'Wall_Corridor';
    map_bounds = [-2 20 -5 5];
    
    % Single robot navigating corridor
    robots = Robot([-1; 0], [18; 0], 0.4, 1.0, 'b');
    
    % Corridor walls as series of small obstacles (wall segments)
    obstacles = [];
    
    % Top wall
    for x = 0:2:16
        obstacles = [obstacles, Obstacle([x; 3], 0.5, [0; 0])];
    end
    
    % Bottom wall
    for x = 0:2:16
        obstacles = [obstacles, Obstacle([x; -3], 0.5, [0; 0])];
    end
    
    % Random dynamic obstacles in corridor
    obstacles = [obstacles, ...
        Obstacle([8; 0], 0.3, [-0.3; 0]),   % Coming toward robot
        Obstacle([12; 1], 0.3, [-0.2; -0.1]) % Diagonal motion
    ];
end
```

#### 5.2.4 Scenario: Interactive Mode (`interactive.m`)
**Purpose:** Allow users to place robots and goals via mouse clicks.

```matlab
function [robots, obstacles, map_bounds, scn_name] = interactive()
    scn_name = 'Interactive';
    map_bounds = [-15 15 -15 15];
    
    % Setup figure for interaction
    fig = figure('Name', 'Interactive Robot Placement', 'Color', 'k');
    ax = axes('Color', 'k', 'XColor', 'w', 'YColor', 'w');
    axis equal; grid on; hold on;
    axis(map_bounds);
    title('LEFT CLICK: Place Robot | RIGHT CLICK: Set Goal | ENTER: Start', 'Color', 'w');
    
    robots = Robot.empty();
    temp_starts = [];
    robot_count = 0;
    placing_goal = false;
    
    % Predefined static obstacles
    obstacles = [
        Obstacle([5; 5], 1.0, [0; 0]),
        Obstacle([-5; -5], 1.0, [0; 0]),
        Obstacle([0; 0], 0.8, [0; 0])
    ];
    
    % Draw obstacles
    for i = 1:length(obstacles)
        viscircles(obstacles(i).pos', obstacles(i).radius, 'Color', 'r');
    end
    
    % Mouse click callback
    set(fig, 'WindowButtonDownFcn', @mouse_callback);
    set(fig, 'KeyPressFcn', @key_callback);
    
    % Wait for user to finish
    uiwait(fig);
    
    % Nested callback functions
    function mouse_callback(~, ~)
        pt = get(ax, 'CurrentPoint');
        click_pos = pt(1, 1:2)';
        
        if strcmp(get(fig, 'SelectionType'), 'normal')  % Left click
            % Place new robot start position
            robot_count = robot_count + 1;
            temp_starts = [temp_starts, click_pos];
            plot(click_pos(1), click_pos(2), 'bo', 'MarkerSize', 15, ...
                'MarkerFaceColor', 'b');
            text(click_pos(1)+0.5, click_pos(2), sprintf('R%d Start', robot_count), ...
                'Color', 'w');
            placing_goal = true;
            
        elseif strcmp(get(fig, 'SelectionType'), 'alt') && placing_goal  % Right click
            % Set goal for most recent robot
            colors = lines(robot_count);
            robots(robot_count) = Robot(temp_starts(:,robot_count), click_pos, ...
                0.4, 1.0, colors(robot_count,:));
            robots(robot_count).id = robot_count;
            
            plot(click_pos(1), click_pos(2), 'gx', 'MarkerSize', 12, 'LineWidth', 2);
            text(click_pos(1)+0.5, click_pos(2), sprintf('R%d Goal', robot_count), ...
                'Color', 'w');
            
            % Draw line connecting start and goal
            plot([temp_starts(1,robot_count), click_pos(1)], ...
                 [temp_starts(2,robot_count), click_pos(2)], '--', ...
                 'Color', colors(robot_count,:), 'LineWidth', 0.5);
            
            placing_goal = false;
        end
    end
    
    function key_callback(~, event)
        if strcmp(event.Key, 'return')
            if ~isempty(robots)
                uiresume(fig);
            else
                disp('Please place at least one robot before starting.');
            end
        end
    end
end
```

### 5.3 Scenario Summary Table

| Scenario | Robots | Obstacles | Purpose |
|:---------|:-------|:----------|:--------|
| `crossing_4.m` | 4 | 0 | Multi-way intersection coordination |
| `swarm_8.m` | 8 | 1 central | Scalability and formation |
| `wall_corridor.m` | 1 | Many walls + 2 dynamic | Constrained space navigation |
| `dense_crowd.m` | 2 | 15+ dynamic | High-density dynamic environment |
| `interactive.m` | User-defined | 3 static | Testing arbitrary configurations |

### 5.4 Implementation Checklist

- [x] Create `scenarios/multi_agent/` directory
- [x] Implement `crossing_4.m`
- [x] Implement `swarm_8.m`
- [x] Implement `wall_corridor.m`
- [x] Implement `dense_crowd.m`
- [x] Implement `interactive.m` with mouse callbacks
- [x] Add scenario loader to `multi_agent_simulation.m`
- [x] Test each scenario with VO, RVO, and HRVO

---

## Phase 6: Maze Demo with Global Planner

### 6.1 Overview
Create a comprehensive demonstration combining:
1. **Random maze generation** with corridors and obstacles
2. **Global path planning** using MATLAB's Robotics System Toolbox (A*, PRM, or RRT)
3. **Local reactive navigation** using VO/RVO/HRVO to handle dynamic obstacles
4. **Mixed obstacle types:** static walls, dynamic agents, and velocity obstacles

### 6.2 Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                        MAZE DEMO                                │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│   ┌─────────────────┐     ┌──────────────────────────────────┐  │
│   │  GLOBAL LAYER   │     │          LOCAL LAYER             │  │
│   │                 │     │                                  │  │
│   │  ┌───────────┐  │     │  ┌─────────┐    ┌───────────┐    │  │
│   │  │   Maze    │  │     │  │ Current │    │  Sensor   │    │  │
│   │  │ Generator │  │     │  │ Waypoint│    │   Range   │    │  │
│   │  └─────┬─────┘  │     │  └────┬────┘    └─────┬─────┘    │  │
│   │        │        │     │       │               │          │  │
│   │        ▼        │     │       ▼               ▼          │  │
│   │  ┌───────────┐  │     │  ┌─────────────────────────┐     │  │
│   │  │ Occupancy │  │     │  │   VO/RVO/HRVO Planner   │     │  │
│   │  │   Grid    │──┼────┼─▶│                         │     │  │
│   │  └─────┬─────┘  │     │  │  - Static obstacles     │     │  │
│   │        │        │     │  │  - Dynamic agents       │     │  │
│   │        ▼        │     │  │  - Waypoint attraction  │     │  │
│   │  ┌───────────┐  │     │  └───────────┬─────────────┘     │  │
│   │  │  A*/PRM   │  │     │              │                   │  │
│   │  │  Planner  │  │     │              ▼                   │  │
│   │  └─────┬─────┘  │     │       ┌─────────────┐            │  │
│   │        │        │     │       │   v_opt     │            │  │
│   │        ▼        │     │       │  (output)   │            │  │
│   │  ┌───────────┐  │     │       └─────────────┘            │  │
│   │  │ Waypoints │──┼─────┼───────────────▶                 │  │
│   │  │   Path    │  │     │                                  │  │
│   │  └───────────┘  │     │                                  │  │
│   └─────────────────┘     └──────────────────────────────────┘  │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### 6.3 Technical Components

#### 6.3.1 Maze Generation (`generate_maze.m`)

```matlab
function [occupancy_map, obstacles, start, goal] = generate_maze(width, height, complexity)
% GENERATE_MAZE Creates a random maze with obstacles
%
% Inputs:
%   width, height - Maze dimensions in meters
%   complexity    - 'easy', 'medium', 'hard' (obstacle density)
%
% Outputs:
%   occupancy_map - binaryOccupancyMap for global planner
%   obstacles     - Array of Obstacle objects for local planner
%   start, goal   - [x; y] positions

    % Create binary occupancy map
    resolution = 10;  % cells per meter
    occupancy_map = binaryOccupancyMap(width, height, resolution);
    
    % Add boundary walls
    walls = [];
    wall_thickness = 0.3;
    
    % Top and bottom walls
    for x = 0:0.5:width
        walls = [walls; x, 0; x, height];
    end
    % Left and right walls
    for y = 0:0.5:height
        walls = [walls; 0, y; width, y];
    end
    setOccupancy(occupancy_map, walls, 1);
    
    % Generate internal obstacles based on complexity
    switch complexity
        case 'easy'
            num_blocks = 5;
            num_dynamic = 2;
        case 'medium'
            num_blocks = 12;
            num_dynamic = 5;
        case 'hard'
            num_blocks = 20;
            num_dynamic = 10;
    end
    
    obstacles = [];
    
    % Random static blocks
    for i = 1:num_blocks
        block_pos = [rand*(width-4)+2; rand*(height-4)+2];
        block_radius = 0.5 + rand*0.5;
        
        % Add to occupancy map
        inflate_radius = block_radius + 0.5;  % Robot radius buffer
        setOccupancy(occupancy_map, block_pos', 1);
        
        % Add to obstacle array
        obstacles = [obstacles, Obstacle(block_pos, block_radius, [0; 0])];
    end
    
    % Random dynamic obstacles (moving agents)
    for i = 1:num_dynamic
        dyn_pos = [rand*(width-4)+2; rand*(height-4)+2];
        dyn_vel = (rand(2,1) - 0.5) * 0.6;  % Random velocity [-0.3, 0.3]
        obstacles = [obstacles, Obstacle(dyn_pos, 0.3, dyn_vel)];
    end
    
    % Find valid start and goal positions
    start = find_free_position(occupancy_map, [1; 1], [3; 3]);
    goal = find_free_position(occupancy_map, [width-3; height-3], [width-1; height-1]);
end

function pos = find_free_position(map, min_bound, max_bound)
    max_attempts = 100;
    for i = 1:max_attempts
        pos = min_bound + rand(2,1) .* (max_bound - min_bound);
        if ~getOccupancy(map, pos')
            return;
        end
    end
    error('Could not find free position');
end
```

#### 6.3.2 Global Path Planning (`plan_global_path.m`)

```matlab
function [waypoints, path_length] = plan_global_path(occupancy_map, start, goal, method)
% PLAN_GLOBAL_PATH Compute waypoints using MATLAB's planning toolbox
%
% Inputs:
%   occupancy_map - binaryOccupancyMap
%   start, goal   - [x; y] positions
%   method        - 'astar', 'prm', 'rrt'
%
% Outputs:
%   waypoints   - Nx2 matrix of [x, y] waypoints
%   path_length - Total path length in meters

    % Inflate map for robot radius
    robot_radius = 0.5;
    inflated_map = copy(occupancy_map);
    inflate(inflated_map, robot_radius);
    
    switch lower(method)
        case 'astar'
            % A* on grid
            planner = plannerAStarGrid(inflated_map);
            [path, info] = plan(planner, start', goal');
            
        case 'prm'
            % Probabilistic Roadmap
            planner = mobileRobotPRM(inflated_map);
            planner.NumNodes = 100;
            planner.ConnectionDistance = 5;
            path = findpath(planner, start', goal');
            
        case 'rrt'
            % RRT (requires Navigation Toolbox)
            ss = stateSpaceSE2;
            ss.StateBounds = [occupancy_map.XWorldLimits; ...
                              occupancy_map.YWorldLimits; 
                              [-pi, pi]];
            
            sv = validatorOccupancyMap(ss);
            sv.Map = inflated_map;
            sv.ValidationDistance = 0.1;
            
            planner = plannerRRT(ss, sv);
            planner.MaxConnectionDistance = 2.0;
            planner.MaxIterations = 1000;
            
            [pthObj, ~] = plan(planner, [start' 0], [goal' 0]);
            path = pthObj.States(:, 1:2);
            
        otherwise
            error('Unknown planning method: %s', method);
    end
    
    % Simplify path (reduce waypoints)
    if size(path, 1) > 2
        waypoints = simplify_path(path, inflated_map);
    else
        waypoints = path;
    end
    
    % Calculate path length
    path_length = 0;
    for i = 2:size(waypoints, 1)
        path_length = path_length + norm(waypoints(i,:) - waypoints(i-1,:));
    end
end

function simplified = simplify_path(path, map)
% Remove unnecessary waypoints using line-of-sight checking
    simplified = path(1, :);
    current_idx = 1;
    
    while current_idx < size(path, 1)
        % Find furthest visible waypoint
        furthest = current_idx + 1;
        for i = current_idx+2:size(path, 1)
            if is_line_free(map, path(current_idx,:), path(i,:))
                furthest = i;
            end
        end
        simplified = [simplified; path(furthest, :)];
        current_idx = furthest;
    end
end

function free = is_line_free(map, p1, p2)
    num_samples = ceil(norm(p2 - p1) * 10);
    for t = linspace(0, 1, num_samples)
        pt = p1 + t * (p2 - p1);
        if getOccupancy(map, pt)
            free = false;
            return;
        end
    end
    free = true;
end
```

#### 6.3.3 Waypoint Following with Local Avoidance

```matlab
function v_opt = plan_with_waypoints(robot, obstacles, waypoints, current_wp_idx)
% PLAN_WITH_WAYPOINTS Combines global waypoints with local VO avoidance
%
% The robot follows waypoints but uses VO/RVO/HRVO to avoid dynamic obstacles

    % Determine current target waypoint
    wp = waypoints(current_wp_idx, :)';
    
    % Create a virtual goal for the local planner
    temp_robot = robot;
    temp_robot.goal = wp;
    
    % Run local planner (HRVO) toward waypoint
    [v_opt, ~] = plan_HRVO_new(temp_robot, obstacles);
end

function [next_idx, reached_goal] = update_waypoint(robot, waypoints, current_idx)
% Check if current waypoint reached, advance to next
    WAYPOINT_THRESHOLD = 0.8;  % meters
    
    dist_to_wp = norm(waypoints(current_idx, :)' - robot.pos);
    
    if dist_to_wp < WAYPOINT_THRESHOLD
        if current_idx < size(waypoints, 1)
            next_idx = current_idx + 1;
            reached_goal = false;
        else
            next_idx = current_idx;
            reached_goal = true;
        end
    else
        next_idx = current_idx;
        reached_goal = false;
    end
end
```

#### 6.3.4 Main Maze Demo (`maze_demo.m`)

```matlab
% maze_demo.m - Full integrated demonstration
% ============================================

clc; clear; close all;
addpath('classes', 'algorithms', 'utils', 'scenarios');

%% Configuration
MAZE_WIDTH = 25;
MAZE_HEIGHT = 25;
COMPLEXITY = 'medium';  % 'easy', 'medium', 'hard'
GLOBAL_PLANNER = 'astar';  % 'astar', 'prm', 'rrt'
LOCAL_PLANNER = 3;  % 1=VO, 2=RVO, 3=HRVO
dt = 0.1;
T_max = 120;

%% 1. Generate Random Maze
fprintf('Generating maze...\n');
[occupancy_map, obstacles, start_pos, goal_pos] = generate_maze(...
    MAZE_WIDTH, MAZE_HEIGHT, COMPLEXITY);

%% 2. Compute Global Path
fprintf('Computing global path using %s...\n', upper(GLOBAL_PLANNER));
[waypoints, path_length] = plan_global_path(occupancy_map, start_pos, goal_pos, GLOBAL_PLANNER);
fprintf('Path found: %.2f meters, %d waypoints\n', path_length, size(waypoints, 1));

%% 3. Initialize Robot
robot = Robot(start_pos, goal_pos, 0.4, 1.2, 'b');
current_wp_idx = 1;

%% 4. Visualization Setup
fig = figure('Name', 'Maze Demo', 'Color', 'k', 'Position', [50 50 1000 800]);

% Left panel: Full maze with global path
subplot(1, 2, 1);
show(occupancy_map); hold on;
plot(waypoints(:,1), waypoints(:,2), 'g-', 'LineWidth', 2);
plot(start_pos(1), start_pos(2), 'go', 'MarkerSize', 15, 'MarkerFaceColor', 'g');
plot(goal_pos(1), goal_pos(2), 'rx', 'MarkerSize', 15, 'LineWidth', 3);
title('Global View', 'Color', 'w');
h_robot_global = plot(robot.pos(1), robot.pos(2), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b');

% Right panel: Local view (robot-centric)
ax_local = subplot(1, 2, 2);
title('Local View (VO/RVO/HRVO)', 'Color', 'w');

%% 5. Main Simulation Loop
reached_goal = false;

for t = 0:dt:T_max
    % Update dynamic obstacles
    for k = 1:length(obstacles)
        if norm(obstacles(k).vel) > 0
            obstacles(k).pos = obstacles(k).pos + obstacles(k).vel * dt;
            % Bounce off walls (simple reflection)
            obstacles(k) = bounce_off_walls(obstacles(k), [0 MAZE_WIDTH 0 MAZE_HEIGHT]);
        end
    end
    
    % Local planning with waypoint target
    v_opt = plan_with_waypoints(robot, obstacles, waypoints, current_wp_idx);
    robot = robot.move(v_opt, dt);
    
    % Update waypoint
    [current_wp_idx, reached_goal] = update_waypoint(robot, waypoints, current_wp_idx);
    
    % Update visualization
    set(h_robot_global, 'XData', robot.pos(1), 'YData', robot.pos(2));
    draw_local_view(ax_local, robot, obstacles, waypoints(current_wp_idx,:)');
    
    drawnow;
    
    if reached_goal
        fprintf('SUCCESS! Goal reached in %.2f seconds\n', t);
        break;
    end
end

if ~reached_goal
    fprintf('TIMEOUT: Did not reach goal in %.2f seconds\n', T_max);
end
```

### 6.4 Required MATLAB Toolboxes

| Toolbox | Required For | Alternative |
|:--------|:-------------|:------------|
| Robotics System Toolbox | `binaryOccupancyMap`, `plannerAStarGrid` | Custom grid-based A* |
| Navigation Toolbox | `mobileRobotPRM`, RRT planners | PRM from scratch |
| Image Processing Toolbox | `viscircles` (already used) | Custom circle drawing |

### 6.5 Obstacle Types in Maze

| Type | Behavior | VO Treatment |
|:-----|:---------|:-------------|
| **Static Wall** | Fixed position, $v=0$ | Standard VO (apex at origin) |
| **Dynamic Agent** | Constant velocity | RVO/HRVO (shared avoidance) |
| **Patrol Agent** | Waypoint-following | RVO/HRVO with current velocity |
| **Random Walker** | Brownian motion | VO (unpredictable) |

### 6.6 Implementation Checklist

- [ ] Implement `generate_maze.m` with configurable complexity
- [ ] Implement `plan_global_path.m` with A* support
- [ ] Implement path simplification algorithm
- [ ] Create `plan_with_waypoints.m` integration function
- [ ] Implement waypoint advancement logic
- [ ] Create `maze_demo.m` main script
- [ ] Add dual-view visualization (global + local)
- [ ] Implement obstacle bouncing/patrol behaviors
- [ ] Test with easy/medium/hard mazes
- [ ] Record demo video

---

## Timeline Estimate

| Phase | Estimated Duration | Dependencies |
|:------|:-------------------|:-------------|
| Phase 4: Multi-Agent | 3-4 days | None |
| Phase 5: Scenarios | 2-3 days | Phase 4 |
| Phase 6: Maze Demo | 4-5 days | Phase 4, Robotics Toolbox |

**Total: ~10-12 days**

---

## Notes & Considerations

1. **Performance:** With N robots, planning complexity is O(N²) per timestep. Consider spatial hashing for large N.

2. **Toolbox Alternatives:** If Robotics System Toolbox unavailable, implement A* on custom grid structure.

3. **Video Recording:** All demos should auto-save to `output/` folder for documentation.

4. **Testing Protocol:** Each scenario should be run with VO, RVO, and HRVO to compare behaviors.
