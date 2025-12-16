%% MAZE_DEMO: Hierarchical Planning (Global A* + Local VO/RVO/HRVO)
%
% Demonstrates a real-world navigation architecture:
%   - Global Layer: A* computes path through static obstacles (walls)
%   - Local Layer:  VO/RVO/HRVO provides reactive avoidance for dynamic agents
%
% The LOCAL PLANNER receives BOTH:
%   1. Wall obstacles (for collision avoidance near walls)
%   2. Dynamic agents (other robots, moving obstacles)
%
% Reference: Thrun, Burgard & Fox (2005), Probabilistic Robotics, Ch. 5

clear; clc; close all;

addpath('algorithms', 'classes', 'utils', 'global_planner');

%% ======================= CONFIGURATION =======================
config.maze_type     = 'simple';   % 'simple', 'corridor', 'rooms'
config.grid_size     = [30, 30];   % Grid cells
config.cell_size     = 0.5;        % Meters per cell
config.local_planner = 'VO';       % 'VO', 'RVO', 'HRVO'
config.dt            = 0.1;        % Time step
config.max_time      = 80;         % Max simulation time
config.robot_radius  = 0.3;        
config.robot_vmax    = 1.0;        
config.record_video  = true;       % Save video for presentation

% Derived
world_width  = config.grid_size(2) * config.cell_size;
world_height = config.grid_size(1) * config.cell_size;

fprintf('=== MAZE DEMO: A* + %s ===\n', config.local_planner);
fprintf('World: %.0fm x %.0fm | Maze: %s\n', world_width, world_height, config.maze_type);

%% ======================= MAZE GENERATION =======================
grid_map = zeros(config.grid_size);
wall_obstacles = [];  % Obstacles for local planner (wall boundaries)

switch config.maze_type
    case 'simple'
        % Border walls
        grid_map(1,:) = 1; grid_map(end,:) = 1;
        grid_map(:,1) = 1; grid_map(:,end) = 1;
        
        % L-shaped internal wall
        mid_r = round(config.grid_size(1)/2);
        mid_c = round(config.grid_size(2)/2);
        grid_map(6:mid_r+4, mid_c) = 1;      % Vertical part
        grid_map(mid_r, mid_c:mid_c+6) = 1;  % Horizontal part
        
        start_cell = [config.grid_size(1)-3, 4];
        goal_cell  = [4, config.grid_size(2)-4];
        
    case 'corridor'
        grid_map(1,:) = 1; grid_map(end,:) = 1;
        grid_map(:,1) = 1; grid_map(:,end) = 1;
        
        % S-curve barriers
        grid_map(8, 1:18) = 1;
        grid_map(15, 12:end) = 1;
        grid_map(22, 1:18) = 1;
        
        start_cell = [config.grid_size(1)-3, 4];
        goal_cell  = [4, config.grid_size(2)-4];
        
    case 'rooms'
        grid_map(1,:) = 1; grid_map(end,:) = 1;
        grid_map(:,1) = 1; grid_map(:,end) = 1;
        
        mid = round(config.grid_size(1)/2);
        grid_map(mid, 1:mid-3) = 1;
        grid_map(mid, mid+3:end) = 1;
        grid_map(1:mid-3, mid) = 1;
        grid_map(mid+3:end, mid) = 1;
        
        start_cell = [config.grid_size(1)-3, 4];
        goal_cell  = [4, config.grid_size(2)-4];
end

% Convert wall grid cells to circular obstacles for VO local planner
% This ensures the robot doesn't cut through walls!
wall_obstacles = grid_to_wall_obstacles(grid_map, config.grid_size, config.cell_size);

start_pos = cell_to_world(start_cell, config.grid_size, config.cell_size);
goal_pos  = cell_to_world(goal_cell, config.grid_size, config.cell_size);

fprintf('Start: (%.1f, %.1f) | Goal: (%.1f, %.1f)\n', ...
    start_pos(1), start_pos(2), goal_pos(1), goal_pos(2));
fprintf('Wall obstacles for VO: %d\n', length(wall_obstacles));

%% ======================= A* GLOBAL PATH =======================
fprintf('\nRunning A* planner...\n');
tic;
[path_cells, path_found] = astar_grid(grid_map, start_cell, goal_cell);
planning_time = toc;

if ~path_found
    error('A* could not find path!');
end

waypoints = path_to_world(path_cells, config.grid_size, config.cell_size);
fprintf('Path: %d waypoints in %.3fs\n', size(waypoints,1), planning_time);

%% ======================= AGENTS & OBSTACLES =======================
% Main robot
robot = Robot(1, start_pos, 0, config.robot_radius, config.robot_vmax, goal_pos);
robot.color = [0, 0.7, 0];

% Other robots using same local planner (demonstrates multi-agent)
% Place in known free areas of the simple maze
other_robots = [];
other_robot_goals = [];

% Robot 2: In the lower-right corridor area, moving left
r2_start = [world_width*0.8, world_height*0.15];
r2_goal  = [world_width*0.15, world_height*0.15];
r2 = Robot(2, r2_start, 0, 0.25, 0.6, r2_goal);
r2.color = [0.8, 0.2, 0.2];
other_robots = [other_robots, r2];
other_robot_goals = [other_robot_goals; r2_goal];

% Robot 3: In the upper corridor area, moving down
r3_start = [world_width*0.85, world_height*0.85];
r3_goal  = [world_width*0.85, world_height*0.15];
r3 = Robot(3, r3_start, 0, 0.25, 0.5, r3_goal);
r3.color = [0.2, 0.2, 0.8];
other_robots = [other_robots, r3];
other_robot_goals = [other_robot_goals; r3_goal];

% Static obstacles (furniture-like)
static_obstacles = [];
static_obs_positions = [
    world_width*0.25, world_height*0.4;
    world_width*0.7,  world_height*0.65;
];
for i = 1:size(static_obs_positions, 1)
    static_obstacles = [static_obstacles, ...
        Obstacle(static_obs_positions(i,:), 0.4, [0,0])];
end

fprintf('Other robots: %d | Static obstacles: %d\n', ...
    length(other_robots), length(static_obstacles));

%% ======================= VISUALIZATION SETUP =======================
fig = figure('Name', 'Maze Demo: A* + Local Avoidance', ...
    'Position', [50, 50, 1500, 650], 'Color', 'k');

% === LEFT PANEL: Global View ===
ax1 = subplot(1, 2, 1);
set(ax1, 'Color', 'k', 'XColor', 'w', 'YColor', 'w');
hold on;
title('Global View', 'FontSize', 14, 'Color', 'w');

% Draw occupancy grid (walls)
imagesc([config.cell_size/2, world_width - config.cell_size/2], ...
    [config.cell_size/2, world_height - config.cell_size/2], ...
    flipud(grid_map));
colormap(ax1, [0 0 0; 0.4 0.4 0.4]);

% A* path
plot(waypoints(:,1), waypoints(:,2), 'b-', 'LineWidth', 2);
plot(waypoints(:,1), waypoints(:,2), 'b.', 'MarkerSize', 8);

% Start/Goal markers
plot(start_pos(1), start_pos(2), 'go', 'MarkerSize', 14, 'LineWidth', 2);
plot(goal_pos(1), goal_pos(2), 'rp', 'MarkerSize', 16, 'MarkerFaceColor', 'r');

% Robot marker (will update)
h_robot_global = plot(robot.pos(1), robot.pos(2), 'o', ...
    'MarkerSize', 10, 'MarkerFaceColor', robot.color, 'MarkerEdgeColor', 'k');

% Other robots markers
h_other_global = [];
for i = 1:length(other_robots)
    h = plot(other_robots(i).pos(1), other_robots(i).pos(2), 's', ...
        'MarkerSize', 8, 'MarkerFaceColor', other_robots(i).color, 'MarkerEdgeColor', 'k');
    h_other_global = [h_other_global, h];
end

% Static obstacles on global view
for i = 1:length(static_obstacles)
    obs = static_obstacles(i);
    th = linspace(0, 2*pi, 20);
    fill(obs.pos(1) + obs.radius*cos(th), obs.pos(2) + obs.radius*sin(th), ...
        [0.6, 0.4, 0.2], 'FaceAlpha', 0.8, 'EdgeColor', 'k');
end

% Trajectory line (will update)
h_traj = plot(robot.pos(1), robot.pos(2), '-', 'Color', [0, 0.7, 0], 'LineWidth', 1.5);

axis equal;
xlim([0, world_width]); ylim([0, world_height]);
xlabel('X (m)', 'Color', 'w'); ylabel('Y (m)', 'Color', 'w');
hold off;

% === RIGHT PANEL: Local View ===
ax2 = subplot(1, 2, 2);
set(ax2, 'Color', 'k', 'XColor', 'w', 'YColor', 'w');
hold on;
title('Local View', 'FontSize', 14, 'Color', 'w');
axis equal;
local_radius = 4;
xlabel('X (m)', 'Color', 'w'); ylabel('Y (m)', 'Color', 'w');
hold off;

% Video setup
if config.record_video
    video_file = sprintf('output/maze_demo_%s_%s.mp4', config.maze_type, config.local_planner);
    vid = VideoWriter(video_file, 'MPEG-4');
    vid.FrameRate = 15;
    open(vid);
    fprintf('Recording: %s\n', video_file);
end

%% ======================= MAIN SIMULATION LOOP =======================
fprintf('\nSimulating...\n');

current_wp = 1;
t = 0;
step = 0;
trajectory = robot.pos;

while t < config.max_time
    step = step + 1;
    
    %% Build obstacle list for main robot's local planner
    obstacles_for_robot = [wall_obstacles, static_obstacles];
    
    % Add other robots as dynamic obstacles
    for i = 1:length(other_robots)
        obs = Obstacle(other_robots(i).pos, other_robots(i).radius, other_robots(i).vel(:)');
        obstacles_for_robot = [obstacles_for_robot, obs];
    end
    
    %% Plan for main robot (waypoint follower + local planner)
    [v_cmd, current_wp, goal_reached, robot_cones] = ...
        waypoint_follower(robot, waypoints, current_wp, obstacles_for_robot, config.local_planner, 1);
    
    % Detect if local planner is deviating from A* path direction
    is_avoiding = false;
    if current_wp <= size(waypoints, 1) && norm(v_cmd) > 0.01
        wp_dir = waypoints(current_wp, :) - robot.pos;
        wp_dir = wp_dir / (norm(wp_dir) + 1e-6);
        v_dir = v_cmd / (norm(v_cmd) + 1e-6);
        angle_diff = acos(max(-1, min(1, dot(wp_dir, v_dir))));
        is_avoiding = (rad2deg(angle_diff) > 25);  % >25° deviation = avoiding
    end
    
    if goal_reached
        fprintf('\n*** GOAL REACHED at t=%.1fs ***\n', t);
        break;
    end
    
    %% Update main robot
    robot.vel = v_cmd(:)';
    robot.pos = robot.pos + robot.vel * config.dt;
    robot.theta = atan2(robot.vel(2), robot.vel(1));
    trajectory = [trajectory; robot.pos];
    
    %% Plan and update other robots (they also use local planners!)
    for i = 1:length(other_robots)
        % Build obstacles for this robot (walls + static + main robot + other robots)
        obs_for_i = [wall_obstacles, static_obstacles];
        obs_for_i = [obs_for_i, Obstacle(robot.pos, robot.radius, robot.vel)];
        for j = 1:length(other_robots)
            if i ~= j
                obs_for_i = [obs_for_i, Obstacle(other_robots(j).pos, other_robots(j).radius, other_robots(j).vel(:)')];
            end
        end
        
        % Simple goal-directed planning (no waypoints, just local planner)
        temp_robot = Robot(other_robots(i).id, other_robots(i).pos, other_robots(i).theta, ...
            other_robots(i).radius, other_robots(i).v_max, other_robot_goals(i,:));
        temp_robot.vel = other_robots(i).vel;
        
        switch upper(config.local_planner)
            case 'VO',   v_i = plan_VO(temp_robot, obs_for_i);
            case 'RVO',  v_i = plan_RVO_new(temp_robot, obs_for_i);
            case 'HRVO', v_i = plan_HRVO_new(temp_robot, obs_for_i);
        end
        
        other_robots(i).vel = v_i(:)';
        other_robots(i).pos = other_robots(i).pos + other_robots(i).vel * config.dt;
        
        % Check if reached goal
        if norm(other_robots(i).pos - other_robot_goals(i,:)) < 0.5
            other_robots(i).vel = [0, 0];
        end
    end
    
    %% Visualization (every 2 steps)
    if mod(step, 2) == 0
        % Update global view
        subplot(ax1);
        set(h_robot_global, 'XData', robot.pos(1), 'YData', robot.pos(2));
        set(h_traj, 'XData', trajectory(:,1), 'YData', trajectory(:,2));
        for i = 1:length(other_robots)
            set(h_other_global(i), 'XData', other_robots(i).pos(1), 'YData', other_robots(i).pos(2));
        end
        
        % Update local view
        subplot(ax2);
        cla;
        hold on;
        
        xlim([robot.pos(1)-local_radius, robot.pos(1)+local_radius]);
        ylim([robot.pos(2)-local_radius, robot.pos(2)+local_radius]);
        
        % Draw VO/RVO/HRVO cones (with correct apex positions)
        % Cones format: [theta_min, theta_max, apex_x, apex_y]
        if ~isempty(robot_cones)
            MAX_CONES_TO_SHOW = 10;  % Limit for visual clarity
            num_cones = min(size(robot_cones, 1), MAX_CONES_TO_SHOW);
            for c = 1:num_cones
                % Calculate apex position in world coordinates
                % apex = robot position + velocity-space apex offset
                if size(robot_cones, 2) >= 4
                    apex_offset = robot_cones(c, 3:4);
                    apex = robot.pos(:) + apex_offset(:);  % Ensure column vector
                else
                    apex = robot.pos(:);  % Ensure column vector
                end
                % Draw cone with magenta color
                plot_cone(apex, robot_cones(c, 1), robot_cones(c, 2), local_radius*0.8, [0.8, 0.2, 0.8]);
            end
        end
        
        % Draw wall obstacles in local view (gray circles)
        for i = 1:length(wall_obstacles)
            obs = wall_obstacles(i);
            if norm(obs.pos - robot.pos) < local_radius + obs.radius
                th = linspace(0, 2*pi, 16);
                fill(obs.pos(1) + obs.radius*cos(th), obs.pos(2) + obs.radius*sin(th), ...
                    [0.4, 0.4, 0.4], 'FaceAlpha', 0.6, 'EdgeColor', 'none');
            end
        end
        
        % Draw static obstacles (brown)
        for i = 1:length(static_obstacles)
            obs = static_obstacles(i);
            if norm(obs.pos - robot.pos) < local_radius + obs.radius
                th = linspace(0, 2*pi, 20);
                fill(obs.pos(1) + obs.radius*cos(th), obs.pos(2) + obs.radius*sin(th), ...
                    [0.6, 0.4, 0.2], 'FaceAlpha', 0.8, 'EdgeColor', 'k');
            end
        end
        
        % Draw other robots (colored squares)
        for i = 1:length(other_robots)
            r = other_robots(i);
            if norm(r.pos - robot.pos) < local_radius + 1
                th = linspace(0, 2*pi, 20);
                fill(r.pos(1) + r.radius*cos(th), r.pos(2) + r.radius*sin(th), ...
                    r.color, 'FaceAlpha', 0.8, 'EdgeColor', 'k', 'LineWidth', 1);
                if norm(r.vel) > 0.05
                    quiver(r.pos(1), r.pos(2), r.vel(1), r.vel(2), 0, ...
                        'Color', 'k', 'LineWidth', 1.5, 'MaxHeadSize', 1);
                end
            end
        end
        
        % Draw main robot
        th = linspace(0, 2*pi, 30);
        fill(robot.pos(1) + robot.radius*cos(th), robot.pos(2) + robot.radius*sin(th), ...
            robot.color, 'FaceAlpha', 0.9, 'EdgeColor', 'k', 'LineWidth', 2);
        
        % Velocity vector
        if norm(robot.vel) > 0.01
            quiver(robot.pos(1), robot.pos(2), robot.vel(1)*1.5, robot.vel(2)*1.5, 0, ...
                'Color', 'b', 'LineWidth', 2, 'MaxHeadSize', 0.8);
        end
        
        % Direction to current waypoint
        if current_wp <= size(waypoints, 1)
            wp = waypoints(current_wp, :);
            plot(wp(1), wp(2), 'mp', 'MarkerSize', 12, 'MarkerFaceColor', 'm');
            plot([robot.pos(1), wp(1)], [robot.pos(2), wp(2)], 'm--', 'LineWidth', 1);
        end
        
        % Title with avoidance indicator (changes color when avoiding)
        if is_avoiding
            title(ax2, sprintf('Local View | t=%.1fs | ⚠ AVOIDING', t), ...
                'FontSize', 13, 'Color', [1, 0.3, 0.3], 'FontWeight', 'bold');
        else
            title(ax2, sprintf('Local View | t=%.1fs', t), ...
                'FontSize', 13, 'Color', 'w', 'FontWeight', 'normal');
        end
        
        hold off;
        drawnow;
        
        if config.record_video && isvalid(fig)
            writeVideo(vid, getframe(fig));
        end
    end
    
    t = t + config.dt;
    
    if mod(step, 100) == 0
        fprintf('  t=%.1fs | WP %d/%d | dist=%.2fm\n', t, current_wp, size(waypoints,1), norm(robot.pos - goal_pos));
    end
end

%% ======================= FINALIZE =======================
if config.record_video
    close(vid);
    fprintf('\nVideo saved: %s\n', video_file);
end

fprintf('\n====== RESULTS ======\n');
fprintf('Time: %.1fs\n', t);
fprintf('Path length: %.2fm (A*: %.2fm)\n', ...
    sum(sqrt(sum(diff(trajectory).^2, 2))), ...
    sum(sqrt(sum(diff(waypoints).^2, 2))));
fprintf('Final distance to goal: %.3fm\n', norm(robot.pos - goal_pos));
fprintf('=====================\n');

%% ======================= HELPER FUNCTIONS =======================

function world_pos = cell_to_world(cell, grid_size, cell_size)
    rows = grid_size(1);
    x = (cell(2) - 0.5) * cell_size;
    y = (rows - cell(1) + 0.5) * cell_size;
    world_pos = [x, y];
end

function obstacles = grid_to_wall_obstacles(grid_map, grid_size, cell_size)
    % Convert grid wall cells to circular obstacles for VO planner
    % Samples boundary wall cells more densely for better collision avoidance
    [rows, cols] = size(grid_map);
    obstacles = [];
    
    % Find wall cells that are adjacent to free cells (boundary walls)
    % This gives better coverage without overwhelming the corridor
    radius = cell_size * 0.5;  % Smaller radius for tighter fit
    
    for r = 2:rows-1
        for c = 2:cols-1
            if grid_map(r, c) == 1
                % Check if adjacent to any free cell
                neighbors = [grid_map(r-1,c), grid_map(r+1,c), grid_map(r,c-1), grid_map(r,c+1)];
                if any(neighbors == 0)
                    pos = cell_to_world([r, c], grid_size, cell_size);
                    obstacles = [obstacles, Obstacle(pos, radius, [0, 0])];
                end
            end
        end
    end
    
    function world_pos = cell_to_world(cell, grid_size, cell_size)
        rows = grid_size(1);
        x = (cell(2) - 0.5) * cell_size;
        y = (rows - cell(1) + 0.5) * cell_size;
        world_pos = [x, y];
    end
end
