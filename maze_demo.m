%% MAZE_DEMO: Global + Local Planning Integration
% This script demonstrates hierarchical motion planning:
%   1. Global Planner (A*): Computes collision-free path on grid map
%   2. Local Planner (VO/RVO/HRVO): Reactive obstacle avoidance
%
% The demo shows how a robot navigates through a maze, where:
%   - A* provides high-level waypoints through the maze
%   - VO/RVO/HRVO handles real-time obstacle avoidance (dynamic obstacles)
%
% This represents a common real-world architecture used in:
%   - Warehouse robots (Amazon Kiva, etc.)
%   - Autonomous vehicles (Apollo, Autoware)
%   - Service robots (hospital delivery, etc.)
%
% Reference:
%   Thrun, S., Burgard, W., & Fox, D. (2005). Probabilistic Robotics. MIT Press.

clear; clc; close all;

% Add paths
addpath('algorithms');
addpath('classes');
addpath('utils');
addpath('global_planner');

%% Configuration
config.maze_type = 'simple';      % 'simple', 'corridor', 'rooms'
config.grid_size = [30, 30];      % Grid dimensions (smaller = faster)
config.cell_size = 0.5;           % Meters per cell
config.local_planner = 'VO';      % 'VO', 'RVO', 'HRVO'
config.dt = 0.1;                  % Time step (seconds)
config.max_time = 60;             % Maximum simulation time
config.robot_radius = 0.3;        % Robot radius (meters)
config.robot_vmax = 1.2;          % Max velocity (m/s)
config.record_video = false;      % Set to true to save video
config.add_dynamic_obs = true;    % Add dynamic obstacles during navigation

fprintf('=== MAZE DEMO: Global + Local Planning ===\n');
fprintf('Maze Type: %s\n', config.maze_type);
fprintf('Local Planner: %s\n', config.local_planner);

world_width = config.grid_size(2) * config.cell_size;
world_height = config.grid_size(1) * config.cell_size;
fprintf('World: %.1fm x %.1fm\n', world_width, world_height);

%% Generate Maze (Grid for A*, dynamic obstacles for VO)
fprintf('\n[1] Generating maze...\n');

% Create grid map for A*
grid_map = zeros(config.grid_size);

% NOTE: Wall obstacles are handled by A* global planner.
% The local VO planner only handles dynamic obstacles that A* can't predict.
% This is a valid hierarchical planning architecture.

switch config.maze_type
    case 'simple'
        % === SIMPLE MAZE ===
        % Border walls (for A*)
        grid_map(1, :) = 1;
        grid_map(end, :) = 1;
        grid_map(:, 1) = 1;
        grid_map(:, end) = 1;
        
        % L-shaped wall in center
        mid_row = round(config.grid_size(1)/2);
        mid_col = round(config.grid_size(2)/2);
        grid_map(8:mid_row+5, mid_col) = 1;
        grid_map(mid_row, mid_col:mid_col+8) = 1;
        
        start_cell = [config.grid_size(1)-3, 4];
        goal_cell = [4, config.grid_size(2)-4];
        
    case 'corridor'
        % === CORRIDOR MAZE ===
        grid_map(1, :) = 1;
        grid_map(end, :) = 1;
        grid_map(:, 1) = 1;
        grid_map(:, end) = 1;
        
        % Horizontal barriers creating S-curve
        grid_map(8, 1:20) = 1;
        grid_map(16, 10:end) = 1;
        grid_map(24, 1:20) = 1;
        
        start_cell = [config.grid_size(1)-3, 4];
        goal_cell = [4, config.grid_size(2)-4];
        
    case 'rooms'
        % === ROOMS MAZE ===
        grid_map(1, :) = 1;
        grid_map(end, :) = 1;
        grid_map(:, 1) = 1;
        grid_map(:, end) = 1;
        
        mid = round(config.grid_size(1)/2);
        
        % Horizontal wall with gap
        grid_map(mid, 1:mid-3) = 1;
        grid_map(mid, mid+3:end) = 1;
        
        % Vertical wall with gap  
        grid_map(1:mid-3, mid) = 1;
        grid_map(mid+3:end, mid) = 1;
        
        start_cell = [config.grid_size(1)-3, 4];
        goal_cell = [4, config.grid_size(2)-4];
end

% Convert cell coordinates to world coordinates
start_pos = cell_to_world(start_cell, config.grid_size, config.cell_size);
goal_pos = cell_to_world(goal_cell, config.grid_size, config.cell_size);

fprintf('    Start: (%.1f, %.1f)\n', start_pos(1), start_pos(2));
fprintf('    Goal:  (%.1f, %.1f)\n', goal_pos(1), goal_pos(2));

%% Run A* Global Planner
fprintf('\n[2] Running A* global planner...\n');

tic;
[path_cells, path_found] = astar_grid(grid_map, start_cell, goal_cell);
planning_time = toc;

if ~path_found
    error('A* could not find a path! Try a different maze configuration.');
end

% Convert path to world coordinates
waypoints = path_to_world(path_cells, config.grid_size, config.cell_size);

% Use all waypoints (no simplification) for reliable following
% waypoints = simplify_path(waypoints, 0.1);

fprintf('    Path found in %.3f seconds\n', planning_time);
fprintf('    Waypoints: %d\n', size(waypoints, 1));

%% Initialize Robot
fprintf('\n[3] Initializing robot...\n');
robot = Robot(1, start_pos, 0, config.robot_radius, config.robot_vmax, goal_pos);
robot.color = [0, 0.6, 0];  % Green

%% Setup Dynamic Obstacles
dynamic_obstacles = [];
if config.add_dynamic_obs
    fprintf('\n[4] Adding dynamic obstacles...\n');
    
    % Moving obstacle 1: patrols horizontally
    dyn1 = Obstacle([world_width*0.7, world_height*0.3], 0.35, [0.4, 0]);
    dynamic_obstacles = [dynamic_obstacles, dyn1];
    
    % Moving obstacle 2: patrols vertically
    dyn2 = Obstacle([world_width*0.3, world_height*0.7], 0.35, [0, -0.35]);
    dynamic_obstacles = [dynamic_obstacles, dyn2];
    
    fprintf('    Dynamic obstacles: %d\n', length(dynamic_obstacles));
end

%% Setup Visualization (Dual View)
fprintf('\n[5] Setting up visualization...\n');
fig = figure('Name', 'Maze Demo: Global + Local Planning', ...
             'Position', [100, 100, 1400, 600]);

% Left panel: Grid map with A* path
ax1 = subplot(1, 2, 1);
title('Global View (A* Path on Grid)');
hold on;

% Draw grid
imagesc([config.cell_size/2, world_width - config.cell_size/2], ...
        [config.cell_size/2, world_height - config.cell_size/2], ...
        flipud(grid_map));
colormap(ax1, [1 1 1; 0.3 0.3 0.3]);
axis equal;
xlim([0, world_width]);
ylim([0, world_height]);

% Plot A* path
plot(waypoints(:,1), waypoints(:,2), 'b-', 'LineWidth', 2.5, 'DisplayName', 'A* Path');
plot(waypoints(:,1), waypoints(:,2), 'b.', 'MarkerSize', 12);

% Plot start and goal
plot(start_pos(1), start_pos(2), 'go', 'MarkerSize', 15, 'LineWidth', 3, 'DisplayName', 'Start');
plot(goal_pos(1), goal_pos(2), 'rp', 'MarkerSize', 18, 'LineWidth', 2, 'MarkerFaceColor', 'r', 'DisplayName', 'Goal');

% Robot marker
robot_marker_global = plot(robot.pos(1), robot.pos(2), 'ko', ...
    'MarkerSize', 12, 'MarkerFaceColor', robot.color, 'DisplayName', 'Robot');

% Trajectory line
traj_line = plot(robot.pos(1), robot.pos(2), 'g-', 'LineWidth', 1.5, 'DisplayName', 'Trajectory');

legend('Location', 'northeast');
xlabel('X (m)'); ylabel('Y (m)');
grid on;
hold off;

% Right panel: Local view 
ax2 = subplot(1, 2, 2);
title(sprintf('Local View (%s Planner)', config.local_planner));
hold on;
axis equal;
local_view_radius = 4;
xlabel('X (m)'); ylabel('Y (m)');
grid on;

% Video recording
if config.record_video
    video_filename = sprintf('output/maze_demo_%s_%s.mp4', config.maze_type, config.local_planner);
    video = VideoWriter(video_filename, 'MPEG-4');
    video.FrameRate = 20;
    open(video);
    fprintf('    Recording to: %s\n', video_filename);
end

%% Main Simulation Loop
fprintf('\n[6] Starting simulation...\n');
fprintf('    Press Ctrl+C to stop early\n\n');

current_waypoint_idx = 1;
t = 0;
step = 0;
trajectory = robot.pos;

while t < config.max_time
    step = step + 1;
    
    % Only dynamic obstacles are passed to local planner
    % (Static walls are handled by A* global planner)
    all_obstacles = dynamic_obstacles;
    
    % Update dynamic obstacles
    for i = 1:length(dynamic_obstacles)
        dynamic_obstacles(i).pos = dynamic_obstacles(i).pos + dynamic_obstacles(i).vel * config.dt;
        
        % Bounce off boundaries
        if dynamic_obstacles(i).pos(1) < 2 || dynamic_obstacles(i).pos(1) > world_width - 2
            dynamic_obstacles(i).vel(1) = -dynamic_obstacles(i).vel(1);
        end
        if dynamic_obstacles(i).pos(2) < 2 || dynamic_obstacles(i).pos(2) > world_height - 2
            dynamic_obstacles(i).vel(2) = -dynamic_obstacles(i).vel(2);
        end
    end
    
    % Get velocity from waypoint follower + local planner
    [v_cmd, current_waypoint_idx, goal_reached] = ...
        waypoint_follower(robot, waypoints, current_waypoint_idx, all_obstacles, ...
                         config.local_planner, 2);
    
    if goal_reached
        fprintf('\n*** GOAL REACHED at t=%.1f seconds! ***\n', t);
        break;
    end
    
    % Update robot
    robot.vel = v_cmd;
    robot.pos = robot.pos + robot.vel * config.dt;
    robot.theta = atan2(robot.vel(2), robot.vel(1));
    trajectory = [trajectory; robot.pos];
    
    % Update visualization every few steps
    if mod(step, 3) == 0
        % Global view update
        subplot(1, 2, 1);
        set(robot_marker_global, 'XData', robot.pos(1), 'YData', robot.pos(2));
        set(traj_line, 'XData', trajectory(:,1), 'YData', trajectory(:,2));
        
        % Local view update
        subplot(1, 2, 2);
        cla;
        hold on;
        
        xlim([robot.pos(1)-local_view_radius, robot.pos(1)+local_view_radius]);
        ylim([robot.pos(2)-local_view_radius, robot.pos(2)+local_view_radius]);
        
        % Draw obstacles in view
        for i = 1:length(all_obstacles)
            obs = all_obstacles(i);
            if norm(obs.pos - robot.pos) < local_view_radius + obs.radius + 1
                theta_c = linspace(0, 2*pi, 30);
                x_c = obs.pos(1) + obs.radius * cos(theta_c);
                y_c = obs.pos(2) + obs.radius * sin(theta_c);
                
                if any(obs.vel ~= 0)
                    fill(x_c, y_c, [1, 0.5, 0], 'FaceAlpha', 0.7, 'EdgeColor', [0.8, 0.3, 0]);
                    quiver(obs.pos(1), obs.pos(2), obs.vel(1)*2, obs.vel(2)*2, 0, ...
                        'Color', [0.8, 0, 0], 'LineWidth', 2);
                else
                    fill(x_c, y_c, [0.5, 0.5, 0.5], 'FaceAlpha', 0.8);
                end
            end
        end
        
        % Draw robot
        theta_c = linspace(0, 2*pi, 30);
        x_r = robot.pos(1) + robot.radius * cos(theta_c);
        y_r = robot.pos(2) + robot.radius * sin(theta_c);
        fill(x_r, y_r, robot.color, 'FaceAlpha', 0.9, 'EdgeColor', 'k', 'LineWidth', 1.5);
        
        % Velocity arrow
        if norm(robot.vel) > 0.01
            quiver(robot.pos(1), robot.pos(2), robot.vel(1), robot.vel(2), 0, ...
                   'Color', 'b', 'LineWidth', 2, 'MaxHeadSize', 1);
        end
        
        % Draw waypoints
        for i = current_waypoint_idx:min(current_waypoint_idx+3, size(waypoints,1))
            plot(waypoints(i,1), waypoints(i,2), 'mp', 'MarkerSize', 10, 'MarkerFaceColor', 'm');
            if i == current_waypoint_idx
                plot([robot.pos(1), waypoints(i,1)], [robot.pos(2), waypoints(i,2)], 'm--', 'LineWidth', 1);
            end
        end
        
        title(sprintf('Local View (%s) | t=%.1fs | WP %d/%d', ...
              config.local_planner, t, current_waypoint_idx, size(waypoints,1)));
        grid on;
        hold off;
        
        drawnow;
        
        if config.record_video
            frame = getframe(fig);
            writeVideo(video, frame);
        end
    end
    
    t = t + config.dt;
    
    if mod(step, 50) == 0
        dist_to_goal = norm(robot.pos - goal_pos);
        fprintf('  t=%.1fs | Waypoint %d/%d | Distance to goal: %.2fm\n', ...
            t, current_waypoint_idx, size(waypoints,1), dist_to_goal);
    end
end

%% Finalize
if config.record_video
    close(video);
    fprintf('\nVideo saved: %s\n', video_filename);
end

% Final trajectory on global view
figure(fig);
subplot(1, 2, 1);
hold on;
plot(trajectory(:,1), trajectory(:,2), 'g-', 'LineWidth', 2);
plot(robot.pos(1), robot.pos(2), 'kp', 'MarkerSize', 15, 'MarkerFaceColor', 'g');
hold off;

% Statistics
fprintf('\n========== RESULTS ==========\n');
fprintf('Simulation time:    %.1f seconds\n', t);
fprintf('Actual path length: %.2f meters\n', sum(sqrt(sum(diff(trajectory).^2, 2))));
fprintf('A* path length:     %.2f meters\n', sum(sqrt(sum(diff(waypoints).^2, 2))));
fprintf('Final dist to goal: %.3f meters\n', norm(robot.pos - goal_pos));
fprintf('Status: %s\n', robot.status);
fprintf('=============================\n');

%% Helper: Path Simplification
function simplified = simplify_path(waypoints, tolerance)
    if size(waypoints, 1) <= 2
        simplified = waypoints;
        return;
    end
    
    simplified = waypoints(1, :);
    last_dir = [];
    
    for i = 2:size(waypoints, 1)
        if i < size(waypoints, 1)
            dir = waypoints(i, :) - waypoints(i-1, :);
            dir = dir / (norm(dir) + 1e-6);
            
            if isempty(last_dir) || norm(dir - last_dir) > tolerance
                simplified = [simplified; waypoints(i, :)];
                last_dir = dir;
            end
        else
            simplified = [simplified; waypoints(i, :)];
        end
    end
end

%% Helper: Cell to World conversion (local copy)
function world_pos = cell_to_world(cell, grid_size, cell_size)
    rows = grid_size(1);
    x = (cell(2) - 0.5) * cell_size;
    y = (rows - cell(1) + 0.5) * cell_size;
    world_pos = [x, y];
end
