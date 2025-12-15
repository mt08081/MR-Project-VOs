% multi_agent_simulation.m
% MR-Project-VOs: Multi-Agent Simulation Framework
% Supports: VO (Phase 1), RVO (Phase 2), HRVO (Phase 3)
% N-Robot Scalable System (Phase 4)
% ---------------------------------------------------------
clc; clear; close all;

% 1. SETUP PATHS
addpath('classes'); 
addpath('algorithms'); 
addpath('utils');
addpath('scenarios/multi_agent'); 

% ---------------------------------------------------------
% 2. CONFIGURATION
% ---------------------------------------------------------
dt = 0.1; 
T_max = 60.0; 
SAVE_VIDEO = true;

% Algorithm Selector
% 1 = Velocity Obstacles (VO)
% 2 = Reciprocal VO (RVO)
% 3 = Hybrid RVO (HRVO)
ALGORITHM = 1; 

% Multi-Agent Scenario Selector
% 1 = Crossing 4-Way Intersection
% 2 = Swarm 8 Robots
% 3 = Dense Crowd
% 4 = Wall Corridor
% 5 = Interactive Mode
SCENARIO_ID = 5; 

% Safety & Deadlock Parameters
MAX_BLOCKED_DURATION = 5.0; 
GOAL_THRESHOLD = 0.5;

% ---------------------------------------------------------
% 3. LOAD SCENARIO
% ---------------------------------------------------------
switch SCENARIO_ID
    case 1, [robots, obstacles, map_bounds, scn_name] = crossing_4();
    case 2, [robots, obstacles, map_bounds, scn_name] = swarm_8();
    case 3, [robots, obstacles, map_bounds, scn_name] = dense_crowd();
    case 4, [robots, obstacles, map_bounds, scn_name] = wall_corridor();
    case 5, [robots, obstacles, map_bounds, scn_name] = interactive();
    otherwise, error('Invalid Scenario Choice');
end

N = length(robots);  % Number of robots
algo_name = get_algo_name(ALGORITHM);
VIDEO_NAME = sprintf('output/MultiAgent_%s_%s.mp4', algo_name, scn_name);
fprintf('>> Loaded Multi-Agent Scenario: %s\n', scn_name);
fprintf('>> Number of Robots: %d\n', N);
fprintf('>> Using Algorithm: %s\n', algo_name);
fprintf('>> Saving Video to: %s\n', VIDEO_NAME);

% Track blocked time for each robot (for deadlock detection)
blocked_start_times = NaN(1, N);

% ---------------------------------------------------------
% 4. INITIALIZE VISUALIZATION
% ---------------------------------------------------------
[fig, ax, h_hud, robot_colors] = setup_multi_visualization(map_bounds, robots, obstacles);

if SAVE_VIDEO
    if ~exist('output', 'dir'), mkdir('output'); end
    v = VideoWriter(VIDEO_NAME, 'MPEG-4');
    v.FrameRate = 10; open(v);
end

% ---------------------------------------------------------
% 5. MAIN SIMULATION LOOP
% ---------------------------------------------------------
h_dynamic = []; 

for t = 0:dt:T_max
    if ~isempty(h_dynamic), delete(h_dynamic); end
    h_dynamic = [];
    
    % --- A. UPDATE DYNAMIC OBSTACLES ---
    for k = 1:length(obstacles)
        if norm(obstacles(k).vel) > 0
            obstacles(k).pos = obstacles(k).pos + obstacles(k).vel * dt;
        end
    end

    % --- B. FOR EACH ROBOT: PERCEPTION, PLANNING, ACTION ---
    all_v_opts = zeros(2, N);  % Store planned velocities
    all_cones = cell(1, N);    % Store cones for visualization
    
    for i = 1:N
        % Skip robots that have finished
        if strcmp(robots(i).status, 'arrived') || strcmp(robots(i).status, 'crashed')
            all_v_opts(:, i) = [0; 0];
            continue;
        end
        
        % 1. Perception: Static obstacles + ALL other robots
        obs_for_robot_i = obstacles;
        for j = 1:N
            if i ~= j
                % Add robot j as dynamic obstacle with its CURRENT velocity
                obs_for_robot_i = [obs_for_robot_i, ...
                    Obstacle(robots(j).pos, robots(j).radius, robots(j).vel)];
            end
        end
        
        % 2. Planning: Call the selected Algorithm
        [v_opt, cones] = run_planner(ALGORITHM, robots(i), obs_for_robot_i);
        all_v_opts(:, i) = v_opt;
        all_cones{i} = cones;
        
        % 3. Action: Move Robot i
        robots(i) = robots(i).move(v_opt, dt);
        
        % 4. Update Status
        [robots(i), blocked_start_times(i)] = update_robot_status(...
            robots(i), t, blocked_start_times(i), MAX_BLOCKED_DURATION, GOAL_THRESHOLD);
    end
    
    % --- C. COLLISION DETECTION ---
    collision_pairs = check_all_collisions(robots, obstacles);
    
    % Mark crashed robots
    for p = 1:size(collision_pairs, 1)
        idx = collision_pairs(p, 1);
        if idx <= N
            robots(idx).status = 'crashed';
        end
    end
    
    % --- D. VISUALIZATION & HUD ---
    h_dynamic = draw_multi_scene(robots, obstacles, all_cones, all_v_opts);
    update_multi_hud(h_hud, robots, t, algo_name);
    
    drawnow limitrate;
    if SAVE_VIDEO, writeVideo(v, getframe(gcf)); end
    
    % --- E. TERMINATION LOGIC ---
    if all_robots_finished(robots)
        fprintf('>> ALL ROBOTS FINISHED at T=%.2fs\n', t);
        break;
    end
end

% Final Statistics
print_final_statistics(robots);

if SAVE_VIDEO, close(v); end
fprintf('>> Simulation Complete!\n');


% =========================================================================
% HELPER FUNCTIONS
% =========================================================================

function [v_opt, cones] = run_planner(algo_id, robot, obstacles)
    switch algo_id
        case 1, [v_opt, cones] = plan_VO(robot, obstacles);
        case 2, [v_opt, cones] = plan_RVO_new(robot, obstacles);
        case 3, [v_opt, cones] = plan_HRVO_new(robot, obstacles);
        otherwise, error('Unknown Algorithm ID');
    end
end

function name = get_algo_name(id)
    if id==1, name='VO'; elseif id==2, name='RVO'; elseif id==3, name='HRVO'; else, name='UNK'; end
end

function [robot, blocked_time] = update_robot_status(robot, t, blocked_start, max_blocked, goal_thresh)
    blocked_time = blocked_start;
    dist_to_goal = norm(robot.goal - robot.pos);
    
    if dist_to_goal < goal_thresh
        robot.status = 'arrived';
        if isnan(robot.arrival_time)
            robot.arrival_time = t;
        end
        blocked_time = NaN;
    elseif norm(robot.vel) < 0.02
        % Robot is waiting/blocked
        if isnan(blocked_time)
            blocked_time = t;
        elseif (t - blocked_time) > max_blocked
            % Deadlock detected - still mark as waiting but could extend
            robot.status = 'waiting';
        else
            robot.status = 'waiting';
        end
    else
        robot.status = 'navigating';
        blocked_time = NaN;
    end
end

function finished = all_robots_finished(robots)
    finished = true;
    for i = 1:length(robots)
        if strcmp(robots(i).status, 'navigating') || strcmp(robots(i).status, 'waiting')
            finished = false;
            return;
        end
    end
end

function collision_pairs = check_all_collisions(robots, obstacles)
    collision_pairs = [];
    N = length(robots);
    
    % Robot-Robot collisions
    for i = 1:N
        for j = i+1:N
            if norm(robots(i).pos - robots(j).pos) <= (robots(i).radius + robots(j).radius)
                collision_pairs = [collision_pairs; i, j];
            end
        end
        
        % Robot-Obstacle collisions
        for k = 1:length(obstacles)
            if norm(robots(i).pos - obstacles(k).pos) <= (robots(i).radius + obstacles(k).radius)
                collision_pairs = [collision_pairs; i, -k];  % Negative index for obstacle
            end
        end
    end
end

function print_final_statistics(robots)
    fprintf('\n========================================\n');
    fprintf('        FINAL STATISTICS\n');
    fprintf('========================================\n');
    
    arrived = 0; crashed = 0; waiting = 0;
    total_time = 0;
    
    for i = 1:length(robots)
        r = robots(i);
        switch r.status
            case 'arrived'
                arrived = arrived + 1;
                fprintf('Robot %d: ARRIVED at T=%.2fs\n', i, r.arrival_time);
                total_time = total_time + r.arrival_time;
            case 'crashed'
                crashed = crashed + 1;
                fprintf('Robot %d: CRASHED\n', i);
            case 'waiting'
                waiting = waiting + 1;
                fprintf('Robot %d: STUCK (waiting)\n', i);
            otherwise
                fprintf('Robot %d: Still navigating\n', i);
        end
    end
    
    fprintf('----------------------------------------\n');
    fprintf('Arrived: %d/%d (%.1f%%)\n', arrived, length(robots), 100*arrived/length(robots));
    fprintf('Crashed: %d/%d\n', crashed, length(robots));
    fprintf('Stuck:   %d/%d\n', waiting, length(robots));
    if arrived > 0
        fprintf('Avg Arrival Time: %.2fs\n', total_time/arrived);
    end
    fprintf('========================================\n');
end

% =========================================================================
% VISUALIZATION FUNCTIONS
% =========================================================================

function [fig, ax, h_hud, robot_colors] = setup_multi_visualization(bounds, robots, obs)
    w = bounds(2)-bounds(1); h = bounds(4)-bounds(3);
    fig = figure('Name', 'Multi-Agent VO Simulation', 'Color', 'black', ...
        'Position', [50 50 min(1200, 800*(w/h)) 800]);
    ax = axes('Position', [0.05 0.12 0.75 0.83], 'Color', 'k', 'XColor','w', 'YColor','w');
    axis equal; grid on; hold on; axis(bounds);
    title('Multi-Agent Velocity Obstacle Simulation', 'Color', 'w', 'FontSize', 14);
    
    N = length(robots);
    robot_colors = lines(N);
    
    % Draw Goals for each robot
    for i = 1:N
        plot(robots(i).goal(1), robots(i).goal(2), 'x', 'Color', robot_colors(i,:), ...
            'MarkerSize', 12, 'LineWidth', 2, 'HandleVisibility', 'off');
        text(robots(i).goal(1)+0.3, robots(i).goal(2)+0.3, sprintf('G%d', i), ...
            'Color', robot_colors(i,:), 'FontSize', 8);
    end
    
    % Draw static obstacles (initial)
    for k = 1:length(obs)
        if norm(obs(k).vel) == 0  % Only static ones
            viscircles(obs(k).pos', obs(k).radius, 'Color', [0.5 0 0], 'LineWidth', 1);
        end
    end
    
    % Setup HUD Panel
    h_hud = annotation('textbox', [0.82 0.3 0.17 0.55], ...
        'String', 'Initializing...', ...
        'Color', 'w', 'BackgroundColor', [0.1 0.1 0.1], ...
        'EdgeColor', 'w', 'FontName', 'Consolas', 'FontSize', 9, ...
        'VerticalAlignment', 'top', 'Interpreter', 'none');
    
    % Legend
    legend_items = [];
    legend_labels = {};
    for i = 1:min(N, 8)  % Show max 8 in legend
        h = plot(NaN, NaN, 'o', 'MarkerFaceColor', robot_colors(i,:), ...
            'MarkerEdgeColor', robot_colors(i,:), 'MarkerSize', 8);
        legend_items = [legend_items, h];
        legend_labels{end+1} = sprintf('Robot %d', i);
    end
    h_obs = plot(NaN, NaN, 'ro', 'MarkerFaceColor', 'r', 'DisplayName', 'Obstacle');
    legend_items = [legend_items, h_obs];
    legend_labels{end+1} = 'Obstacle';
    
    legend(legend_items, legend_labels, 'Location', 'northeastoutside', ...
        'Color', 'k', 'TextColor', 'w', 'FontSize', 8);
end

function h_handles = draw_multi_scene(robots, obstacles, all_cones, all_v_opts)
    h_handles = [];
    N = length(robots);
    robot_colors = lines(N);
    
    % Draw velocity cones for navigating robots (optional, first robot only to reduce clutter)
    if N <= 4 && ~isempty(all_cones{1})
        for c = 1:min(size(all_cones{1}, 1), 5)  % Limit cones shown
            if size(all_cones{1}, 1) >= c
                p = plot_cone(robots(1).pos, all_cones{1}(c,1), all_cones{1}(c,2), 4.0, 'm');
                set(p, 'HandleVisibility', 'off');
                h_handles = [h_handles; p];
            end
        end
    end
    
    % Draw all robots with status-based visualization
    for i = 1:N
        r = robots(i);
        v_opt = all_v_opts(:, i);
        
        % Get edge color based on status
        switch r.status
            case 'navigating', edge_color = robot_colors(i,:);
            case 'arrived',    edge_color = [0 1 0];    % Green
            case 'waiting',    edge_color = [1 0.6 0];  % Orange
            case 'crashed',    edge_color = [1 0 0];    % Red
            otherwise,         edge_color = robot_colors(i,:);
        end
        
        % Robot body
        h_body = viscircles(r.pos', r.radius, 'Color', edge_color, 'LineWidth', 2);
        set(findobj(h_body), 'HandleVisibility', 'off');
        h_handles = [h_handles; h_body];
        
        % Robot ID label
        h_txt = text(r.pos(1), r.pos(2)+r.radius+0.3, sprintf('R%d', i), ...
            'Color', 'w', 'HorizontalAlignment', 'center', 'FontSize', 8, 'FontWeight', 'bold');
        h_handles = [h_handles; h_txt];
        
        % Heading direction
        h_head = quiver(r.pos(1), r.pos(2), 0.4*cos(r.theta), 0.4*sin(r.theta), 0, ...
            'Color', 'g', 'LineWidth', 1.5, 'HandleVisibility', 'off');
        h_handles = [h_handles; h_head];
        
        % Velocity vector (dashed)
        if norm(v_opt) > 0.01
            h_vel = quiver(r.pos(1), r.pos(2), v_opt(1), v_opt(2), 0, ...
                'Color', edge_color, 'LineStyle', '--', 'LineWidth', 1.5, 'HandleVisibility', 'off');
            h_handles = [h_handles; h_vel];
        end
    end
    
    % Draw dynamic obstacles
    for k = 1:length(obstacles)
        obs = obstacles(k);
        h_o = viscircles(obs.pos', obs.radius, 'Color', 'r', 'LineWidth', 1.5);
        set(findobj(h_o), 'HandleVisibility', 'off');
        h_handles = [h_handles; h_o];
        
        % Show velocity for dynamic obstacles
        if norm(obs.vel) > 0.01
            h_ov = quiver(obs.pos(1), obs.pos(2), obs.vel(1), obs.vel(2), 0, ...
                'Color', 'r', 'LineWidth', 1, 'HandleVisibility', 'off');
            h_handles = [h_handles; h_ov];
        end
    end
end

function update_multi_hud(h_hud, robots, t, algo_name)
    hud_str = sprintf('Algorithm: %s\nTime: %.2fs\n', algo_name, t);
    hud_str = [hud_str, '------------------------   '];
    
    for i = 1:length(robots)
        r = robots(i);
        dist = norm(r.goal - r.pos);
        speed = norm(r.vel);
        
        % Status indicator
        switch r.status
            case 'navigating', icon = '[NAV]';
            case 'arrived',    icon = '[OK] ';
            case 'waiting',    icon = '[WAIT]';
            case 'crashed',    icon = '[X]  ';
            otherwise,         icon = '[?]  ';
        end
        
        hud_str = [hud_str, sprintf('R%d %s\n  Dist:%.1fm Spd:%.1f\n', ...
            i, icon, dist, speed)];
    end
    
    % Summary counts
    arrived = sum(strcmp({robots.status}, 'arrived'));
    crashed = sum(strcmp({robots.status}, 'crashed'));
    hud_str = [hud_str, '------------------------ '];
    hud_str = [hud_str, sprintf('Done: %d/%d\n', arrived, length(robots))];
    if crashed > 0
        hud_str = [hud_str, sprintf('Crashed: %d\n', crashed)];
    end
    
    h_hud.String = hud_str;
end
