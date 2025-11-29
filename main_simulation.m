% main_simulation.m
% Phase 1.9: Modularized VO Simulation
% ---------------------------------------------------------
clc; clear; close all;
addpath('classes'); addpath('algorithms'); addpath('utils');
addpath('scenarios/VOs'); % Make sure this path exists!

% --- CONFIGURATION ---
dt = 0.1; 
T_max = 50; 
SAVE_VIDEO = true;

% Auto-stop logic
blocked_start_sim_time = NaN; 
MAX_BLOCKED_DURATION = 5.0;   

% --- SCENARIO SELECTION ---
% 1=Random, 2=U-Trap, 3=Hallway, 4=Somewhat Busy, 5=Very Busy
CHOICE = 5; 

switch CHOICE
    case 1
        [robot, robot2, obstacles, map_bounds, scn_name] = basic();
    case 2
        [robot, robot2, obstacles, map_bounds, scn_name] = u_trap();
    case 3
        [robot, robot2, obstacles, map_bounds, scn_name] = setup_hallway();
    case 4
        [robot, robot2, obstacles, map_bounds, scn_name] = somewhat_busy();
    case 5
        [robot, robot2, obstacles, map_bounds, scn_name] = very_busy();
    otherwise
        error('Invalid Scenario Choice');
end

VIDEO_NAME = sprintf('output/VO_%s.mp4', scn_name);
fprintf('Running Scenario: %s\nSaving to: %s\n', scn_name, VIDEO_NAME);

% --- VIDEO WRITER ---
if SAVE_VIDEO
    if ~exist('output', 'dir'), mkdir('output'); end
    v = VideoWriter(VIDEO_NAME, 'MPEG-4');
    v.FrameRate = 10; open(v);
end

% --- VISUALIZATION SETUP ---
% Calculate aspect ratio of the map
map_width = map_bounds(2) - map_bounds(1);
map_height = map_bounds(4) - map_bounds(3);
aspect_ratio = map_width / map_height;

% Define screen height you want (e.g., 800 pixels)
fig_height = 800;
fig_width = fig_height * aspect_ratio;

% Create figure with tight fit
fig = figure('Name', 'VO Simulation', 'Color', 'black', ...
    'Position', [100 100 fig_width fig_height]); % Adjust width to match map

ax = axes('Position', [0.05 0.05 0.9 0.9]); % Maximize plot area (removing gray borders)
axis equal; grid on; hold on;
axis(map_bounds); 

% Style the plot
xlabel('X (m)'); ylabel('Y (m)');
set(ax, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'GridColor', 'w', 'GridAlpha', 0.3);

% Draw Static Elements
plot(robot.goal(1), robot.goal(2), 'gx', 'MarkerSize', 10, 'LineWidth', 2);
if ~isempty(robot2), plot(robot2.goal(1), robot2.goal(2), 'rx', 'MarkerSize', 10, 'LineWidth', 2); end

% Legend
h_rob_leg  = plot(NaN,NaN, 'bo', 'MarkerFaceColor', 'b', 'DisplayName', 'Robot 1');
h_obs_leg  = plot(NaN,NaN, 'ro', 'MarkerFaceColor', 'r', 'DisplayName', 'Obstacle');
h_inf_leg  = plot(NaN,NaN, 'r--', 'DisplayName', 'Safety Margin');
h_cone_leg = patch(NaN, NaN, 'm', 'FaceAlpha', 0.2, 'EdgeColor', 'm', 'LineStyle', '--', 'DisplayName', 'VO Cone');
if ~isempty(robot2)
    h_rob2_leg = plot(NaN,NaN, 'ro', 'MarkerFaceColor', 'r', 'DisplayName', 'Robot 2');
    lgd = legend([h_rob_leg, h_rob2_leg, h_obs_leg, h_inf_leg, h_cone_leg], 'Location', 'northeastoutside', 'Color', 'k', 'TextColor', 'w');
else
    lgd = legend([h_rob_leg, h_obs_leg, h_inf_leg, h_cone_leg], 'Location', 'northeastoutside', 'Color', 'k', 'TextColor', 'w');
end
lgd.AutoUpdate = 'off'; 

% Update HUD Text Position (moved +1 in X and -1 in Y for safety)
h_hud = text(map_bounds(1) + map_width*0.05, map_bounds(4) - map_height*0.05, '', ...
    'BackgroundColor', 'b', 'Color', 'w', 'EdgeColor', 'k', 'FontName', 'Consolas');

% --- MAIN LOOP ---
h_dynamic = []; 
collision_detected = false;

for t = 0:dt:T_max
    if ~isempty(h_dynamic), delete(h_dynamic); end
    h_dynamic = [];
    
    % 1. MOVE DYNAMIC OBSTACLES
    for k = 1:length(obstacles)
        if norm(obstacles(k).vel) > 0
            obstacles(k).pos = obstacles(k).pos + obstacles(k).vel * dt;
        end
    end

    % 2. ROBOT 1 PLAN & MOVE
    obs_for_r1 = obstacles;
    if ~isempty(robot2)
        % Treat Robot 2 as a physical obstacle
        % FIX: Use robot2.vel instead of [0;0]
        obs_for_r1 = [obs_for_r1, Obstacle(robot2.pos, robot2.radius, robot2.vel)];
    end
    [v_opt, cones] = plan_VO(robot, obs_for_r1);
    robot = robot.move(v_opt, dt);
    
    % 3. ROBOT 2 PLAN & MOVE
    if ~isempty(robot2)
        % FIX: Use robot.vel instead of [0;0]
        obs_for_r2 = [obstacles, Obstacle(robot.pos, robot.radius, robot.vel)];
        [v_opt2, ~] = plan_VO(robot2, obs_for_r2);
        robot2 = robot2.move(v_opt2, dt);
    end
    
    % --- COLLISION CHECK ---
    % Check Robot 1 vs All Obstacles
    for k = 1:length(obstacles)
        dist = norm(robot.pos - obstacles(k).pos);
        if dist <= (robot.radius + obstacles(k).radius)
            collision_detected = true;
        end
    end
    % Check Robot 1 vs Robot 2
    if ~isempty(robot2)
        dist = norm(robot.pos - robot2.pos);
        if dist <= (robot.radius + robot2.radius)
            collision_detected = true;
        end
    end
    
    % --- VISUALIZATION ---
    h_cones = [];
    if ~isempty(cones)
        for i = 1:size(cones, 1)
            p_h = plot_cone(robot.pos, cones(i,1), cones(i,2), 5.0, 'm');
            h_cones = [h_cones; p_h];
        end
    end
    
    % Draw Robot 1
    h_rob_body = viscircles(robot.pos', robot.radius, 'Color', 'b');
    h_head = quiver(robot.pos(1), robot.pos(2), cos(robot.theta), sin(robot.theta), 0.5, 'Color', 'g', 'LineWidth', 1);
    h_vel = quiver(robot.pos(1), robot.pos(2), v_opt(1), v_opt(2), 0, 'Color', 'r', 'LineStyle', '--', 'LineWidth', 2);
    
    % Draw Obstacles (Redrawn for motion)
    h_obs_all = [];
    for k = 1:length(obstacles)
        h_o = viscircles(obstacles(k).pos', obstacles(k).radius, 'Color', 'r');
        h_s = viscircles(obstacles(k).pos', obstacles(k).radius + robot.radius, 'Color', 'r', 'LineStyle', '--', 'LineWidth', 0.5);
        h_obs_all = [h_obs_all; h_o; h_s];
    end
    
    if ~isempty(robot2)
        h_r2_body = viscircles(robot2.pos', robot2.radius, 'Color', 'r');
        h_r2_head = quiver(robot2.pos(1), robot2.pos(2), cos(robot2.theta), sin(robot2.theta), 0.5, 'Color', 'r', 'LineWidth', 1);
        h_dynamic = [h_cones; h_rob_body; h_head; h_vel; h_r2_body; h_r2_head; h_obs_all];
    else
        h_dynamic = [h_cones; h_rob_body; h_head; h_vel; h_obs_all];
    end
    
    % HUD
    dist_to_goal = norm(robot.goal - robot.pos);
    v_mag = norm(v_opt);
    
    if collision_detected
        state = 'CRASHED!';
        hud_color = 'r';
    else
        state = ij_status(dist_to_goal, v_mag);
        hud_color = 'b';
    end
    
    h_hud.String = sprintf([...
        'Time:        %.2f s\n' ...
        'Dist to Goal: %.2f m\n' ...
        'Cmd Speed:    %.2f m/s\n' ...
        'Status:       %s'], ...
        t, dist_to_goal, v_mag, string(state));
    h_hud.BackgroundColor = hud_color;
    
    drawnow limitrate;
    if SAVE_VIDEO, writeVideo(v, getframe(gcf)); else, pause(0.01); end
    
    if collision_detected
        disp('COLLISION DETECTED! Stopping Simulation.');
        break;
    end
    
    if strcmp(state, 'WAITING / BLOCKED')
        if isnan(blocked_start_sim_time), blocked_start_sim_time = t; end
        if (t - blocked_start_sim_time) > MAX_BLOCKED_DURATION
            disp('DEADLOCK DETECTED. Stopping.'); break;
        end
    else
        blocked_start_sim_time = NaN;
    end
    
    if dist_to_goal < 0.5
        disp('Robot 1 Goal Reached!');
        break;
    end
end

if SAVE_VIDEO, close(v); disp(['Video saved to: ' VIDEO_NAME]); end

function s = ij_status(d, v)
    if d < 0.5, s = 'ARRIVED';
    elseif v < 0.01, s = 'WAITING / BLOCKED';
    else, s = 'NAVIGATING';
    end
end