% main_simulation.m
% Phase 1.5: VO Scenario 3 (Final logic fix)
% ---------------------------------------------------------
clc; clear; close all;
addpath('classes'); addpath('algorithms'); addpath('utils');

% --- CONFIGURATION ---
dt = 0.1; 
T_max = 30; 
SAVE_VIDEO = true;

% Logic to auto-stop if stuck
blocked_start_sim_time = NaN; % Use NaN to flag "not currently blocked"
MAX_BLOCKED_DURATION = 5.0;   % In Simulation Seconds (not real seconds)

% --- SCENARIO SELECTION ---
CHOICE = 3; % 1=Random, 2=U-Trap, 3=Hallway

% Dynamic Video Name
if CHOICE == 1, scenario_name = 'Random_Blocks';
elseif CHOICE == 2, scenario_name = 'U_Shape_Trap';
elseif CHOICE == 3, scenario_name = 'Hallway_Collision';
else, scenario_name = 'Unknown'; end

VIDEO_NAME = sprintf('output/VO_%s.mp4', scenario_name);
fprintf('Running Scenario: %s\nSaving to: %s\n', scenario_name, VIDEO_NAME);

% Initialize 'robot2' as empty
robot2 = []; 

if CHOICE == 1
    % Scenario 1: Random Blocks
    robot = Robot(1, [0; 0], 0, 0.5, 2.0, [10; 10]);
    obstacles = [
        Obstacle([5; 5], 1.0), Obstacle([7; 2], 0.8), Obstacle([3; 8], 0.8)
    ];
elseif CHOICE == 2
    % Scenario 2: The U-Shape Trap
    robot = Robot(1, [0; 5], 0, 0.5, 2.0, [12; 5]);
    wall_back = [Obstacle([6; 5], 0.6), Obstacle([6; 4.2], 0.6), Obstacle([6; 5.8], 0.6)];
    wall_top  = [Obstacle([7; 6.2], 0.6), Obstacle([8; 6.2], 0.6)];
    wall_bot  = [Obstacle([7; 3.8], 0.6), Obstacle([8; 3.8], 0.6)];
    obstacles = [wall_back, wall_top, wall_bot];
elseif CHOICE == 3
    % Scenario 3: The Hallway
    robot = Robot(1, [0; 5], 0, 0.5, 2.0, [12; 5]);
    
    % Note: Radius 2.0 is very large! (Diameter 4m). 
    % In a 6m hallway, this robot takes up 66% of the space alone.
    % This forces the collision very early, which is good for testing.
    robot2 = Robot(2, [12; 5], pi, 2.0, 2.0, [0; 5]); 
    robot2.color = 'r';
    
    obstacles = [];
    for x = 0:2:12
        obstacles = [obstacles, Obstacle([x; 8], 0.5), Obstacle([x; 2], 0.5)];
    end
end

% --- VIDEO WRITER SETUP ---
if SAVE_VIDEO
    if ~exist('output', 'dir'), mkdir('output'); end
    v = VideoWriter(VIDEO_NAME, 'MPEG-4');
    v.FrameRate = 10; 
    open(v);
end

% --- VISUALIZATION SETUP ---
fig = figure('Name', 'VO Simulation', 'Color', 'black', 'Position', [100 100 1000 800]);
ax = axes('Position', [0.1 0.1 0.6 0.8]); 
axis equal; grid on; hold on;
axis([-2 14 -2 12]);
xlabel('X (m)'); ylabel('Y (m)');
set(ax, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'GridColor', 'w', 'GridAlpha', 0.3);

% Draw Static Elements
plot(robot.goal(1), robot.goal(2), 'gx', 'MarkerSize', 10, 'LineWidth', 2);
if ~isempty(robot2)
    plot(robot2.goal(1), robot2.goal(2), 'rx', 'MarkerSize', 10, 'LineWidth', 2);
end

for i = 1:length(obstacles)
    viscircles(obstacles(i).pos', obstacles(i).radius, 'Color', 'r');
    viscircles(obstacles(i).pos', obstacles(i).radius + robot.radius, ...
        'Color', 'r', 'LineStyle', '--', 'LineWidth', 0.5);
end

% Static Legend
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

% HUD
ax_lims = axis(ax);
h_hud = text(ax_lims(1)+0.5, ax_lims(4)-1, '', ...
    'BackgroundColor', 'b', 'Color', 'w', 'EdgeColor', 'k', 'FontName', 'Consolas');

% --- MAIN LOOP ---
h_dynamic = []; 
for t = 0:dt:T_max
    if ~isempty(h_dynamic), delete(h_dynamic); end
    h_dynamic = [];
    
    % --- LOGIC FOR ROBOT 1 ---
    obs_for_r1 = obstacles;
    if ~isempty(robot2)
        obs_for_r1 = [obs_for_r1, Obstacle(robot2.pos, robot2.radius, [0;0])];
    end
    
    [v_opt, cones] = plan_VO(robot, obs_for_r1);
    robot = robot.move(v_opt, dt);
    
    % --- LOGIC FOR ROBOT 2 ---
    if ~isempty(robot2)
        obs_for_r2 = [obstacles, Obstacle(robot.pos, robot.radius, [0;0])];
        [v_opt2, ~] = plan_VO(robot2, obs_for_r2);
        robot2 = robot2.move(v_opt2, dt);
    end
    
    % --- VISUALIZATION ---
    h_cones = [];
    if ~isempty(cones)
        for i = 1:size(cones, 1)
            p_h = plot_cone(robot.pos, cones(i,1), cones(i,2), 5.0, 'm');
            h_cones = [h_cones; p_h];
        end
    end
    h_rob_body = viscircles(robot.pos', robot.radius, 'Color', 'b');
    h_head = quiver(robot.pos(1), robot.pos(2), cos(robot.theta), sin(robot.theta), 0.5, 'Color', 'g', 'LineWidth', 1);
    h_vel = quiver(robot.pos(1), robot.pos(2), v_opt(1), v_opt(2), 0, 'Color', 'r', 'LineStyle', '--', 'LineWidth', 2);
    
    if ~isempty(robot2)
        h_r2_body = viscircles(robot2.pos', robot2.radius, 'Color', 'r');
        h_r2_head = quiver(robot2.pos(1), robot2.pos(2), cos(robot2.theta), sin(robot2.theta), 0.5, 'Color', 'r', 'LineWidth', 1);
        h_dynamic = [h_cones; h_rob_body; h_head; h_vel; h_r2_body; h_r2_head];
    else
        h_dynamic = [h_cones; h_rob_body; h_head; h_vel];
    end
    
    % HUD Update
    dist_to_goal = norm(robot.goal - robot.pos);
    velocity_mag = norm(v_opt);
    state = ij_status(dist_to_goal, velocity_mag);
    
    h_hud.String = sprintf([...
        'Time:        %.2f s\n' ...
        'Dist to Goal: %.2f m\n' ...
        'Cmd Speed:    %.2f m/s\n' ...
        'Status:       %s'], ...
        t, dist_to_goal, velocity_mag, ...
        string(state));
    
    % --- DEADLOCK DETECTION LOGIC (Fixed) ---
    if strcmp(state, 'WAITING / BLOCKED')
        if isnan(blocked_start_sim_time)
            % First time being blocked, record SIMULATION time
            blocked_start_sim_time = t; 
        end
        
        % Check if blocked duration > 5.0 simulation seconds
        if (t - blocked_start_sim_time) > MAX_BLOCKED_DURATION
            disp('DEADLOCK DETECTED (Stuck > 5s). Stopping Simulation.');
            break; % Exit loop. File closing happens below.
        end
    else
        % Reset if we moved
        blocked_start_sim_time = NaN;
    end
        
    drawnow limitrate;
    
    if SAVE_VIDEO
        frame = getframe(gcf);
        writeVideo(v, frame);
    else
        pause(0.05); 
    end
    
    if dist_to_goal < 0.5
        disp('Robot 1 Goal Reached!');
        break;
    end
end

if SAVE_VIDEO
    close(v);
    disp(['Video saved to: ' VIDEO_NAME]);
end

function s = ij_status(d, v)
    if d < 0.5, s = 'ARRIVED';
    elseif v < 0.01, s = 'WAITING / BLOCKED';
    else, s = 'NAVIGATING';
    end
end