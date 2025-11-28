% main_simulation.m
% Phase 1.5: VO with Professional Visualization (Legend Fix)
% ---------------------------------------------------------
clc; clear; close all;

addpath('classes'); addpath('algorithms'); addpath('utils');

% --- CONFIGURATION ---
dt = 0.1; 
T_max = 30; 
SAVE_VIDEO = true;

% --- SCENARIO SELECTION ---
CHOICE = 2; % Change this to 1 or 2

% Dynamic Video Name based on Choice
if CHOICE == 1
    scenario_name = 'Random_Blocks';
elseif CHOICE == 2
    scenario_name = 'U_Shape_Trap';
else
    scenario_name = 'Unknown';
end

VIDEO_NAME = sprintf('output/VO_%s.mp4', scenario_name);
fprintf('Running Scenario: %s\nSaving to: %s\n', scenario_name, VIDEO_NAME);

if CHOICE == 1
    % Scenario 1: Random Blocks
    robot = Robot(1, [0; 0], 0, 0.5, 2.0, [10; 10]);
    obstacles = [
        Obstacle([5; 5], 1.0),      % Center
        Obstacle([7; 2], 0.8),      % Bottom Right
        Obstacle([3; 8], 0.8)       % Top Left
    ];
elseif CHOICE == 2
    % Scenario 2: The U-Shape Trap (Stress Test)
    robot = Robot(1, [0; 5], 0, 0.5, 2.0, [12; 5]); % Start Left, Go Right
    
    % Define walls separately to avoid "vertcat" errors
    wall_back = [Obstacle([6; 5], 0.6), Obstacle([6; 4.2], 0.6), Obstacle([6; 5.8], 0.6)];
    wall_top  = [Obstacle([7; 6.2], 0.6), Obstacle([8; 6.2], 0.6)];
    wall_bot  = [Obstacle([7; 3.8], 0.6), Obstacle([8; 3.8], 0.6)];
    
    % Combine them into one list
    obstacles = [wall_back, wall_top, wall_bot];
end

% --- VIDEO WRITER SETUP ---
if SAVE_VIDEO
    % Ensure output directory exists
    if ~exist('output', 'dir'), mkdir('output'); end
    
    v = VideoWriter(VIDEO_NAME, 'MPEG-4');
    v.FrameRate = 10; % Playback speed
    open(v);
end

% --- VISUALIZATION SETUP ---
fig = figure('Name', 'VO Simulation', 'Color', 'w', 'Position', [100 100 1000 800]);
ax = axes('Position', [0.1 0.1 0.6 0.8]); % Leave room for Legend
axis equal; grid on; hold on;
axis([-2 12 -2 12]);
xlabel('X (m)'); ylabel('Y (m)');

% Draw Static Elements
plot(robot.goal(1), robot.goal(2), 'gx', 'MarkerSize', 10, 'LineWidth', 2);

for i = 1:length(obstacles)
    viscircles(obstacles(i).pos', obstacles(i).radius, 'Color', 'r');
    viscircles(obstacles(i).pos', obstacles(i).radius + robot.radius, ...
        'Color', 'r', 'LineStyle', '--', 'LineWidth', 0.5);
end

% Static Legend
h_rob_leg  = plot(NaN,NaN, 'bo', 'MarkerFaceColor', 'b', 'DisplayName', 'Robot');
h_obs_leg  = plot(NaN,NaN, 'ro', 'MarkerFaceColor', 'r', 'DisplayName', 'Obstacle (Physical)');
h_inf_leg  = plot(NaN,NaN, 'r--', 'DisplayName', 'Obstacle (Safety Margin)');
h_goal_leg = plot(NaN,NaN, 'gx', 'LineWidth', 2, 'DisplayName', 'Goal');
h_cone_leg = patch(NaN, NaN, 'm', 'FaceAlpha', 0.2, 'EdgeColor', 'm', 'LineStyle', '--', 'DisplayName', 'VO Cone (Forbidden)');

lgd = legend([h_rob_leg, h_obs_leg, h_inf_leg, h_goal_leg, h_cone_leg], 'Location', 'northeastoutside');
lgd.AutoUpdate = 'off'; 

% HUD
ax_lims = axis(ax);
h_hud = text(ax_lims(1)+0.5, ax_lims(4)-1, '', ...
    'BackgroundColor', 'b', 'Color', 'w', 'EdgeColor', 'k', 'FontName', 'Consolas');

% --- MAIN LOOP ---
h_dynamic = []; 

for t = 0:dt:T_max
    if ~isempty(h_dynamic), delete(h_dynamic); end
    
    % PLAN & ACT
    [v_opt, cones] = plan_VO(robot, obstacles);
    robot = robot.move(v_opt, dt);
    
    % VISUALIZE
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
    
    h_dynamic = [h_cones; h_rob_body; h_head; h_vel];
    
    dist_to_goal = norm(robot.goal - robot.pos);
    velocity_mag = norm(v_opt);
    
    h_hud.String = sprintf([...
        'Time:        %.2f s\n' ...
        'Dist to Goal: %.2f m\n' ...
        'Cmd Speed:    %.2f m/s\n' ...
        'Status:       %s'], ...
        t, dist_to_goal, velocity_mag, ...
        string(ij_status(dist_to_goal, velocity_mag)));

    drawnow limitrate;

    if SAVE_VIDEO
        frame = getframe(gcf);
        writeVideo(v, frame);
    else
        pause(0.05); 
    end
    
    if dist_to_goal < 0.5
        disp('Goal Reached!');
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