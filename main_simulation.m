% main_simulation.m
% Phase 1.5: VO with Advanced Visualization & HUD (Fixed)
% ---------------------------------------------------------
clc; clear; close all;

addpath('classes'); addpath('algorithms'); addpath('utils');

% --- CONFIGURATION ---
dt = 0.1; 
T_max = 30; 
robot = Robot(1, [0; 0], 0, 0.5, 2.0, [10; 10]);
obstacles = [
    Obstacle([5; 5], 1.0),      % Center
    Obstacle([7; 2], 0.8),      % Bottom Right
    Obstacle([3; 8], 0.8)       % Top Left
];

% --- VISUALIZATION SETUP ---
fig = figure('Name', 'VO Simulation', 'Color', 'w', 'Position', [100 100 1000 800]);
ax = axes('Position', [0.1 0.1 0.6 0.8]); % Leave room on right for Legend
axis equal; grid on; hold on;
axis([-2 12 -2 12]);
xlabel('X (m)'); ylabel('Y (m)');

% Create Static Legend (Dummy Handles)
h_rob = plot(NaN,NaN, 'bo', 'MarkerFaceColor', 'b', 'DisplayName', 'Robot');
h_obs = plot(NaN,NaN, 'ro', 'MarkerFaceColor', 'r', 'DisplayName', 'Obstacle (Physical)');
h_inf = plot(NaN,NaN, 'r--', 'DisplayName', 'Obstacle (Minkowski/Safety)');
h_goal = plot(NaN,NaN, 'gx', 'LineWidth', 2, 'DisplayName', 'Goal');
h_cone = patch(NaN, NaN, 'm', 'FaceAlpha', 0.2, 'EdgeColor', 'm', 'LineStyle', '--', 'DisplayName', 'Velocity Obstacle (Forbidden)');
h_cmd = quiver(NaN,NaN,0,0, 'Color', 'm', 'LineStyle', '--', 'LineWidth', 1.5, 'DisplayName', 'Commanded Velocity');

legend([h_rob, h_obs, h_inf, h_goal, h_cone, h_cmd], 'Location', 'northeastoutside');

% --- MAIN LOOP ---
for t = 0:dt:T_max
    
    % 1. PLAN: Get velocity AND cones
    [v_opt, cones] = plan_VO(robot, obstacles);
    
    % 2. ACT
    robot = robot.move(v_opt, dt);
    
    % 3. VISUALIZE
    cla(ax); % Clear only the plotting area
    
    % -- Draw Static Elements --
    plot(robot.goal(1), robot.goal(2), 'gx', 'MarkerSize', 10, 'LineWidth', 2);
    
    % -- Draw Obstacles & Safety Bounds --
    for i = 1:length(obstacles)
        viscircles(obstacles(i).pos', obstacles(i).radius, 'Color', 'r');
        viscircles(obstacles(i).pos', obstacles(i).radius + robot.radius, 'Color', 'r', 'LineStyle', '--', 'LineWidth', 0.5);
    end
    
    % -- Draw Cones --
    if ~isempty(cones)
        for i = 1:size(cones, 1)
            plot_cone(robot.pos, cones(i,1), cones(i,2), 5.0, 'm');
        end
    end
    
    % -- Draw Robot --
    viscircles(robot.pos', robot.radius, 'Color', 'b');
    
    % -- Draw Vectors --
    quiver(robot.pos(1), robot.pos(2), cos(robot.theta), sin(robot.theta), 0.5, 'Color', 'g');
    quiver(robot.pos(1), robot.pos(2), v_opt(1), v_opt(2), 0, 'Color', 'r', 'LineStyle', '--', 'LineWidth', 2);
    
    % -- HUD (Fixed) --
    dist_to_goal = norm(robot.goal - robot.pos);
    velocity_mag = norm(v_opt);
    
    hud_text = sprintf([...
        'Time:        %.2f s\n' ...
        'Dist to Goal: %.2f m\n' ...
        'Cmd Speed:    %.2f m/s\n' ...
        'Status:       %s'], ...
        t, dist_to_goal, velocity_mag, ...
        string(ij_status(dist_to_goal, velocity_mag)));
    
    % FIX: Retrieve axis limits into a variable first
    ax_lims = axis(ax); 
    text(ax_lims(1)+0.5, ax_lims(4)-1, hud_text, 'BackgroundColor', 'b', 'EdgeColor', 'k', 'FontName', 'Consolas');

    drawnow limitrate;
    
    if dist_to_goal < 0.5
        disp('Goal Reached!');
        break;
    end
end

% Helper function
function s = ij_status(d, v)
    if d < 0.5, s = 'ARRIVED';
    elseif v < 0.01, s = 'WAITING / BLOCKED';
    else, s = 'NAVIGATING';
    end
end