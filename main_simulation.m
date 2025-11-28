% main_simulation.m
% 0. Environment Setup
clc; clear; close all;

% Add paths so MATLAB sees your functions
addpath('algorithms'); 
addpath('utils'); 

% 1. Simulation Parameters
dt = 0.1;       % Time step
T_max = 20;     % Total time

% 2. Initialize Robot and Obstacle
% Note: We use structures here as planned in Phase 1
robot.pos = [0; 0];      % Start at origin
robot.theta = 0;         % Facing East
robot.goal = [10; 10];   % Target position
robot.radius = 0.5;      % Size of robot
robot.v_max = 2.0;       % Maximum speed

obstacle.pos = [5; 5];   % Obstacle in the middle
obstacle.radius = 1.0;   % Size of obstacle

% 3. Main Simulation Loop
figure(1); hold on; grid on; axis equal;
axis([-2 12 -2 12]);

for t = 0:dt:T_max
    % --- PLAN: Get velocity from VO Algorithm ---
    % Ensure your function filename matches this call (e.g., plan_vo.m)
    [vx_ref, vy_ref] = plan_VO(robot, obstacle);
    
    % --- CONTROL: Differential Drive Kinematics ---
    % Calculate desired heading
    target_heading = atan2(vy_ref, vx_ref);
    current_heading = robot.theta;
    
    % Error in heading
    error_theta = angdiff(current_heading, target_heading);
    
    % Controller Gains
    Kp = 4.0; 
    
    % Calculate wheel commands (v = linear, w = angular)
    % If the velocity vector is 0 (stop), force v and w to 0
    if norm([vx_ref, vy_ref]) < 0.01
        v = 0;
        w = 0;
    else
        w = Kp * error_theta;
        % Slow down if we have to turn a lot (cos reduces speed as error increases)
        v = norm([vx_ref, vy_ref]) * cos(error_theta);
        if v < 0, v = 0; end % Prevent backing up
    end
    
    % --- ACT: Update State (Euler Integration) ---
    robot.theta = robot.theta + w * dt;
    robot.pos(1) = robot.pos(1) + v * cos(robot.theta) * dt;
    robot.pos(2) = robot.pos(2) + v * sin(robot.theta) * dt;
    
    % --- VISUALIZE ---
    clf; hold on; grid on; axis equal;
    axis([-2 12 -2 12]);
    
    % Draw Goal
    plot(robot.goal(1), robot.goal(2), 'gx', 'MarkerSize', 10, 'LineWidth', 2);
    
    % Draw Obstacle (Red)
    viscircles(obstacle.pos, obstacle.radius, 'Color', 'r');
    % Draw Minkowski Sum (Dashed Red - Safety Margin)
    viscircles(obstacle.pos, obstacle.radius + robot.radius, 'Color', 'r', 'LineStyle', '--');
    
    % Draw Robot (Blue)
    viscircles(robot.pos', robot.radius, 'Color', 'b');
    
    % Draw Robot Heading (Green Arrow)
    quiver(robot.pos(1), robot.pos(2), cos(robot.theta), sin(robot.theta), 0.5, 'Color', 'g', 'LineWidth', 2);
    
    % Draw Desired Velocity Vector (Dashed Black Arrow)
    quiver(robot.pos(1), robot.pos(2), vx_ref, vy_ref, 0, 'Color', 'k', 'LineStyle', '--');
    
    title(sprintf('Time: %.2fs', t));
    drawnow;
    
    % Stop if close to goal
    if norm(robot.pos - robot.goal') < 0.5
        disp('Goal Reached!');
        break;
    end
end