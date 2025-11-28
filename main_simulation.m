% main_simulation.m
% Velocity Obstacle (VO) - Base Implementation
% ---------------------------------------------------------

clc; clear; close all;

% Add paths
addpath('classes');
addpath('algorithms');
addpath('utils');

% 1. Simulation Parameters
dt = 0.1;           % Time step
T_max = 20;         % Total simulation time
frame_rate = 0.05;  % Visualization update rate

% 2. Setup Scenario
% Create Robot: ID=1, Start=[0;0], Heading=0, Radius=0.5, V_max=2.0, Goal=[10;10]
my_robot = Robot(1, [0; 0], 0, 0.5, 2.0, [10; 10]);

% Create Obstacles: (Position, Radius)
obstacles = [
    Obstacle([5; 5], 1.0),      % Center block
    Obstacle([7; 2], 0.8),      % Side block
    Obstacle([3; 8], 0.8)       % Another block
];

% 3. Main Loop
figure('Name', 'VO Simulation', 'Color', 'w');
axis equal; grid on; hold on;
axis([-2 12 -2 12]);
xlabel('X (m)'); ylabel('Y (m)');

for t = 0:dt:T_max
    % --- PLAN ---
    % Compute the VO velocity (Holonomic)
    v_opt = plan_VO(my_robot, obstacles);
    
    % --- ACT ---
    % Robot updates its own physics based on the command
    my_robot = my_robot.move(v_opt, dt);
    
    % --- VISUALIZE ---
    cla; % Clear axes
    
    % Draw Goal
    plot(my_robot.goal(1), my_robot.goal(2), 'gx', 'MarkerSize', 10, 'LineWidth', 2);
    
    % Draw Obstacles (Red)
    for i = 1:length(obstacles)
        viscircles(obstacles(i).pos', obstacles(i).radius, 'Color', 'r');
        % Optional: Draw Safety Margin (Minkowski Sum boundary)
        viscircles(obstacles(i).pos', obstacles(i).radius + my_robot.radius, ...
            'Color', 'r', 'LineStyle', '--', 'LineWidth', 0.5);
    end
    
    % Draw Robot (Blue)
    viscircles(my_robot.pos', my_robot.radius, 'Color', 'b');
    
    % Draw Heading Vector
    quiver(my_robot.pos(1), my_robot.pos(2), ...
           cos(my_robot.theta), sin(my_robot.theta), ...
           0.7, 'Color', 'k', 'LineWidth', 2, 'MaxHeadSize', 0.5);
           
    % Draw Velocity Vector (The command from VO)
    quiver(my_robot.pos(1), my_robot.pos(2), ...
           v_opt(1), v_opt(2), ...
           0, 'Color', 'm', 'LineWidth', 1.5, 'LineStyle', '--');

    title(sprintf('Time: %.2fs', t));
    drawnow limitrate;
    
    % Check Termination
    if norm(my_robot.pos - my_robot.goal) < 0.5
        disp('Goal Reached!');
        break;
    end
end