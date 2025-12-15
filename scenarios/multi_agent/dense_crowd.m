function [robots, obstacles, map_bounds, scn_name] = dense_crowd()
% DENSE_CROWD: High Density Dynamic Environment
% Two robots navigating through a crowded space with many dynamic agents
% Tests robustness to high obstacle density and dynamic environments
%
% This scenario simulates a crowded plaza or pedestrian area where robots
% must navigate among many moving entities.

    scn_name = 'Dense_Crowd';
    map_bounds = [-5 35 -5 35];
    
    % Main robots
    robot_radius = 0.5;
    robot_v_max = 2.0;
    
    robots = Robot.empty(0, 2);
    
    % Robot 1: Bottom-Left to Top-Right (diagonal)
    robots(1) = Robot(1, [0; 0], pi/4, robot_radius, robot_v_max, [30; 30]);
    robots(1).color = [0 0.5 1];  % Blue
    
    % Robot 2: Top-Left to Bottom-Right (crossing)
    robots(2) = Robot(2, [0; 30], -pi/4, robot_radius, robot_v_max, [30; 0]);
    robots(2).color = [1 0.3 0.3];  % Red
    
    % Generate dense crowd of dynamic obstacles
    obstacles = [];
    
    % Static obstacles (pillars/structures)
    static_positions = [
        15, 15;  % Center
        8, 8;
        22, 22;
        8, 22;
        22, 8
    ];
    
    for i = 1:size(static_positions, 1)
        obstacles = [obstacles, Obstacle(static_positions(i,:)', 1.0, [0; 0])];
    end
    
    % Dynamic agents - various directions and speeds
    % Group 1: Moving right (like pedestrians walking east)
    dynamic_agents = [
        % pos_x, pos_y, vel_x, vel_y, radius
        0, 5, 0.8, 0, 0.4;
        0, 10, 0.7, 0.1, 0.4;
        0, 15, 0.9, -0.1, 0.4;
        0, 20, 0.6, 0.15, 0.4;
        0, 25, 0.75, 0, 0.4;
    ];
    
    % Group 2: Moving left
    dynamic_agents = [dynamic_agents;
        30, 7, -0.7, 0.1, 0.4;
        30, 12, -0.8, 0, 0.4;
        30, 18, -0.65, -0.1, 0.4;
        30, 23, -0.7, 0.05, 0.4;
    ];
    
    % Group 3: Moving up
    dynamic_agents = [dynamic_agents;
        5, 0, 0, 0.75, 0.4;
        12, 0, 0.1, 0.8, 0.4;
        18, 0, -0.1, 0.7, 0.4;
        25, 0, 0, 0.65, 0.4;
    ];
    
    % Group 4: Moving down
    dynamic_agents = [dynamic_agents;
        7, 30, 0.1, -0.7, 0.4;
        13, 30, 0, -0.8, 0.4;
        20, 30, -0.1, -0.75, 0.4;
        27, 30, 0.05, -0.65, 0.4;
    ];
    
    % Group 5: Diagonal movers (crossing the main path)
    dynamic_agents = [dynamic_agents;
        5, 5, 0.5, 0.5, 0.35;
        10, 5, 0.4, 0.6, 0.35;
        5, 10, 0.6, 0.4, 0.35;
        25, 5, -0.5, 0.5, 0.35;
        25, 25, -0.4, -0.4, 0.35;
    ];
    
    % Create all dynamic obstacle objects
    for i = 1:size(dynamic_agents, 1)
        pos = dynamic_agents(i, 1:2)';
        vel = dynamic_agents(i, 3:4)';
        rad = dynamic_agents(i, 5);
        obstacles = [obstacles, Obstacle(pos, rad, vel)];
    end
    
    fprintf('   >> Dense Crowd: %d dynamic agents + %d static obstacles\n', ...
        size(dynamic_agents, 1), size(static_positions, 1));
end
