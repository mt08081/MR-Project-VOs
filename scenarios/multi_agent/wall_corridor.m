function [robots, obstacles, map_bounds, scn_name] = wall_corridor()
% WALL_CORRIDOR: Constrained Space Navigation
% Robot navigates through a narrow corridor with dynamic obstacles
% Tests navigation in tight spaces where maneuvering room is limited
%
% This scenario is useful for demonstrating how VO algorithms handle
% narrow passages and head-on encounters with dynamic obstacles.

    scn_name = 'Wall_Corridor';
    map_bounds = [-2 22 -6 6];
    
    % Main robot navigating through corridor
    robot_radius = 0.4;
    robot_v_max = 1.5;
    
    robots = Robot.empty(0, 1);
    robots(1) = Robot(1, [0; 0], 0, robot_radius, robot_v_max, [20; 0]);
    robots(1).color = [0 0.5 1];  % Blue
    
    % Build corridor walls using small obstacles
    obstacles = [];
    
    % Top wall (y = 3)
    for x = 0:1.5:18
        obstacles = [obstacles, Obstacle([x; 3.5], 0.6, [0; 0])];
    end
    
    % Bottom wall (y = -3)
    for x = 0:1.5:18
        obstacles = [obstacles, Obstacle([x; -3.5], 0.6, [0; 0])];
    end
    
    % Narrow section in the middle (additional walls)
    obstacles = [obstacles, Obstacle([10; 2], 0.5, [0; 0])];
    obstacles = [obstacles, Obstacle([10; -2], 0.5, [0; 0])];
    
    % Dynamic obstacles in corridor (pedestrians)
    obstacles = [obstacles, Obstacle([8; 0.5], 0.35, [-0.5; 0])];   % Coming toward robot
    obstacles = [obstacles, Obstacle([14; -0.5], 0.35, [-0.6; 0.1])]; % Diagonal
    obstacles = [obstacles, Obstacle([18; 0], 0.4, [-0.4; 0])];      % Near goal
    
    fprintf('   >> Wall Corridor: Corridor with %d wall segments and 3 dynamic agents\n', ...
        length(obstacles) - 3);
end
