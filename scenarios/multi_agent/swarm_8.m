function [robots, obstacles, map_bounds, scn_name] = swarm_8()
% SWARM_8: 8 Robot Circle Swap Scenario
% Eight robots arranged in a circle, each navigating to the opposite point
% Tests scalability and dense multi-agent coordination
%
% This scenario is inspired by the classic "antipodal swap" benchmark used
% in multi-robot motion planning research (e.g., ORCA paper).

    scn_name = 'Swarm_8';
    map_bounds = [-15 15 -15 15];
    
    % Robot parameters
    N = 8;
    robot_radius = 0.4;
    robot_v_max = 1.2;
    circle_radius = 10;
    
    % Pre-allocate robots array
    robots = Robot.empty(0, N);
    
    % Generate distinct colors using MATLAB's lines colormap
    colors = lines(N);
    
    % Create robots in circular formation
    for i = 1:N
        % Starting angle (evenly distributed around circle)
        angle_start = (i-1) * 2*pi/N;
        
        % Goal angle (opposite side of circle)
        angle_goal = angle_start + pi;
        
        % Calculate positions
        start_pos = circle_radius * [cos(angle_start); sin(angle_start)];
        goal_pos = circle_radius * [cos(angle_goal); sin(angle_goal)];
        
        % Initial heading toward goal
        dir_to_goal = goal_pos - start_pos;
        theta = atan2(dir_to_goal(2), dir_to_goal(1));
        
        robots(i) = Robot(i, start_pos, theta, robot_radius, robot_v_max, goal_pos);
        robots(i).color = colors(i, :);
    end
    
    % Add a central pillar obstacle to make it more interesting
    obstacles = [
        Obstacle([0; 0], 1.5, [0; 0])   % Central pillar
    ];
end
