function [robots, obstacles, map_bounds, scn_name] = crossing_4()
% CROSSING_4: 4-Way Intersection Scenario
% Four robots cross from cardinal directions to opposite sides
% Tests multi-agent coordination and VO/RVO/HRVO collision avoidance
%
% This is a classic benchmark for testing reciprocal collision avoidance.
% All robots meet at the center simultaneously, requiring coordinated
% navigation without explicit communication.

    scn_name = 'Crossing_4Way';
    map_bounds = [-12 12 -12 12];
    
    % Robot parameters
    robot_radius = 0.8;
    robot_v_max = 1.5;
    
    % Pre-allocate robots array
    robots = Robot.empty(0, 4);
    
    % Define colors for each robot
    colors = [
        0 0.4 0.8;    % Blue (East)
        0.8 0.2 0.2;  % Red (West)
        0.2 0.7 0.2;  % Green (North)
        0.7 0 0.7     % Purple (South)
    ];
    
    % Start positions (at edges) and goals (opposite edges)
    start_positions = [
        8, 0;    % Robot 1: East
       -8, 0;    % Robot 2: West
        0, 8;    % Robot 3: North
        0, -8    % Robot 4: South
    ];
    
    goal_positions = [
       -8, 0;    % Robot 1: East -> West
        8, 0;    % Robot 2: West -> East
        0, -8;   % Robot 3: North -> South
        0, 8     % Robot 4: South -> North
    ];
    
    % Create robots
    for i = 1:4
        start_pos = start_positions(i, :)';
        goal_pos = goal_positions(i, :)';
        
        % Calculate initial heading toward goal
        dir_to_goal = goal_pos - start_pos;
        theta = atan2(dir_to_goal(2), dir_to_goal(1));
        
        robots(i) = Robot(i, start_pos, theta, robot_radius, robot_v_max, goal_pos);
        robots(i).color = colors(i, :);
    end
    
    % No static obstacles - pure multi-agent coordination test
    obstacles = [];
end
