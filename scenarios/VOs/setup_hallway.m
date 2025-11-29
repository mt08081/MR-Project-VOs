function [robot, robot2, obstacles, map_bounds, name] = setup_hallway()
    name = 'setup_hallway';
    map_bounds = [-2 14 -2 12];
    
    % Robot 1 (Blue)
    robot = Robot(1, [0; 5], 0, 0.5, 2.0, [12; 5]);
    
    % Robot 2 (Red)
    robot2 = Robot(2, [12; 5], pi, 2.0, 2.0, [0; 5]); 
    robot2.color = 'r';
    
    % Walls along top and bottom
    obstacles = [];
    for x = 0:2:12
        obstacles = [obstacles, Obstacle([x; 8], 0.5), Obstacle([x; 2], 0.5)];
    end
end