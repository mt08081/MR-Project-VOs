function [robot, robot2, obstacles, map_bounds, name] = very_busy()
    name = 'very_busy';
    map_bounds = [-5 30 -5 30];
    
    % Robot 1: Bottom-Left -> Top-Right
    robot = Robot(1, [0; 0], 0, 0.5, 3.0, [25; 25]);
    
    % Robot 2: Bottom-Right -> Top-Left (Crossing)
    robot2 = Robot(2, [25; 0], pi, 0.5, 3.0, [0; 25]); 
    robot2.color = 'r';
    
    obstacles = [];
    
    % Static Obstacles
    obstacles = [obstacles, ...
        Obstacle([12.5; 12.5], 2.0), ... 
        Obstacle([5; 20], 1.0), ...
        Obstacle([20; 5], 1.0), ...
        Obstacle([8; 8], 0.8), ...
        Obstacle([17; 17], 0.8)];
        
    % Dynamic Obstacles
    obstacles = [obstacles, Obstacle([0; 10], 0.6, [1.2; 0])];
    obstacles = [obstacles, Obstacle([25; 15], 0.6, [-1.4; 0])];
    obstacles = [obstacles, Obstacle([10; 0], 0.6, [0; 1.0])];
    obstacles = [obstacles, Obstacle([15; 25], 0.6, [0; -1.2])];
    obstacles = [obstacles, Obstacle([5; 25], 0.6, [0.5; -0.5])];
end