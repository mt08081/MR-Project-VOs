function [robot, robot2, obstacles, map_bounds, name] = somewhat_busy()
    name = 'somewhat_busy';
    map_bounds = [-5 30 -5 30];
    
    robot = Robot(1, [0; 0], 0, 0.5, 3.0, [25; 25]);
    robot2 = [];
    
    obstacles = [];
    % Static Pillars
    obstacles = [obstacles, Obstacle([10; 10], 1.5), Obstacle([18; 5], 1.0), Obstacle([5; 18], 1.0)];
    
    % Dynamic Agents [vx; vy]
    obstacles = [obstacles, Obstacle([0; 12], 0.6, [1.5; -0.2])]; 
    obstacles = [obstacles, Obstacle([20; 0], 0.6, [-1.0; 1.0])]; 
    obstacles = [obstacles, Obstacle([25; 15], 0.6, [0.0; -2.0])]; 
end