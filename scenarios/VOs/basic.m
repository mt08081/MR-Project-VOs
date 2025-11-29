function [robot, robot2, obstacles, map_bounds, name] = basic()
    name = 'basic';
    map_bounds = [-2 12 -2 12];
    
    robot = Robot(1, [0; 0], 0, 0.5, 2.0, [10; 10]);
    robot2 = []; % No second robot
    
    obstacles = [
        Obstacle([5; 5], 1.0), ...
        Obstacle([7; 2], 0.8), ...
        Obstacle([3; 8], 0.8)
    ];
end