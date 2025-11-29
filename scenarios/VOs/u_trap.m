function [robot, robot2, obstacles, map_bounds, name] = u_trap()
    name = 'u_trap';
    map_bounds = [-2 14 -2 12];
    
    robot = Robot(1, [0; 5], 0, 0.5, 2.0, [12; 5]);
    robot2 = [];
    
    % Define walls
    wall_back = [Obstacle([6; 5], 0.6), Obstacle([6; 4.2], 0.6), Obstacle([6; 5.8], 0.6)];
    wall_top  = [Obstacle([7; 6.2], 0.6), Obstacle([8; 6.2], 0.6)];
    wall_bot  = [Obstacle([7; 3.8], 0.6), Obstacle([8; 3.8], 0.6)];
    
    obstacles = [wall_back, wall_top, wall_bot];
end