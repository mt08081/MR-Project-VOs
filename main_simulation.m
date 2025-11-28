% Setup
robot.pos = [0; 0]; robot.theta = 0; robot.goal = [10; 10];
obstacle.pos = [5; 5]; obstacle.radius = 1;

% Loop
for t = 0:dt:T_max
    % PLAN: Get velocity from VO Algorithm
    [vx_ref, vy_ref] = plan_VO(robot, obstacle);
    
    % CONTROL: Convert to v, w
    [v, w] = diff_drive_control(robot, vx_ref, vy_ref);
    
    % ACT: Update position (Euler Integration)
    robot.theta = robot.theta + w * dt;
    robot.pos(1) = robot.pos(1) + v * cos(robot.theta) * dt;
    robot.pos(2) = robot.pos(2) + v * sin(robot.theta) * dt;
    
    % VISUALIZE
    clf; hold on;
    % Draw Robot
    viscircles(robot.pos', robot.radius, 'Color', 'b');
    % Draw Obstacle (inflated radius for visualization effect)
    viscircles(obstacle.pos', obstacle.radius, 'Color', 'r');
    % Draw Goal
    plot(robot.goal(1), robot.goal(2), 'gx');
    % Draw Velocity Vector (The "Intention")
    quiver(robot.pos(1), robot.pos(2), vx_ref, vy_ref, 'MaxHeadSize', 2);
    
    axis([0 12 0 12]);
    drawnow;
end