% INPUT: v_opt (Target velocity vector from VO)
% OUTPUT: v (Linear velocity), w (Angular velocity) for the robot

target_heading = atan2(v_opt_y, v_opt_x);
current_heading = robot.theta;

% Calculate error (Smallest angle difference)
error_theta = angdiff(current_heading, target_heading);

% Proportional Controller for turning
Kp = 2.0; 
w = Kp * error_theta;

% Speed Controller
% If we are facing roughly the right way, move forward. 
% If we are turning sharply, slow down (cos(error) does this naturally).
v = norm([v_opt_x, v_opt_y]) * cos(error_theta);

if v < 0, v = 0; end % Prevent backing up for simple VO