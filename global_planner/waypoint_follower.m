function [v_cmd, current_waypoint_idx, waypoint_reached] = waypoint_follower(robot, waypoints, current_waypoint_idx, obstacles, planner_type, lookahead)
% WAYPOINT_FOLLOWER Follow waypoints using local reactive planner
%
% This function demonstrates hierarchical planning:
%   - Global planner (A*) provides waypoints
%   - Local planner (VO/RVO/HRVO) provides reactive obstacle avoidance
%
% Inputs:
%   robot              - Robot object with current state
%   waypoints          - Nx2 matrix of [x, y] waypoints from global planner
%   current_waypoint_idx - Index of current target waypoint
%   obstacles          - Array of Obstacle objects (can include dynamic)
%   planner_type       - 'VO', 'RVO', or 'HRVO'
%   lookahead          - Number of waypoints to look ahead (default: 1)
%
% Outputs:
%   v_cmd              - [vx, vy] velocity command
%   current_waypoint_idx - Updated waypoint index
%   waypoint_reached   - True if final waypoint reached

    if nargin < 6
        lookahead = 1;  % Direct to current waypoint
    end
    
    waypoint_reached = false;
    num_waypoints = size(waypoints, 1);
    
    % Check if done
    if current_waypoint_idx > num_waypoints
        v_cmd = [0, 0];
        waypoint_reached = true;
        return;
    end
    
    % Distance threshold to consider waypoint reached
    reach_threshold = 0.5;  % meters
    
    % Advance through reached waypoints
    while current_waypoint_idx <= num_waypoints
        current_target = waypoints(current_waypoint_idx, :);
        dist = norm(robot.pos - current_target);
        
        if dist < reach_threshold
            current_waypoint_idx = current_waypoint_idx + 1;
        else
            break;
        end
    end
    
    % Check if final goal reached
    if current_waypoint_idx > num_waypoints
        v_cmd = [0, 0];
        waypoint_reached = true;
        return;
    end
    
    % Select target waypoint (with optional lookahead)
    target_idx = min(current_waypoint_idx + lookahead - 1, num_waypoints);
    target_waypoint = waypoints(target_idx, :);
    
    % Create a temporary robot pointing at this waypoint
    % Robot constructor: Robot(id, start_pos, start_theta, radius, v_max, goal)
    temp_robot = Robot(robot.id, robot.pos, robot.theta, ...
                       robot.radius, robot.v_max, target_waypoint);
    temp_robot.vel = robot.vel;
    
    % Call the local planner
    switch upper(planner_type)
        case 'VO'
            v_cmd = plan_VO(temp_robot, obstacles);
        case 'RVO'
            v_cmd = plan_RVO_new(temp_robot, obstacles);
        case 'HRVO'
            v_cmd = plan_HRVO_new(temp_robot, obstacles);
        otherwise
            error('Unknown planner type: %s', planner_type);
    end
    
    % Ensure output is row vector
    v_cmd = v_cmd(:)';
end
