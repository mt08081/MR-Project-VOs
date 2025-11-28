function [v_opt, forbidden_intervals] = plan_VO(robot, obstacles)
    % PLAN_VO Computes optimal velocity and returns cone constraints for visualization.
    % Returns:
    %   v_opt: [vx; vy]
    %   forbidden_intervals: [min_angle, max_angle] matrix of cones
    
    % 1. Determine Preferred Velocity (Vector to Goal)
    vec_to_goal = robot.goal - robot.pos;
    dist_to_goal = norm(vec_to_goal);
    
    if dist_to_goal > 0.1
        v_pref = (vec_to_goal / dist_to_goal) * robot.v_max;
    else
        v_pref = [0; 0];
        v_opt = [0; 0];
        forbidden_intervals = []; % No cones if stopped
        return;
    end
    
    % 2. Build Constraints (VO Cones)
    forbidden_intervals = [];
    
    for i = 1:length(obstacles)
        obs = obstacles(i);
        p_rel = obs.pos - robot.pos;
        dist = norm(p_rel);
        r_combined = robot.radius + obs.radius;
        
        if dist <= r_combined
            v_opt = [0; 0]; return; 
        end
        
        phi = atan2(p_rel(2), p_rel(1));
        alpha = asin(r_combined / dist);
        
        % Store the interval [phi - alpha, phi + alpha]
        forbidden_intervals = [forbidden_intervals; phi - alpha, phi + alpha];
    end
    
    % 3. Optimization (Sampling)
    current_test_angle = atan2(v_pref(2), v_pref(1));
    best_v = [0; 0];
    found_safe = false;
    
    % Check angles: 0, +5, -5, +10, -10 ...
    search_pattern = [0, 5, -5, 10, -10, 15, -15, 30, -30, 45, -45, 60, -60, 90, -90]; 
    
    for k = search_pattern
        test_angle = current_test_angle + deg2rad(k);
        in_cone = check_angles(test_angle, forbidden_intervals);
        
        if ~in_cone
            best_v = [robot.v_max * cos(test_angle); robot.v_max * sin(test_angle)];
            found_safe = true;
            break;
        end
    end
    
    if found_safe
        v_opt = best_v;
    else
        v_opt = [0; 0];
    end
end