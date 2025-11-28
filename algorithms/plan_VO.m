function [v_opt, forbidden_intervals] = plan_VO(robot, obstacles)
    % PLAN_VO Computes optimal velocity with a Sensor Range limit.
    
    % 1. Determine Preferred Velocity
    vec_to_goal = robot.goal - robot.pos;
    dist_to_goal = norm(vec_to_goal);
    
    if dist_to_goal > 0.1
        v_pref = (vec_to_goal / dist_to_goal) * robot.v_max;
    else
        v_pref = [0; 0];
        v_opt = [0; 0];
        forbidden_intervals = [];
        return;
    end
    
    % 2. Build Constraints (VO Cones)
    forbidden_intervals = [];
    
    % --- CONFIG: SENSOR RANGE ---
    % Only consider obstacles within 4 meters. 
    % This prevents "Paralysis by Infinity" in hallways.
    SENSOR_RANGE = 5.0; 
    
    for i = 1:length(obstacles)
        obs = obstacles(i);
        p_rel = obs.pos - robot.pos;
        dist = norm(p_rel);
        
        % --- FIX: Ignore far away obstacles ---
        if dist > SENSOR_RANGE
            continue; 
        end
        
        r_combined = robot.radius + obs.radius;
        
        if dist <= r_combined
            v_opt = [0; 0]; return; 
        end
        
        phi = atan2(p_rel(2), p_rel(1));
        alpha = asin(r_combined / dist);
        
        forbidden_intervals = [forbidden_intervals; phi - alpha, phi + alpha];
    end
    
    % 3. Optimization (Sampling)
    current_test_angle = atan2(v_pref(2), v_pref(1));
    best_v = [0; 0];
    found_safe = false;
    
    % Check angles
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
        % If trapped, stop
        v_opt = [0; 0];
    end
end