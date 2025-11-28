function [v_opt_x, v_opt_y] = plan_VO(robot, obstacles)
    % 1. Determine Preferred Velocity (Vector to Goal)
    vec_to_goal = robot.goal - robot.pos;
    dist_to_goal = norm(vec_to_goal);
    v_pref = (vec_to_goal / dist_to_goal) * robot.v_max;
    
    % 2. Initialize constraints
    % In VO, constraints are "Cones". We build a list of forbidden angles.
    forbidden_intervals =;
    
    for i = 1:length(obstacles)
        % Calculate Relative Position
        p_rel = obstacles(i).pos - robot.pos;
        dist = norm(p_rel);
        
        % Minkowski Sum Radius
        r_combined = robot.radius + obstacles(i).radius;
        
        % Check if collision is already happening (Safety)
        if dist <= r_combined
            v_opt_x = 0; v_opt_y = 0; return; % Emergency Stop
        end
        
        % Calculate Geometric Tangents (The Cone)
        % Angle to center of obstacle
        phi = atan2(p_rel(2), p_rel(1));
        % Half-angle of the cone
        alpha = asin(r_combined / dist);
        
        % Define the "Forbidden" angular interval [phi - alpha, phi + alpha]
        % (Note: In full code, handle wrapping -pi to pi carefully!)
        forbidden_intervals = [forbidden_intervals; phi - alpha, phi + alpha];
    end
    
    % 3. Optimization (Sampling)
    % Simple approach for Phase 1: Check if v_pref is inside any cone.
    % If yes, rotate v_pref until it is free.
    
    current_test_angle = atan2(v_pref(2), v_pref(1));
    is_safe = false;
    
    % Check angles deviating from goal (+/- 5 deg, +/- 10 deg, etc.)
    search_pattern = [0, 5, -5, 10, -10, 15, -15, 30, -30, 45, -45, 90, -90]; 
    
    for k = search_pattern
        test_angle = current_test_angle + deg2rad(k);
        
        % Check if 'test_angle' is inside 'forbidden_intervals'
        in_cone = check_angles(test_angle, forbidden_intervals);
        
        if ~in_cone
            v_opt_x = robot.v_max * cos(test_angle);
            v_opt_y = robot.v_max * sin(test_angle);
            is_safe = true;
            break;
        end
    end
    
    % Fallback: If trapped, stop.
    if ~is_safe
        v_opt_x = 0; v_opt_y = 0;
    end
end