function [v_opt, forbidden_intervals] = plan_VO(robot, obstacles)
    % PLAN_VO Computes optimal velocity considering DYNAMIC OBSTACLES.
    
    % 1. Determine Preferred Velocity (Vector to Goal)
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
    
    % 2. Build Constraints (Geometric Cones)
    % We store the geometric details to check against later
    % Structure: [min_angle, max_angle, obs_vx, obs_vy]
    cone_constraints = []; 
    
    % For visualization return only
    forbidden_intervals = []; 
    
    SENSOR_RANGE = 5.0; 
    
    for i = 1:length(obstacles)
        obs = obstacles(i);
        p_rel = obs.pos - robot.pos;
        dist = norm(p_rel);
        
        if dist > SENSOR_RANGE, continue; end
        
        r_combined = robot.radius + obs.radius;
        
        % Safety Padding: If we are too close, treat as collision
        if dist <= r_combined
            v_opt = [0; 0]; forbidden_intervals=[]; return; 
        end
        
        % Geometric Cone (Relative Position)
        phi = atan2(p_rel(2), p_rel(1));
        alpha = asin(r_combined / dist);
        
        % Define the cone angles
        theta_min = phi - alpha;
        theta_max = phi + alpha;
        
        % Store constraints WITH obstacle velocity
        cone_constraints = [cone_constraints; theta_min, theta_max, obs.vel(1), obs.vel(2)];
        
        % For visualization, we just show the static cone (approximation)
        forbidden_intervals = [forbidden_intervals; theta_min, theta_max];
    end
    
    % 3. Optimization (Sampling Search)
    current_test_angle = atan2(v_pref(2), v_pref(1));
    best_v = [0; 0];
    found_safe = false;
    
    % Search pattern: 0, +/-5, +/-10 ...
    search_pattern = [0, 5, -5, 10, -10, 15, -15, 30, -30, 45, -45, 60, -60, 90, -90, 135, -135, 180]; 
    
    for k = search_pattern
        test_angle = current_test_angle + deg2rad(k);
        v_cand = [robot.v_max * cos(test_angle); robot.v_max * sin(test_angle)];
        
        is_candidate_safe = true;
        
        % --- DYNAMIC CHECK ---
        for c = 1:size(cone_constraints, 1)
            c_min = cone_constraints(c, 1);
            c_max = cone_constraints(c, 2);
            v_obs = cone_constraints(c, 3:4)';
            
            % Calculate RELATIVE velocity
            v_rel = v_cand - v_obs;
            
            % Check if v_rel points into the cone
            angle_rel = atan2(v_rel(2), v_rel(1));
            
            % Re-use check_angles for a single interval
            if check_angles(angle_rel, [c_min, c_max])
                is_candidate_safe = false;
                break; % Fails this cone, try next candidate
            end
        end
        
        if is_candidate_safe
            best_v = v_cand;
            found_safe = true;
            break;
        end
    end
    
    if found_safe
        v_opt = best_v;
    else
        v_opt = [0; 0]; % No safe velocity found
    end
end