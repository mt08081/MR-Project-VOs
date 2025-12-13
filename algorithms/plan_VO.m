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

    BUFFER_RADII = 0.3;
    
    for i = 1:length(obstacles)
        obs = obstacles(i);
        p_rel = obs.pos - robot.pos;
        dist = norm(p_rel);
        
        if dist > SENSOR_RANGE, continue; end
        
        r_combined = robot.radius + obs.radius + BUFFER_RADII;
        
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
    
    % 3. Optimization (Sampling Search with Multiple Speeds)
    current_test_angle = atan2(v_pref(2), v_pref(1));
    best_v = [0; 0];
    best_cost = inf;
    found_safe = false;
    
    % Finer search pattern: 0, +/-5, +/-10, etc. (degrees)
    angle_offsets = [0, 5, -5, 10, -10, 15, -15, 20, -20, 30, -30, 45, -45, 60, -60, 75, -75, 90, -90, 120, -120, 150, -150, 180];
    
    % Search at multiple speeds (important for letting others pass)
    speed_fractions = [1.0, 0.7, 0.4, 0.2];
    
    for speed_frac = speed_fractions
        test_speed = robot.v_max * speed_frac;
        
        for k = angle_offsets
            test_angle = current_test_angle + deg2rad(k);
            v_cand = [test_speed * cos(test_angle); test_speed * sin(test_angle)];
            
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
                % Cost function: prefer velocities close to v_pref
                cost = norm(v_cand - v_pref);
                if cost < best_cost
                    best_cost = cost;
                    best_v = v_cand;
                    found_safe = true;
                end
                % For first safe velocity at this speed, break angle loop
                % (greedy for speed, we found a safe angle)
                break;
            end
        end
        
        % If we found a safe velocity at this speed, stop searching slower speeds
        if found_safe
            break;
        end
    end
    
    if found_safe
        v_opt = best_v;
    else
        % Emergency: try to back away slowly from nearest obstacle
        if ~isempty(cone_constraints)
            % Move perpendicular to preferred direction (sidestep)
            perp_angle = current_test_angle + pi/2;
            v_opt = 0.3 * robot.v_max * [cos(perp_angle); sin(perp_angle)];
        else
            v_opt = [0; 0]; % No safe velocity found
        end
    end
end