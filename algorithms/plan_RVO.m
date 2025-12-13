function [v_opt, forbidden_intervals] = plan_RVO(robot, obstacles)
    % PLAN_RVO Reciprocal Velocity Obstacles - Solves oscillation problem
    % 
    % Key Insight: Instead of assuming obstacles won't react, RVO assumes
    % both agents will share the avoidance responsibility equally.
    % The cone apex is shifted from v_obs to (v_robot + v_obs)/2.
    
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
    
    % 2. Build RVO Constraints
    % For RVO, the cone is centered at (v_robot + v_obs)/2 instead of v_obs
    % This means: v_relative = v_candidate - (v_robot + v_obs)/2
    cone_constraints = []; 
    forbidden_intervals = []; 
    
    SENSOR_RANGE = 7.0; 
    BUFFER_RADII = 0.25;
    
    for i = 1:length(obstacles)
        obs = obstacles(i);
        p_rel = obs.pos - robot.pos;
        dist = norm(p_rel);
        
        if dist > SENSOR_RANGE, continue; end
        
        r_combined = robot.radius + obs.radius + BUFFER_RADII;
        
        % Safety: If too close, don't freeze - try to escape
        if dist <= r_combined
            % Emergency escape: move away from obstacle
            escape_dir = -p_rel / dist;
            v_opt = escape_dir * robot.v_max * 0.5;
            forbidden_intervals = [];
            return;
        end
        
        % Geometric Cone angles (same as VO)
        phi = atan2(p_rel(2), p_rel(1));
        alpha = asin(r_combined / dist);
        theta_min = phi - alpha;
        theta_max = phi + alpha;
        
        % RVO apex: midpoint of current velocities
        % For static obstacles, robot.vel is used; obs.vel is [0;0]
        rvo_apex = (robot.vel + obs.vel) / 2;
        
        % Store: [cone_min, cone_max, apex_vx, apex_vy, is_dynamic]
        is_dynamic = norm(obs.vel) > 0.01;
        cone_constraints = [cone_constraints; theta_min, theta_max, rvo_apex(1), rvo_apex(2), is_dynamic];
        
        forbidden_intervals = [forbidden_intervals; theta_min, theta_max];
    end
    
    % 3. Optimization (Sampling Search with Cost Function)
    current_test_angle = atan2(v_pref(2), v_pref(1));
    best_v = [0; 0];
    best_cost = inf;
    found_safe = false;
    
    % Dense angular sampling
    angle_offsets = [0];
    for d = 5:5:180
        angle_offsets = [angle_offsets, d, -d];
    end
    
    % Multiple speed levels
    speed_fractions = [1.0, 0.75, 0.5, 0.3, 0.15];
    
    for speed_frac = speed_fractions
        test_speed = robot.v_max * speed_frac;
        
        for k = angle_offsets
            test_angle = current_test_angle + deg2rad(k);
            v_cand = [test_speed * cos(test_angle); test_speed * sin(test_angle)];
            
            is_candidate_safe = true;
            
            % --- RVO CHECK ---
            for c = 1:size(cone_constraints, 1)
                c_min = cone_constraints(c, 1);
                c_max = cone_constraints(c, 2);
                rvo_apex = cone_constraints(c, 3:4)';
                
                % RVO: Check relative to the shifted apex
                v_rel = v_cand - rvo_apex;
                angle_rel = atan2(v_rel(2), v_rel(1));
                
                if check_angles(angle_rel, [c_min, c_max])
                    is_candidate_safe = false;
                    break;
                end
            end
            
            if is_candidate_safe
                % Cost: prefer velocities close to v_pref
                % Also slightly penalize slower speeds
                cost = norm(v_cand - v_pref) + 0.5 * (1 - speed_frac);
                if cost < best_cost
                    best_cost = cost;
                    best_v = v_cand;
                    found_safe = true;
                end
            end
        end
    end
    
    if found_safe
        v_opt = best_v;
    else
        % Fallback: sidestep maneuver
        perp_angle = current_test_angle + pi/2;
        v_opt = 0.25 * robot.v_max * [cos(perp_angle); sin(perp_angle)];
    end
end
