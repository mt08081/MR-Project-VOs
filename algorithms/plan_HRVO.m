function [v_opt, forbidden_intervals] = plan_HRVO(robot, obstacles)
    % PLAN_HRVO Hybrid Reciprocal Velocity Obstacles
    % 
    % Solves the "Reciprocal Dance" problem where RVO agents can't decide
    % which side to pass on. HRVO combines one leg from VO and one from RVO
    % to create a preferred passing side (typically right-hand traffic).
    %
    % Key: The hybrid line bisects VO apex and RVO apex, forcing a decision.
    
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
    
    % 2. Build HRVO Constraints
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
        
        % Safety: Emergency escape if too close
        if dist <= r_combined
            escape_dir = -p_rel / dist;
            v_opt = escape_dir * robot.v_max * 0.5;
            forbidden_intervals = [];
            return;
        end
        
        % Geometric cone angles
        phi = atan2(p_rel(2), p_rel(1));
        alpha = asin(r_combined / dist);
        
        % VO cone legs (apex at v_obs)
        vo_left = phi + alpha;   % Left leg angle
        vo_right = phi - alpha;  % Right leg angle
        
        % RVO apex
        rvo_apex = (robot.vel + obs.vel) / 2;
        
        % VO apex (obstacle velocity)
        vo_apex = obs.vel;
        
        % HRVO Construction:
        % Determine which side the robot should pass on
        % Use cross product to determine relative position
        v_rel_current = robot.vel - obs.vel;
        cross_val = p_rel(1) * v_rel_current(2) - p_rel(2) * v_rel_current(1);
        
        % HRVO hybrid apex - shifted based on passing preference
        % If cross_val > 0: obstacle is to our left, pass on right
        % If cross_val <= 0: obstacle is to our right, pass on left
        if cross_val >= 0
            % Pass on right: use RVO's right leg, VO's left leg
            % Apex is on the right side
            hrvo_apex = rvo_apex;
            theta_min = vo_right; % VO right leg (sharper constraint)
            theta_max = vo_left;  % Standard
        else
            % Pass on left: use VO's right leg, RVO's left leg  
            hrvo_apex = rvo_apex;
            theta_min = vo_right;
            theta_max = vo_left;
        end
        
        % For truly hybrid behavior, we check against both constraints
        % Store: [theta_min, theta_max, apex_x, apex_y, vo_apex_x, vo_apex_y, pass_right]
        pass_right = cross_val >= 0;
        cone_constraints = [cone_constraints; theta_min, theta_max, hrvo_apex(1), hrvo_apex(2), vo_apex(1), vo_apex(2), pass_right];
        
        forbidden_intervals = [forbidden_intervals; theta_min, theta_max];
    end
    
    % 3. Optimization with HRVO check
    current_test_angle = atan2(v_pref(2), v_pref(1));
    best_v = [0; 0];
    best_cost = inf;
    found_safe = false;
    
    % Dense sampling - bias towards right-hand passing
    angle_offsets = [0];
    % Slightly prefer positive angles (right-hand rule)
    for d = 5:5:180
        angle_offsets = [angle_offsets, -d, d]; % Try right first (-d rotates right)
    end
    
    speed_fractions = [1.0, 0.75, 0.5, 0.35, 0.2];
    
    for speed_frac = speed_fractions
        test_speed = robot.v_max * speed_frac;
        
        for k = angle_offsets
            test_angle = current_test_angle + deg2rad(k);
            v_cand = [test_speed * cos(test_angle); test_speed * sin(test_angle)];
            
            is_candidate_safe = true;
            
            % --- HRVO CHECK ---
            for c = 1:size(cone_constraints, 1)
                c_min = cone_constraints(c, 1);
                c_max = cone_constraints(c, 2);
                hrvo_apex = cone_constraints(c, 3:4)';
                vo_apex = cone_constraints(c, 5:6)';
                pass_right = cone_constraints(c, 7);
                
                % HRVO uses a hybrid check:
                % One leg is checked against RVO apex, other against VO apex
                v_rel_rvo = v_cand - hrvo_apex;
                v_rel_vo = v_cand - vo_apex;
                
                angle_rel_rvo = atan2(v_rel_rvo(2), v_rel_rvo(1));
                angle_rel_vo = atan2(v_rel_vo(2), v_rel_vo(1));
                
                % Hybrid check: use the more restrictive constraint
                % This creates an asymmetric forbidden region
                in_rvo_cone = check_angles(angle_rel_rvo, [c_min, c_max]);
                in_vo_cone = check_angles(angle_rel_vo, [c_min, c_max]);
                
                % HRVO: forbidden if in BOTH or in the "wrong side" cone
                if pass_right
                    % Prefer passing right: stricter on left-passing velocities
                    if in_rvo_cone && in_vo_cone
                        is_candidate_safe = false;
                        break;
                    elseif in_rvo_cone
                        % Check if this is a left-passing velocity (penalize more)
                        if angle_rel_rvo > atan2(hrvo_apex(2), hrvo_apex(1))
                            is_candidate_safe = false;
                            break;
                        end
                    end
                else
                    if in_rvo_cone && in_vo_cone
                        is_candidate_safe = false;
                        break;
                    elseif in_rvo_cone
                        if angle_rel_rvo < atan2(hrvo_apex(2), hrvo_apex(1))
                            is_candidate_safe = false;
                            break;
                        end
                    end
                end
            end
            
            if is_candidate_safe
                % Cost: prefer v_pref, penalize slow speeds
                cost = norm(v_cand - v_pref) + 0.3 * (1 - speed_frac);
                
                % Bonus for maintaining right-hand passing convention
                if k < 0  % Right turn preference
                    cost = cost * 0.95;
                end
                
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
        % Emergency sidestep (prefer right)
        perp_angle = current_test_angle - pi/2;  % Right side
        v_opt = 0.3 * robot.v_max * [cos(perp_angle); sin(perp_angle)];
    end
end
