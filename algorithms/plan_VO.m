function [v_opt, forbidden_intervals] = plan_VO(robot, obstacles)
% PLAN_VO Velocity Obstacles for Dynamic Collision Avoidance
%
% Implementation based on:
%   Fiorini, P., & Shiller, Z. (1998). Motion Planning in Dynamic 
%   Environments Using Velocity Obstacles. IJRR, 17(7), 760-772.
%
% THEORY (Fiorini & Shiller, 1998):
% ---------------------------------
% The Velocity Obstacle VO_{A|B} is the set of all velocities of robot A
% that will result in a collision with obstacle B at some future time t,
% assuming B maintains constant velocity v_B.
%
% Mathematically:
%   VO_{A|B} = { v_A | exists t > 0 : p_A + v_A*t in D(p_B + v_B*t, r_A+r_B) }
%
% This can be rewritten as (Theorem 1):
%   VO_{A|B} = v_B + CC_{A|B}
%
% Where CC_{A|B} is the Collision Cone - a cone with apex at origin,
% tangent to the Minkowski sum disk of radii (r_A + r_B) centered at
% the relative position (p_B - p_A).
%
% The cone half-angle is:
%   alpha = arcsin((r_A + r_B) / ||p_B - p_A||)
%
% VELOCITY SELECTION:
% The optimal velocity is chosen outside all VOs while minimizing
% deviation from the preferred velocity (typically toward goal).
%
% LIMITATIONS:
% - Assumes obstacles maintain constant velocity (reactive agents violate this)
% - When two VO-using agents meet, oscillations occur ("mirror effect")
% - This motivates RVO (Van den Berg et al., 2008)
%
% See also: plan_RVO, plan_HRVO
%
% References:
%   [1] Fiorini & Shiller (1998) - Original VO formulation
%   [2] Choset et al. (2005) - Motion planning fundamentals
%   [3] Siegwart et al. (2011) - Mobile robot navigation

    % =====================================================================
    % 1. PREFERRED VELOCITY COMPUTATION
    % =====================================================================
    % v_pref points toward goal at maximum speed (Section 5, Fiorini 1998)
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
    
    % =====================================================================
    % 2. VELOCITY OBSTACLE CONSTRUCTION
    % =====================================================================
    % For each obstacle, compute collision cone CC and translate by v_B
    % Structure: [theta_min, theta_max, v_obs_x, v_obs_y]
    cone_constraints = []; 
    forbidden_intervals = []; 
    
    % Sensor range - finite time horizon (Section 4.2, Fiorini 1998)
    SENSOR_RANGE = 2; 
    % Safety buffer for uncertainty (Siegwart et al. 2011, Chapter 6)
    BUFFER_RADII = 0.25;
    
    for i = 1:length(obstacles)
        obs = obstacles(i);
        
        % Relative position: p_B - p_A (Section 3.1, Fiorini 1998)
        p_rel = obs.pos - robot.pos;
        dist = norm(p_rel);
        
        if dist > SENSOR_RANGE, continue; end
        
        % Minkowski sum radius (r_A + r_B) with safety buffer
        r_combined = robot.radius + obs.radius + BUFFER_RADII;
        
        % Emergency escape if already in collision configuration
        if dist <= r_combined
            escape_dir = -p_rel / max(dist, 0.01);
            v_opt = escape_dir * robot.v_max * 0.4;
            forbidden_intervals = [];
            return;
        end
        
        % -----------------------------------------------------------------
        % Collision Cone Computation (Theorem 1, Fiorini 1998)
        % -----------------------------------------------------------------
        % Direction to obstacle center
        phi = atan2(p_rel(2), p_rel(1));
        
        % Cone half-angle: alpha = arcsin(r_combined / dist)
        alpha = asin(r_combined / dist);
        
        % Cone boundary angles (right and left tangents)
        theta_min = phi - alpha;  % Right tangent
        theta_max = phi + alpha;  % Left tangent
        
        % Store cone with obstacle velocity for VO translation
        % VO = v_B + CC, so we check if (v_A - v_B) is in CC
        cone_constraints = [cone_constraints; theta_min, theta_max, obs.vel(1), obs.vel(2)];
        forbidden_intervals = [forbidden_intervals; theta_min, theta_max];
    end
    
    % =====================================================================
    % 3. VELOCITY OPTIMIZATION (Sampling-based search)
    % =====================================================================
    % Find v_opt minimizing ||v - v_pref|| s.t. v not in any VO
    % (Section 5 "Avoidance Maneuver", Fiorini 1998)
    current_test_angle = atan2(v_pref(2), v_pref(1));
    best_v = [0; 0];
    best_cost = inf;
    found_safe = false;
    
    % Angular search pattern centered on v_pref direction
    % Coin-flip to break symmetry bias (prevents "vortex" effect)
    if rand > 0.5
        angle_offsets = [0, 5, -5, 10, -10, 15, -15, 20, -20, 30, -30, 45, -45, 60, -60, 75, -75, 90, -90, 120, -120, 150, -150, 180];
    else
        angle_offsets = [0, -5, 5, -10, 10, -15, 15, -20, 20, -30, 30, -45, 45, -60, 60, -75, 75, -90, 90, -120, 120, -150, 150, 180];
    end
    
    % Speed discretization - allows slowing down
    % (Important for non-holonomic systems, Siegwart et al. 2011, Sec 6.3)
    speed_fractions = [1.0, 0.7, 0.5, 0.3, 0.15];
    
    for speed_frac = speed_fractions
        test_speed = robot.v_max * speed_frac;
        
        for k = angle_offsets
            test_angle = current_test_angle + deg2rad(k);
            v_cand = [test_speed * cos(test_angle); test_speed * sin(test_angle)];
            
            is_candidate_safe = true;
            
            % -----------------------------------------------------------------
            % VO Membership Test (Definition 2, Fiorini 1998)
            % -----------------------------------------------------------------
            % v_A is in VO_{A|B} iff (v_A - v_B) is in CC_{A|B}
            for c = 1:size(cone_constraints, 1)
                theta_min_c = cone_constraints(c, 1);
                theta_max_c = cone_constraints(c, 2);
                v_obs = cone_constraints(c, 3:4)';
                
                % Relative velocity: v_rel = v_A - v_B
                v_rel = v_cand - v_obs;
                
                % Check if v_rel direction falls within collision cone
                angle_rel = atan2(v_rel(2), v_rel(1));
                
                if check_angles(angle_rel, [theta_min_c, theta_max_c])
                    is_candidate_safe = false;
                    break;
                end
            end
            
            if is_candidate_safe
                % Cost: prefer velocities close to v_pref
                cost = norm(v_cand - v_pref);
                if cost < best_cost
                    best_cost = cost;
                    best_v = v_cand;
                    found_safe = true;
                end
                break; % Greedy: first safe angle at this speed
            end
        end
        
        if found_safe
            break;
        end
    end
    
    % =====================================================================
    % 4. FALLBACK BEHAVIOR
    % =====================================================================
    if found_safe
        v_opt = best_v;
    else
        % Emergency sidestep when completely blocked
        if ~isempty(cone_constraints)
            perp_angle = current_test_angle + pi/2;
            v_opt = 0.2 * robot.v_max * [cos(perp_angle); sin(perp_angle)];
        else
            v_opt = [0; 0];
        end
    end
end