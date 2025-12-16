function [v_opt, forbidden_intervals] = plan_RVO_new(robot, obstacles)
% PLAN_RVO Reciprocal Velocity Obstacles for Multi-Agent Navigation
%
% Implementation based on:
%   Van den Berg, J., Lin, M., & Manocha, D. (2008). Reciprocal Velocity
%   Obstacles for Real-Time Multi-Agent Navigation. IEEE ICRA.
%
% THEORY (Van den Berg et al., 2008):
% ------------------------------------
% Standard VO assumes obstacles don't react, causing oscillations when
% two reactive agents meet (the "mirror effect" or "jitter").
%
% RVO introduces RECIPROCITY: both agents share collision avoidance
% responsibility equally. Instead of agent A fully avoiding B's VO,
% A only moves halfway, expecting B to do the same.
%
% Mathematical Formulation (Equation 3-4, Van den Berg 2008):
%   RVO_{A|B} = { v_A | 2*v_A - v_A^{cur} in VO_{A|B} }
%
% Geometrically, this translates the VO apex:
%   RVO apex = (v_A^{cur} + v_B^{cur}) / 2
%   (instead of v_B for standard VO)
%
% CRITICAL IMPLEMENTATION NOTE:
% -----------------------------
% RVO assumes BOTH agents are reactive and using RVO. For STATIC obstacles,
% we MUST fall back to standard VO (apex at v_obs = [0,0]).
% Otherwise, the robot will get too close before reacting!
%
% See also: plan_VO, plan_HRVO
%
% References:
%   [1] Van den Berg et al. (2008) - RVO formulation
%   [2] Fiorini & Shiller (1998) - Original VO theory
%   [3] LaValle (2006) - Planning Algorithms, Chapter 8

    % =====================================================================
    % 1. PREFERRED VELOCITY COMPUTATION
    % =====================================================================
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
    % 2. BUILD VELOCITY OBSTACLES (VO for static, RVO for dynamic)
    % =====================================================================
    % Structure: [theta_min, theta_max, apex_vx, apex_vy]
    cone_constraints = []; 
    forbidden_intervals = []; 
    
    SENSOR_RANGE = 2; 
    BUFFER_RADII = 0.25;
    DYNAMIC_THRESHOLD = 0.05; % Velocity threshold to consider obstacle "dynamic"
    
    for i = 1:length(obstacles)
        obs = obstacles(i);
        p_rel = obs.pos - robot.pos;
        dist = norm(p_rel);
        
        if dist > SENSOR_RANGE, continue; end
        
        r_combined = robot.radius + obs.radius + BUFFER_RADII;
        
        % Emergency escape if in collision
        if dist <= r_combined
            escape_dir = -p_rel / max(dist, 0.01);
            v_opt = escape_dir * robot.v_max * 0.4;
            forbidden_intervals = [];
            return;
        end
        
        % -----------------------------------------------------------------
        % Collision Cone Geometry (same as VO)
        % -----------------------------------------------------------------
        phi = atan2(p_rel(2), p_rel(1));
        alpha = asin(r_combined / dist);
        theta_min = phi - alpha;
        theta_max = phi + alpha;
        
        % -----------------------------------------------------------------
        % APEX SELECTION: VO vs RVO
        % -----------------------------------------------------------------
        is_dynamic = norm(obs.vel) > DYNAMIC_THRESHOLD;
        
        if is_dynamic
            % DYNAMIC OBSTACLE (other robot): Use RVO
            % RVO apex = (v_robot + v_obs) / 2
            % This shares avoidance responsibility 50-50
            apex = (robot.vel + obs.vel) / 2;
        else
            % STATIC OBSTACLE: Use standard VO
            % VO apex = v_obs = [0, 0]
            apex = obs.vel;  % [0, 0] for static
        end
        
        cone_constraints = [cone_constraints; theta_min, theta_max, apex(1), apex(2)];
        forbidden_intervals = [forbidden_intervals; theta_min, theta_max];
    end
    
    % =====================================================================
    % 3. VELOCITY OPTIMIZATION
    % =====================================================================
    % Same sampling approach as VO
    current_test_angle = atan2(v_pref(2), v_pref(1));
    best_v = [0; 0];
    best_cost = inf;
    found_safe = false;
    
    % Coin-flip to break symmetry bias (prevents "vortex" effect)
    if rand > 0.5
        angle_offsets = [0, 5, -5, 10, -10, 15, -15, 20, -20, 30, -30, 45, -45, 60, -60, 75, -75, 90, -90, 120, -120, 150, -150, 180];
    else
        angle_offsets = [0, -5, 5, -10, 10, -15, 15, -20, 20, -30, 30, -45, 45, -60, 60, -75, 75, -90, 90, -120, 120, -150, 150, 180];
    end
    speed_fractions = [1.0, 0.7, 0.5, 0.3, 0.15];
    
    for speed_frac = speed_fractions
        test_speed = robot.v_max * speed_frac;
        
        for k = angle_offsets
            test_angle = current_test_angle + deg2rad(k);
            v_cand = [test_speed * cos(test_angle); test_speed * sin(test_angle)];
            
            is_candidate_safe = true;
            
            % -----------------------------------------------------------------
            % VO/RVO Membership Test
            % -----------------------------------------------------------------
            % v is forbidden if (v - apex) direction is in collision cone
            for c = 1:size(cone_constraints, 1)
                theta_min_c = cone_constraints(c, 1);
                theta_max_c = cone_constraints(c, 2);
                apex = cone_constraints(c, 3:4)';
                
                v_rel = v_cand - apex;
                angle_rel = atan2(v_rel(2), v_rel(1));
                
                if check_angles(angle_rel, [theta_min_c, theta_max_c])
                    is_candidate_safe = false;
                    break;
                end
            end
            
            if is_candidate_safe
                cost = norm(v_cand - v_pref);
                if cost < best_cost
                    best_cost = cost;
                    best_v = v_cand;
                    found_safe = true;
                end
                break;
            end
        end
        
        if found_safe
            break;
        end
    end
    
    % =====================================================================
    % 4. FALLBACK
    % =====================================================================
    if found_safe
        v_opt = best_v;
    else
        perp_angle = current_test_angle + pi/2;
        v_opt = 0.2 * robot.v_max * [cos(perp_angle); sin(perp_angle)];
    end
end
