function [v_opt, forbidden_intervals] = plan_HRVO_new(robot, obstacles)
% PLAN_HRVO Hybrid Reciprocal Velocity Obstacles
%
% Implementation based on:
%   Snape, J., Van Den Berg, J., Guy, S.J., & Manocha, D. (2011). 
%   The Hybrid Reciprocal Velocity Obstacle. IEEE Transactions on Robotics.
%
% THEORY (Snape et al., 2011):
% ----------------------------
% RVO solves oscillations but introduces a new problem: the "Reciprocal
% Dance" where agents cannot decide which side to pass on. Both agents
% may simultaneously choose left, then both switch to right, etc.
%
% HRVO solves this by combining ONE leg from VO with ONE leg from RVO,
% creating an asymmetric forbidden region that implicitly encodes a
% passing-side preference (typically right-hand traffic).
%
% CONSTRUCTION (Section III, Snape 2011):
% - Determine passing side using cross product of p_rel and v_rel
% - If passing RIGHT: use right leg from VO, left leg from RVO
% - If passing LEFT: use left leg from VO, right leg from RVO
%
% CRITICAL IMPLEMENTATION NOTE:
% -----------------------------
% HRVO assumes BOTH agents are reactive. For STATIC obstacles,
% we fall back to standard VO (same behavior as plan_VO).
%
% See also: plan_VO, plan_RVO
%
% References:
%   [1] Snape et al. (2011) - HRVO formulation
%   [2] Van den Berg et al. (2008) - RVO foundation
%   [3] Fiorini & Shiller (1998) - Original VO theory

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
    % 2. BUILD VELOCITY OBSTACLES
    % =====================================================================
    % For static: standard VO
    % For dynamic: HRVO with hybrid cone
    %
    % Structure: [theta_min, theta_max, apex_vx, apex_vy, is_hrvo, pass_right]
    cone_constraints = []; 
    forbidden_intervals = []; 
    
    SENSOR_RANGE = 2; 
    BUFFER_RADII = 0.25;
    DYNAMIC_THRESHOLD = 0.05;
    
    for i = 1:length(obstacles)
        obs = obstacles(i);
        p_rel = obs.pos - robot.pos;
        dist = norm(p_rel);
        
        if dist > SENSOR_RANGE, continue; end
        
        r_combined = robot.radius + obs.radius + BUFFER_RADII;
        
        % Emergency escape
        if dist <= r_combined
            escape_dir = -p_rel / max(dist, 0.01);
            v_opt = escape_dir * robot.v_max * 0.4;
            forbidden_intervals = [];
            return;
        end
        
        % -----------------------------------------------------------------
        % Collision Cone Geometry
        % -----------------------------------------------------------------
        phi = atan2(p_rel(2), p_rel(1));
        alpha = asin(r_combined / dist);
        theta_right = phi - alpha;  % Right leg
        theta_left = phi + alpha;   % Left leg
        
        % -----------------------------------------------------------------
        % STATIC vs DYNAMIC handling
        % -----------------------------------------------------------------
        is_dynamic = norm(obs.vel) > DYNAMIC_THRESHOLD;
        
        if is_dynamic
            % ---------------------------------------------------------
            % DYNAMIC: Build HRVO (hybrid of VO and RVO)
            % ---------------------------------------------------------
            % HRVO is defined as intersection of two cones (Snape et al. 2011):
            %   - One leg from VO (apex at v_B)
            %   - One leg from RVO (apex at (v_A + v_B)/2)
            % The hybrid apex is where these two legs intersect.
            
            vo_apex = obs.vel;
            rvo_apex = (robot.vel + obs.vel) / 2;
            
            % Determine passing side (Section III-B, Snape 2011)
            v_rel_current = robot.vel - obs.vel;
            cross_val = p_rel(1) * v_rel_current(2) - p_rel(2) * v_rel_current(1);
            
            % cross > 0: obstacle to left, pass on right
            pass_right = (cross_val >= 0);
            
            % ---------------------------------------------------------
            % Construct TRUE HRVO by intersecting legs from VO and RVO
            % ---------------------------------------------------------
            if pass_right
                % HRVO = Right leg of VO + Left leg of RVO
                % Right leg from VO (origin at vo_apex = v_B)
                angle_vo_leg = theta_right;
                origin_vo_leg = vo_apex;
                
                % Left leg from RVO (origin at rvo_apex)
                angle_rvo_leg = theta_left;
                origin_rvo_leg = rvo_apex;
                
                % The hybrid cone angles (relative to new apex)
                hrvo_theta_right = theta_right;  % From VO
                hrvo_theta_left = theta_left;    % From RVO
            else
                % HRVO = Left leg of VO + Right leg of RVO
                % Left leg from VO (origin at vo_apex = v_B)
                angle_vo_leg = theta_left;
                origin_vo_leg = vo_apex;
                
                % Right leg from RVO (origin at rvo_apex)
                angle_rvo_leg = theta_right;
                origin_rvo_leg = rvo_apex;
                
                % The hybrid cone angles
                hrvo_theta_right = theta_right;  % From RVO
                hrvo_theta_left = theta_left;    % From VO
            end
            
            % Calculate the TRUE hybrid apex (intersection of the two legs)
            hrvo_apex = intersect_rays(origin_vo_leg, angle_vo_leg, ...
                                       origin_rvo_leg, angle_rvo_leg);
            
            cone_constraints = [cone_constraints; ...
                hrvo_theta_right, hrvo_theta_left, hrvo_apex(1), hrvo_apex(2), 1, pass_right];
        else
            % ---------------------------------------------------------
            % STATIC: Use standard VO (apex at v_obs = [0,0])
            % ---------------------------------------------------------
            apex = obs.vel;  % [0,0]
            
            cone_constraints = [cone_constraints; ...
                theta_right, theta_left, apex(1), apex(2), 0, 0];
        end
        
        forbidden_intervals = [forbidden_intervals; theta_right, theta_left];
    end
    
    % =====================================================================
    % 3. VELOCITY OPTIMIZATION
    % =====================================================================
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
            % Membership Test (different for VO vs HRVO cones)
            % -----------------------------------------------------------------
            for c = 1:size(cone_constraints, 1)
                theta_right = cone_constraints(c, 1);
                theta_left = cone_constraints(c, 2);
                apex = cone_constraints(c, 3:4)';
                is_hrvo = cone_constraints(c, 5);
                pass_right = cone_constraints(c, 6);
                
                v_rel = v_cand - apex;
                angle_rel = atan2(v_rel(2), v_rel(1));
                
                if is_hrvo
                    % HRVO: asymmetric check based on passing side
                    % This creates preference for consistent passing
                    in_cone = check_angles(angle_rel, [theta_right, theta_left]);
                    
                    if in_cone
                        % Additional HRVO logic: penalize wrong-side passing
                        % For now, use same check (simplified HRVO)
                        is_candidate_safe = false;
                        break;
                    end
                else
                    % Standard VO check
                    if check_angles(angle_rel, [theta_right, theta_left])
                        is_candidate_safe = false;
                        break;
                    end
                end
            end
            
            if is_candidate_safe
                cost = norm(v_cand - v_pref);
                
                % Small bonus for right-hand passing convention
                if k < 0
                    cost = cost * 0.98;
                end
                
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
        % Prefer right sidestep
        perp_angle = current_test_angle - pi/2;
        v_opt = 0.2 * robot.v_max * [cos(perp_angle); sin(perp_angle)];
    end
end
