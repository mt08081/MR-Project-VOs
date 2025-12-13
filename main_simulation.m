% main_simulation.m
% MR-Project-VOs: Modular Simulation Framework
% Supports: VO (Phase 1), RVO (Phase 2), HRVO (Phase 3)
% ---------------------------------------------------------
clc; clear; close all;

% 1. SETUP PATHS
addpath('classes'); 
addpath('algorithms'); 
addpath('utils');
addpath('scenarios/VOs'); 

% ---------------------------------------------------------
% 2. CONFIGURATION
% ---------------------------------------------------------
dt = 0.1; 
T_max = 50.0; 
SAVE_VIDEO = true;

% Algorithm Selector
% 1 = Velocity Obstacles (VO)
% 2 = Reciprocal VO (RVO)
% 3 = Hybrid RVO (HRVO)
ALGORITHM = 3; 

% Scenario Selector
% 1 = Random Blocks
% 2 = U-Shape Trap
% 3 = Hallway (Head-on)
% 4 = Somewhat Busy Plaza
% 5 = Very Busy Plaza
SCENARIO_ID = 5; 

% Safety & Deadlock Parameters
MAX_BLOCKED_DURATION = 5.0; 
blocked_start_sim_time = NaN;

% ---------------------------------------------------------
% 3. LOAD SCENARIO
% ---------------------------------------------------------
robot2 = []; % Default to empty
switch SCENARIO_ID
    case 1, [robot, robot2, obstacles, map_bounds, scn_name] = basic();
    case 2, [robot, robot2, obstacles, map_bounds, scn_name] = u_trap();
    case 3, [robot, robot2, obstacles, map_bounds, scn_name] = setup_hallway();
    case 4, [robot, robot2, obstacles, map_bounds, scn_name] = somewhat_busy();
    case 5, [robot, robot2, obstacles, map_bounds, scn_name] = very_busy();
    otherwise, error('Invalid Scenario Choice');
end

algo_name = get_algo_name(ALGORITHM);
VIDEO_NAME = sprintf('output/%s_%s.mp4', algo_name, scn_name);
fprintf('>> Loaded Scenario: %s\n', scn_name);
fprintf('>> Using Algorithm: %s\n', algo_name);
fprintf('>> Saving Video to: %s\n', VIDEO_NAME);

% ---------------------------------------------------------
% 4. INITIALIZE VISUALIZATION
% ---------------------------------------------------------
[fig, ax, h_hud, lgd] = setup_visualization(map_bounds, robot, robot2, obstacles);

if SAVE_VIDEO
    if ~exist('output', 'dir'), mkdir('output'); end
    v = VideoWriter(VIDEO_NAME, 'MPEG-4');
    v.FrameRate = 10; open(v);
end

% ---------------------------------------------------------
% 5. MAIN SIMULATION LOOP
% ---------------------------------------------------------
h_dynamic = []; 
collision_detected = false;

for t = 0:dt:T_max
    if ~isempty(h_dynamic), delete(h_dynamic); end
    h_dynamic = [];
    
    % --- A. UPDATE DYNAMIC OBSTACLES ---
    % Move dumb agents based on their constant velocity
    for k = 1:length(obstacles)
        if norm(obstacles(k).vel) > 0
            obstacles(k).pos = obstacles(k).pos + obstacles(k).vel * dt;
        end
    end

    % --- B. ROBOT 1: PERCEPTION & PLANNING ---
    % 1. Perception: Scan for walls + dynamic agents + other robots
    obs_for_r1 = obstacles;
    if ~isempty(robot2)
        % Dynamic VO: See Robot 2 with its CURRENT Velocity
        obs_for_r1 = [obs_for_r1, Obstacle(robot2.pos, robot2.radius, robot2.vel)];
    end
    
    % 2. Planning: Call the selected Algorithm
    [v_opt1, cones1] = run_planner(ALGORITHM, robot, obs_for_r1);
    
    % 3. Action: Move Robot 1
    robot = robot.move(v_opt1, dt);
    
    % --- C. ROBOT 2: PERCEPTION & PLANNING (Optional) ---
    v_opt2 = [0;0]; cones2 = [];
    if ~isempty(robot2)
        obs_for_r2 = [obstacles, Obstacle(robot.pos, robot.radius, robot.vel)];
        [v_opt2, cones2] = run_planner(ALGORITHM, robot2, obs_for_r2);
        robot2 = robot2.move(v_opt2, dt);
    end
    
    % --- D. COLLISION CHECK ---
    if check_collision(robot, obstacles, robot2)
        collision_detected = true;
    end
    
    % --- E. VISUALIZATION & HUD ---
    h_dynamic = draw_scene(robot, robot2, obstacles, cones1, v_opt1, v_opt2);
    
    % Check Status
    dist_to_goal = norm(robot.goal - robot.pos);
    if collision_detected
        status = 'CRASHED!'; hud_color = 'r';
    elseif dist_to_goal < 0.5
        status = 'ARRIVED'; hud_color = 'g';
    elseif norm(v_opt1) < 0.01
        status = 'WAITING'; hud_color = [0.9 0.6 0]; % Orange
    else
        status = 'NAVIGATING'; hud_color = 'b';
    end
    
    h_hud.String = sprintf('Alg: %s | T: %.2fs\nGoal Dist: %.2fm\nStatus: %s', ...
        algo_name, t, dist_to_goal, status);
    h_hud.BackgroundColor = hud_color;
    
    drawnow limitrate;
    if SAVE_VIDEO, writeVideo(v, getframe(gcf)); end
    
    % --- F. TERMINATION LOGIC ---
    if collision_detected
        disp('!! COLLISION DETECTED !!'); break;
    end
    if dist_to_goal < 0.5
        disp('>> SUCCESS: Goal Reached!'); break;
    end
    % Deadlock
    if strcmp(status, 'WAITING')
        if isnan(blocked_start_sim_time), blocked_start_sim_time = t; end
        if (t - blocked_start_sim_time) > MAX_BLOCKED_DURATION
            disp('!! DEADLOCK DETECTED !!'); break;
        end
    else
        blocked_start_sim_time = NaN;
    end
end

if SAVE_VIDEO, close(v); end


% ---------------------------------------------------------
% 6. HELPER FUNCTIONS
% ---------------------------------------------------------

function [v_opt, cones] = run_planner(algo_id, robot, obstacles)
    switch algo_id
        case 1, [v_opt, cones] = plan_VO(robot, obstacles);
        case 2, [v_opt, cones] = plan_RVO(robot, obstacles);
        case 3, [v_opt, cones] = plan_HRVO(robot, obstacles);
        otherwise, error('Unknown Algorithm ID');
    end
end

function is_colliding = check_collision(r1, obstacles, r2)
    is_colliding = false;
    for k = 1:length(obstacles)
        if norm(r1.pos - obstacles(k).pos) <= (r1.radius + obstacles(k).radius)
            is_colliding = true; return;
        end
    end
    if ~isempty(r2)
        if norm(r1.pos - r2.pos) <= (r1.radius + r2.radius)
            is_colliding = true; return;
        end
    end
end

function name = get_algo_name(id)
    if id==1, name='VO'; elseif id==2, name='RVO'; elseif id==3, name='HRVO'; else, name='UNK'; end
end

% --- VISUALIZATION HELPERS (FIXED LEGEND) ---

function h_handles = draw_scene(r1, r2, obstacles, cones1, v1, v2)
    h_handles = [];
    
    % Draw Cones (HandleVisibility off prevents legend clutter)
    if ~isempty(cones1)
        for i = 1:size(cones1, 1)
            p = plot_cone(r1.pos, cones1(i,1), cones1(i,2), 5.0, 'm');
            set(p, 'HandleVisibility', 'off'); % <--- FIX
            h_handles = [h_handles; p];
        end
    end
    
    % Draw Robot 1
    [h_b, h_h, h_v] = draw_robot(r1, v1);
    h_handles = [h_handles; h_b; h_h; h_v];
    
    % Draw Robot 2
    if ~isempty(r2)
        [h_b2, h_h2, h_v2] = draw_robot(r2, v2);
        h_handles = [h_handles; h_b2; h_h2; h_v2];
    end
    
    % Draw Obstacles
    for k = 1:length(obstacles)
        % Create circles with invisible handles
        h_o = viscircles(obstacles(k).pos', obstacles(k).radius, 'Color', 'r');
        set(findobj(h_o), 'HandleVisibility', 'off'); % <--- FIX for viscircles group
        
        h_s = viscircles(obstacles(k).pos', obstacles(k).radius + r1.radius, 'Color', 'r', 'LineStyle', '--', 'LineWidth', 0.5);
        set(findobj(h_s), 'HandleVisibility', 'off'); % <--- FIX
        
        h_handles = [h_handles; h_o; h_s];
    end
end

function [h_body, h_head, h_vel] = draw_robot(r, v_cmd)
    h_body = viscircles(r.pos', r.radius, 'Color', r.color);
    set(findobj(h_body), 'HandleVisibility', 'off'); % <--- FIX
    
    h_head = quiver(r.pos(1), r.pos(2), cos(r.theta), sin(r.theta), 0.5, 'Color', 'g', 'LineWidth', 1, 'HandleVisibility', 'off');
    h_vel  = quiver(r.pos(1), r.pos(2), v_cmd(1), v_cmd(2), 0, 'Color', r.color, 'LineStyle', '--', 'LineWidth', 2, 'HandleVisibility', 'off');
end

function [fig, ax, h_hud, lgd] = setup_visualization(bounds, r1, r2, obs)
    w = bounds(2)-bounds(1); h = bounds(4)-bounds(3);
    fig = figure('Name', 'VO Simulation', 'Color', 'black', 'Position', [50 50 800*(w/h) 800]);
    ax = axes('Position', [0.05 0.05 0.9 0.9], 'Color', 'k', 'XColor','w', 'YColor','w');
    axis equal; grid on; hold on; axis(bounds);
    
    % Draw Static Goals (Keep these in legend/plot logic if desired, or exclude)
    plot(r1.goal(1), r1.goal(2), 'gx', 'MarkerSize', 10, 'LineWidth', 2, 'HandleVisibility', 'off'); 
    if ~isempty(r2), plot(r2.goal(1), r2.goal(2), 'rx', 'MarkerSize', 10, 'LineWidth', 2, 'HandleVisibility', 'off'); end
    
    % --- LEGEND SETUP (The "Dummy" Trick) ---
    % We draw invisible points just to create perfect legend entries
    h_rob = plot(NaN,NaN,'bo','MarkerFaceColor','b','DisplayName','Robot 1');
    h_obs = plot(NaN,NaN,'ro','MarkerFaceColor','r','DisplayName','Obstacle');
    h_inf = plot(NaN,NaN,'r--','DisplayName','Safety Margin');
    h_con = patch(NaN,NaN,'m','FaceAlpha',0.2,'EdgeColor','m','LineStyle','--','DisplayName','VO Cone');
    
    if ~isempty(r2)
        h_rob2 = plot(NaN,NaN,'ro','MarkerFaceColor','r','DisplayName','Robot 2');
        lgd = legend([h_rob, h_rob2, h_obs, h_inf, h_con], 'Location', 'northeastoutside', 'Color', 'k', 'TextColor', 'w');
    else
        lgd = legend([h_rob, h_obs, h_inf, h_con], 'Location', 'northeastoutside', 'Color', 'k', 'TextColor', 'w');
    end
    
    % Setup HUD
    h_hud = text(bounds(1)+w*0.05, bounds(4)-h*0.05, 'Init...', ...
        'BackgroundColor', 'b', 'Color', 'w', 'EdgeColor', 'k', 'FontName', 'Consolas');
end