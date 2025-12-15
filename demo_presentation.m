% demo_presentation.m
% MR-Project-VOs: Quick Demo Script for Presentations
% ---------------------------------------------------------
% This script provides a quick way to showcase different scenarios
% during your evaluation. Just change DEMO_TYPE and run!
% ---------------------------------------------------------
clc; clear; close all;

% =========================================================================
%                    QUICK DEMO CONFIGURATION
% =========================================================================
% Choose your demo:
%   1 = Original 2-robot scenarios (main_simulation.m style)
%   2 = Multi-agent: 4-way crossing
%   3 = Multi-agent: 8-robot swarm
%   4 = Multi-agent: Dense crowd
%   5 = Multi-agent: Wall corridor

DEMO_TYPE = 2;

% Algorithm: 1=VO, 2=RVO, 3=HRVO
ALGORITHM = 1;

% =========================================================================
%                         RUN DEMO
% =========================================================================

addpath('classes'); 
addpath('algorithms'); 
addpath('utils');
addpath('scenarios/VOs'); 
addpath('scenarios/multi_agent');

switch DEMO_TYPE
    case 1
        % Run original simulation
        fprintf('\n>> Running Original 2-Robot Demo...\n');
        fprintf('>> Please use main_simulation.m directly for full control.\n\n');
        main_simulation;
        
    case {2, 3, 4, 5}
        % Run multi-agent simulation
        fprintf('\n>> Running Multi-Agent Demo...\n\n');
        
        % Map demo type to scenario
        scenario_map = containers.Map(...
            {2, 3, 4, 5}, ...
            {'crossing_4', 'swarm_8', 'dense_crowd', 'wall_corridor'});
        
        % Temporarily modify multi_agent_simulation settings
        dt = 0.1;
        T_max = 60.0;
        SAVE_VIDEO = false;  % Change to true to save
        GOAL_THRESHOLD = 0.5;
        MAX_BLOCKED_DURATION = 5.0;
        
        % Load scenario
        switch DEMO_TYPE
            case 2, [robots, obstacles, map_bounds, scn_name] = crossing_4();
            case 3, [robots, obstacles, map_bounds, scn_name] = swarm_8();
            case 4, [robots, obstacles, map_bounds, scn_name] = dense_crowd();
            case 5, [robots, obstacles, map_bounds, scn_name] = wall_corridor();
        end
        
        N = length(robots);
        algo_names = {'VO', 'RVO', 'HRVO'};
        algo_name = algo_names{ALGORITHM};
        
        fprintf('╔════════════════════════════════════════════════╗\n');
        fprintf('║           DEMO: %s + %s                \n', upper(scn_name), algo_name);
        fprintf('║           Robots: %d                           \n', N);
        fprintf('╚════════════════════════════════════════════════╝\n\n');
        
        % Initialize
        blocked_start_times = NaN(1, N);
        [fig, ax, h_hud, robot_colors] = setup_demo_vis(map_bounds, robots, obstacles, algo_name, scn_name);
        h_dynamic = [];
        
        % Main loop
        for t = 0:dt:T_max
            if ~isempty(h_dynamic), delete(h_dynamic); end
            h_dynamic = [];
            
            % Update obstacles
            for k = 1:length(obstacles)
                if norm(obstacles(k).vel) > 0
                    obstacles(k).pos = obstacles(k).pos + obstacles(k).vel * dt;
                end
            end
            
            % Plan and move each robot
            all_v_opts = zeros(2, N);
            
            for i = 1:N
                if strcmp(robots(i).status, 'arrived') || strcmp(robots(i).status, 'crashed')
                    continue;
                end
                
                % Perception
                obs_for_robot_i = obstacles;
                for j = 1:N
                    if i ~= j
                        obs_for_robot_i = [obs_for_robot_i, ...
                            Obstacle(robots(j).pos, robots(j).radius, robots(j).vel)];
                    end
                end
                
                % Planning
                v_opt = run_planner(ALGORITHM, robots(i), obs_for_robot_i);
                all_v_opts(:, i) = v_opt;
                
                % Action
                robots(i) = robots(i).move(v_opt, dt);
                
                % Status
                [robots(i), blocked_start_times(i)] = update_status(...
                    robots(i), t, blocked_start_times(i), MAX_BLOCKED_DURATION, GOAL_THRESHOLD);
            end
            
            % Visualization
            h_dynamic = draw_demo_scene(robots, obstacles, all_v_opts);
            update_demo_hud(h_hud, robots, t, algo_name);
            
            drawnow;
            
            % Check termination
            if all_finished(robots)
                fprintf('>> All robots arrived at T=%.2fs\n', t);
                break;
            end
        end
        
        % Print results
        arrived = sum(strcmp({robots.status}, 'arrived'));
        fprintf('\n>> Demo Complete: %d/%d robots arrived\n', arrived, N);
        
    otherwise
        error('Invalid DEMO_TYPE. Choose 1-5.');
end

% =========================================================================
%                     HELPER FUNCTIONS
% =========================================================================

function [v_opt] = run_planner(algo_id, robot, obstacles)
    switch algo_id
        case 1, [v_opt, ~] = plan_VO(robot, obstacles);
        case 2, [v_opt, ~] = plan_RVO_new(robot, obstacles);
        case 3, [v_opt, ~] = plan_HRVO_new(robot, obstacles);
    end
end

function [robot, blocked_time] = update_status(robot, t, blocked_start, max_blocked, goal_thresh)
    blocked_time = blocked_start;
    dist_to_goal = norm(robot.goal - robot.pos);
    
    if dist_to_goal < goal_thresh
        robot.status = 'arrived';
        if isnan(robot.arrival_time), robot.arrival_time = t; end
        blocked_time = NaN;
    elseif norm(robot.vel) < 0.02
        if isnan(blocked_time), blocked_time = t; end
        robot.status = 'waiting';
    else
        robot.status = 'navigating';
        blocked_time = NaN;
    end
end

function finished = all_finished(robots)
    finished = true;
    for i = 1:length(robots)
        if strcmp(robots(i).status, 'navigating') || strcmp(robots(i).status, 'waiting')
            finished = false; return;
        end
    end
end

function [fig, ax, h_hud, colors] = setup_demo_vis(bounds, robots, obs, algo, scn)
    fig = figure('Name', sprintf('Demo: %s + %s', scn, algo), 'Color', 'black', ...
        'Position', [50 50 1000 800]);
    ax = axes('Position', [0.08 0.1 0.7 0.85], 'Color', 'k', 'XColor','w', 'YColor','w');
    axis equal; grid on; hold on; axis(bounds);
    title(sprintf('%s - %s Algorithm', strrep(scn, '_', ' '), algo), 'Color', 'w', 'FontSize', 14);
    
    N = length(robots);
    colors = lines(N);
    
    % Draw goals
    for i = 1:N
        plot(robots(i).goal(1), robots(i).goal(2), 'x', 'Color', colors(i,:), ...
            'MarkerSize', 14, 'LineWidth', 3);
    end
    
    % Draw static obstacles
    for k = 1:length(obs)
        if norm(obs(k).vel) == 0
            viscircles(obs(k).pos', obs(k).radius, 'Color', [0.6 0.1 0.1], 'LineWidth', 1.5);
        end
    end
    
    h_hud = annotation('textbox', [0.8 0.3 0.18 0.5], 'String', 'Init...', ...
        'Color', 'w', 'BackgroundColor', [0.1 0.1 0.1], 'EdgeColor', 'w', ...
        'FontName', 'Consolas', 'FontSize', 10, 'VerticalAlignment', 'top');
end

function h = draw_demo_scene(robots, obstacles, v_opts)
    h = [];
    N = length(robots);
    colors = lines(N);
    
    for i = 1:N
        r = robots(i);
        switch r.status
            case 'navigating', ec = colors(i,:);
            case 'arrived', ec = [0 1 0];
            case 'waiting', ec = [1 0.6 0];
            case 'crashed', ec = [1 0 0];
            otherwise, ec = colors(i,:);
        end
        
        hb = viscircles(r.pos', r.radius, 'Color', ec, 'LineWidth', 2.5);
        set(findobj(hb), 'HandleVisibility', 'off');
        h = [h; hb];
        
        ht = text(r.pos(1), r.pos(2)+r.radius+0.4, sprintf('R%d', i), ...
            'Color', 'w', 'HorizontalAlignment', 'center', 'FontSize', 10, 'FontWeight', 'bold');
        h = [h; ht];
        
        if norm(v_opts(:,i)) > 0.01
            hv = quiver(r.pos(1), r.pos(2), v_opts(1,i), v_opts(2,i), 0, ...
                'Color', ec, 'LineWidth', 2);
            h = [h; hv];
        end
    end
    
    for k = 1:length(obstacles)
        ho = viscircles(obstacles(k).pos', obstacles(k).radius, 'Color', 'r', 'LineWidth', 1.5);
        set(findobj(ho), 'HandleVisibility', 'off');
        h = [h; ho];
    end
end

function update_demo_hud(h_hud, robots, t, algo)
    str = sprintf('Algorithm: %s\nTime: %.1fs\n\n', algo, t);
    for i = 1:length(robots)
        r = robots(i);
        d = norm(r.goal - r.pos);
        switch r.status
            case 'navigating', s = 'NAV';
            case 'arrived', s = 'DONE';
            case 'waiting', s = 'WAIT';
            case 'crashed', s = 'CRASH';
            otherwise, s = '?';
        end
        str = [str, sprintf('R%d [%s] %.1fm\n', i, s, d)];
    end
    arrived = sum(strcmp({robots.status}, 'arrived'));
    str = [str, sprintf('\n--------------\nArrived: %d/%d', arrived, length(robots))];
    h_hud.String = str;
end
