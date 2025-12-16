% run_all_simulations.m
% Batch runner for all VO/RVO/HRVO simulations across all scenarios
% Generates videos for presentation/documentation
% ---------------------------------------------------------
clc; clear; close all;

fprintf('============================================\n');
fprintf('  BATCH SIMULATION RUNNER\n');
fprintf('  Generating all videos for all scenarios\n');
fprintf('============================================\n\n');

% Create output directory if needed
if ~exist('output', 'dir'), mkdir('output'); end

% Track results
results = {};
total_start = tic;

%% =========================================================
%  PART 1: MAIN SIMULATION (2-robot scenarios)
%  =========================================================
fprintf('\n>>> PART 1: Main Simulation Scenarios <<<\n');
fprintf('==========================================\n');

main_scenarios = {
    1, 'basic';
    2, 'u_trap';
    3, 'hallway';
    4, 'somewhat_busy';
    5, 'very_busy'
};

algorithms = {1, 'VO'; 2, 'RVO'; 3, 'HRVO'};

for s = 1:size(main_scenarios, 1)
    for a = 1:size(algorithms, 1)
        scenario_id = main_scenarios{s, 1};
        scenario_name = main_scenarios{s, 2};
        algo_id = algorithms{a, 1};
        algo_name = algorithms{a, 2};
        
        fprintf('\n[Main] Running: %s + %s\n', algo_name, scenario_name);
        
        try
            run_main_simulation(scenario_id, algo_id);
            results{end+1, 1} = sprintf('Main_%s_%s', algo_name, scenario_name);
            results{end, 2} = 'SUCCESS';
            fprintf('  >> Completed successfully\n');
        catch ME
            results{end+1, 1} = sprintf('Main_%s_%s', algo_name, scenario_name);
            results{end, 2} = sprintf('FAILED: %s', ME.message);
            fprintf('  >> FAILED: %s\n', ME.message);
        end
        
        close all;  % Clean up figures
    end
end

%% =========================================================
%  PART 2: MULTI-AGENT SIMULATION (N-robot scenarios)
%  =========================================================
fprintf('\n\n>>> PART 2: Multi-Agent Scenarios <<<\n');
fprintf('======================================\n');

multi_scenarios = {
    1, 'crossing_4';
    2, 'swarm_8';
    3, 'dense_crowd';
    4, 'wall_corridor'
    % 5, 'interactive' - skip interactive mode for batch
};

for s = 1:size(multi_scenarios, 1)
    for a = 1:size(algorithms, 1)
        scenario_id = multi_scenarios{s, 1};
        scenario_name = multi_scenarios{s, 2};
        algo_id = algorithms{a, 1};
        algo_name = algorithms{a, 2};
        
        fprintf('\n[Multi] Running: %s + %s\n', algo_name, scenario_name);
        
        try
            run_multi_agent_simulation(scenario_id, algo_id);
            results{end+1, 1} = sprintf('Multi_%s_%s', algo_name, scenario_name);
            results{end, 2} = 'SUCCESS';
            fprintf('  >> Completed successfully\n');
        catch ME
            results{end+1, 1} = sprintf('Multi_%s_%s', algo_name, scenario_name);
            results{end, 2} = sprintf('FAILED: %s', ME.message);
            fprintf('  >> FAILED: %s\n', ME.message);
        end
        
        close all;
    end
end

%% =========================================================
%  PART 3: MAZE DEMO (Global + Local planning)
%  =========================================================
fprintf('\n\n>>> PART 3: Maze Demo Scenarios <<<\n');
fprintf('====================================\n');

maze_types = {'simple', 'corridor', 'rooms'};
local_planners = {'VO', 'RVO', 'HRVO'};

for m = 1:length(maze_types)
    for p = 1:length(local_planners)
        maze_type = maze_types{m};
        planner = local_planners{p};
        
        fprintf('\n[Maze] Running: %s + %s\n', planner, maze_type);
        
        try
            run_maze_demo(maze_type, planner);
            results{end+1, 1} = sprintf('Maze_%s_%s', planner, maze_type);
            results{end, 2} = 'SUCCESS';
            fprintf('  >> Completed successfully\n');
        catch ME
            results{end+1, 1} = sprintf('Maze_%s_%s', planner, maze_type);
            results{end, 2} = sprintf('FAILED: %s', ME.message);
            fprintf('  >> FAILED: %s\n', ME.message);
        end
        
        close all;
    end
end

%% =========================================================
%  SUMMARY
%  =========================================================
total_time = toc(total_start);

fprintf('\n\n============================================\n');
fprintf('  BATCH SIMULATION COMPLETE\n');
fprintf('============================================\n');
fprintf('Total time: %.1f seconds (%.1f minutes)\n', total_time, total_time/60);
fprintf('\nResults Summary:\n');
fprintf('----------------\n');

success_count = 0;
fail_count = 0;
for i = 1:size(results, 1)
    if strcmp(results{i, 2}, 'SUCCESS')
        fprintf('  [OK]   %s\n', results{i, 1});
        success_count = success_count + 1;
    else
        fprintf('  [FAIL] %s: %s\n', results{i, 1}, results{i, 2});
        fail_count = fail_count + 1;
    end
end

fprintf('\n----------------\n');
fprintf('Success: %d/%d\n', success_count, size(results, 1));
fprintf('Failed:  %d/%d\n', fail_count, size(results, 1));
fprintf('\nVideos saved to: output/\n');
fprintf('============================================\n');


%% =========================================================
%  HELPER FUNCTIONS
%  =========================================================

function run_main_simulation(SCENARIO_ID, ALGORITHM)
    % Adapted from main_simulation.m for batch execution
    
    addpath('classes'); 
    addpath('algorithms'); 
    addpath('utils');
    addpath('scenarios/VOs'); 
    
    dt = 0.1; 
    T_max = 50.0; 
    MAX_BLOCKED_DURATION = 5.0; 
    blocked_start_sim_time = NaN;
    
    % Load scenario
    robot2 = [];
    switch SCENARIO_ID
        case 1, [robot, robot2, obstacles, map_bounds, scn_name] = basic();
        case 2, [robot, robot2, obstacles, map_bounds, scn_name] = u_trap();
        case 3, [robot, robot2, obstacles, map_bounds, scn_name] = setup_hallway();
        case 4, [robot, robot2, obstacles, map_bounds, scn_name] = somewhat_busy();
        case 5, [robot, robot2, obstacles, map_bounds, scn_name] = very_busy();
    end
    
    algo_name = get_algo_name(ALGORITHM);
    VIDEO_NAME = sprintf('output/%s_%s.mp4', algo_name, scn_name);
    
    % Setup visualization
    [fig, ax, h_hud, lgd] = setup_main_vis(map_bounds, robot, robot2, obstacles);
    
    v = VideoWriter(VIDEO_NAME, 'MPEG-4');
    v.FrameRate = 10; open(v);
    
    h_dynamic = []; 
    collision_detected = false;
    
    for t = 0:dt:T_max
        if ~isempty(h_dynamic), delete(h_dynamic); end
        h_dynamic = [];
        
        % Update dynamic obstacles
        for k = 1:length(obstacles)
            if norm(obstacles(k).vel) > 0
                obstacles(k).pos = obstacles(k).pos + obstacles(k).vel * dt;
            end
        end
        
        % Planning
        obs_for_r1 = obstacles;
        if ~isempty(robot2)
            obs_for_r1 = [obs_for_r1, Obstacle(robot2.pos, robot2.radius, robot2.vel)];
        end
        [v_opt1, cones1] = run_planner(ALGORITHM, robot, obs_for_r1);
        
        v_opt2 = [0;0]; cones2 = [];
        if ~isempty(robot2)
            obs_for_r2 = [obstacles, Obstacle(robot.pos, robot.radius, robot.vel)];
            [v_opt2, cones2] = run_planner(ALGORITHM, robot2, obs_for_r2);
        end
        
        % Execution
        robot = robot.move(v_opt1, dt);
        if ~isempty(robot2)
            robot2 = robot2.move(v_opt2, dt);
        end
        
        % Collision check
        if check_collision_main(robot, obstacles, robot2)
            collision_detected = true;
        end
        
        % Visualization
        h_dynamic = draw_main_scene(robot, robot2, obstacles, cones1, v_opt1, v_opt2);
        
        dist_to_goal = norm(robot.goal - robot.pos);
        if collision_detected
            status = 'CRASHED!'; hud_color = 'r';
        elseif dist_to_goal < 0.5
            status = 'ARRIVED'; hud_color = 'g';
        elseif norm(v_opt1) < 0.01
            status = 'WAITING'; hud_color = [0.9 0.6 0];
        else
            status = 'NAVIGATING'; hud_color = 'b';
        end
        
        h_hud.String = sprintf('Alg: %s | T: %.2fs\nGoal Dist: %.2fm\nStatus: %s', ...
            algo_name, t, dist_to_goal, status);
        h_hud.BackgroundColor = hud_color;
        
        drawnow limitrate;
        writeVideo(v, getframe(gcf));
        
        % Termination
        if collision_detected, break; end
        if dist_to_goal < 0.5, break; end
        if strcmp(status, 'WAITING')
            if isnan(blocked_start_sim_time), blocked_start_sim_time = t; end
            if (t - blocked_start_sim_time) > MAX_BLOCKED_DURATION, break; end
        else
            blocked_start_sim_time = NaN;
        end
    end
    
    close(v);
end

function run_multi_agent_simulation(SCENARIO_ID, ALGORITHM)
    % Adapted from multi_agent_simulation.m for batch execution
    
    addpath('classes'); 
    addpath('algorithms'); 
    addpath('utils');
    addpath('scenarios/multi_agent'); 
    
    dt = 0.1; 
    T_max = 60.0; 
    MAX_BLOCKED_DURATION = 5.0; 
    GOAL_THRESHOLD = 0.5;
    
    % Load scenario
    switch SCENARIO_ID
        case 1, [robots, obstacles, map_bounds, scn_name] = crossing_4();
        case 2, [robots, obstacles, map_bounds, scn_name] = swarm_8();
        case 3, [robots, obstacles, map_bounds, scn_name] = dense_crowd();
        case 4, [robots, obstacles, map_bounds, scn_name] = wall_corridor();
    end
    
    N = length(robots);
    algo_name = get_algo_name(ALGORITHM);
    VIDEO_NAME = sprintf('output/MultiAgent_%s_%s.mp4', algo_name, scn_name);
    
    blocked_start_times = NaN(1, N);
    
    % Setup visualization
    [fig, ax, h_hud, robot_colors] = setup_multi_vis(map_bounds, robots, obstacles);
    
    v = VideoWriter(VIDEO_NAME, 'MPEG-4');
    v.FrameRate = 10; open(v);
    
    h_dynamic = []; 
    
    for t = 0:dt:T_max
        if ~isempty(h_dynamic), delete(h_dynamic); end
        h_dynamic = [];
        
        % Update dynamic obstacles
        for k = 1:length(obstacles)
            if norm(obstacles(k).vel) > 0
                obstacles(k).pos = obstacles(k).pos + obstacles(k).vel * dt;
            end
        end
        
        % Planning for all robots
        all_v_opts = zeros(2, N);
        all_cones = cell(1, N);
        
        for i = 1:N
            if strcmp(robots(i).status, 'arrived') || strcmp(robots(i).status, 'crashed')
                all_v_opts(:, i) = [0; 0];
                continue;
            end
            
            obs_for_robot_i = obstacles;
            for j = 1:N
                if i ~= j
                    obs_for_robot_i = [obs_for_robot_i, ...
                        Obstacle(robots(j).pos, robots(j).radius, robots(j).vel)];
                end
            end
            
            [v_opt, cones] = run_planner(ALGORITHM, robots(i), obs_for_robot_i);
            all_v_opts(:, i) = v_opt;
            all_cones{i} = cones;
        end
        
        % Execution
        for i = 1:N
            if strcmp(robots(i).status, 'arrived') || strcmp(robots(i).status, 'crashed')
                continue;
            end
            robots(i) = robots(i).move(all_v_opts(:, i), dt);
            [robots(i), blocked_start_times(i)] = update_robot_status(...
                robots(i), t, blocked_start_times(i), MAX_BLOCKED_DURATION, GOAL_THRESHOLD);
        end
        
        % Collision detection
        collision_pairs = check_all_collisions(robots, obstacles);
        for p = 1:size(collision_pairs, 1)
            idx = collision_pairs(p, 1);
            if idx <= N
                robots(idx).status = 'crashed';
            end
        end
        
        % Visualization
        h_dynamic = draw_multi_scene(robots, obstacles, all_cones, all_v_opts);
        update_multi_hud(h_hud, robots, t, algo_name);
        
        drawnow limitrate;
        writeVideo(v, getframe(gcf));
        
        if all_robots_finished(robots), break; end
    end
    
    close(v);
end

function run_maze_demo(maze_type, local_planner)
    % Adapted from maze_demo.m for batch execution
    
    addpath('algorithms', 'classes', 'utils', 'global_planner');
    
    grid_size = [30, 30];
    cell_size = 0.5;
    dt = 0.1;
    max_time = 80;
    robot_radius = 0.3;
    robot_vmax = 1.0;
    
    world_width = grid_size(2) * cell_size;
    world_height = grid_size(1) * cell_size;
    
    % Generate maze
    grid_map = zeros(grid_size);
    
    switch maze_type
        case 'simple'
            grid_map(1,:) = 1; grid_map(end,:) = 1;
            grid_map(:,1) = 1; grid_map(:,end) = 1;
            mid_r = round(grid_size(1)/2);
            mid_c = round(grid_size(2)/2);
            grid_map(6:mid_r+4, mid_c) = 1;
            grid_map(mid_r, mid_c:mid_c+6) = 1;
        case 'corridor'
            grid_map(1,:) = 1; grid_map(end,:) = 1;
            grid_map(:,1) = 1; grid_map(:,end) = 1;
            grid_map(8, 1:18) = 1;
            grid_map(15, 12:end) = 1;
            grid_map(22, 1:18) = 1;
        case 'rooms'
            grid_map(1,:) = 1; grid_map(end,:) = 1;
            grid_map(:,1) = 1; grid_map(:,end) = 1;
            mid = round(grid_size(1)/2);
            grid_map(mid, 1:mid-3) = 1;
            grid_map(mid, mid+3:end) = 1;
            grid_map(1:mid-3, mid) = 1;
            grid_map(mid+3:end, mid) = 1;
    end
    
    start_cell = [grid_size(1)-3, 4];
    goal_cell = [4, grid_size(2)-4];
    
    wall_obstacles = grid_to_wall_obs(grid_map, grid_size, cell_size);
    start_pos = cell_to_world_local(start_cell, grid_size, cell_size);
    goal_pos = cell_to_world_local(goal_cell, grid_size, cell_size);
    
    % A* path
    [path_cells, path_found] = astar_grid(grid_map, start_cell, goal_cell);
    if ~path_found, error('A* failed'); end
    waypoints = path_to_world(path_cells, grid_size, cell_size);
    
    % Robot
    robot = Robot(1, start_pos, 0, robot_radius, robot_vmax, goal_pos);
    robot.color = [0, 0.7, 0];
    
    % Other robots
    other_robots = [];
    other_robot_goals = [];
    r2 = Robot(2, [world_width*0.8, world_height*0.15], 0, 0.25, 0.6, [world_width*0.15, world_height*0.15]);
    r2.color = [0.8, 0.2, 0.2];
    other_robots = [other_robots, r2];
    other_robot_goals = [other_robot_goals; r2.goal];
    
    r3 = Robot(3, [world_width*0.85, world_height*0.85], 0, 0.25, 0.5, [world_width*0.85, world_height*0.15]);
    r3.color = [0.2, 0.2, 0.8];
    other_robots = [other_robots, r3];
    other_robot_goals = [other_robot_goals; r3.goal];
    
    static_obstacles = [];
    
    % Video setup
    video_file = sprintf('output/maze_demo_%s_%s.mp4', maze_type, local_planner);
    vid = VideoWriter(video_file, 'MPEG-4');
    vid.FrameRate = 15;
    open(vid);
    
    % Figure setup
    fig = figure('Name', 'Maze Demo', 'Position', [50, 50, 1200, 500], 'Color', 'k');
    
    current_wp = 1;
    t = 0;
    step = 0;
    trajectory = robot.pos;
    local_radius = 4;
    
    while t < max_time
        step = step + 1;
        
        % Build obstacles
        obstacles_for_robot = [wall_obstacles, static_obstacles];
        for i = 1:length(other_robots)
            obs = Obstacle(other_robots(i).pos, other_robots(i).radius, other_robots(i).vel(:)');
            obstacles_for_robot = [obstacles_for_robot, obs];
        end
        
        % Plan
        [v_cmd, current_wp, goal_reached, robot_cones] = ...
            waypoint_follower(robot, waypoints, current_wp, obstacles_for_robot, local_planner, 1);
        
        if goal_reached, break; end
        
        % Update main robot
        robot.vel = v_cmd(:)';
        robot.pos = robot.pos + robot.vel * dt;
        robot.theta = atan2(robot.vel(2), robot.vel(1));
        trajectory = [trajectory; robot.pos];
        
        % Update other robots
        for i = 1:length(other_robots)
            obs_for_i = [wall_obstacles, static_obstacles];
            obs_for_i = [obs_for_i, Obstacle(robot.pos, robot.radius, robot.vel)];
            for j = 1:length(other_robots)
                if i ~= j
                    obs_for_i = [obs_for_i, Obstacle(other_robots(j).pos, other_robots(j).radius, other_robots(j).vel(:)')];
                end
            end
            
            temp_robot = Robot(other_robots(i).id, other_robots(i).pos, other_robots(i).theta, ...
                other_robots(i).radius, other_robots(i).v_max, other_robot_goals(i,:));
            temp_robot.vel = other_robots(i).vel;
            
            switch upper(local_planner)
                case 'VO',   v_i = plan_VO(temp_robot, obs_for_i);
                case 'RVO',  v_i = plan_RVO_new(temp_robot, obs_for_i);
                case 'HRVO', v_i = plan_HRVO_new(temp_robot, obs_for_i);
            end
            
            other_robots(i).vel = v_i(:)';
            other_robots(i).pos = other_robots(i).pos + other_robots(i).vel * dt;
            
            if norm(other_robots(i).pos - other_robot_goals(i,:)) < 0.5
                other_robots(i).vel = [0, 0];
            end
        end
        
        % Visualization (every 2 steps)
        if mod(step, 2) == 0
            clf;
            
            % Global view
            subplot(1, 2, 1);
            set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w');
            hold on;
            imagesc([cell_size/2, world_width - cell_size/2], ...
                [cell_size/2, world_height - cell_size/2], flipud(grid_map));
            colormap(gca, [0 0 0; 0.4 0.4 0.4]);
            plot(waypoints(:,1), waypoints(:,2), 'b-', 'LineWidth', 2);
            plot(trajectory(:,1), trajectory(:,2), '-', 'Color', [0, 0.7, 0], 'LineWidth', 1.5);
            plot(robot.pos(1), robot.pos(2), 'o', 'MarkerSize', 10, 'MarkerFaceColor', robot.color);
            for i = 1:length(other_robots)
                plot(other_robots(i).pos(1), other_robots(i).pos(2), 's', ...
                    'MarkerSize', 8, 'MarkerFaceColor', other_robots(i).color);
            end
            axis equal; xlim([0, world_width]); ylim([0, world_height]);
            title('Global View', 'Color', 'w');
            hold off;
            
            % Local view
            subplot(1, 2, 2);
            set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w');
            hold on;
            xlim([robot.pos(1)-local_radius, robot.pos(1)+local_radius]);
            ylim([robot.pos(2)-local_radius, robot.pos(2)+local_radius]);
            
            % Draw cones
            if ~isempty(robot_cones)
                for c = 1:min(size(robot_cones, 1), 10)
                    if size(robot_cones, 2) >= 4
                        apex = robot.pos(:) + robot_cones(c, 3:4)';
                    else
                        apex = robot.pos(:);
                    end
                    plot_cone(apex, robot_cones(c, 1), robot_cones(c, 2), local_radius*0.8, [0.8, 0.2, 0.8]);
                end
            end
            
            % Draw walls
            for i = 1:length(wall_obstacles)
                obs = wall_obstacles(i);
                if norm(obs.pos - robot.pos) < local_radius + obs.radius
                    th = linspace(0, 2*pi, 16);
                    fill(obs.pos(1) + obs.radius*cos(th), obs.pos(2) + obs.radius*sin(th), ...
                        [0.4, 0.4, 0.4], 'FaceAlpha', 0.6, 'EdgeColor', 'none');
                end
            end
            
            % Draw other robots
            for i = 1:length(other_robots)
                r = other_robots(i);
                if norm(r.pos - robot.pos) < local_radius + 1
                    th = linspace(0, 2*pi, 20);
                    fill(r.pos(1) + r.radius*cos(th), r.pos(2) + r.radius*sin(th), ...
                        r.color, 'FaceAlpha', 0.8, 'EdgeColor', 'k');
                end
            end
            
            % Draw main robot
            th = linspace(0, 2*pi, 30);
            fill(robot.pos(1) + robot.radius*cos(th), robot.pos(2) + robot.radius*sin(th), ...
                robot.color, 'FaceAlpha', 0.9, 'EdgeColor', 'k', 'LineWidth', 2);
            
            if norm(robot.vel) > 0.01
                quiver(robot.pos(1), robot.pos(2), robot.vel(1)*1.5, robot.vel(2)*1.5, 0, ...
                    'Color', 'b', 'LineWidth', 2);
            end
            
            title(sprintf('Local View (%s) | t=%.1fs', local_planner, t), 'Color', 'w');
            axis equal;
            hold off;
            
            drawnow;
            writeVideo(vid, getframe(fig));
        end
        
        t = t + dt;
    end
    
    close(vid);
end

%% --- Shared Helper Functions ---

function [v_opt, cones] = run_planner(algo_id, robot, obstacles)
    switch algo_id
        case 1, [v_opt, cones] = plan_VO(robot, obstacles);
        case 2, [v_opt, cones] = plan_RVO_new(robot, obstacles);
        case 3, [v_opt, cones] = plan_HRVO_new(robot, obstacles);
    end
end

function name = get_algo_name(id)
    if id==1, name='VO'; elseif id==2, name='RVO'; elseif id==3, name='HRVO'; else, name='UNK'; end
end

function is_colliding = check_collision_main(r1, obstacles, r2)
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

function [robot, blocked_time] = update_robot_status(robot, t, blocked_start, max_blocked, goal_thresh)
    blocked_time = blocked_start;
    dist_to_goal = norm(robot.goal - robot.pos);
    
    if dist_to_goal < goal_thresh
        robot.status = 'arrived';
        if isnan(robot.arrival_time)
            robot.arrival_time = t;
        end
        blocked_time = NaN;
    elseif norm(robot.vel) < 0.02
        if isnan(blocked_time)
            blocked_time = t;
        elseif (t - blocked_time) > max_blocked
            robot.status = 'waiting';
        else
            robot.status = 'waiting';
        end
    else
        robot.status = 'navigating';
        blocked_time = NaN;
    end
end

function finished = all_robots_finished(robots)
    finished = true;
    for i = 1:length(robots)
        if strcmp(robots(i).status, 'navigating') || strcmp(robots(i).status, 'waiting')
            finished = false;
            return;
        end
    end
end

function collision_pairs = check_all_collisions(robots, obstacles)
    collision_pairs = [];
    N = length(robots);
    for i = 1:N
        for j = i+1:N
            if norm(robots(i).pos - robots(j).pos) <= (robots(i).radius + robots(j).radius)
                collision_pairs = [collision_pairs; i, j];
            end
        end
        for k = 1:length(obstacles)
            if norm(robots(i).pos - obstacles(k).pos) <= (robots(i).radius + obstacles(k).radius)
                collision_pairs = [collision_pairs; i, -k];
            end
        end
    end
end

%% --- Visualization Helpers ---

function [fig, ax, h_hud, lgd] = setup_main_vis(bounds, r1, r2, obs)
    w = bounds(2)-bounds(1); h = bounds(4)-bounds(3);
    fig = figure('Name', 'VO Simulation', 'Color', 'black', 'Position', [50 50 800*(w/h) 800]);
    ax = axes('Position', [0.05 0.05 0.9 0.9], 'Color', 'k', 'XColor','w', 'YColor','w');
    axis equal; grid on; hold on; axis(bounds);
    
    plot(r1.goal(1), r1.goal(2), 'gx', 'MarkerSize', 10, 'LineWidth', 2, 'HandleVisibility', 'off'); 
    if ~isempty(r2), plot(r2.goal(1), r2.goal(2), 'rx', 'MarkerSize', 10, 'LineWidth', 2, 'HandleVisibility', 'off'); end
    
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
    
    h_hud = text(bounds(1)+w*0.05, bounds(4)-h*0.05, 'Init...', ...
        'BackgroundColor', 'b', 'Color', 'w', 'EdgeColor', 'k', 'FontName', 'Consolas');
end

function h_handles = draw_main_scene(r1, r2, obstacles, cones1, v1, v2)
    h_handles = [];
    
    if ~isempty(cones1)
        for i = 1:size(cones1, 1)
            if size(cones1, 2) >= 4
                apex = r1.pos + cones1(i, 3:4)';
            else
                apex = r1.pos;
            end
            p = plot_cone(apex, cones1(i,1), cones1(i,2), 5.0, 'm');
            set(p, 'HandleVisibility', 'off');
            h_handles = [h_handles; p];
        end
    end
    
    [h_b, h_h, h_v] = draw_robot_main(r1, v1);
    h_handles = [h_handles; h_b; h_h; h_v];
    
    if ~isempty(r2)
        [h_b2, h_h2, h_v2] = draw_robot_main(r2, v2);
        h_handles = [h_handles; h_b2; h_h2; h_v2];
    end
    
    for k = 1:length(obstacles)
        h_o = viscircles(obstacles(k).pos', obstacles(k).radius, 'Color', 'r');
        set(findobj(h_o), 'HandleVisibility', 'off');
        h_s = viscircles(obstacles(k).pos', obstacles(k).radius + r1.radius, 'Color', 'r', 'LineStyle', '--', 'LineWidth', 0.5);
        set(findobj(h_s), 'HandleVisibility', 'off');
        h_handles = [h_handles; h_o; h_s];
    end
end

function [h_body, h_head, h_vel] = draw_robot_main(r, v_cmd)
    h_body = viscircles(r.pos', r.radius, 'Color', r.color);
    set(findobj(h_body), 'HandleVisibility', 'off');
    h_head = quiver(r.pos(1), r.pos(2), cos(r.theta), sin(r.theta), 0.5, 'Color', 'g', 'LineWidth', 1, 'HandleVisibility', 'off');
    h_vel = quiver(r.pos(1), r.pos(2), v_cmd(1), v_cmd(2), 0, 'Color', r.color, 'LineStyle', '--', 'LineWidth', 2, 'HandleVisibility', 'off');
end

function [fig, ax, h_hud, robot_colors] = setup_multi_vis(bounds, robots, obs)
    w = bounds(2)-bounds(1); h = bounds(4)-bounds(3);
    fig = figure('Name', 'Multi-Agent VO Simulation', 'Color', 'black', ...
        'Position', [50 50 min(1200, 800*(w/h)) 800]);
    ax = axes('Position', [0.05 0.12 0.75 0.83], 'Color', 'k', 'XColor','w', 'YColor','w');
    axis equal; grid on; hold on; axis(bounds);
    title('Multi-Agent Velocity Obstacle Simulation', 'Color', 'w', 'FontSize', 14);
    
    N = length(robots);
    robot_colors = lines(N);
    
    for i = 1:N
        plot(robots(i).goal(1), robots(i).goal(2), 'x', 'Color', robot_colors(i,:), ...
            'MarkerSize', 12, 'LineWidth', 2, 'HandleVisibility', 'off');
    end
    
    for k = 1:length(obs)
        if norm(obs(k).vel) == 0
            viscircles(obs(k).pos', obs(k).radius, 'Color', [0.5 0 0], 'LineWidth', 1);
        end
    end
    
    h_hud = annotation('textbox', [0.82 0.3 0.17 0.55], ...
        'String', 'Initializing...', ...
        'Color', 'w', 'BackgroundColor', [0.1 0.1 0.1], ...
        'EdgeColor', 'w', 'FontName', 'Consolas', 'FontSize', 9, ...
        'VerticalAlignment', 'top', 'Interpreter', 'none');
end

function h_handles = draw_multi_scene(robots, obstacles, all_cones, all_v_opts)
    h_handles = [];
    N = length(robots);
    robot_colors = lines(N);
    cone_colors = robot_colors * 0.8;
    
    MAX_CONES_PER_ROBOT = 8;
    SHOW_CONES_FOR_N_ROBOTS = min(N, 8);
    
    for i = 1:SHOW_CONES_FOR_N_ROBOTS
        if ~isempty(all_cones{i}) && ~strcmp(robots(i).status, 'arrived') && ~strcmp(robots(i).status, 'crashed')
            num_cones = min(size(all_cones{i}, 1), MAX_CONES_PER_ROBOT);
            for c = 1:num_cones
                if size(all_cones{i}, 2) >= 4
                    apex = robots(i).pos + all_cones{i}(c, 3:4)';
                else
                    apex = robots(i).pos;
                end
                p = plot_cone(apex, all_cones{i}(c,1), all_cones{i}(c,2), 4.0, cone_colors(i,:));
                set(p, 'HandleVisibility', 'off');
                h_handles = [h_handles; p];
            end
        end
    end
    
    for i = 1:N
        r = robots(i);
        v_opt = all_v_opts(:, i);
        
        switch r.status
            case 'navigating', edge_color = robot_colors(i,:);
            case 'arrived',    edge_color = [0 1 0];
            case 'waiting',    edge_color = [1 0.6 0];
            case 'crashed',    edge_color = [1 0 0];
            otherwise,         edge_color = robot_colors(i,:);
        end
        
        h_body = viscircles(r.pos', r.radius, 'Color', edge_color, 'LineWidth', 2);
        set(findobj(h_body), 'HandleVisibility', 'off');
        h_handles = [h_handles; h_body];
        
        h_txt = text(r.pos(1), r.pos(2)+r.radius+0.3, sprintf('R%d', i), ...
            'Color', 'w', 'HorizontalAlignment', 'center', 'FontSize', 8, 'FontWeight', 'bold');
        h_handles = [h_handles; h_txt];
        
        h_head = quiver(r.pos(1), r.pos(2), 0.4*cos(r.theta), 0.4*sin(r.theta), 0, ...
            'Color', 'g', 'LineWidth', 1.5, 'HandleVisibility', 'off');
        h_handles = [h_handles; h_head];
        
        if norm(v_opt) > 0.01
            h_vel = quiver(r.pos(1), r.pos(2), v_opt(1), v_opt(2), 0, ...
                'Color', edge_color, 'LineStyle', '--', 'LineWidth', 1.5, 'HandleVisibility', 'off');
            h_handles = [h_handles; h_vel];
        end
    end
    
    for k = 1:length(obstacles)
        obs = obstacles(k);
        h_o = viscircles(obs.pos', obs.radius, 'Color', 'r', 'LineWidth', 1.5);
        set(findobj(h_o), 'HandleVisibility', 'off');
        h_handles = [h_handles; h_o];
        
        if norm(obs.vel) > 0.01
            h_ov = quiver(obs.pos(1), obs.pos(2), obs.vel(1), obs.vel(2), 0, ...
                'Color', 'r', 'LineWidth', 1, 'HandleVisibility', 'off');
            h_handles = [h_handles; h_ov];
        end
    end
end

function update_multi_hud(h_hud, robots, t, algo_name)
    hud_str = sprintf('Algorithm: %s\nTime: %.2fs\n', algo_name, t);
    hud_str = [hud_str, '------------------------'];
    
    for i = 1:length(robots)
        r = robots(i);
        dist = norm(r.goal - r.pos);
        
        switch r.status
            case 'navigating', icon = '[NAV]';
            case 'arrived',    icon = '[OK] ';
            case 'waiting',    icon = '[WAIT]';
            case 'crashed',    icon = '[X]  ';
            otherwise,         icon = '[?]  ';
        end
        
        hud_str = [hud_str, sprintf('\nR%d %s D:%.1f', i, icon, dist)];
    end
    
    arrived = sum(strcmp({robots.status}, 'arrived'));
    hud_str = [hud_str, sprintf('\n------------------------\nDone: %d/%d', arrived, length(robots))];
    
    h_hud.String = hud_str;
end

%% --- Maze Helpers ---

function world_pos = cell_to_world_local(cell, grid_size, cell_size)
    rows = grid_size(1);
    x = (cell(2) - 0.5) * cell_size;
    y = (rows - cell(1) + 0.5) * cell_size;
    world_pos = [x, y];
end

function obstacles = grid_to_wall_obs(grid_map, grid_size, cell_size)
    [rows, cols] = size(grid_map);
    obstacles = [];
    radius = cell_size * 0.5;
    
    for r = 2:rows-1
        for c = 2:cols-1
            if grid_map(r, c) == 1
                neighbors = [grid_map(r-1,c), grid_map(r+1,c), grid_map(r,c-1), grid_map(r,c+1)];
                if any(neighbors == 0)
                    pos = cell_to_world_local([r, c], grid_size, cell_size);
                    obstacles = [obstacles, Obstacle(pos, radius, [0, 0])];
                end
            end
        end
    end
end
