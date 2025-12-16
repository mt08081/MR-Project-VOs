% benchmark_algorithms.m
% MR-Project-VOs: Algorithm Comparison Benchmark
% Runs all algorithms on all multi-agent scenarios and reports statistics
% Perfect for evaluation/presentation comparisons!
% ---------------------------------------------------------
clc; clear; close all;

% 1. SETUP PATHS
addpath('classes'); 
addpath('algorithms'); 
addpath('utils');
addpath('scenarios/multi_agent'); 

% ---------------------------------------------------------
% 2. BENCHMARK CONFIGURATION
% ---------------------------------------------------------
dt = 0.1;
T_max = 45.0;
GOAL_THRESHOLD = 0.5;
MAX_BLOCKED_DURATION = 5.0;

% Algorithms to test
ALGORITHMS = {'VO', 'RVO', 'HRVO'};
ALGO_IDS = [1, 2, 3];

% Scenarios to test (excluding interactive)
SCENARIOS = {'crossing_4', 'swarm_8', 'dense_crowd', 'wall_corridor'};
SCENARIO_FUNCS = {@crossing_4, @swarm_8, @dense_crowd, @wall_corridor};

% Results storage
results = struct();

fprintf('╔════════════════════════════════════════════════════════════════╗\n');
fprintf('║       MULTI-AGENT VELOCITY OBSTACLE BENCHMARK                 ║\n');
fprintf('╠════════════════════════════════════════════════════════════════╣\n');
fprintf('║  Comparing: VO, RVO, HRVO across %d scenarios                 ║\n', length(SCENARIOS));
fprintf('╚════════════════════════════════════════════════════════════════╝\n\n');

% ---------------------------------------------------------
% 3. RUN BENCHMARKS
% ---------------------------------------------------------
for s = 1:length(SCENARIOS)
    scn_name = SCENARIOS{s};
    scn_func = SCENARIO_FUNCS{s};
    
    fprintf('\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
    fprintf('  SCENARIO: %s\n', upper(scn_name));
    fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
    
    for a = 1:length(ALGORITHMS)
        algo_name = ALGORITHMS{a};
        algo_id = ALGO_IDS(a);
        
        % Load fresh scenario
        [robots_orig, obstacles_orig, map_bounds, ~] = scn_func();
        
        % Deep copy for this run
        robots = copy_robots(robots_orig);
        obstacles = copy_obstacles(obstacles_orig);
        
        N = length(robots);
        blocked_start_times = NaN(1, N);
        
        % Run simulation (no visualization for speed)
        collision_count = 0;
        
        for t = 0:dt:T_max
            % Update dynamic obstacles
            for k = 1:length(obstacles)
                if norm(obstacles(k).vel) > 0
                    obstacles(k).pos = obstacles(k).pos + obstacles(k).vel * dt;
                end
            end
            
            % PHASE 1: Planning for all robots (parallel - same world state)
            all_v_opts = zeros(2, N);
            for i = 1:N
                if strcmp(robots(i).status, 'arrived') || strcmp(robots(i).status, 'crashed')
                    continue;
                end
                
                % Perception: obstacles + other robots (all at time t)
                obs_for_robot_i = obstacles;
                for j = 1:N
                    if i ~= j
                        obs_for_robot_i = [obs_for_robot_i, ...
                            Obstacle(robots(j).pos, robots(j).radius, robots(j).vel)];
                    end
                end
                
                % Planning
                all_v_opts(:, i) = run_planner(algo_id, robots(i), obs_for_robot_i);
            end
            
            % PHASE 2: Execution for all robots (parallel - simultaneous)
            for i = 1:N
                if strcmp(robots(i).status, 'arrived') || strcmp(robots(i).status, 'crashed')
                    continue;
                end
                
                % Action
                robots(i) = robots(i).move(all_v_opts(:, i), dt);
                
                % Update status
                [robots(i), blocked_start_times(i)] = update_status(...
                    robots(i), t, blocked_start_times(i), MAX_BLOCKED_DURATION, GOAL_THRESHOLD);
            end
            
            % Check collisions
            collision_count = collision_count + count_collisions(robots, obstacles);
            
            % Termination check
            if all_finished(robots)
                break;
            end
        end
        
        % Collect statistics
        arrived = sum(strcmp({robots.status}, 'arrived'));
        crashed = sum(strcmp({robots.status}, 'crashed'));
        waiting = sum(strcmp({robots.status}, 'waiting'));
        
        arrival_times = [robots.arrival_time];
        valid_times = arrival_times(~isnan(arrival_times));
        avg_time = mean(valid_times);
        max_time = max(valid_times);
        if isempty(valid_times), avg_time = NaN; max_time = NaN; end
        
        success_rate = 100 * arrived / N;
        
        % Store results
        results.(scn_name).(algo_name).arrived = arrived;
        results.(scn_name).(algo_name).crashed = crashed;
        results.(scn_name).(algo_name).waiting = waiting;
        results.(scn_name).(algo_name).avg_time = avg_time;
        results.(scn_name).(algo_name).max_time = max_time;
        results.(scn_name).(algo_name).success_rate = success_rate;
        results.(scn_name).(algo_name).collisions = collision_count;
        
        % Print result
        fprintf('  %5s: %d/%d arrived (%.0f%%) | Avg: %.2fs | Max: %.2fs | Collisions: %d\n', ...
            algo_name, arrived, N, success_rate, avg_time, max_time, collision_count);
    end
end

% ---------------------------------------------------------
% 4. PRINT SUMMARY TABLE
% ---------------------------------------------------------
fprintf('\n\n');
fprintf('╔══════════════════════════════════════════════════════════════════════════════╗\n');
fprintf('║                           BENCHMARK SUMMARY                                  ║\n');
fprintf('╠════════════════════╦════════════════╦════════════════╦════════════════╦═════╣\n');
fprintf('║     Scenario       ║       VO       ║      RVO       ║     HRVO       ║Best ║\n');
fprintf('╠════════════════════╬════════════════╬════════════════╬════════════════╬═════╣\n');

for s = 1:length(SCENARIOS)
    scn_name = SCENARIOS{s};
    
    rates = [results.(scn_name).VO.success_rate, ...
             results.(scn_name).RVO.success_rate, ...
             results.(scn_name).HRVO.success_rate];
    times = [results.(scn_name).VO.avg_time, ...
             results.(scn_name).RVO.avg_time, ...
             results.(scn_name).HRVO.avg_time];
    
    [~, best_idx] = min(times);
    best_names = {'VO', 'RVO', 'HRVO'};
    
    fprintf('║ %-18s ║ %5.0f%% / %.1fs  ║ %5.0f%% / %.1fs  ║ %5.0f%% / %.1fs  ║ %-3s ║\n', ...
        scn_name, ...
        rates(1), times(1), ...
        rates(2), times(2), ...
        rates(3), times(3), ...
        best_names{best_idx});
end

fprintf('╚════════════════════╩════════════════╩════════════════╩════════════════╩═════╝\n');

fprintf('\n>> Benchmark Complete!\n');

% ---------------------------------------------------------
% HELPER FUNCTIONS
% ---------------------------------------------------------

function [v_opt, cones] = run_planner(algo_id, robot, obstacles)
    switch algo_id
        case 1, [v_opt, cones] = plan_VO(robot, obstacles);
        case 2, [v_opt, cones] = plan_RVO_new(robot, obstacles);
        case 3, [v_opt, cones] = plan_HRVO_new(robot, obstacles);
        otherwise, error('Unknown Algorithm ID');
    end
end

function [robot, blocked_time] = update_status(robot, t, blocked_start, max_blocked, goal_thresh)
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

function finished = all_finished(robots)
    finished = true;
    for i = 1:length(robots)
        if strcmp(robots(i).status, 'navigating') || strcmp(robots(i).status, 'waiting')
            finished = false;
            return;
        end
    end
end

function count = count_collisions(robots, obstacles)
    count = 0;
    N = length(robots);
    
    for i = 1:N
        for j = i+1:N
            if norm(robots(i).pos - robots(j).pos) <= (robots(i).radius + robots(j).radius)
                count = count + 1;
            end
        end
        for k = 1:length(obstacles)
            if norm(robots(i).pos - obstacles(k).pos) <= (robots(i).radius + obstacles(k).radius)
                count = count + 1;
            end
        end
    end
end

function robots_copy = copy_robots(robots_orig)
    N = length(robots_orig);
    robots_copy = Robot.empty(0, N);
    for i = 1:N
        r = robots_orig(i);
        robots_copy(i) = Robot(r.id, r.pos, r.theta, r.radius, r.v_max, r.goal);
        robots_copy(i).color = r.color;
        robots_copy(i).vel = r.vel;
        robots_copy(i).status = r.status;
        robots_copy(i).arrival_time = r.arrival_time;
    end
end

function obstacles_copy = copy_obstacles(obstacles_orig)
    M = length(obstacles_orig);
    obstacles_copy = [];
    for k = 1:M
        o = obstacles_orig(k);
        obstacles_copy = [obstacles_copy, Obstacle(o.pos, o.radius, o.vel)];
    end
end
