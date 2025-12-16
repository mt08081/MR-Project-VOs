function [path, path_found] = astar_grid(grid_map, start_cell, goal_cell)
% ASTAR_GRID A* pathfinding on a 2D grid
%
% Implementation based on:
%   Hart, P. E., Nilsson, N. J., & Raphael, B. (1968). A Formal Basis for 
%   the Heuristic Determination of Minimum Cost Paths. IEEE Transactions 
%   on Systems Science and Cybernetics, 4(2), 100-107.
%
% Inputs:
%   grid_map   - MxN binary matrix (0 = free, 1 = obstacle)
%   start_cell - [row, col] start position
%   goal_cell  - [row, col] goal position
%
% Outputs:
%   path       - Kx2 matrix of [row, col] waypoints from start to goal
%   path_found - Boolean indicating if path was found
%
% The algorithm uses:
%   f(n) = g(n) + h(n)
%   where g(n) = cost from start to n
%         h(n) = heuristic estimate from n to goal (Euclidean distance)

    [rows, cols] = size(grid_map);
    
    % Validate inputs
    if grid_map(start_cell(1), start_cell(2)) == 1
        warning('Start position is in obstacle!');
        path = [];
        path_found = false;
        return;
    end
    if grid_map(goal_cell(1), goal_cell(2)) == 1
        warning('Goal position is in obstacle!');
        path = [];
        path_found = false;
        return;
    end
    
    % Initialize data structures
    % Using linear indices for efficiency
    start_idx = sub2ind([rows, cols], start_cell(1), start_cell(2));
    goal_idx = sub2ind([rows, cols], goal_cell(1), goal_cell(2));
    
    % g_score: cost from start to each node
    g_score = inf(rows, cols);
    g_score(start_cell(1), start_cell(2)) = 0;
    
    % f_score: g + heuristic
    f_score = inf(rows, cols);
    f_score(start_cell(1), start_cell(2)) = heuristic(start_cell, goal_cell);
    
    % came_from: for path reconstruction
    came_from = zeros(rows, cols);
    
    % Open set (nodes to explore) - using a simple list
    % Each entry: [f_score, row, col]
    open_set = [f_score(start_cell(1), start_cell(2)), start_cell(1), start_cell(2)];
    
    % Closed set (already explored)
    closed_set = false(rows, cols);
    
    % 8-connected neighbors (includes diagonals)
    % [delta_row, delta_col, cost]
    neighbors = [
        -1, 0, 1.0;    % Up
         1, 0, 1.0;    % Down
         0, -1, 1.0;   % Left
         0, 1, 1.0;    % Right
        -1, -1, 1.414; % Up-Left (diagonal)
        -1, 1, 1.414;  % Up-Right
         1, -1, 1.414; % Down-Left
         1, 1, 1.414   % Down-Right
    ];
    
    path_found = false;
    
    while ~isempty(open_set)
        % Get node with lowest f_score
        [~, min_idx] = min(open_set(:, 1));
        current = open_set(min_idx, 2:3);
        current_row = current(1);
        current_col = current(2);
        
        % Remove from open set
        open_set(min_idx, :) = [];
        
        % Check if goal reached
        if current_row == goal_cell(1) && current_col == goal_cell(2)
            path_found = true;
            break;
        end
        
        % Add to closed set
        closed_set(current_row, current_col) = true;
        
        % Explore neighbors
        for n = 1:size(neighbors, 1)
            neighbor_row = current_row + neighbors(n, 1);
            neighbor_col = current_col + neighbors(n, 2);
            move_cost = neighbors(n, 3);
            
            % Check bounds
            if neighbor_row < 1 || neighbor_row > rows || ...
               neighbor_col < 1 || neighbor_col > cols
                continue;
            end
            
            % Check if obstacle or already closed
            if grid_map(neighbor_row, neighbor_col) == 1 || ...
               closed_set(neighbor_row, neighbor_col)
                continue;
            end
            
            % For diagonal moves, check if both adjacent cells are free
            % (prevents cutting corners)
            if neighbors(n, 3) > 1  % Diagonal move
                if grid_map(current_row + neighbors(n, 1), current_col) == 1 || ...
                   grid_map(current_row, current_col + neighbors(n, 2)) == 1
                    continue;
                end
            end
            
            % Calculate tentative g_score
            tentative_g = g_score(current_row, current_col) + move_cost;
            
            if tentative_g < g_score(neighbor_row, neighbor_col)
                % This path is better
                came_from(neighbor_row, neighbor_col) = ...
                    sub2ind([rows, cols], current_row, current_col);
                g_score(neighbor_row, neighbor_col) = tentative_g;
                f_score(neighbor_row, neighbor_col) = tentative_g + ...
                    heuristic([neighbor_row, neighbor_col], goal_cell);
                
                % Add to open set if not already there
                in_open = any(open_set(:,2) == neighbor_row & open_set(:,3) == neighbor_col);
                if ~in_open
                    open_set = [open_set; ...
                        f_score(neighbor_row, neighbor_col), neighbor_row, neighbor_col];
                end
            end
        end
    end
    
    % Reconstruct path
    if path_found
        path = reconstruct_path(came_from, goal_cell, [rows, cols]);
    else
        path = [];
        warning('A* could not find a path!');
    end
end

function h = heuristic(cell, goal)
    % Euclidean distance heuristic
    h = sqrt((cell(1) - goal(1))^2 + (cell(2) - goal(2))^2);
end

function path = reconstruct_path(came_from, goal_cell, grid_size)
    path = goal_cell;
    current = goal_cell;
    
    while came_from(current(1), current(2)) ~= 0
        prev_idx = came_from(current(1), current(2));
        [prev_row, prev_col] = ind2sub(grid_size, prev_idx);
        current = [prev_row, prev_col];
        path = [current; path];
    end
end
