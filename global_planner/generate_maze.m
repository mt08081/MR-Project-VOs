function [grid_map, obstacles, start_pos, goal_pos] = generate_maze(maze_type, grid_size, cell_size)
% GENERATE_MAZE Create a maze/map with obstacles for global+local planning demo
%
% Inputs:
%   maze_type  - String: 'simple', 'corridor', 'rooms', 'cluttered', 'custom'
%   grid_size  - [rows, cols] size of the grid
%   cell_size  - Size of each grid cell in world coordinates (meters)
%
% Outputs:
%   grid_map   - Binary matrix for A* (0 = free, 1 = obstacle)
%   obstacles  - Array of Obstacle objects for VO
%   start_pos  - [x, y] world coordinates for robot start
%   goal_pos   - [x, y] world coordinates for goal

    if nargin < 2
        grid_size = [40, 40];
    end
    if nargin < 3
        cell_size = 0.5;  % 0.5m per cell
    end
    
    rows = grid_size(1);
    cols = grid_size(2);
    
    % Initialize empty grid
    grid_map = zeros(rows, cols);
    
    switch maze_type
        case 'simple'
            % Simple maze with a few walls - good for demonstration
            [grid_map, start_cell, goal_cell] = create_simple_maze(grid_map);
            
        case 'corridor'
            % Long corridor with narrow passages
            [grid_map, start_cell, goal_cell] = create_corridor_maze(grid_map);
            
        case 'rooms'
            % Room-like structure with doorways
            [grid_map, start_cell, goal_cell] = create_rooms_maze(grid_map);
            
        case 'cluttered'
            % Random obstacles scattered around
            [grid_map, start_cell, goal_cell] = create_cluttered_maze(grid_map);
            
        case 'custom'
            % Load from file or use default
            [grid_map, start_cell, goal_cell] = create_simple_maze(grid_map);
            
        otherwise
            error('Unknown maze type: %s', maze_type);
    end
    
    % Convert cell coordinates to world coordinates
    % Grid: row increases downward, col increases rightward
    % World: x increases rightward, y increases upward
    start_pos = cell_to_world(start_cell, grid_size, cell_size);
    goal_pos = cell_to_world(goal_cell, grid_size, cell_size);
    
    % Convert grid obstacles to Obstacle objects for VO
    obstacles = grid_to_obstacles(grid_map, grid_size, cell_size);
end

%% Maze Generators

function [grid_map, start_cell, goal_cell] = create_simple_maze(grid_map)
    [rows, cols] = size(grid_map);
    
    % Add border walls
    grid_map(1, :) = 1;
    grid_map(rows, :) = 1;
    grid_map(:, 1) = 1;
    grid_map(:, cols) = 1;
    
    % Add internal walls with gaps
    % Vertical wall in the middle
    wall_col = round(cols/2);
    grid_map(5:rows-10, wall_col) = 1;
    grid_map(5:rows-10, wall_col+1) = 1;
    
    % Horizontal wall
    wall_row = round(rows/2);
    grid_map(wall_row, wall_col+5:cols-5) = 1;
    grid_map(wall_row+1, wall_col+5:cols-5) = 1;
    
    % Another obstacle block
    grid_map(rows-15:rows-10, 8:12) = 1;
    
    % Set start and goal
    start_cell = [rows-5, 5];
    goal_cell = [5, cols-5];
end

function [grid_map, start_cell, goal_cell] = create_corridor_maze(grid_map)
    [rows, cols] = size(grid_map);
    
    % Add border walls
    grid_map(1, :) = 1;
    grid_map(rows, :) = 1;
    grid_map(:, 1) = 1;
    grid_map(:, cols) = 1;
    
    % Create winding corridor
    % First horizontal section
    corridor_width = 5;
    wall_top = round(rows/4);
    wall_bottom = rows - round(rows/4);
    
    % Upper and lower walls creating corridor
    grid_map(1:wall_top, 1:cols) = 1;
    grid_map(wall_bottom:rows, 1:cols) = 1;
    
    % Add baffles to create turns
    baffle_length = round((wall_bottom - wall_top) * 0.7);
    
    % Baffle from top
    grid_map(wall_top:wall_top+baffle_length, round(cols/4)) = 1;
    grid_map(wall_top:wall_top+baffle_length, round(cols/4)+1) = 1;
    
    % Baffle from bottom
    grid_map(wall_bottom-baffle_length:wall_bottom, round(cols/2)) = 1;
    grid_map(wall_bottom-baffle_length:wall_bottom, round(cols/2)+1) = 1;
    
    % Baffle from top
    grid_map(wall_top:wall_top+baffle_length, round(3*cols/4)) = 1;
    grid_map(wall_top:wall_top+baffle_length, round(3*cols/4)+1) = 1;
    
    start_cell = [round((wall_top + wall_bottom)/2), 4];
    goal_cell = [round((wall_top + wall_bottom)/2), cols-4];
end

function [grid_map, start_cell, goal_cell] = create_rooms_maze(grid_map)
    [rows, cols] = size(grid_map);
    
    % Add border walls
    grid_map(1, :) = 1;
    grid_map(rows, :) = 1;
    grid_map(:, 1) = 1;
    grid_map(:, cols) = 1;
    
    % Create 4 rooms
    mid_row = round(rows/2);
    mid_col = round(cols/2);
    
    % Horizontal divider
    grid_map(mid_row, :) = 1;
    grid_map(mid_row+1, :) = 1;
    
    % Vertical divider
    grid_map(:, mid_col) = 1;
    grid_map(:, mid_col+1) = 1;
    
    % Create doorways
    door_width = 4;
    
    % Door in horizontal wall (left side)
    grid_map(mid_row:mid_row+1, round(mid_col/2)-door_width:round(mid_col/2)+door_width) = 0;
    
    % Door in horizontal wall (right side)
    grid_map(mid_row:mid_row+1, mid_col+round(mid_col/2)-door_width:mid_col+round(mid_col/2)+door_width) = 0;
    
    % Door in vertical wall (top side)
    grid_map(round(mid_row/2)-door_width:round(mid_row/2)+door_width, mid_col:mid_col+1) = 0;
    
    % Door in vertical wall (bottom side)
    grid_map(mid_row+round(mid_row/2)-door_width:mid_row+round(mid_row/2)+door_width, mid_col:mid_col+1) = 0;
    
    % Add some furniture in rooms
    grid_map(5:8, 5:8) = 1;
    grid_map(rows-8:rows-5, cols-8:cols-5) = 1;
    
    start_cell = [rows-10, 10];
    goal_cell = [10, cols-10];
end

function [grid_map, start_cell, goal_cell] = create_cluttered_maze(grid_map)
    [rows, cols] = size(grid_map);
    
    % Add border walls
    grid_map(1, :) = 1;
    grid_map(rows, :) = 1;
    grid_map(:, 1) = 1;
    grid_map(:, cols) = 1;
    
    % Add random rectangular obstacles
    num_obstacles = 15;
    rng(42);  % For reproducibility
    
    for i = 1:num_obstacles
        % Random position and size
        obs_row = randi([5, rows-10]);
        obs_col = randi([5, cols-10]);
        obs_height = randi([2, 5]);
        obs_width = randi([2, 5]);
        
        % Don't block start or goal areas
        if (obs_row > rows - 15 && obs_col < 15) || ...
           (obs_row < 15 && obs_col > cols - 15)
            continue;
        end
        
        % Place obstacle
        r_end = min(obs_row + obs_height, rows-2);
        c_end = min(obs_col + obs_width, cols-2);
        grid_map(obs_row:r_end, obs_col:c_end) = 1;
    end
    
    start_cell = [rows-5, 5];
    goal_cell = [5, cols-5];
end

%% Conversion Functions

function world_pos = cell_to_world(cell, grid_size, cell_size)
    % Convert grid cell [row, col] to world coordinates [x, y]
    % Grid origin is top-left, world origin is bottom-left
    rows = grid_size(1);
    
    % x = col * cell_size (shifted to center of cell)
    % y = (rows - row) * cell_size (inverted, shifted to center)
    x = (cell(2) - 0.5) * cell_size;
    y = (rows - cell(1) + 0.5) * cell_size;
    
    world_pos = [x, y];
end

function cell = world_to_cell(world_pos, grid_size, cell_size)
    % Convert world coordinates [x, y] to grid cell [row, col]
    rows = grid_size(1);
    
    col = round(world_pos(1) / cell_size + 0.5);
    row = round(rows - world_pos(2) / cell_size + 0.5);
    
    % Clamp to valid range
    row = max(1, min(rows, row));
    col = max(1, min(grid_size(2), col));
    
    cell = [row, col];
end

function obstacles = grid_to_obstacles(grid_map, grid_size, cell_size)
    % Convert grid obstacles to Obstacle objects
    % Each grid cell becomes an individual obstacle for better VO handling
    
    [rows, cols] = size(grid_map);
    obstacles = [];
    
    % Step size for subsampling (every Nth cell creates an obstacle)
    % This reduces the number of obstacles while maintaining coverage
    step = 2;
    
    for r = 1:step:rows
        for c = 1:step:cols
            % Check if this region has obstacles
            r_end = min(r + step - 1, rows);
            c_end = min(c + step - 1, cols);
            
            if any(any(grid_map(r:r_end, c:c_end) == 1))
                % Create obstacle at center of this cell group
                center_row = (r + r_end) / 2;
                center_col = (c + c_end) / 2;
                center_world = cell_to_world([center_row, center_col], grid_size, cell_size);
                
                % Radius covers the cell group with some padding
                radius = step * cell_size * 0.7;  % Slightly smaller to avoid excessive blocking
                
                % Create obstacle (static, velocity = [0,0])
                obs = Obstacle(center_world, radius, [0, 0]);
                obstacles = [obstacles, obs];
            end
        end
    end
    
    if isempty(obstacles)
        % Return empty Obstacle array of correct type
        obstacles = Obstacle.empty(1, 0);
    end
end

function path_world = path_to_world(path_cells, grid_size, cell_size)
    % Convert path from cell coordinates to world coordinates
    path_world = zeros(size(path_cells));
    for i = 1:size(path_cells, 1)
        path_world(i, :) = cell_to_world(path_cells(i, :), grid_size, cell_size);
    end
end
