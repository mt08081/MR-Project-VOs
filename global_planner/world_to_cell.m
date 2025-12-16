function world_to_cell = world_to_cell(world_pos, grid_size, cell_size)
% WORLD_TO_CELL Convert world coordinates to grid cell indices
%
% Inputs:
%   world_pos  - [x, y] world coordinates
%   grid_size  - [rows, cols] size of the grid
%   cell_size  - Size of each grid cell in meters
%
% Outputs:
%   cell       - [row, col] cell indices

    rows = grid_size(1);
    
    col = round(world_pos(1) / cell_size + 0.5);
    row = round(rows - world_pos(2) / cell_size + 0.5);
    
    % Clamp to valid range
    row = max(1, min(rows, row));
    col = max(1, min(grid_size(2), col));
    
    world_to_cell = [row, col];
end
