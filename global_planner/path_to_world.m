function path_world = path_to_world(path_cells, grid_size, cell_size)
% PATH_TO_WORLD Convert path from cell coordinates to world coordinates
%
% Inputs:
%   path_cells - Nx2 matrix of [row, col] cell coordinates
%   grid_size  - [rows, cols] size of the grid
%   cell_size  - Size of each grid cell in meters
%
% Outputs:
%   path_world - Nx2 matrix of [x, y] world coordinates

    rows = grid_size(1);
    path_world = zeros(size(path_cells));
    
    for i = 1:size(path_cells, 1)
        cell = path_cells(i, :);
        % x = col * cell_size (shifted to center of cell)
        % y = (rows - row) * cell_size (inverted, shifted to center)
        x = (cell(2) - 0.5) * cell_size;
        y = (rows - cell(1) + 0.5) * cell_size;
        path_world(i, :) = [x, y];
    end
end
