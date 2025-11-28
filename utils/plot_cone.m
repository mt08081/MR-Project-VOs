function h = plot_cone(apex, theta_min, theta_max, length, color)
    % PLOT_CONE Draws a semi-transparent cone and returns its graphics handle.
    % Inputs:
    %   apex: [x, y] position of the cone tip
    %   theta_min, theta_max: Angles of the cone legs
    %   length: Visual length of the legs
    %   color: Color char or vector
    % Output:
    %   h: Handle to the patch object (used for deletion)
    
    % 1. Calculate the three points of the triangle
    p1 = apex; 
    p2 = apex + length * [cos(theta_min); sin(theta_min)];
    p3 = apex + length * [cos(theta_max); sin(theta_max)];
    
    % 2. Create the shape coordinates
    X = [p1(1), p2(1), p3(1)];
    Y = [p1(2), p2(2), p3(2)];
    
    % 3. Draw and RETURN the handle 'h'
    h = patch(X, Y, color, 'FaceAlpha', 0.2, 'EdgeColor', color, 'LineStyle', '--');
end