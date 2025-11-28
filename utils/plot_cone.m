function plot_cone(apex, theta_min, theta_max, length, color)
    % PLOT_CONE Draws a semi-transparent cone (triangle) on the current plot.
    % Inputs:
    %   apex: [x, y] position of the cone tip (usually robot position)
    %   theta_min, theta_max: Angles of the cone legs
    %   length: How long to draw the cone legs (visual only)
    %   color: Color string or RGB vector
    
    % 1. Calculate the three points of the triangle
    p1 = apex; % The tip
    
    % The two outer edges
    p2 = apex + length * [cos(theta_min); sin(theta_min)];
    p3 = apex + length * [cos(theta_max); sin(theta_max)];
    
    % 2. Create the shape coordinates
    X = [p1(1), p2(1), p3(1)];
    Y = [p1(2), p2(2), p3(2)];
    
    % 3. Draw using 'patch' for a filled shape
    patch(X, Y, color, 'FaceAlpha', 0.2, 'EdgeColor', color, 'LineStyle', '--');
end