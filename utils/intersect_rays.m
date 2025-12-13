function p_inter = intersect_rays(origin1, angle1, origin2, angle2)
% INTERSECT_RAYS Compute intersection point of two rays
%
% Inputs:
%   origin1 - 2x1 starting point of first ray
%   angle1  - direction angle of first ray (radians)
%   origin2 - 2x1 starting point of second ray  
%   angle2  - direction angle of second ray (radians)
%
% Output:
%   p_inter - 2x1 intersection point, or midpoint if rays are parallel
%
% Method: Solves origin1 + t1*dir1 = origin2 + t2*dir2
%         Using parametric form and Cramer's rule

    % Direction vectors
    dir1 = [cos(angle1); sin(angle1)];
    dir2 = [cos(angle2); sin(angle2)];
    
    % Difference between origins
    d = origin2(:) - origin1(:);
    
    % Solve: t1 * dir1 - t2 * dir2 = d
    % Matrix form: [dir1, -dir2] * [t1; t2] = d
    % Using Cramer's rule:
    
    det_A = dir1(1) * (-dir2(2)) - dir1(2) * (-dir2(1));
    % Simplifies to: det_A = dir2(1)*dir1(2) - dir1(1)*dir2(2)
    det_A = dir1(1) * dir2(2) - dir1(2) * dir2(1);
    
    % Check for parallel rays (determinant near zero)
    if abs(det_A) < 1e-10
        % Rays are parallel - return midpoint of origins as fallback
        p_inter = (origin1(:) + origin2(:)) / 2;
        return;
    end
    
    % Solve for t1 using Cramer's rule
    % t1 = det([d, -dir2]) / det_A
    t1 = (d(1) * dir2(2) - d(2) * dir2(1)) / det_A;
    
    % Compute intersection point
    p_inter = origin1(:) + t1 * dir1;
end
