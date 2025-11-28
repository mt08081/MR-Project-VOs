function in_cone = check_angles(test_angle, intervals)
    % CHECK_ANGLES Checks if a test_angle lies within any forbidden angular intervals.
    
    in_cone = false;
    if isempty(intervals)
        return;
    end
    
    % Normalize test_angle to [-pi, pi]
    test_angle = atan2(sin(test_angle), cos(test_angle));
    
    for i = 1:size(intervals, 1)
        % Normalize interval bounds
        lower = atan2(sin(intervals(i,1)), cos(intervals(i,1)));
        upper = atan2(sin(intervals(i,2)), cos(intervals(i,2)));
        
        % Check if the interval wraps around pi/-pi
        if lower <= upper
            % Standard case (e.g., -0.5 to 0.5)
            if test_angle >= lower && test_angle <= upper
                in_cone = true; return;
            end
        else
            % Wrap-around case (e.g., 3.0 to -3.0)
            if test_angle >= lower || test_angle <= upper
                in_cone = true; return;
            end
        end
    end
end