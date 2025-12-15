function [robots, obstacles, map_bounds, scn_name] = interactive()
% INTERACTIVE: User-Defined Robot Placement Scenario
% Allows users to place robots and goals via mouse clicks
% Great for testing arbitrary configurations during presentations
%
% Instructions:
%   LEFT CLICK:  Place robot starting position
%   RIGHT CLICK: Set goal for most recently placed robot
%   ENTER:       Start simulation
%   ESC:         Cancel and exit
%
% Note: Place start first, then goal for each robot before placing next robot

    scn_name = 'Interactive';
    map_bounds = [-15 15 -15 15];
    
    % Setup interactive figure
    fig = figure('Name', 'Interactive Robot Placement', 'Color', 'k', ...
        'Position', [100 100 800 800], 'NumberTitle', 'off');
    ax = axes('Color', 'k', 'XColor', 'w', 'YColor', 'w', 'GridColor', [0.3 0.3 0.3]);
    axis equal; grid on; hold on;
    axis(map_bounds);
    xlabel('X (m)', 'Color', 'w');
    ylabel('Y (m)', 'Color', 'w');
    title({'INTERACTIVE MODE', ...
           'LEFT CLICK: Place Robot Start | RIGHT CLICK: Set Goal | ENTER: Start'}, ...
           'Color', 'w', 'FontSize', 12);
    
    % Predefined static obstacles for interaction testing (ROW array with commas)
    obstacles = [Obstacle([5; 5], 1.2, [0; 0]), ...
                 Obstacle([-5; -5], 1.2, [0; 0]), ...
                 Obstacle([0; 0], 0.8, [0; 0]), ...
                 Obstacle([-5; 5], 0.8, [0; 0]), ...
                 Obstacle([5; -5], 0.8, [0; 0])];
    
    % Draw obstacles
    for i = 1:length(obstacles)
        viscircles(obstacles(i).pos', obstacles(i).radius, 'Color', [0.7 0.2 0.2], 'LineWidth', 2);
    end
    
    % Initialize robot tracking
    robots = [];  % Will be built as proper array
    temp_starts = [];
    robot_count = 0;
    placing_goal = false;
    colors = lines(20);  % Pre-generate colors for up to 20 robots
    
    % Handles for dynamic graphics
    h_markers = [];
    
    % Store data in figure for callbacks
    setappdata(fig, 'robots', robots);
    setappdata(fig, 'temp_starts', temp_starts);
    setappdata(fig, 'robot_count', robot_count);
    setappdata(fig, 'placing_goal', placing_goal);
    setappdata(fig, 'colors', colors);
    setappdata(fig, 'h_markers', h_markers);
    setappdata(fig, 'ax', ax);
    setappdata(fig, 'finished', false);
    
    % Instructions text
    h_inst = text(map_bounds(1)+0.5, map_bounds(4)-1, ...
        'Click to place Robot 1 start position...', ...
        'Color', 'y', 'FontSize', 10, 'FontWeight', 'bold');
    setappdata(fig, 'h_inst', h_inst);
    
    % Mouse click callback
    set(fig, 'WindowButtonDownFcn', @mouse_callback);
    set(fig, 'KeyPressFcn', @key_callback);
    
    % Wait for user to finish
    try
        uiwait(fig);
    catch
        % Figure was closed
        robots = Robot.empty();
        return;
    end
    
    % Check if figure still exists
    if ~ishandle(fig)
        robots = Robot.empty();
        return;
    end
    
    % Retrieve final robots array
    robots = getappdata(fig, 'robots');
    
    % Close the interactive figure
    if ishandle(fig)
        close(fig);
    end
    
    % Verify we have valid robots
    if isempty(robots)
        warning('No robots were placed. Creating default configuration.');
        default_robot = Robot(1, [-10; 0], 0, 0.4, 1.5, [10; 0]);
        default_robot.color = [0 0.5 1];
        robots = default_robot;
    end
    
    fprintf('   >> Interactive: %d robots placed\n', length(robots));
end

% =========================================================================
% CALLBACK FUNCTIONS (Nested for access to figure data)
% =========================================================================

function mouse_callback(fig, ~)
    ax = getappdata(fig, 'ax');
    robots = getappdata(fig, 'robots');
    temp_starts = getappdata(fig, 'temp_starts');
    robot_count = getappdata(fig, 'robot_count');
    placing_goal = getappdata(fig, 'placing_goal');
    colors = getappdata(fig, 'colors');
    h_markers = getappdata(fig, 'h_markers');
    h_inst = getappdata(fig, 'h_inst');
    
    % Get click position
    pt = get(ax, 'CurrentPoint');
    click_pos = pt(1, 1:2)';
    
    if strcmp(get(fig, 'SelectionType'), 'normal')  % Left click - Place start
        if placing_goal
            % Must place goal first before new robot
            h_inst.String = sprintf('Place GOAL for Robot %d first (RIGHT CLICK)', robot_count);
            return;
        end
        
        % Place new robot start position
        robot_count = robot_count + 1;
        temp_starts = [temp_starts, click_pos];
        
        % Draw start marker - use color index with modulo
        color_idx = mod(robot_count - 1, size(colors, 1)) + 1;
        h_s = plot(click_pos(1), click_pos(2), 'o', 'MarkerSize', 20, ...
            'MarkerFaceColor', colors(color_idx,:), 'MarkerEdgeColor', 'w', 'LineWidth', 2);
        h_t = text(click_pos(1), click_pos(2), sprintf('%d', robot_count), ...
            'Color', 'w', 'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontSize', 12);
        h_markers = [h_markers; h_s; h_t];
        
        placing_goal = true;
        h_inst.String = sprintf('Now RIGHT CLICK to set goal for Robot %d', robot_count);
        
    elseif strcmp(get(fig, 'SelectionType'), 'alt') && placing_goal  % Right click - Place goal
        % Set goal for most recent robot
        start_pos = temp_starts(:, robot_count);
        goal_pos = click_pos;
        
        % Calculate initial heading
        dir_to_goal = goal_pos - start_pos;
        theta = atan2(dir_to_goal(2), dir_to_goal(1));
        
        % Create robot - use color index with modulo for >20 robots
        color_idx = mod(robot_count - 1, size(colors, 1)) + 1;
        new_robot = Robot(robot_count, start_pos, theta, 0.4, 1.5, goal_pos);
        new_robot.color = colors(color_idx, :);
        
        % Build robots array properly (as 1xN array)
        if isempty(robots)
            robots = new_robot;
        else
            robots = [robots, new_robot];
        end
        
        % Draw goal marker
        h_g = plot(click_pos(1), click_pos(2), 'x', 'MarkerSize', 15, ...
            'Color', colors(color_idx,:), 'LineWidth', 3);
        h_gt = text(click_pos(1)+0.5, click_pos(2)+0.5, sprintf('G%d', robot_count), ...
            'Color', colors(color_idx,:), 'FontSize', 10);
        
        % Draw line connecting start and goal
        h_line = plot([start_pos(1), click_pos(1)], [start_pos(2), click_pos(2)], '--', ...
            'Color', colors(color_idx,:), 'LineWidth', 1);
        h_markers = [h_markers; h_g; h_gt; h_line];
        
        placing_goal = false;
        h_inst.String = sprintf('Robot %d placed! LEFT CLICK for Robot %d or ENTER to start', ...
            robot_count, robot_count+1);
    end
    
    % Save updated data
    setappdata(fig, 'robots', robots);
    setappdata(fig, 'temp_starts', temp_starts);
    setappdata(fig, 'robot_count', robot_count);
    setappdata(fig, 'placing_goal', placing_goal);
    setappdata(fig, 'h_markers', h_markers);
end

function key_callback(fig, event)
    robots = getappdata(fig, 'robots');
    placing_goal = getappdata(fig, 'placing_goal');
    h_inst = getappdata(fig, 'h_inst');
    
    if strcmp(event.Key, 'return') || strcmp(event.Key, 'space')
        if ~isempty(robots) && ~placing_goal
            h_inst.String = 'Starting simulation...';
            drawnow;
            pause(0.5);
            uiresume(fig);
        else
            h_inst.String = 'Place at least one complete robot (start + goal) first!';
        end
    elseif strcmp(event.Key, 'escape')
        close(fig);
    end
end
