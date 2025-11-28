classdef Robot
    % ROBOT Represents a differential drive robot with unicycle kinematics.
    
    properties
        id          % Unique identifier
        pos         % [x; y] position (m)
        theta       % Heading angle (rad)
        radius      % Physical radius (m)
        v_max       % Maximum linear speed (m/s)
        goal        % [x; y] target position
        
        % Visualization colors
        color = 'b'; 
    end
    
    methods
        function obj = Robot(id, start_pos, start_theta, radius, v_max, goal)
            obj.id = id;
            obj.pos = start_pos;
            obj.theta = start_theta;
            obj.radius = radius;
            obj.v_max = v_max;
            obj.goal = goal;
        end
        
        function obj = move(obj, v_opt_vector, dt)
            % MOVE Executes the differential drive controller and updates kinematics.
            % Input: 
            %   v_opt_vector: [vx; vy] The desired holonomic velocity from VO
            %   dt: Time step
            
            % 1. Controller (Section 4.3 of Guide)
            % Deconstruct the optimal holonomic velocity
            v_opt_x = v_opt_vector(1);
            v_opt_y = v_opt_vector(2);
            
            % Calculate desired heading
            target_heading = atan2(v_opt_y, v_opt_x);
            
            % Calculate heading error (using angdiff logic)
            error_theta = atan2(sin(target_heading - obj.theta), cos(target_heading - obj.theta));
            
            % If desired velocity is effectively zero, stop
            if norm(v_opt_vector) < 0.01
                v = 0;
                w = 0;
            else
                % Proportional Controller for Angular Velocity
                Kp = 4.0; 
                w = Kp * error_theta;
                
                % Linear Velocity: Slow down when turning (The "Cos" term)
                v = norm(v_opt_vector) * cos(error_theta);
                
                % Prevent reverse motion (Standard VO assumes forward motion)
                if v < 0, v = 0; end 
            end
            
            % 2. Kinematics Update (Euler Integration)
            obj.theta = obj.theta + w * dt;
            obj.pos(1) = obj.pos(1) + v * cos(obj.theta) * dt;
            obj.pos(2) = obj.pos(2) + v * sin(obj.theta) * dt;
        end
    end
end