classdef Robot
    properties
        id          
        pos         % [x; y] position (m)
        theta       % Heading angle (rad)
        vel         % [vx; vy] Current velocity vector <--- NEW PROPERTY
        radius      
        v_max       
        goal        
        color = 'b'; 
    end
    
    methods
        function obj = Robot(id, start_pos, start_theta, radius, v_max, goal)
            obj.id = id;
            obj.pos = start_pos;
            obj.theta = start_theta;
            obj.vel = [0; 0]; % <--- Initialize to zero
            obj.radius = radius;
            obj.v_max = v_max;
            obj.goal = goal;
        end
        
        function obj = move(obj, v_opt_vector, dt)
            % Deconstruct optimal velocity
            v_opt_x = v_opt_vector(1);
            v_opt_y = v_opt_vector(2);
            
            target_heading = atan2(v_opt_y, v_opt_x);
            error_theta = atan2(sin(target_heading - obj.theta), cos(target_heading - obj.theta));
            
            if norm(v_opt_vector) < 0.01
                v = 0;
                w = 0;
            else
                Kp = 4.0; 
                w = Kp * error_theta;
                v = norm(v_opt_vector) * cos(error_theta);
                if v < 0, v = 0; end 
            end
            
            % Kinematics Update
            obj.theta = obj.theta + w * dt;
            
            % Calculate actual velocity vector based on heading
            vx_actual = v * cos(obj.theta);
            vy_actual = v * sin(obj.theta);
            
            % Update Position
            obj.pos(1) = obj.pos(1) + vx_actual * dt;
            obj.pos(2) = obj.pos(2) + vy_actual * dt;
            
            % Store Velocity for other agents to see <--- NEW
            obj.vel = [vx_actual; vy_actual];
        end
    end
end