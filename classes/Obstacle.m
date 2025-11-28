classdef Obstacle
    properties
        pos     % [x; y] position
        radius  % radius (m)
        vel     % [vx; vy] velocity (defaults to [0;0] for static)
    end
    
    methods
        function obj = Obstacle(pos, radius, vel)
            obj.pos = pos;
            obj.radius = radius;
            if nargin < 3
                obj.vel = [0; 0];
            else
                obj.vel = vel;
            end
        end
    end
end