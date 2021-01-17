classdef zonotope_dynamic_obstacle < cuboid_dynamic_obstacle
    properties
        zono
    end
    
    methods
        function O = zonotope_dynamic_obstacle(L,W,H,position,speed)
            O@cuboid_dynamic_obstacle(L,W,H,position,speed) ;
            O.zono = zonotope([zeros(3,1), [L/2 0 0; 0 W/2 0; 0 0 H/2] ]);
        end
    end
end