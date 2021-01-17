classdef zonotope_obstacle < cuboid_obstacle
    properties
        zono ;
    end
    
    methods
        function O = zonotope_obstacle(l,w,h,c)
            %ZONOTOPE_OBSTACLE is pretty much a cuboid obstacle, but hold
            %on to the zonotope representation
            O@cuboid_obstacle(l,w,h,c);
            O.zono = zonotope([c, [l/2 0 0; 0 w/2 0; 0 0 h/2] ]);
        end
    end
end