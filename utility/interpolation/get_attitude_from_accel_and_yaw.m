function R = get_attitude_from_accel_and_yaw(accel,yaw,add_gravity_flag)
    if nargin < 2
        yaw = 0 ;
    end
    
    % add gravity
    if nargin < 3 || add_gravity_flag
        accel = accel + [0;0;9.81] ;
    end
    
    % body z axis
    n = vecnorm(accel) ;
    if n ~= 0
        z_B = accel ./ n ;
    else
        z_B = [0;0;1] ;
    end
    
    % intermediate x axis for given yaw
    x_C = [cos(yaw) ; sin(yaw) ; 0] ;
    
    % body y axis
    y_B = cross(z_B,x_C) ;
    y_B = y_B./vecnorm(y_B) ;
    
    % body x axis
    x_B = cross(y_B,z_B) ;
    
    % output
    R = [x_B,y_B,z_B] ;
end