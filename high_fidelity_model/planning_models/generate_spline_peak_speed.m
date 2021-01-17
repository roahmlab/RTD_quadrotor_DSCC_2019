function [T,Z,Z_plan] = generate_spline_peak_speed(v_0,a_0,v_peak,t_plan,t_peak,t_total,dt,t_extra)
    % note that, given the current initial condition set, the time horizon
    % should be chosen so that the robot can come to a stop with some max
    % allowable acceleration magnitude from the peak speed
    
%% ensure inputs are right
    v_0 = v_0(:) ;
    a_0 = a_0(:) ;
    v_peak = v_peak(:) ;
    
%% compute the first part of the spline, up to v_peak
    % time vector
    T_to_peak = 0:dt:t_peak ; % assume t_peak/dt is an integer
    
    % desired acceleration at peak speed
    a_peak = [0;0;0] ;
    
    % compute change in velocity/accel for each axis
    Dv = v_peak - v_0 - a_0*t_peak ;
    Da = a_peak - a_0 ;
    
    % compute spline parameters
    [ax,bx,cx] = single_axis_params(Dv(1),Da(1),t_peak) ;
    [ay,by,cy] = single_axis_params(Dv(2),Da(2),t_peak) ;
    [az,bz,cz] = single_axis_params(Dv(3),Da(3),t_peak) ;
    
    a = [ax ay az] ;
    b = [bx by bz] ;
    c = [cx cy cz] ;
    
    % compute spline
    p_0 = [0;0;0] ; % default initial position
    Z_to_peak = make_spline(T_to_peak,p_0,v_0,a_0,a,b,c) ;

%% compute second part of the spline, coming to a stop
    % create time vector for second half
    t_to_stop = t_total - t_peak ;
    T_to_stop = 0:dt:t_to_stop ;    

    % desired end speed and acceleration
    v_f = [0;0;0] ;
    a_f = [0;0;0] ;

    % for each axis, compute the change in velocity/accel
    Dv = v_f - v_peak - a_peak.*t_to_stop ;
    Da = a_f - a_peak ;

    [ax,bx,cx] = single_axis_params(Dv(1),Da(1),t_to_stop) ;
    [ay,by,cy] = single_axis_params(Dv(2),Da(2),t_to_stop) ;
    [az,bz,cz] = single_axis_params(Dv(3),Da(3),t_to_stop) ;

    a = [ax ay az] ;
    b = [bx by bz] ;
    c = [cx cy cz] ;
    
    % compute spline
    p_peak = Z_to_peak(1:3,end) ;
    Z_to_stop = make_spline(T_to_stop,p_peak,v_peak,a_peak,a,b,c) ;

%% connect splines and times
    T = [T_to_peak(1:end-1), T_to_stop + t_peak] ;
    Z = [Z_to_peak(:,1:end-1), Z_to_stop] ;
    
%% add extra time to end of spline if desired
    if nargin > 7 && (t_extra > 0)
            T = [T, T(end) + dt, T(end) + t_extra] ;
            Z = [Z, [Z(1:3,end);zeros(12,1)], [Z(1:3,end);zeros(12,1)]] ;
    end
    
%% compute v and a at t_plan
    if nargout > 2
        Z_plan = match_trajectories(t_plan,T_to_peak,Z_to_peak) ;
    end
    
end

function [a,b,c] = single_axis_params(Dv,Da,T)
    M = [0 0 ;
         -12 6*T ;
         6*T -2*T^2] ;
         
    out = (1/T^3)*M*[Dv;Da] ;
    a = out(1) ; b = out(2) ; c = out(3) ;
end

function Z_out = make_spline(T_in,p_0,v_0,a_0,a,b,c)
    % get spline params
    ax = a(1) ; ay = a(2) ; az = a(3) ;
    bx = b(1) ; by = b(2) ; bz = b(3) ;
    cx = c(1) ; cy = c(2) ; cz = c(3) ;

    % position:
    px = (ax/120).*T_in.^5 + (bx/24).*T_in.^4 + (cx/6).*T_in.^3 + (a_0(1)/2).*T_in.^2 + v_0(1).*T_in + p_0(1) ;
    py = (ay/120).*T_in.^5 + (by/24).*T_in.^4 + (cy/6).*T_in.^3 + (a_0(2)/2).*T_in.^2 + v_0(2).*T_in + p_0(2) ;
    pz = (az/120).*T_in.^5 + (bz/24).*T_in.^4 + (cz/6).*T_in.^3 + (a_0(3)/2).*T_in.^2 + v_0(3).*T_in + p_0(3) ;
    
    % speed:
    vx = (ax/24).*T_in.^4 + (bx/6).*T_in.^3 + (cx/2).*T_in.^2 + (a_0(1)).*T_in + v_0(1) ;
    vy = (ay/24).*T_in.^4 + (by/6).*T_in.^3 + (cy/2).*T_in.^2 + (a_0(2)).*T_in + v_0(2) ;
    vz = (az/24).*T_in.^4 + (bz/6).*T_in.^3 + (cz/2).*T_in.^2 + (a_0(3)).*T_in + v_0(3) ;
    
    % acceleration:
    ax = (ax/6).*T_in.^3 + (bx/2).*T_in.^2 + (cx).*T_in + (a_0(1)) ;
    ay = (ay/6).*T_in.^3 + (by/2).*T_in.^2 + (cy).*T_in + (a_0(2)) ;
    az = (az/6).*T_in.^3 + (bz/2).*T_in.^2 + (cz).*T_in + (a_0(3)) ;
    
    % jerk
    jx = (ax/2).*T_in.^2 + (bx).*T_in + (cx) ;
    jy = (ay/2).*T_in.^2 + (by).*T_in + (cy) ;
    jz = (az/2).*T_in.^2 + (bz).*T_in + (cz) ;
    
    % snap
    sx = ax.*T_in + (bx) ;
    sy = ay.*T_in + (by) ;
    sz = az.*T_in + (bz) ;
    
    % create output traj
    Z_out = [px ; py ; pz ;
             vx ; vy ; vz ;
             ax ; ay ; az ;
             jx ; jy ; jz ;
             sx ; sy ; sz ] ;
end
