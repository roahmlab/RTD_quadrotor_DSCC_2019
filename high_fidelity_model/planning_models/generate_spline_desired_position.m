function [T,U,Z] = generate_spline_desired_position(pf,v0,a0,time_horizon,sample_time)
% [T,U,Z] = generate_spline_desired_position(pf,v0,a0,time_horizon,sample_time)
%
% Create a spline, with associated nominal control inputs, for a quadcopter
% to track as in [1]
%
% Inputs:
%   pf - desired final location (relative to (x,y,z) = (0,0,0))
%   v0 - initial speed (vx,vy,vz)_0
%   a0 - initial acceleration
%   time_horizon - default is 1.5 s
%   sample_time - default is 0.05 s
%
% Outputs:
%   T - time vector of output
%   U - nominal thrust and body rates (in inertial frame)
%   Z - desired center-of-mass traj (position, speed, accel, jerk in x,y,z)
%
% % [1] Mueller, M., et al., "A Computationally Efficient Motion Primitive
%     for Quadrocopter Trajectory Generation," T-RO 2015
%
% See also: generate_spline_peak_speed

    % parse inputs
    if nargin < 5
        sample_time = 0.05 ;
        if nargin < 4
            time_horizon = 1.5 ;
        end
    end
    
    % create a time vector
    T = 0:sample_time:time_horizon ;
    
    % create default initial position and gravity
    p0 = [0;0;0] ;
    g = [0;0;-9.81] ;

    % desired future speed and acceleration (QC stops at end of desired traj)
    vf = [0;0;0] ;
    af = [0;0;0] ;

    % for each axis, compute the change in position/velocity/accel
    Dp = pf - p0 - v0.*time_horizon - 0.5.*a0.*time_horizon^2 ;
    Dv = vf - v0 - a0.*time_horizon ;
    Da = af - a0 ;

    [ax,bx,cx] = single_axis_params(Dp(1),Dv(1),Da(1),time_horizon) ;
    [ay,by,cy] = single_axis_params(Dp(2),Dv(2),Da(2),time_horizon) ;
    [az,bz,cz] = single_axis_params(Dp(3),Dv(3),Da(3),time_horizon) ;

% compute the trajectory in each dimension
    % position:
    px = (ax/120).*T.^5 + (bx/24).*T.^4 + (cx/6).*T.^3 + (a0(1)/2).*T.^2 + v0(1).*T + p0(1) ;
    py = (ay/120).*T.^5 + (by/24).*T.^4 + (cy/6).*T.^3 + (a0(2)/2).*T.^2 + v0(2).*T + p0(2) ;
    pz = (az/120).*T.^5 + (bz/24).*T.^4 + (cz/6).*T.^3 + (a0(3)/2).*T.^2 + v0(3).*T + p0(3) ;
    
    % speed:
    vx = (ax/24).*T.^4 + (bx/6).*T.^3 + (cx/2).*T.^2 + (a0(1)).*T + v0(1) ;
    vy = (ay/24).*T.^4 + (by/6).*T.^3 + (cy/2).*T.^2 + (a0(2)).*T + v0(2) ;
    vz = (az/24).*T.^4 + (bz/6).*T.^3 + (cz/2).*T.^2 + (a0(3)).*T + v0(3) ;
    
    % acceleration:
    ax = (ax/6).*T.^3 + (bx/2).*T.^2 + (cx).*T + (a0(1)) ;
    ay = (ay/6).*T.^3 + (by/2).*T.^2 + (cy).*T + (a0(2)) ;
    az = (az/6).*T.^3 + (bz/2).*T.^2 + (cz).*T + (a0(3)) ;
    
    % jerk
    jx = (ax/2).*T.^2 + (bx).*T + (cx) ;
    jy = (ay/2).*T.^2 + (by).*T + (cy) ;
    jz = (az/2).*T.^2 + (bz).*T + (cz) ;
    
    % snap
    sx = ax.*T + (bx) ;
    sy = ay.*T + (by) ;
    sz = az.*T + (bz) ;
    
    % create output traj
    Z = [px ; py ; pz ;
         vx ; vy ; vz ;
         ax ; ay ; az ;
         jx ; jy ; jz ;
         sx ; sy ; sz ] ;
     
% compute the control input
    % thrust
    f = sum(([ax ; ay ; az] - g).^2,1).^(1/2) ;
    
    % thrust direction
    n_f = [ax;ay;az] - repmat(g,1,length(ax));
    
    % check for zeros
    n_zero = all(n_f == 0,1) ;
    n_f(:,n_zero) = repmat([0;0;1],1,sum(n_zero)) ;
    n = n_f./repmat(vecnorm(n_f,2,1),3,1) ;
    
    % get thrust directions 1 ms ahead of current ones
    T_adv = T + 0.001 ;
    n_adv = [match_trajectories(T_adv(1:end-1),T,n), n(:,end)] ;
    
    % get body rates in inertial frame numerically
    w = cross(n,n_adv,1) ;
    
    % create nominal input
    U = [f ; w] ;
end

function [a,b,c] = single_axis_params(Dp,Dv,Da,T)
    M = [720, -360*T, 60*T^2 ;
         -360*T, 168*T^2, -24*T^3 ;
         60*T^2, -24*T^3, 3*T^4] ;
         
    out = (1/T^5)*M*[Dp;Dv;Da] ;
    a = out(1) ; b = out(2) ; c = out(3) ;
end