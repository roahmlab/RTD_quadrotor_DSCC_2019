function [y,R,a_ref,w_dot] = get_quadrotor_state_using_diff_flatness(varargin)
% [y,R] = get_quadrotor_state_using_diff_flatness(z_ref)
% [y,R] = get_quadrotor_state_using_diff_flatness(t,T_ref,Z_ref)
%
% Given a time and reference trajectory defined by T_ref and Z_ref, back
% out the quadrotor state y and attitude R along the reference trajectory.
%
% INPUTS:
%   A - a quadrotor_agent instance
%
%   t - a scalar time within [T_ref(1), T_ref(end)]
%
%   T_ref - a 1-by-N array of monotonically increasing time values
%
%   Z_ref - a 12-by-N array of (position,velocity,acceleration,jerk)
%       states, each of which is 12-by-1, corresponding to the times in
%       the array T_ref; needs to be 15-by-N, including snap, if you need
%       the w_dot (angular acceleration) output
%
% OUTPUTS:
%   y - a 9-by-1 array of the (position,velocity,angular_velocity) states
%       of the drone at time t
%
%   R - a 3-by-3 rotation matrix of the drone's required attitude at time t
%
%   a - a 3-by-1 vector of the reference acceleration
%
%   w_dot - a 3-by-1 vector of the reference angular acceleration
%
% See "Minimum Snap Trajectory Generation and Control for Quadrotors"
% or an explanation of how differential flatness lets us back out the
% states y and R for a quadrotor from a trajectory in the format of Z_ref.
% Note that this function assumes 0 yaw, but one could extend it with the
% techniques in that paper to modify the output R so that the quadrotor can
% achieve a given yaw angle at the time t.
%
% See also: Mellinger_LLC, quadrotor_agent
%
% Author: Shreyas Kousik
% Created: 11 Aug 2020

    % get reference state at time t
    if nargin == 1
        z = varargin{1} ;
    elseif nargin == 3
        t = varargin{1} ;
        T_ref = varargin{2} ;
        Z_ref = varargin{3} ;
        z = match_trajectories(t,T_ref,Z_ref) ;
    else
        error('Incorrect number of input arguments!')
    end
    x_ref = z(1:3) ;
    v_ref = z(4:6) ;
    a_ref = z(7:9) ; % note that this is an output!
    j_ref = z(10:12) ;

    % reference body frame z axis
    f = a_ref + [0;0;9.81] ; % add gravity force in
    z_B = f/norm(f) ;
    
    % normalized thrust force
    u_1 = z_B(3) ;

    % reference world x axis assuming yaw ref is 0
    x_C = [1;0;0] ;

    % reference body y axis
    y_B_ir = cross(z_B,x_C) ;
    y_B = y_B_ir/norm(y_B_ir) ;

    % reference body x axis
    x_B = cross(y_B,z_B) ;

    % reference angular velocity
    h_w = (1/u_1)*(j_ref - (z_B'*j_ref)*z_B) ;
    p = -h_w'*y_B ;
    q = h_w'*x_B ;
    r = 0 ; % since reference yaw and yaw rate are 0
    w_BW = [p;q;r] ;

    % output state
    y = [x_ref ; v_ref ; w_BW] ;

    % output attitude
    R = [x_B,y_B,z_B] ;
    
    % output angular acceleration
    if nargout > 2
        % get reference snap
        s = z(13:15) ;
        
        % compute h_w_dot
        w_BW_x_z_B = cross(w_BW,z_B) ;
        h_w_dot_inner = (z_B'*j_ref)*w_BW_x_z_B + (w_BW_x_z_B'*j_ref + z_B'*s)*z_B ;
        h_w_dot = (1/u_1)*(s - h_w_dot_inner) ;
        
        p_dot = -h_w_dot'*y_B - h_w'*cross(w_BW,y_B) ;
        q_dot = h_w_dot'*x_B + h_w'*cross(w_BW,x_B) ;
        r_dot = 0 ;
        
        w_dot = [p_dot ; q_dot ; r_dot] ;
    end
end