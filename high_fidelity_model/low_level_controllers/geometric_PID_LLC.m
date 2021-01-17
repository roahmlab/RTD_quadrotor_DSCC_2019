classdef geometric_PID_LLC < low_level_controller
% Adapted from:
% [1] Lee et al. https://arxiv.org/pdf/1003.2005.pdf
% [2] Mellinger and Kumar. http://lewissoft.com/assets/pdf/uPennTrajectoryGeneration.pdf
% [3] Goodarzi et al. https://arxiv.org/pdf/1304.6765.pdf
% [4] Lee et al. https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=6315143
% [5] Mueller et al. https://ieeexplore.ieee.org/abstract/document/7299672
% [6] Lee et al. 2010. https://arxiv.org/pdf/1003.2005v3.pdf

properties
    k_x = 15 ;
    k_v = 5 ;
    k_R = 8 ;
    k_O = 2.5 ;
end
    
methods
%% constructorxn
function LLC = geometric_PID_LLC(varargin)
    LLC@low_level_controller(varargin{:}) ;
end

%% get control inputs
function U = get_control_inputs(LLC,A,t,z,varargin)
    % method: U = get_control_inputs(LLC,A,t,z,T_ref,U_ref,Z_ref,R,a_est,j_est)
    %
    % This is used for PID on the quadrotor_agent. It takes in the agent,
    % the current time t, the current state z, the current orientation
    % matrix R, and a reference time/input/trajectory in the inputs
    % T_ref/U_ref/Z_ref.
    %
    % The output of this method is a 4-by-1 vector where
    % the first element contains the commanded thrust, and elements 2--4
    % contain the commanded moments (exerted on the quadrotor body in the
    % global frame)
    
    % parse inputs
    T_ref = varargin{1} ;
    Z_ref = varargin{3} ;
    R = varargin{4} ;
    a_est = varargin{5} ; % estimated acceleration
    j_est = varargin{6} ; % estimated jerk
    
    % get current states
    x = z(A.position_indices) ;
    v = z(A.velocity_indices) ;
    O = z(A.angular_velocity_indices) ;    
    
    % get current desired trajectory, which should have its states as
    % position, velocity, acceleration, jerk, and snap, written as:
    % (x,v,a,j,s) \in R^15
    z_d = match_trajectories(t,T_ref,Z_ref) ;
    x_d = z_d(1:3) ;
    v_d = z_d(4:6) ;
    a_d = z_d(7:9) ;    
    j_d = z_d(10:12) ;
    s_d = z_d(13:15) ;
    
    % get agent properties
    e1 = A.e1 ;
    e3 = A.e3 ;
    m = A.m ;
    g = A.g ;
    J = A.J ;
    Jinv = A.Jinv ;
    
    % get position error as in (17) [6]    
    e_x = x - x_d ;
    
    % get velocity error as in (18) [6]
    e_v = v - v_d ;
    
    % get b3c as in (23) with A as in Appendix F [6]
    A = -LLC.k_x*e_x - LLC.k_v*e_v + m*g*e3 + m*a_d ;
    b3c_norm = norm(A) ;
    if b3c_norm > 1e-6
        b3c = A./b3c_norm ;
    else
        b3c = e3 ;
    end
    
    % get b1c by projecting global first axis onto the plane normal to b3c
    b1d = e1 ; % we want the yaw rate to be 0, so we pick b1d = e3
    C = cross(b3c,b1d) ; % as in Appendix F
    b1c_dir = cross(b3c,C) ; % (36) [6]
    b1c_norm = norm(b1c_dir) ;
    if b1c_norm > 1e-6
        b1c = -b1c_dir./b1c_norm ;
    else
        b1c = [1;0;0] ;
    end
    
    % bet b2d from b1d and b3c
    b2c = cross(b3c,b1c) ;
    
    % get desired attitude with (22) [6]
    R_c = [b1c b2c b3c] ;
    
    % get the time derivatives of A and C
    Ad = -LLC.k_x*e_v - LLC.k_v*(a_est - a_d) + m*j_d ;
    b3cd = -Ad/b3c_norm + ((A'*Ad)/b3c_norm^3)*A ;
    Cd = cross(b3cd,b1d) ;
    b2cd = -Cd/norm(C) + ((C'*Cd)/norm(C)^3)*C ;
    b1cd = cross(b2cd,b3c) + cross(b2c,b3cd) ;
    
    % get the second order time derivatives of A and C
    Add = -LLC.k_x*(a_est - a_d) - LLC.k_v*(j_est - j_d) + m*s_d ;
    Cdd = cross(b3cd,b1d) ;
    b3cdd = -Add/norm(Add) + ((2*A'*Ad)/norm(A)^3)*Ad + ...
            + ((norm(Ad)^2 + A'*Add)/norm(A)^3)*A - 3*(((A'*Ad)^2)/(norm(A)^5))*A ;
    b2cdd = -Cdd/norm(Cdd) + ((2*C'*Cd)/norm(C)^3)*Cd + ...
            + ((norm(Cd)^2 + C'*Cdd)/norm(C)^3)*C - 3*(((C'*Cd)^2)/(norm(C)^5))*C ;
	b1cdd = cross(b2cdd,b3c) + 2*cross(b2cd,b3cd) + cross(b2c,b3cdd) ;
    
    % get attitude rate and acceleration
    Rd_c = [b1cd,b2cd,b3cd] ;
    Rdd_c = [b1cdd,b2cdd,b3cdd] ;
    
    % get body rate and acceleration
    O_c_mat = R_c'*Rd_c ;
    O_c_mat([1 5 9]) = 0 ;
    O_c = unskew(O_c_mat) ;
    
    Od_c_mat = R_c'*Rdd_c - skew(O_c)^2 ;
    Od_c_mat([1 5 9]) = 0 ;
    Od_c = unskew(Od_c_mat) ;
    
%% compute control inputs
    % get desired thrust vector as in (19)
    f = m*A'*R(:,3) ;
    
    % compute attitude error
    e_R = 0.5*unskew(R_c'*R - R'*R_c) ;
    e_O = O - R'*R_c*O_c ;
    
    % compute desired moment as in (20)
    M = -LLC.k_R*e_R - LLC.k_O*e_O + cross(O,J*O) + ...
        -J*(skew(O)*R'*R_c*O_c -R'*R_c*Od_c) ;
    
    % output desired force and moment
    U = [f ; M] ;
    
end
end
end