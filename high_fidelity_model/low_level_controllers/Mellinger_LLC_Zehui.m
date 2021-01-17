classdef Mellinger_LLC_Zehui < low_level_controller
    % Adapted from: http://lewissoft.com/assets/pdf/uPennTrajectoryGeneration.pdf
properties
    % these gain matrices were tuned by hand for the quadrotor_agent
%     K_p = 4.5*eye(3) ;
%     K_v = 1.1*eye(3) ;
%     K_R = 0.15*eye(3) ;
%     K_w = 0.01*eye(3) ;

    K_p = 4.5*eye(3) ;
    K_v = 1.1*eye(3) ;
    K_R = 0.2*eye(3) ;
    K_w = 0.01*eye(3) ;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    K_I_p = 0.00005*eye(3) ;
    K_I_R = 0.00005*eye(3) ;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end

methods
%% constructor
    function LLC = Mellinger_LLC_Zehui(varargin)
        LLC@low_level_controller(varargin{:}) ;
    end
    
%% get control inputs
function U = get_control_inputs(LLC,A,t,z,varargin)
    % method: U = get_control_inputs(LLC,A,t,z,T_ref,U_ref,Z_ref,R)
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    global e_accumulated_p e_accumulated_R
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % parse inputs
    T_ref = varargin{1} ;
    Z_ref = varargin{3} ;
    R = varargin{4} ;
    
    % get agent properties
    m = A.m ;
    g = A.g*A.gravity_direction ;
    
    % get current states
    x = z(A.position_indices) ;
    v = z(A.velocity_indices) ;
    O = z(A.angular_velocity_indices) ;
    
    % get current desired trajectory, which should have its states as
    % position, velocity, acceleration, jerk, and snap, written as
    % (x,v,a,j,s) \in R^15; note that the reference yaw is 0
    z_d = match_trajectories(t,T_ref,Z_ref) ;
    x_d = z_d(1:3) ;
    v_d = z_d(4:6) ;
    a_d = z_d(7:9) ;
    j_d = z_d(10:12) ;
    s_d = z_d(13:15) ;
    
    % desired body frame z axis
    e_p = x - x_d ;
    e_v = v - v_d ;
%     F_d = -LLC.K_p*e_p - LLC.K_v*e_v - m*g + m*a_d ; % desired body force
    
    %%%%%%%%%%%%%%
    e_accumulated_p = e_accumulated_p + e_p ;
    F_d = -LLC.K_p*e_p - LLC.K_v*e_v - LLC.K_I_p*e_accumulated_p - m*g + m*a_d ; % desired body force
    %%%%%%%%%%%%%%
    
    z_B_d = F_d/norm(F_d) ;
    
    % desired thrust force
    z_B = R(:,3) ;
    u_1 = F_d'*z_B ; % desired net body force
    
    % desired world x axis assuming yaw ref is 0
    x_C_d = [1;0;0] ;
    
    % desired body y axis
    y_B_d_dir = cross(z_B_d,x_C_d) ;
    y_B_d = y_B_d_dir/norm(y_B_d_dir) ;
    
    % desired body x axis
    x_B_d = cross(y_B_d,z_B_d) ;
    
    % desired attitude
    R_d = [x_B_d,y_B_d,z_B_d] ;
    
    % orientation error
    e_R = 0.5*unskew(R_d'*R - R'*R_d) ;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % e_R is 3 by 1
    e_accumulated_R = e_accumulated_R + e_R ;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    
    % desired angular velocity
    h_w = (m/u_1)*(j_d - (z_B_d'*j_d)*z_B_d) ;
    p_d = -h_w'*y_B_d ;
    q_d = h_w'*x_B_d ;
    r_d = 0 ; % since desired yaw rate is 0
    O_d = [p_d;q_d;r_d] ;
    
    % angular velocity error
    e_w = O - O_d ;
    
    % compute desired moments
%     M_des = -LLC.K_R*e_R - LLC.K_w*e_w ;
    
    %%%%%%%%%%%%%%%%
    M_des = -LLC.K_R*e_R - LLC.K_w*e_w - LLC.K_I_R*e_accumulated_R ;
    %%%%%%%%%%%%%%%%
    
    % output!
    U = [u_1 ; M_des] ;
end
end
end