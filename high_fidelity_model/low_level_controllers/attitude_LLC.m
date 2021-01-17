classdef attitude_LLC < low_level_controller
    % Adapted from: https://arxiv.org/pdf/1003.2005.pdf
properties
    % these gain matrices were tuned by hand for the quadrotor_agent
    K_p = 2*eye(3) ;
    K_v = 0.5*eye(3) ;
    K_R = 1*eye(3) ;
    K_w = 0.03*eye(3) ;
end

methods
%% constructor
    function LLC = attitude_LLC(varargin)
        LLC@low_level_controller(varargin{:}) ;
    end
    
%% get control inputs
function U = get_control_inputs(LLC,A,t,z,varargin)
    % method: U = get_control_inputs(LLC,A,t,z,T_ref,U_ref,Z_ref,R)
    
    % parse inputs
%     T_ref = varargin{1} ;
%     Z_ref = varargin{3} ;
    U_ref = varargin{2} ;
    flip_rate = U_ref(1) ;
    flip_axis = U_ref(2:4) ;
    R = varargin{4} ;
    
    % get agent properties
    m = A.body_mass ;
    J = A.body_inertia_matrix ;
    Jinv = A.body_inertia_matrix_inverse ;
    g = A.gravity_acceleration*A.gravity_direction;
    
    % get current states
%     x = z(A.position_indices) ;
%     v = z(A.velocity_indices) ;
    O = z(A.angular_velocity_indices) ;
    
    % get current desired trajectory, which should have its states as
    % position, velocity, acceleration and angular velocity
%     z_d = match_trajectories(t,T_ref,Z_ref,'previous') ;
%     x_d = z_d(1:3) ;c
%     v_d = z_d(4:6) ;
%     a_d = z_d(7:9) ;
%     j_d = z_d(10:12) ;
    
    % get desired rotation matrix
    
    R_d = expm(flip_rate*t*skew(flip_axis)) ;
    Rd_d = flip_rate*skew(flip_axis)*R_d ;
    O_d = unskew(R_d'*Rd_d) ;
    
%     % desired body frame z axis
%     e_p = x - x_d ;
%     e_v = v - v_d ;
%     F_d = -LLC.K_p*e_p - LLC.K_v*e_v - m*g + m*a_d ; % desired body force
%     z_B_d = F_d/norm(F_d) ;
%     
%     % desired thrust force
%     z_B = R(:,3) ;
%     u_1 = F_d'*z_B ; % desired net body force
%     
%     % desired world x axis assuming yaw ref is 0
%     x_C_d = R_d(:,1) ;
%     
%     % desired body y axis
%     y_B_d_dir = cross(z_B_d,x_C_d) ;
%     y_B_d = y_B_d_dir/norm(y_B_d_dir) ;
%     
%     % desired body x axis
%     x_B_d = cross(y_B_d,z_B_d) ;
%     
%     % desired attitude
%     R_d = [x_B_d,y_B_d,z_B_d] ;

    % default thrust
    u_1 = 0.5*norm(m*g)  ;
    
    % orientation error
    e_R = 0.5*unskew(R_d'*R - R'*R_d) ;
    
    % angular velocity error
    e_w = O - O_d ;
    
    % compute desired moments
    M_des = -LLC.K_R*e_R - LLC.K_w*e_w ;
    
    % output!
    U = [u_1 ; M_des] ;
end
end
end