classdef INDI_LLC < low_level_controller
    % Adapted from: https://arxiv.org/pdf/1809.04048.pdf
    %
    % Control moment computed with https://arxiv.org/pdf/1003.2005.pdf
properties
    k_x = 10 ;
    k_v = 20 ;
    k_R = 0.0001 ;
    k_O = 0.0001 ;
end

methods
%% constructor
    function LLC = INDI_LLC(varargin)
        LLC@low_level_controller(varargin{:}) ;
    end
    
%% get control inputs
function U = get_control_inputs(LLC,A,t,z,varargin)
    % method: U = get_control_inputs(LLC,A,t,z,T_ref,U_ref,Z_ref,R)
    
    % parse inputs
    T_ref = varargin{1} ;
    Z_ref = varargin{3} ;
    R = varargin{4} ;
    
    % get agent properties
    e3 = A.e3 ;
    m = A.m ;
    g = A.g*A.gravity_direction ;
    J = A.J ;
    J_inv = A.Jinv ;
    
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
    
%% Tal and Karaman 2018
    % compute the desired thrust and body z-axis direction
    tau = norm(a_d - g) ;
    b_z_dir = a_d - g ;
    b_z_norm = norm(b_z_dir) ;
    if b_z_norm > 1e-6
        b_z = b_z_dir./b_z_norm ;
    else
        b_z = e3 ;
    end
    
    % compute the desired roll and pitch angles using (90) and (91), which
    % should be okay as long as the quadrotor doesn't flip
    r = asin(-b_z(2)) ;
    p = atan(b_z(1)/b_z(3)) ;
    
    % compute the desired body x- and y-axes using the desired attitude
    R_d = eul2rotm([r p 0],'XYZ') ;
    b_x = R_d(:,1) ;
    b_y = R_d(:,2) ;
    
    % compute the desired attitude rate and thrust rate using (93)
    sth = sin(p) ; sph = sin(r) ;
    cth = cos(p) ; cph = cos(r) ;
    tth = tan(p) ;
    C = [tau*(-sph*sth) tau*cph*cth sth*cph ;
         -tau*cph 0 -sph ;
         -tau*cth*sph -tau*cph*sth cth*cph] ;
    rd_pd_taud = pinv(C)*j_d ;
    rd_d = rd_pd_taud(1) ;
    pd_d = rd_pd_taud(2) ;
    taud_d = rd_pd_taud(3) ;
    
    % compute the desired attitude rate using (94)
    M = (1/tau)*[-b_y' ; b_x'./cos(r) ; zeros(1,3)] ;
    xid_d = M*j_d ;
    
    % transform to desired body rates
    S_inv = [1 0 -sth ;
            0 cph cth*sph ;
            0 -sph cph*cth] ;
    O_d = S_inv*xid_d ;
    
    % compute the desired attitude acceleration using (100) and (101)    
    S_dot = rd_d*[0 cph*tth -sph*tth ;
                     0 -sph -cph ;
                     0 cph/cth -sph/cth] + ...
            + pd_d*[0 -sph/(sth^2 - 1) cph/(cth^2) ;
                     0 0 0 ;
                     0 (-sph*sth)/(sth^2 - 1) (cph*sth)/(cth^2)] ;
    
    e = R_d*((2*taud_d + tau*skew(O_d))*O_d(3) - tau*[0 0 1]*S_inv*S_dot*O_d) ;
    if all(e(:) == 0)
        e = zeros(3,1) ;
    end
     
    xidd_d = M*s_d - diag([1 1 0])*pinv(C)*e ;
    
    % compute desired body angular acceleration with (98)
    Od_d = -S_inv*S_dot*O_d + S_inv*xidd_d ;
    
%% Lee et al 2011
    % compute attitude error
    e_R = 0.5*unskew(R_d'*R - R'*R_d) ;
    
    % compute angular velocity error
    e_O = O - R'*R_d*O_d ;
    
    % compute control moment with (11)
    M_c = -LLC.k_R*e_R - LLC.k_O*e_O + cross(O,J*O) + ...
        -J*(skew(O_d)*R'*R_d*O_d - R'*R_d*Od_d) ;
    
    % output!
    U = [m*tau ; M_c] ;
    
end
end
end