% from http://lewissoft.com/assets/pdf/uPennTrajectoryGeneration.pdf and
% http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.169.1687&rep=rep1&type=pdf
function state_dot = QC_dyn_flat(time,state,T_in,U_in,Z_in,params)
    % extract states (x,y,z,phi,theta,psi,xd,yd,zd,p,q,r)
    pos = state(1:3) ;
    ph = state(4) ;
    th = state(5) ;
    ps = state(6) ;
    vel = state(7:9) ;
    p = state(10) ;
    q = state(11) ;
    r = state(12) ;
    
    % extract params
    m = params.m ;
    g = params.g ;
    K_pos = params.K_pos ;
    K_vel = params.K_vel ;
    K_R = params.K_R ;
    K_w = params.K_w ;
    
    % extract desired traj
    Z_des = matchTrajectories(time,T_in,Z_in) ;
    pos_des = Z_des(1:3) ;
    vel_des = Z_des(4:6) ;
    acc_des = Z_des(7:9) ;
    ps_des = Z_des(10) ; % should be 0
    
    % get actual body frame
    cph = cos(ph) ; cth = cos(th) ; cps = cos(ps) ;
    sph = sin(ph) ; sth = sin(th) ; sps = sin(ps) ;
    
%     % rotation matrices:
%     Rps = [cps -sps 0 ;
%            sps  cps 0 ;
%            0    0   1] ;
%     Rth = [cth  0 sth ;
%            0    1 0 ;
%            -sth 0 cth] ;
%     Rph = [1 0 0 ;
%            0 cph -sph ;
%            0 sph cph] ;
%     R_B2W = Rps*Rph*Rth ;
    
    R_B2W = [cps*cth - sph*sps*sth, -cph*sps, cps*sth + cth*sph*sps ;
             cth*sps + cps*sph*sth,  cph*cps, sps*sth - cps*cth*sph ;
             -cph*sth,               sph,     cph*cth] ;
    x_B = R_B2W(:,1) ;
    y_B = R_B2W(:,2) ;
    z_B = R_B2W(:,3) ;
    
    % get position and velocity error
    e_pos = pos - pos_des ;
    e_vel = vel - vel_des ;
    
    % get desired force vector
    z_W = [0;0;1] ;
    F_des = -K_pos*e_pos - K_vel*e_vel + m*g*z_W + m*acc_des ;
    
    % get desired body frame axes in inertial frame
    t = acc_des + [0;0;g] ;
    if norm(t) > 1e-3
        z_B_des = t./norm(t) ;
    else
        z_B_des = z_W ;
    end
    x_C_des = [cos(ps_des); sin(ps_des); 0] ;
    z_B_x_x_C = cross(z_B_des,x_C_des) ;
    y_B_des = z_B_x_x_C./norm(z_B_x_x_C) ;
    x_B_des = cross(y_B_des,z_B_des) ;
    
    % get desired rotation matrix
    R_des = [x_B_des, y_B_des, z_B_des] ;
    
    % get first input
    u_1 = F_des'*z_B_des ;
    
    % get error in orientation
    e_R_mat = 0.5*(R_des'*R_B2W - R_B2W'*R_des) ;
    e_R = [e_R_mat(3,2) ; e_R_mat(1,3) ; e_R_mat(2,1)] ;
    
    % get angular velocity
    w_BW = p*x_B + q*y_B + r*z_B ;
    
    % get angular velocity error
    w_BW_des = p*x_B_des + q*y_B_des + r*z_B_des ;
    e_w = R_B2W'*(w_BW - w_BW_des) ;
    
    % get remaining inputs
    u_234 = -K_R*e_R - K_w*e_w ;
    u_2 = u_234(1) ;
    u_3 = u_234(2) ;
    u_4 = u_234(3) ;
    
    % compute desired rotor speeds squared
    error('Left off here!')
end