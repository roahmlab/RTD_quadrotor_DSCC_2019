% from http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.169.1687&rep=rep1&type=pdf
function state_dot = QC_dyn_GRASP(time,state,T_in,~,Z_in,params)
    % extract states (x,y,z,phi,theta,psi,xd,yd,zd,p,q,r)
    pos = state(1:3) ;
    ph = state(4) ;
    th = state(5) ;
    ps = state(6) ;
    vel = state(7:9) ;
    p = state(10) ;
    q = state(11) ;
    r = state(12) ;
    p_des = state(13) ;
    q_des = state(14) ;
    r_des = state(15) ;
    
    % extract params
    m = params.m ;
    g = params.g ;
    L = params.L ;
    I = params.I ;
    k_F = params.k_F ;
    k_M = params.k_M ;
    K_p = params.K_p ;
    K_d = params.K_d ;
    k_pph = params.k_pph ;
    k_dph = params.k_dph ;
    k_pth = params.k_pth ;
    k_dth = params.k_dph ;
    k_pps = params.k_pps ;
    k_dps = params.k_dps ;
    max_rotor_speed = params.max_rotor_speed ;
    min_rotor_speed = params.min_rotor_speed ;
    
    % compute additional params
    w_h = sqrt(m*g/(4*k_F)) ;
    I_xx = I(1,1) ;
    I_yy = I(2,2) ;
    I_zz = I(3,3) ;
    
% get desired trajectory and current error
    % get desired traj
    Z_des = matchTrajectories(time,T_in,Z_in) ;
    pos_des = Z_des(1:3) ;
    vel_des = Z_des(4:6) ;
    acc_des = Z_des(7:9) ;
    ps_des = 0 ; % should be 0
    
    % get rotation matrix
    cph = cos(ph) ; cth = cos(th) ; cps = cos(ps) ;
    sph = sin(ph) ; sth = sin(th) ; sps = sin(ps) ;
    
    R = [cps*cth - sph*sps*sth, -cph*sps, cps*sth + cth*sph*sps ;
         cth*sps + cps*sph*sth,  cph*cps, sps*sth - cps*cth*sph ;
         -cph*sth,               sph,     cph*cth] ;
    
	% get normal and binormal vectors
    acc_val = norm(acc_des) ;
    if acc_val > 1e-3
        n_hat = acc_des./acc_val ;
    else
        n_hat = [0;0;1] ;
    end
    
    vel_val = norm(vel_des) ;
    if vel_val > 1e-3
        t_hat = vel_des./vel_val ;
    else
        t_hat = [1;0;0] ;
    end
    
    b_hat = cross(t_hat,n_hat) ;
        
    % get position and velocity error
%     e_p = ((pos_des - pos)'*n_hat)*n_hat + ((pos_des - pos)'*b_hat)*b_hat ;
    e_p = pos_des - pos ;
    e_v = vel_des - vel ;
    
% compute desired attitude from inner loop
    % get commanded acceleration
    acc_cmd = K_p*e_p + K_d*e_v + acc_des ;
    
    % compute desired roll, pitch, and net force
    ph_des = (1/g)*(acc_cmd(1)*sin(ps_des) - acc_cmd(2)*cos(ps_des)) ;
    th_des = (1/g)*(acc_cmd(1)*cos(ps_des) + acc_cmd(2)*sin(ps_des)) ;
    Dw_F = (m/8*k_F*w_h)*acc_cmd(3) ;
    
    % compute attitude control inputs
    Dw_ph = k_pph*(ph_des - ph) + k_dph*(p_des - p) ;
    Dw_th = k_pth*(th_des - th) + k_dth*(q_des - q) ;
    Dw_ps = k_pps*(ps_des - ps) + k_dps*(r_des - r) ;
    
% compute rotor control inputs (outer loop)
    %  compute rotor speeds
    A = [1 0 -1 1 ;
         1 1 0 -1 ;
         1 0 1 1 ;
         1 -1 0 -1] ;
    rotor_speeds = A*[w_h+Dw_F; Dw_ph; Dw_th; Dw_ps] ;
    
    % saturate rotor speeds
    rotor_speeds = boundValues(rotor_speeds,min_rotor_speed,max_rotor_speed) ;
    
    % compute forces and moments
    F = k_F.*rotor_speeds.^2 ;
    M = k_M.*rotor_speeds.^2 ;
    
% compute dynamics
    pos_dot = vel ;
    vel_dot = [0;0;-m*g] + R*[0;0;sum(F)] ;
    R_att = [cth 0 -cph*sth ;
             0   1 sph ;
             sth 0 cph*cth] ;
    att_dot = pinv(R_att)*[p;q;r] ;
    
    % note that I is a diagonal matrix so the cross term in (2) is 0
    atv_dot = pinv(I)*[L*(F(2)-F(4)) ; L*(F(3)-F(1)) ; M(1)-M(2)+M(3)-M(4)] ;
    
    atv_des_dot = [(4*k_F*L*w_h/I_xx)*Dw_ph ;
                   (4*k_F*L*w_h/I_yy)*Dw_th ;
                   (8*k_M*w_h/I_zz)*Dw_ps ] ;
    
    state_dot = [pos_dot ;
                 att_dot ;
                 vel_dot ;
                 atv_dot ;
                 atv_des_dot] ;                 
end