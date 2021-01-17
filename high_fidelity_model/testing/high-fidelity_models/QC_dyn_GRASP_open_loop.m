% from http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.169.1687&rep=rep1&type=pdf
% and http://lewissoft.com/assets/pdf/uPennTrajectoryGeneration.pdf
function state_dot = QC_dyn_GRASP_open_loop(time,state,T_in,U_in,params)
    % extract states (x,y,z,phi,theta,psi,xd,yd,zd,p,q,r)
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
    L = params.L ;
    I = params.I ;
    k_F = params.k_F ;
    k_M = params.k_M ;
    max_rotor_speed = params.max_rotor_speed ;
    min_rotor_speed = params.min_rotor_speed ;
    
    I_xx = I(1,1) ; I_yy = I(2,2) ; I_zz = I(3,3) ;
    
% get commanded inputs
    % get desired traj
    rotor_speeds = matchTrajectories(time,T_in,U_in) ;
    
    % get rotation matrix
    cph = cos(ph) ; cth = cos(th) ; cps = cos(ps) ;
    sph = sin(ph) ; sth = sin(th) ; sps = sin(ps) ;
    
    R = [cps*cth - sph*sps*sth, -cph*sps, cps*sth + cth*sph*sps ;
         cth*sps + cps*sph*sth,  cph*cps, sps*sth - cps*cth*sph ;
         -cph*sth,               sph,     cph*cth] ;
     
% compute generalized inputs
    rotor_speeds = boundValues(rotor_speeds,min_rotor_speed,max_rotor_speed) ;
    w2 = rotor_speeds.^2 ;
    
    F = k_F.*w2 ;
%     M = k_M.*rotor_speeds.^2 ;
    
% compute dynamics
    R_att = [cth 0 -cph*sth ;
             0   1 sph ;
             sth 0 cph*cth] ;
         
    pos_dot = vel ;
	vel_dot = [0;0;-g] + (1/m).*R*[0;0;sum(F)] ;
    att_dot = pinv(R_att)*[p;q;r] ;
    
    % note that I is a diagonal matrix so the cross term in (2) is 0
%     atv_dot = pinv(I)*[L*(F(2)-F(4)) ; L*(F(3)-F(1)) ; M(1)-M(2)+M(3)-M(4)] ;
    p_dot = (1/I_xx)*(L*k_F*(w2(2) - w2(4)) - q*r*(I_zz - I_yy)) ;
    q_dot = (1/I_yy)*(L*k_F*(w2(3) - w2(1)) - p*r*(I_xx - I_zz)) ;
    r_dot = (1/I_zz)*k_M*(w2(1) - w2(2) + w2(3) - w2(4)) ;
    
    atv_dot = [p_dot ; q_dot ; r_dot] ;
    
    atv_des_dot = [0;0;0];
    
    state_dot = [pos_dot ;
                 att_dot ;
                 vel_dot ;
                 atv_dot ;
                 atv_des_dot] ;                 
end