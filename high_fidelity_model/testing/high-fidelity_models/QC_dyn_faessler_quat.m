% from http://rpg.ifi.uzh.ch/docs/ICRA15_Faessler.pdf
function state_dot = QC_dyn_faessler_quat(time,state,T_in,Z_in,params)
% extract states
    r = state(1:3) ; % position (x,y,z)
    v = state(4:6) ; % velocity (vx,vy,vz)
    q = state(7:10) ; % quaternion for rotation (qw,qx,qy,qz)
    o = state(11:13) ; % angular velocity in body coords (p,q,r)
    
% extract params
    % physical properties
    m = params.m ;
    g = params.g ;
    J = params.J ;
    J_inv = params.J_inv ;
    
    % control gains
    p_xy = params.p_xy ;
    d_xy = params.d_xy ;
    p_z = params.p_z ;
    d_z = params.d_z ;
    p_rp = params.p_rp ;
    p_yaw = params.p_yaw ;
    p_pq = params.p_pq ;
    p_r = params.p_r ;
    
    P_pos = diag([p_xy,p_xy,p_z]) ;
    D_pos = diag([d_xy,d_xy,d_z]) ;
    
% get reference trajectory at current time
    Z_ref = matchTrajectories(time,T_in,Z_in) ;
    r_ref = Z_ref(1:3) ;
    v_ref = Z_ref(4:6) ;
    a_ref = Z_ref(7:9) ;
    % ps_ref = 0 ; % no desired yaw
    
% get current and desired body z axis in world frame
    eB_z = quat2RotMatrix(q)*[0;0;1] ; % (10)
    
    a_tot = norm(a_ref) ;
    if a_tot > 1e-6
        eB_z_des = a_ref./a_tot ;
    else
        eB_z_des = [0;0;1] ; % by default
    end
    
% compute error quaternion for rotation to align W_e_z with W_e_z_des
    al = acos(eB_z'*eB_z_des) ; % (11)
    e_cross_e = cross(eB_z,eB_z_des) ;
    if al > 1e-6
        n = e_cross_e./norm(e_cross_e) ;
        B_n = quat2RotMatrix(quatInverse(q))*n ;
        q_erp = [cos(al/2) ; B_n.*sin(al/2)] ; % (14)
    else
        q_erp = [1;0;0;0] ; % if rotation axis is undetermined
    end
    
% compute desired roll, pitch, and yaw rates
    % roll and pitch
    pq_des = sign(q_erp(1))*2*p_rp.*q_erp(2:3) ; % (15)
    
    % compute intermediate coordinates C
    eC_x = [1;0;0] ; % recall ps_ref = 0
    eC_y = [0;1;0] ;
    
    % compute desired body x and y axes
    eC_cross_eB_z = cross(eC_y,eB_z_des) ;
    n_eC_cross_eB_z = norm(eC_cross_eB_z) ;
    
    if n_eC_cross_eB_z > 1e-6
        % desired body x, with sign choice
        eB_x_des = sign(eB_z_des(3))*eC_cross_eB_z./n_eC_cross_eB_z ;
        
        % desired body y
        eB_x_cross_eB_z = cross(eB_z_des,eB_x_des) ;
        eB_y_des = eB_x_cross_eB_z./norm(eB_x_cross_eB_z) ;
        
        % get desired quaternion (note this is not the body rate q)
        q_des = rotMatrix2Quat([eB_x_des,eB_y_des,eB_z_des]) ;
        
        % error quaternion for rotation after using q_erp
        q_ey = quatMultiplication(quatInverse(quatMultiplication(q,q_erp)),q_des) ;
        
        % compute desired yaw rate
        r_des = q_ey(3) ;
    else
        r_des = 0 ;
    end
    
% compute desired torques
    % desired thrust
    c_des = a_ref'*eB_z_des ; % (9)

    % desired moments (21)
    P_att = diag([p_pq,p_pq,p_r]) ;
    O_des = [pq_des ; r_des] ;
    tau_des = J*P_att*(O_des - o) + cross(o,J*o) ;
    
    % NOTE I am skipping computing the blade speeds
    
% compute dynamics
    rd = v ;
    vd = c_des.*eB_z ;
    qd = omega2QuatSkew(o)*q ;
    od = J_inv*(tau_des - cross(o,J*o)) ;
%     od = P_att*(O_des - o) ;
    
    state_dot = [rd ; vd ; qd ; od] ;
end