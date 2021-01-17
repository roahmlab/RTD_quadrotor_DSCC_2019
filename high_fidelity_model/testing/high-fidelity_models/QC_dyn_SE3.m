% from https://arxiv.org/pdf/1304.6765.pdf
%
% Note that this should get plugged in to a regular ode solver, not a fancy
% schmancy symplectic integrator
%
% Some of this script uses "A computationally efficient motion primitive 
% for quadrotor trajectory generation" by Mueller et al., 2015, and
%
function state_dot = QC_dyn_SE3(time,state,T_input,~,Z_input,params)
%% setup
% extract states
    pos = state(1:3) ;
    vel = state(4:6) ;
    R   = reshape(state(7:15),3,3) ; det(R)
    O   = state(16:18) ;
    e_I = state(19:21) ; % integral of position error
    e_i = state(22:24) ; % integral of attitude error
    
% extract params
    m = params.m ;
    g = params.g ;
    J = params.J ;
    k_x = params.k_x ;
    k_v = params.k_v ;
    k_i = params.k_i ;
    k_R = params.k_R ;
    k_O = params.k_O ;
    k_I = params.k_I ;
    c_1 = params.c_1 ;
    c_2 = params.c_2 ;
    sg  = params.sg ;
    
% extract desired trajectory
    Z_des = matchTrajectories(time,T_input,Z_input) ;
    pos_des = Z_des(1:3) ;
    vel_des = Z_des(4:6) ;
    acc_des = Z_des(7:9) ;
    jrk_des = Z_des(10:12) ;
    snp_des = Z_des(13:15) ;
    
%% compute control inputs
  % outer loop (position control)
    % compute position and velocity tracking errors
    e_x = pos - pos_des ;
    e_v = vel - vel_des ;
    
    % compute desired thrust and direction
    e_3 = [0;0;1] ;
    f_des = norm(acc_des + g*e_3) ; % (9) of Mueller et al. 2015
    b_3c_dir = -k_x*e_x - k_v*e_v - k_i*sat(e_i,sg) + m*g*e_3 + m*acc_des ;
    b_3c_norm = norm(b_3c_dir) ;
    if b_3c_norm > 1e-6
        b_3c = b_3c_dir./b_3c_norm ; % (20)
    else
        b_3c = [0;0;1] ;
    end
    
    % compute first body-fixed axis
    b_1d = [1;0;0] ; % zero yaw rate desired
    b_1c_dir = b_1d - (b_1d'*b_3c).*b_3c ; % on plane normal to b_3c
    b_1c_norm = norm(b_1c_dir) ;
    if b_1c_norm > 1e-6
        b_1c = b_1c_dir./b_1c_norm ;
    else
        b_1c = [1;0;0] ;
    end
    
    % compute second body-fixed axis
    b_2c = cross(b_3c,b_1c) ; % (22)
    b_2c = b_2c./norm(b_2c) ; % normalize just in case
    
    % compute desired attitude
    R_des = [b_1c, b_2c, b_3c] ; % (22)
    det(R_des)
    inv_R_des = pinv(R_des) ;
    
    % compute desired angular velocity as per (12) of Mueller et al. 2015; 
    % note, this fixes the desired yaw rate to 0
    O_des = (1/f_des)*diag([1 1 0])*pinv(R)*jrk_des ;
    
    % compute desired attitude rate
    Rd_des = R_des*skew(O_des) ; % (5)
    
    % compute desired angular acceleration by taking derivative of O_des
    inv_Rd_des = -inv_R_des*Rd_des*inv_R_des ;
    Od_des = (1/f_des)*diag([1 1 0])*(inv_R_des*snp_des + inv_Rd_des*jrk_des) ;
    
  % inner loop (attitude control)
    % compute attitude error
    e_R = 0.5*unskew(R_des'*R - R'*R_des) ;
    
    % compute angular rate error
    e_O = O - R'*R_des*O_des ;
    
    % compute moment input (15)
    M = -k_R*e_R - k_O*e_O - k_I*e_I + ...
        + skew(R'*R_des*O_des)*J*R'*R_des*O_des + ...
        + J*R'*R_des*Od_des ;

%% compute dynamics
    pos_dot = vel ;
    vel_dot = (1/m)*(f_des*R*e_3) - g*e_3 ;
    R_dot = R*skew(O) ;
    O_dot = pinv(J)*(M - cross(O,J*O)) ;
    e_I_dot = e_O + c_2*e_R ;
    e_i_dot = e_v + c_1*e_x ;    
    
    state_dot = [pos_dot ;
                 vel_dot ;
                 R_dot(:) ;
                 O_dot ;
                 e_I_dot ;
                 e_i_dot] ;

end

function yout = sat(y,sg)
    yout = y ;
    yout(yout > +sg) = +sg ;
    yout(yout < -sg) = -sg ;
end