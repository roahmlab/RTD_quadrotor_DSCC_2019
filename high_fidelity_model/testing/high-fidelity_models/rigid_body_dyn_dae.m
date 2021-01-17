function state_dot = rigid_body_dyn_dae(time,state,T_input,U_input,params)
    % extract states
    v = state(4:6) ; % speed
    R = reshape(state(7:15),3,3) ; % attitude
    O = state(16:18) ; % angular speed
    
    % display R determinant
    disp(R'*R)
    disp(det(R))
    
    % extract params
    m = params.m ;
    g = params.g ;
    J = params.J ;
    
    % get current inputs
    U = matchTrajectories(time,T_input,U_input) ;
    f = U(1) ;
    M = U(2:4) ;
    
    % compute dynamics
    e3 = [0;0;1] ;
    x_dot = v ;
    v_dot = -g*e3 + f*R*e3 ;
    R_dot = R*skew(O) ;
    O_dot = pinv(J)*(M - cross(O,J*O)) ;
    
    % compute rotation matrix constraint
    c1 = R'*R - eye(3) ;
    c2 = det(R) - 1;
    
    state_dot = [x_dot ;
                 v_dot ;
                 R_dot(:) ;
                 O_dot ;
                 c1(:) ;
                 c2 ] ;
    
end