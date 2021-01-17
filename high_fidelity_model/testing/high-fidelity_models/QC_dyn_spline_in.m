function state_d = QC_dyn_spline_in(time,state,T,U,Z)
    % extract states (x,y,z,xd,yd,zd,phi,theta,phid,thetad)
    xd = state(4) ; 
    yd = state(5) ;
    zd = state(6) ;
    ph = state(7) ;
    th = state(8) ;
    
    % compute rotation matrix
    cph = cos(ph) ; sph = sin(ph) ; cth = cos(th) ; sth = sin(th) ;
    R = [cth, sth*sph-cph, sth*cph+sph;
         cth, sth*sph+cph, sth*cph-sph ;
         -sth, cth*sph, cth*cph] ;
    
    % interpolate input and desired traj
    [U_cur,Z_cur] = matchTrajectories(time,T,U,T,Z) ;
    
    % compute accelerations due to current thrust
    state_dd = R*[0;0;1]*U_cur - [0;0;9.81] ;
    xdd = state_dd(1) ;
    ydd = state_dd(2) ;
    zdd = state_dd(3) ;    
    
    % compute body rates
    j_cur = Z_cur(10:12) ;
    w = (1/U_cur)*diag([1 1 0])*R'*j_cur ;
    
    w1 = -w(2) ;
    w2 = w(1) ;
    
    w = R*[w1;w2;0] ;
    w1 = w(1) ; w2 = w(2) ;
    
%     % create w^x matrix
%     wx = [0 0 w2 ;
%           0 0 -w1 ;
%           -w2 w1 0] ;
      
    % compute dynamics
    state_d = [xd ; yd ; zd ;
               xdd ; ydd ; zdd ;
               w1 ; w2] ;
    
end

