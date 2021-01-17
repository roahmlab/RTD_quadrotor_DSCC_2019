function state_dot = QC_dyn_motion_prim(t,state,params)
% extract state (x,y,z,xd,yd,zd,R(:))
    x = state(1) ;
    y = state(2) ;
    z = state(3) ;
    xd = state(4) ;
    yd = state(5) ;
    zd = state(6) ;
    R = state(7:end) ; % 9-by-1
end