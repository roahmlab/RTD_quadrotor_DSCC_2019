function dyn = quadcopter_dynamics_PID(t,state,t_des,traj_des,QC_params,PID_params)
% extract states, z = (px,vx,tx,ox,py,vy,ty,oy,pz,vz)
    px = state(1) ;
    vx = state(2) ;
    tx = state(3) ;
    ox = state(4) ;
    py = state(5) ;
    vy = state(6) ;
    ty = state(7) ;
    oy = state(8) ;
    pz = state(9) ;
    vz = state(10) ;
    
% extract error states e = (ex,ey,ez) appended to end of dynamics
    ex = state(11) ;
    ey = state(12) ;
    ez = state(13) ;
    
% extract system parameters
    g = QC_params.gravity ;
    n0 = QC_params.angle_gain ;
    kT = QC_params.thrust_gain ;   
    
% extract PID parameters
    kPx = PID_params.kPx ;
    kPy = PID_params.kPy ;
    kPz = PID_params.kPz ;
    
    kDx = PID_params.kDx ;
    kDy = PID_params.kDy ;
    kDz = PID_params.kDz ;
    
    kIx = PID_params.kIx ;
    kIy = PID_params.kIy ;
    kIz = PID_params.kIz ;
    
% get desired trajectory
    z_des_cur = matchTrajectories(t,t_des,traj_des) ;
    
    px_des = z_des_cur(1) ;
    py_des = z_des_cur(3) ;
    pz_des = z_des_cur(5) ;
    
    vx_des = z_des_cur(2) ;
    vy_des = z_des_cur(4) ;
    vz_des = z_des_cur(6) ;
    
% compute inputs
    Sx = -kPx*(px - px_des) - kDx*(vx - vx_des) - kIx*ex ;
    Sy = -kPy*(py - py_des) - kDy*(vy - vy_des) - kIy*ey ;
    Tz = -kPz*(pz - pz_des) - kDz*(vz - vz_des) - kIz*ez + 10.5 ;
    
% saturate inputs
    Sx = boundValues(Sx, QC_params.Sx_bounds) ;
    Sy = boundValues(Sy, QC_params.Sy_bounds) ;
    Tz = boundValues(Tz, QC_params.Tz_bounds) ;
    
% compute dynamics
    dyn = [vx ;
           g*tan(tx) ;
           ox ;
           n0*Sx ;
           vy ;
           g*tan(ty) ;
           oy ;
           n0*Sy ;
           vz ;
           kT*Tz - g
           px - z_des_cur(1) ;
           py - z_des_cur(2) ;
           pz - z_des_cur(3)] ;
end