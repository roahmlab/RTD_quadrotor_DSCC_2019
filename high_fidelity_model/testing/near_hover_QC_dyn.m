function dzdt = quadcopter_dynamics(~,z,u,params)
% extract states, z = (px,vx,tx,ox,py,vy,ty,oy,pz,vz)
    vx = z(2) ;
    tx = z(3) ;
    ox = z(4) ;
    vy = z(6) ;
    ty = z(7) ;
    oy = z(8) ;
    vz = z(10) ;

% extract inputs
    Sx = u(1) ;
    Sy = u(2) ;
    Tz = u(3) ;
    
% extract system parameters
    g = params.gravity ;
    n0 = params.angle_gain ;
    kT = params.thrust_gain ;   
    
% compute dynamics
    dzdt = [vx ;
            g*tan(tx) ;
            ox ;
            n0*Sx ;
            vy ;
            g*tan(ty) ;
            oy ;
            n0*Sy ;
            vz ;
            kT*Tz - g] ; 
end