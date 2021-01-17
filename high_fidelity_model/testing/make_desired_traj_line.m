function [t_des,traj_des] = make_desired_traj_line(traj_params)
% extract trajectory info
    t_total = traj_params.t_total ;
    t_brake = traj_params.t_brake ;
    t_sample = traj_params.t_sample ;
    
% extract traj parameters
    kx = traj_params.kx ;
    ky = traj_params.ky ;
    kz = traj_params.kz ;
    
% get timing
    t_move = t_total - t_brake ;
    t_move_vec = 0:t_sample:t_move ;
    t_brake_vec = (t_move+t_sample):t_sample:t_total ;
    t_des = [t_move_vec,t_brake_vec] ;
    
% make traj states (x,vx,y,vy,z,vz) ;
    vx_vec = [kx.*ones(1,length(t_move_vec)-1), linspace(kx,0,length(t_brake_vec)+1)] ;
    vy_vec = [ky.*ones(1,length(t_move_vec)-1), linspace(ky,0,length(t_brake_vec)+1)] ;
    vz_vec = [kz.*ones(1,length(t_move_vec)-1), linspace(kz,0,length(t_brake_vec)+1)] ;
    
    px_vec = cumsum(t_sample.*vx_vec) ;
    py_vec = cumsum(t_sample.*vy_vec) ;
    pz_vec = cumsum(t_sample.*vz_vec) ;
    
% compile output
    traj_des = [px_vec ;
                vx_vec ;
                py_vec ;
                vy_vec ;
                pz_vec ;
                vz_vec] ;    
end