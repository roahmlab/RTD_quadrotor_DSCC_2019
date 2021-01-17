function R_out = match_attitudes(t_des,t_in,R_in)
    % Given desired times t_des and an input time t_in with associated
    % attitudes R_in, interpolate between the attitudes (which are rotation
    % matrices in SO(3)) and return a 3-by-3-by-N_des array of rotation
    % matrices.

    % get attitudes into cell array format
    N_R = size(R_in,3) ;
    R_cel = mat2cell(R_in,3,3,ones(1,N_R)) ;
    
    % get euler angles for each attitude
    eul_in = cellfun(@(x) {rotm2eul(x)'},R_cel) ;
    eul_in = permute(cell2mat(eul_in),[1 3 2]) ;
    
    % interpolate euler angles
    eul_des = match_trajectories(t_des,t_in,eul_in)' ;
    
    % make cells
    N_des = length(t_des) ;
    eul_cel = mat2cell(eul_des,ones(N_des,1),3) ;
    
    % transform euler angles back to rotation matrices
    R_out_cel = permute(cellfun(@(x) {eul2rotm(x)},eul_cel),[3 2 1]) ;
    
    % create output
    R_out = cell2mat(R_out_cel) ;
end