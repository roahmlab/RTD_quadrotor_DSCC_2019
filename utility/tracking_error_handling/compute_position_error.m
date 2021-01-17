function E = compute_position_error(A,T_ref,Z_ref)
    % extract agent info
    T = A.time ;
    Z = A.state ;

    % get position trajectories
    Z = match_trajectories(T_ref,T,Z) ;
    X_ref = Z_ref(1:3,:) ;
    X = Z(1:3,:) ;

    % compute position error
    E = X_ref - X ;
end