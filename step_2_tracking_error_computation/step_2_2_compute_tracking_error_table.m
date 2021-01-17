%% user parameters
% file i/o
filename = 'quadrotor_tracking_error_table_dt0.02_v5.25.mat' ;

% initial acceleration
a_0 = zeros(3,1) ;

% timing
t_move  = 5 ;
t_plan  = 0.75 ;
t_peak  = 1.0 ;
t_total = 3 ;
t_extra = 15 ;
dt_ref  = 0.001 ;
dt_int  = 0.01 ;

%% automated from here
% load file
f = load(filename) ;
tbl = f.tracking_error_table ;
dv = f.dv ;
v_max = f.v_max ;
a_max = f.a_max ;
v_c = f.bin_center_velocities ;
N_v = size(v_c,1) ;

% preallocate error storage
v_idx = v_c(1,:) ;
T_log = query_tracking_error_table(tbl,v_idx) ;
N_t_bins = sum(T_log) ;
Ex_lo_all = zeros(N_t_bins,N_v) ;
Ey_lo_all = zeros(N_t_bins,N_v) ;
Ez_lo_all = zeros(N_t_bins,N_v) ;
Ex_hi_all = zeros(N_t_bins,N_v) ;
Ey_hi_all = zeros(N_t_bins,N_v) ;
Ez_hi_all = zeros(N_t_bins,N_v) ;

%% run loop
% for each bin center...
start_tic = tic ;
parfor bin_c_idx = 1:N_v
    disp(['Bin ',num2str(bin_c_idx)])
    
    loop_tic_cur = tic ;
    
    % get the vertices of the bin
    v_idx = v_c(bin_c_idx,:) ;
    
    % get the relevant bins of the table
    T_log = query_tracking_error_table(tbl,v_idx) ;
    T_idx = tbl(T_log,:) ;
    
    % get the time bins
    t_lo = tbl.t_lo(T_log) ;
    t_hi = tbl.t_hi(T_log) ;
    
    % get the eight vertices of the current bin
    [~,v_verts] = make_cuboid_for_patch(dv,dv,dv,v_idx) ;
    
    % set up error for comparison
    Ex_lo_cur = zeros(N_t_bins,1) ;
    Ey_lo_cur = zeros(N_t_bins,1) ;
    Ez_lo_cur = zeros(N_t_bins,1) ;
    Ex_hi_cur = zeros(N_t_bins,1) ;
    Ey_hi_cur = zeros(N_t_bins,1) ;
    Ez_hi_cur = zeros(N_t_bins,1) ;
    
    % set up quadrotor agent
    A = quadrotor_agent() ;
    A.integrator_time_discretization = dt_int ;

    % create acceleration cuboid
    [~,a_verts] = make_cuboid_for_patch() ;
    a_verts = a_verts./repmat(vecnorm(a_verts,2,2),1,3) ;
    a_verts = a_verts.*a_max ;
    
    % use each vertex as an initial velocity...
    for v_vert_idx = 1:8
        % get the initial velocity
        v0_idx = v_verts(v_vert_idx,:) ;
        v0_idx = v0_idx(:) ;
        
        % use each acceleration to generate a new peak speed...
        for a_vert_idx = 1:8
            % get current desired acceleration
            a_idx = a_verts(a_vert_idx,:) ;
            a_idx = a_idx(:) ;
            
            % make peak speed
            v_peak = v0_idx + t_peak*a_idx ;
            if norm(v_peak) > v_max
                v_peak = (v_peak./norm(v_peak)).*v_max ;
            end
            v_peak = v_peak(:) ;
            
            % generate spline (notice that a_0 = 0)
            [T_ref,Z_ref] = generate_spline_peak_speed(v0_idx,a_0,v_peak,...
                                t_plan,t_peak,t_total,dt_ref,t_extra) ;
            U_ref = zeros(4,length(T_ref)) ;
            
            % reset the agent
            z0 = [zeros(3,1) ; v0_idx(:) ; zeros(3,1)] ;
            A.reset(z0)
            
            % move agent
            A.move(t_move,T_ref,U_ref,Z_ref)
            
            % compute new error bounds
            E_cur = compute_position_error(A,T_ref,Z_ref) ;
            [E_lo, E_hi] = put_error_in_time_bins(E_cur,T_ref,t_lo(:),t_hi(:)) ;
            ex_lo_idx = E_lo(:,1) ;
            ey_lo_idx = E_lo(:,2) ;
            ez_lo_idx = E_lo(:,3) ;
            ex_hi_idx = E_hi(:,1) ;
            ey_hi_idx = E_hi(:,2) ;
            ez_hi_idx = E_hi(:,3) ;
            
            % get new error bounds (max and min for each time interval over
            % all of the current v_peaks for the given v0_idx)
            Ex_lo_cur = min([ex_lo_idx,Ex_lo_cur],[],2) ;
            Ey_lo_cur = min([ey_lo_idx,Ey_lo_cur],[],2) ;
            Ez_lo_cur = min([ez_lo_idx,Ez_lo_cur],[],2) ;
            Ex_hi_cur = max([ex_hi_idx,Ex_hi_cur],[],2) ;
            Ey_hi_cur = max([ey_hi_idx,Ey_hi_cur],[],2) ;
            Ez_hi_cur = max([ez_hi_idx,Ez_hi_cur],[],2) ;
            
            % plot if something broke (tracking error is > 20 cm)
            if min(Ex_lo_cur) < -0.2 || max(Ex_hi_cur) > 0.2
                clf
                plot(A)
                axis equal
            end
        end
    end
    
    % update error in preallocated arrays; now we save the tracking error
    % max and min for ALL of the initial velocities and ALL corresponding
    % peak velocities of the current bins (i.e., the current interval of
    % initial velocities and each corresponding time interval)
    Ex_lo_all(:,bin_c_idx) = Ex_lo_cur ;
    Ey_lo_all(:,bin_c_idx) = Ey_lo_cur ;
    Ez_lo_all(:,bin_c_idx) = Ez_lo_cur ;
    Ex_hi_all(:,bin_c_idx) = Ex_hi_cur ;
    Ey_hi_all(:,bin_c_idx) = Ey_hi_cur ;
    Ez_hi_all(:,bin_c_idx) = Ez_hi_cur ;
    
    % timing
    toc(loop_tic_cur)
end
toc(start_tic)

%% update table
disp('Updating error table')

for bin_c_idx = 1:N_v    
    % get the vertices of the bin
    v_idx = v_c(bin_c_idx,:) ;
    
    % get the relevant bins of the table
    T_log = query_tracking_error_table(tbl,v_idx) ;
    
    % update with the found error
    tbl.ex_lo(T_log) = Ex_lo_all(:,bin_c_idx) ;
    tbl.ey_lo(T_log) = Ey_lo_all(:,bin_c_idx) ;
    tbl.ez_lo(T_log) = Ez_lo_all(:,bin_c_idx) ;
    tbl.ex_hi(T_log) = Ex_hi_all(:,bin_c_idx) ;
    tbl.ey_hi(T_log) = Ey_hi_all(:,bin_c_idx) ;
    tbl.ez_hi(T_log) = Ez_hi_all(:,bin_c_idx) ;    
end

%% save
save('temporary_updated_error_table.mat','tbl')