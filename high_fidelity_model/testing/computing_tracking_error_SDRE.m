%% notes
% Left off on mar 11 at 11:32 PM: something is causing a bug that makes the
% quadrotor accelerate to a much higher peak speed than we want
%
%% user parameters
% timing
t_max = 5 ; % total s for simulation
t_plan = 0.5 ;
t_peak = 1 ;
t_total = 3 ; % total time per spline 
t_after_stop = 3 ; % time after spline for stopping
dt = 0.025 ;

% initial condition bin
dt_bin = 0.1 ; % time in s for each "bin" of error
vx_bin = [3.0 3.5] ;
vy_bin = [-0.25 0.25] ;
vz_bin = [-0.25 0.25] ;

% number of extra initial conditions to test inside bin
N_rand_init_conds_to_add = 5 ;

% traj bounds
v_max = 3.75 ; % m/s
a_max = 1 ; % m/s^2

% plotting
plot_flag = true ;
plot_largest_error = true ;
plot_agent_while_running = true ;
t_bin_to_plot = t_peak ; % plot the error for the time bin containing this time if plot_largest_error is false

%% automated from here
% make agent
A = QC_SDRE_agent ;
v_cur = A.state([2 4 6]') ;
a_cur = zeros(3,1) ;

% create spline time vector
[T_des,~,~] = generate_spline_peak_speed([0;0;0],[0;0;0],[1;0;0],t_plan,t_peak,t_total,dt) ;

% create time bin vector
T_bin = 0:dt_bin:(t_total-dt_bin) ; % these are the lower bounds of the time bins
T_des_bin = zeros(size(T_des)) ;
for bin_idx = 2:length(T_bin)
    t_bin = T_bin(bin_idx) ;
    T_bin_log = T_des >= t_bin ;
    T_des_bin(T_bin_log) = T_des_bin(T_bin_log) + dt_bin ;
end

% create error structure for bins
for bin_idx = 1:length(T_bin)
    error_struct(bin_idx).t_lo = T_bin(bin_idx) ;
    error_struct(bin_idx).t_hi = error_struct(bin_idx).t_lo + dt_bin ;
    error_struct(bin_idx).points = [] ;
    error_struct(bin_idx).points_indices = [] ;
end
[~,T_bin_idx_to_plot] = min(abs(T_bin - t_bin_to_plot)) ;

% create vertices of velocity initial condition bin
[VX,VY,VZ] = ndgrid(vx_bin,vy_bin,vz_bin) ;
v_vertices = [VX(:),VY(:),VZ(:)]' ;
N_init_conds_orig = size(v_vertices,2) ;

if N_rand_init_conds_to_add > 0
    vx_add = randRange(vx_bin(1),vx_bin(2),[],[],1,N_rand_init_conds_to_add) ;
    vy_add = randRange(vy_bin(1),vy_bin(2),[],[],1,N_rand_init_conds_to_add) ;
    vz_add = randRange(vz_bin(1),vz_bin(2),[],[],1,N_rand_init_conds_to_add) ;
    v_vertices = [v_vertices, [vx_add;vy_add;vz_add]] ;
end

% create desired accelerations for each bin vertex
a_range = [-1,1] ;
[AX,AY,AZ] = ndgrid(a_range,a_range,a_range) ;
a_vertices = [AX(:),AY(:),AZ(:)]' ;
a_vertices = a_max.*a_vertices./repmat(vecnorm(a_vertices,2,1),3,1) ;

%% run loop to compute error
for v_idx = size(v_vertices,2) %1:size(v_vertices,2)
    tic
%% setup
    % reset agent to 0
    A.reset()
    
    % get current initial condition speed
    v_cur = v_vertices(:,v_idx) ;
    disp(['v: ',num2str(v_cur(:)')])
    
    % try to get agent to current initial condition
    A.feedback_type = 'P' ;
    T_des = [0,3] ;
    U_des = [v_cur,v_cur] ;
    A.move(T_des(end),T_des,U_des,[])
    
    % find state that is closest to initial condition
    v_all = A.state(A.speed_indices,:) - repmat(v_cur,1,size(A.state,2)) ;
    [~,v_closest_idx] = min(vecnorm(v_all)) ;
    Z_end = A.state(:,v_closest_idx) ;
    
    % reset position to zero and set the speeds to v_idx
    Z_end(A.position_indices) = 0 ;
%     Z_end(A.speed_indices) = v_cur ;
    
%% compute error for each acceleration
    for a_idx = 1:size(a_vertices,2)
        
        %% move agent
        % get acceleration
        a_cur = a_vertices(:,a_idx) ;
        disp(['a: ',num2str(a_cur(:)')])
        
        % generate new peak speed
        v_peak = v_cur + a_cur.*t_peak ;
        
        % bound new peak speed if necessary
        if norm(v_peak) > v_max
            v_peak = v_max.*v_peak./norm(v_peak) ;
            disp('accel adjusted for max speed')
        end
        
        % generate spline (note that initial position is 0)
        [T_des,Z_des,~] = generate_spline_peak_speed(v_cur,a_cur,v_peak,t_plan,t_peak,t_total,dt) ;

        % add braking to end of spline
        T_des = [T_des, (T_des(end)+dt):dt:(T_des(end)+t_after_stop)] ;
        Z_des = [Z_des, repmat(Z_des(:,end),1,length(T_des)-length(Z_des))] ;    

        % create nominal inputs, which are advanced by a bit
        U_des = [Z_des(4:6,4:end), zeros(3,3)] ;

        % move agent along spline
        A.feedback_type = 'PI' ;
        A.reset(Z_end) ;
        A.move(T_des(end),T_des,U_des,Z_des)
        
        %% plotting agent
        if plot_flag && plot_agent_while_running
            figure(1) ; clf ; hold on ; axis equal ;
            plot(A)
            plot3(Z_des(1,:),Z_des(2,:),Z_des(3,:),'b--')
            view(3)
            pause(0.001) ;
        end
        
        %% compute error
        % get agent's state output
        T_out = A.time ;
        Z_out = A.state ;

        % reinterpolate spline to match T_out
        Z_des = matchTrajectories(T_out,T_des,Z_des) ;
        Z_des(:,end) = Z_des(:,end-1) ;

        x_out = Z_out(1,:) ;
        y_out = Z_out(3,:) ;
        z_out = Z_out(5,:) ;

        % compute position error
        x_des = Z_des(1,:) ;
        y_des = Z_des(2,:) ;
        z_des = Z_des(3,:) ;

        x_err = x_out - x_des ;
        y_err = y_out - y_des ;
        z_err = z_out - z_des ;
        
        % for each time bin, get the boundary of the error in that bin
        largest_err_val_cur = 0 ;
        largest_err_idx = 1 ;
        for bin_idx = 1:length(T_bin)
            t_bin_idx = T_bin(bin_idx) ;
            T_bin_log = abs(T_des_bin - t_bin_idx) < 1e-6 ;
            err_in_bin = [x_err(T_bin_log) ;
                          y_err(T_bin_log) ;
                          z_err(T_bin_log) ] ;
            err_old = error_struct(bin_idx).points ;
            err_combined = [err_in_bin, err_old] ;
            if length(err_combined) > 8
                err_idxs = unique(convhull(err_combined')) ;
            else
                err_idxs = 1:size(err_combined,2) ;
            end
            error_struct(bin_idx).points = err_combined(:,err_idxs) ;
            
            % find largest error
            largest_err_val_new = max(vecnorm(err_combined,2,1)) ;
            
            if largest_err_val_new > largest_err_val_cur
                largest_err_idx = bin_idx ;
            end            
        end
        
        % if additional points are added, check if they have increased the
        % size of the convex hull
        %% LEFT OFF HERE
        
        %% plotting     
        if plot_flag            
            % plot error blobs
            if plot_largest_error
                plot_idx = largest_err_idx ;
            else
                plot_idx = T_bin_idx_to_plot ;
            end
            
            err_to_plot = error_struct(plot_idx).points ;
            idx_to_plot = convhull(err_to_plot') ;
            figure(2) ; cla ; axis equal ;
            trisurf(idx_to_plot,err_to_plot(1,:),err_to_plot(2,:),err_to_plot(3,:),'FaceColor','r','FaceAlpha','0.5')
            title(['Bin: t =[',num2str(error_struct(plot_idx).t_lo),',',num2str(error_struct(plot_idx).t_hi),']'])

            % plot error values
            figure(3) ; clf ; hold on ;
            plot(T_out,x_err,'r',T_out,y_err,'g',T_out,z_err,'b')
            legend('x','y','z')
            title('position error')
            
            % pause to refresh plots
            pause(0.001) ;
        end
    end
    toc
end