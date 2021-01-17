classdef quadrotor_zono_RTD_planner < planner
    properties
        % planning
        FRS
        FRS_time_step
        FRS_time
        FRS_N_steps
        trajopt_fmincon_options % optimoptions used for fmincon
        use_fmincon_online_flag = true ;
        add_noise_to_waypoint = true ; % helps the quadrotor get unstuck
        N_sample = 15 ; % this should be an odd number
        N_stop = 0 ; % counts how long the quadrotor has been stuck
        N_stop_threshold = 3 ; % reduce N_sample after this is reached
        use_agent_for_initial_condition = true ;
        lookahead_scale = 1 ;
        obs_dyn_buffer = 0 ;
        stop_planning_when_goal_reached_flag = true ;
        
        % planning constraints
        v_max = 2 ;
        a_max = 3 ;
        delta_v_max = 2 ;
        
        % tracking error
        tracking_error_table
        tracking_error_type = 'table' ; % 'none', 'constant', or 'table'
        tracking_error_zono = [] ;
        tracking_error_constant_value = 0.1 ;
        
        % timing
        t_peak = 1 ; % desired time to peak speed
        t_total = 3 ; % total duration of trajectory
        t_sample = 0.01 ; % time discretization
        t_extra = 2 ; % extra time added to end of trajectory so the
                      % low-level controller has a reference after the
                      % quadrotor comes to a stop
        
        % plotting
        plot_local_view = false ;
        plot_zonotope_reach_set_flag = true ;
        plot_zono_style = 'face' ; % 'face' or 'tube'
        plot_zono_face_color = 1/256*[199,234,229] ;
        plot_zono_face_opacity = 0.0 ;
        plot_zono_edge_color = 1/256*[199,234,229] ;
        plot_zono_edge_opacity = 0.5 ;
        plot_zono_edge_width = 2 ;
        plot_zono_lighting = 'flat' ;
        plot_zono_skip_idxs = 5 ; % # of time steps to skip when plotting zonos
        
        plot_obstacle_predictions_flag = false ;
        plot_obstacle_predictions_face_color = [1 0.7 0.7] ;
        plot_obstacle_predictions_face_opacity = 0.1 ;
        plot_obstacle_predictions_edge_color = [1 0.3 0.3] ;
        plot_obstacle_predictions_edge_opacity = 0 ;
        plot_obstacle_predictions_edge_width = 1 ;
        plot_obstacle_predictions_lighting = 'flat' ;
        
        plot_desired_trajectory_flag = true ;
        plot_desired_trajectory_line_width = 2 ;
    end
    
    methods
        %% constructor
        function P = quadrotor_zono_RTD_planner(FRS,tracking_error_table,varargin)
            P@planner(varargin{:}) ;
            P.FRS = FRS.Rcont ;
            P.tracking_error_table = tracking_error_table ;
            
            % try extracting timing info from FRS
            try
                P.t_peak = FRS.t_peak ;
                P.t_total = FRS.t_total ;
                P.FRS_time_step = FRS.options1.timeStep ;
                P.FRS_time = 0:P.FRS_time_step:P.t_total ;
                P.FRS_N_steps = length(P.FRS) ;
                P.vdisp('Timing info extracted from FRS',2)
            catch
                P.vdisp('Timing info not included in FRS file! Using defaults.',2)
            end
            
            % current plan
            P.current_plan.T_des = [] ;
            P.current_plan.Z_des = [] ;
            P.current_plan.v_peak = [] ;
            P.current_plan.a_est = zeros(3,1) ;
            
            % default solver options
            o = optimoptions('fmincon') ;
            o.OptimalityTolerance = 1e-3 ;
            o.MaxIterations = 100000 ;
            o.MaxFunctionEvaluations = 10000 ;
            o.SpecifyConstraintGradient = true ;
            o.SpecifyObjectiveGradient = true ;
            % o.CheckGradients = true ;
            P.trajopt_fmincon_options = o ;
            
            % create tracking error zonotope
            tracking_c = zeros(size(P.FRS{1}{1}.Z, 1), 1);
            tracking_G = zeros(size(P.FRS{1}{1}.Z, 1), 3);
            tracking_G(1, 1) = P.tracking_error_constant_value ;
            tracking_G(6, 2) = P.tracking_error_constant_value ;
            tracking_G(11, 3) = P.tracking_error_constant_value ;
            P.tracking_error_zono = zonotope([tracking_c, tracking_G]);
            
            % default high level planner
            if isempty(P.HLP)
                P.HLP = straight_line_HLP() ; 
            end
            
            % set up info struct
            P.setup_info_struct ;
        end
        
        
        %% setup
        function setup(P,agent_info,world_info)
            % planning speed constraint
            P.delta_v_max = min(P.v_max,P.t_peak*P.a_max) ;
            
            % high level planner
            P.HLP.goal = world_info.goal ;
            P.HLP.dimension = 3 ;
            P.HLP.setup(agent_info,world_info)
            
            % plot data
            P.plot_data.FRS_plot = [] ;
            P.plot_data.current_plan = [] ;
            P.plot_data.obstacles = [] ;
            P.plot_data.FRS_history = [] ;
            
            % clear current plan
            P.current_plan.T_des = [] ;
            P.current_plan.Z_des = [] ;
            
            % info
            P.current_obstacles = [] ;
            P.setup_info_struct() ;
        end
        
        function setup_info_struct(P)
            P.info.obstacles = [] ;
            P.info.t_start_plan = [] ;
            P.info.x_0 = [] ;
            P.info.v_0 = [] ;
            P.info.a_0 = [] ;
            P.info.v_peak = [] ;
            P.info.a_est = [] ;
            P.info.a_est = [] ;
            P.info.T_des = [] ;
            P.info.U_des = [] ;
            P.info.Z_des = [] ;
            P.info.FRS_plot.Faces = [] ;
            P.info.FRS_plot.Vertices = [] ;
        end
        
        %% replan
        function [T_des,U_des,Z_des] = replan(P,agent_info,world_info)    
        % do all the setup stuff required for replanning
            % start timer
            replan_start_tic = tic ;
            
            [x_0,v_0,a_0] = P.get_initial_condition(agent_info,world_info) ;
            
            % get zonotopes from world and shift them to the agent's
            % current position
            P.vdisp('Getting obstacles',4) ;
            O_zono = P.process_obstacles(world_info.obstacles,x_0) ;
            
            % get desired waypoint IN GLOBAL COORDINATES
            P.vdisp('Getting waypoint',4)
            lookahead_distance = P.lookahead_scale*P.t_peak*P.v_max ;
            x_des = P.HLP.get_waypoint(agent_info,world_info,lookahead_distance) ;
            
            % % FOR DEBUGGING:
            % if check_if_plot_is_available(P,'current_waypoint')
            %     P.plot_data.current_waypoint.XData = x_des(1) ;
            %     P.plot_data.current_waypoint.YData = x_des(2) ;
            %     P.plot_data.current_waypoint.ZData = x_des(3) ;
            % else
            %     P.plot_data.current_waypoint = plot3(x_des(1),x_des(2),x_des(3),'kp') ;
            % end
            
            % add noise to x_des if v_0 is low
            if P.add_noise_to_waypoint
                mn = 0 ;
                sd = 0.5*(7/(1+norm(v_0))^4) ;
                x_des = x_des + [rand_range(-sd,sd,mn,sd,2,1); 0] ;
                
                % FOR DEBUGGING:
                % plot3(x_des(1),x_des(2),x_des(3),'ko')
            end
            
            % move x_des to local coordinates
            x_des = x_des - x_0 ;
            
        % check if the previous plan exists, and if it reached the goal
            replan_flag = true ;
            if P.stop_planning_when_goal_reached_flag
                P.vdisp('Checking if we should replan',4)
                if ~isempty(P.current_plan.T_des)
                    T = P.current_plan.T_des ;
                    Z = P.current_plan.Z_des ;
                    
                    if any(dist_point_to_points(P.HLP.goal,Z(1:3,:)) < 0.75.*world_info.goal_radius)
                        % shift old plan by P.t_plan
                        P.vdisp('Trying to continue old plan that reached goal!',3)
                        
                        T_log = T >= P.t_move ;
                        if any(T_log)
                            % make output trajectory
                            T_des = T(T_log) ;
                            
                            if T_des(1) < P.t_move
                                T_des = [P.t_move, T_des] ;
                            end
                            
                            % get state and input for next chunk of plan
                            Z_des = match_trajectories(T_des,T,Z) ;
                            T_des = T_des - P.t_move ;
                            
                            % append extra time at end
                            T_des = [T_des, T_des(end) + P.t_move] ;
                            Z_des = [Z_des, Z_des(:,end)] ;
                        
                            % don't replan
                            replan_flag = false ;
                            v_peak = nan(3,1) ;
                            
                            P.vdisp('Continuing old plan!',2)
                        end
                    end
                end
            end
            
        % if the previous plan didn't reach the goal, then replan
            if replan_flag
                P.vdisp('Replanning',3)
                try
                    [A_con,b_con] = P.generate_constraints(x_0,v_0,a_0,O_zono) ;
                    
                    v_peak = P.trajopt(A_con,b_con,v_0,a_0,x_des,replan_start_tic) ;
                    
                    % compute desired trajectory with output
                    [T_des,Z_des,~] = generate_spline_peak_speed(v_0,a_0,v_peak,...
                        P.t_move,P.t_peak,P.t_total,P.t_sample,...
                        P.t_extra) ;
                    
                    % translate reference trajectory to where quadrotor is
                    Z_des(1:3,:) = Z_des(1:3,:) + x_0 ;
                    
                    P.vdisp('New trajectory found!',3)
                catch
                    P.vdisp('Unable to find new trajectory!',3)
                    v_peak = nan(3,1) ;
                    
                    % try to use the old plan
                    T_old = P.current_plan.T_des ;
                    T_log = T_old > P.t_move ;
                    if any(T_log)
                        % shift old plan
                        P.vdisp('Continuing previous plan',4)
                        
                        % get the remainder of the old plan
                        T_des = [P.t_move, T_old(T_log)] ;
                        Z_old = P.current_plan.Z_des ;
                        Z_des = match_trajectories(T_des,T_old,Z_old) ;
                        
                        % shift by P.t_move to make sure the new plan
                        % starts at 0
                        T_des = T_des - P.t_move;
                        
                        % make sure new plan is long enough
                        if T_des(end) < (P.t_move + P.t_extra)
                            T_des = [T_des, T_des(end)+0.01, P.t_move + P.t_extra] ;
                            Z_des = [Z_des, repmat([Z_des(1:3,end);zeros(12,1)],1,2)] ;
                        end
                    else
                        % command a hover where the quadrotor already is
                        P.vdisp('Hovering',4)
                        
                        T_des = 0:P.t_sample:(P.t_move + P.t_extra) ;
                        N_t_des = size(T_des,2) ;
                        Z_des = [repmat(x_0,1,N_t_des) ; zeros(12,N_t_des)] ;
                    end
                end
            end
            
            % create dummy control input
            N_t_des = size(T_des,2) ;
            U_des = zeros(4,N_t_des) ;
            
            % update acceleration estimate
            a_est_new = match_trajectories(P.t_plan,T_des,Z_des(7:9,:)) ;
            
            % update current plan and obstacles
            P.current_plan.T_des = T_des ;
            P.current_plan.Z_des = Z_des ;
            P.current_plan.a_est = Z_des(7:9,:) ;
            
            % update planner info
            P.info.obstacles = [P.info.obstacles, {P.current_obstacles}] ;
            P.info.t_start_plan = [P.info.t_start_plan,agent_info.time(end)] ;
            P.info.x_0 = [P.info.x_0,x_0] ;
            P.info.v_0 = [P.info.v_0,v_0] ;
            P.info.a_0 = [P.info.a_0,a_0] ;
            P.info.v_peak = [P.info.v_peak,v_peak] ;
            P.info.a_est = [P.info.a_est, a_est_new] ;
            P.info.T_des = [P.info.T_des, {T_des}] ;
            P.info.U_des = [P.info.U_des, {U_des}] ;
            P.info.Z_des = [P.info.Z_des, {Z_des}] ;
        end
        
        
        %% get initial condition
        function [x_0,v_0,a_0] = get_initial_condition(P,agent_info,world_info)
            % get the initial condition for the next traj
            if P.use_agent_for_initial_condition
                % get the agent's current state
                x_0 = agent_info.position(:,end) ;
                v_0 = agent_info.velocity(:,end) ;
                
                if ~isempty(P.current_plan.T_des)
                    a_0 = match_trajectories(P.t_move,P.current_plan.T_des,P.current_plan.a_est) ;
                else
                    a_0 = P.current_plan.a_est(:,end) ;
                end
            else
                % get the previous traj at t_plan
                if isempty(P.current_plan.T_des)
                    z_0 = [world_info.start ; zeros(6,1)] ;
                else
                    T = P.current_plan.T_des ;
                    Z = P.current_plan.Z_des ;
                    t_0 = min(P.t_move,T(end)) ;
                    z_0 = match_trajectories(t_0,T,Z) ;
                end
                
                x_0 = z_0(1:3) ;
                v_0 = z_0(4:6) ;
                a_0 = z_0(7:9) ;
                agent_info.position(:,end) = x_0 ;
                
                % FOR DEBUGGING:
                % plot3(x_0(1),x_0(2),x_0(3),'ko')
            end
        end
        
        %% constraints
        function [A_con, b_con] = generate_constraints(P,~,v_0,a_0,O_zono)
            % generate constraint matrices for trajopt
            switch P.tracking_error_type
                case 'none'
                    [A_con, b_con] = generate_quadrotor_trajopt_constraints(v_0, a_0,...
                        P.FRS, O_zono) ;
                case 'constant'
                    [A_con, b_con] = generate_quadrotor_trajopt_constraints(v_0, a_0,...
                        P.FRS, O_zono, P.tracking_error_zono) ;
                case 'table'
                    [A_con, b_con] = generate_quadrotor_trajopt_constraints(v_0, a_0,...
                        P.FRS, O_zono, P.tracking_error_table) ;
                otherwise
                    error('Tracking error type must be none, constant, or table.')
            end
        end
        
        %% processing obstacles
        function O_zono = process_obstacles(P,O,x_0)
            % get obstacle number
            N_obstacles = length(O) ;
            P.current_obstacles = O ;
            P.vdisp(['N obstacles: ',num2str(length(O))],5) ;
            
            % fill in a cell array that is the length of the FRS in time
            % steps, where each cell contains all a cell array of all the
            % zonotope obstacles at that time
            O_zono = cell(1,P.FRS_N_steps) ;

            for FRS_idx = 1:P.FRS_N_steps
                % create cell array to put into full cell array that will
                % contain each obstacle zonotope for the current time step
                O_zono_idx = cell(1,N_obstacles) ;
                
                for o_idx = 1:N_obstacles
                    % get the current obstacle at the current time step
                    o = O{o_idx} ;
                    
                    if isa(o,'zonotope_dynamic_obstacle')
                        % get the current time in the horizon [0,t_f] of
                        % the zonotope FRS, and add the obstacle's current
                        % time to it (the obstacle's current time does not
                        % update until W.collision_check is called, which
                        % is after P.replan in simulator.run)
                        t_idx = P.FRS_time(FRS_idx) + o.current_time ;
                        
                        G = o.zono.generators ;
                        % c = o.get_position_at_time(t_idx) ;
                        
                        % dilate the obstacle zonotope to cover the whole
                        % time interval, plus an additional buffer distance
                        % for safety's sake
                        if FRS_idx > 1
                            t_idx_prev = P.FRS_time(FRS_idx-1) + o.current_time ;
                            p0 = o.get_position_at_time(t_idx_prev) ;
                            p1 = o.get_position_at_time(t_idx) ;
                            
                            c = (p0 + p1)./2 ; % average position
                            dp = abs(p0 - p1)./2 ; % average change in position
                            G = diag(G) + dp + P.obs_dyn_buffer ;
                            
                            % % FOR DEBUGGING:
                            % [f,v] = make_cuboid_for_patch(2*G(1),2*G(2),2*G(3),c) ;
                            % patch('faces',f,'vertices',v,'facecolor',[1 0.7 0.7],'facealpha',0.05,'edgealpha',0.05) ;
                        else
                            c = o.get_position_at_time(t_idx) ;
                        end
                        
                        % subtract the initial location
                        c = c - x_0 ;
                        
                        % make zonotope
                        Z = zonotope([c,diag(G)]) ;
                    else
                        Z = o.zono - x_0 ;
                    end
                    
                    % fill in the obstacle cell array 
                    O_zono_idx{o_idx} = Z ;
                end
                
                % fill the FRS obstacles in
                O_zono{FRS_idx} = O_zono_idx ;
            end
        end
        
        function [F_out,V_out] = get_dynamic_obs_patch_data(P,o,t)
            % get info for plotting obstacle
            % F = o.plot_faces ;
            % V = o.plot_vertices ;
            F = o.collision_check_input_struct.faces ;
            V = o.collision_check_input_struct.vertices ;
            N = size(V,1) ;
            
            % dilate the obstacle by the buffer amount
            if P.obs_dyn_buffer > 0
                B = points_to_bounds(V') ;
                [l,w,h,~] = bounds_to_box(B) ;
                b = P.obs_dyn_buffer ;
                l = l + b ;
                w = w + b ;
                h = h + b ;
                
                % remake vertices
                Vx = l.*[0 1 1 0 0 1 1 0]' - l/2 ;
                Vy = w.*[0 0 1 1 0 0 1 1]' - w/2 ;
                Vz = h.*[0 0 0 0 1 1 1 1]' - h/2 ;
                V = [Vx Vy Vz] ;
            end
            
            % set up output
            F_out = [] ;
            V_out = [] ;
            
            cnt = 0 ;
            
            for FRS_idx = 1:P.plot_zono_skip_idxs:P.FRS_N_steps
                t_idx = P.FRS_time(FRS_idx) + t ;
                p_t = o.get_position_at_time(t_idx) ;
                V_idx = V + repmat(p_t',N,1) ;
                V_out = [V_out ; V_idx] ;
                F_out = [F_out ; F + N*cnt] ;
                cnt = cnt + 1 ;
            end
        end
        
        %% trajectory optimization
        function v_peak = trajopt(P,A_con,b_con,v_0,a_0,x_des,replan_start_tic)
            P.vdisp('Running trajopt',3)
            
            if P.use_fmincon_online_flag
                [v_peak,exitflag] = P.trajopt_fmincon(A_con,b_con,v_0,a_0,x_des,replan_start_tic) ;
            else
                [v_peak,exitflag] = P.trajopt_sample(A_con,b_con,v_0,a_0,x_des,replan_start_tic) ;
            end
            
            if exitflag <= 0
                error('Solver did not converge.')
            end
        end
        
        function [v_peak,exitflag] = trajopt_fmincon(P,A_con,b_con,v_0,a_0,x_des,replan_start_tic)
            P.vdisp('Setting up cost and constraint functions',7)
            % make cost and nonlcon funcs
            cost = @(v_peak) eval_cost(v_peak, v_0, a_0, x_des, replan_start_tic, P.t_plan) ;
            % cons = @(v_peak) eval_zono_cons(v_peak, A_con, b_con) ;
            cons = @(v_peak) eval_cons(v_peak, v_0, P.v_max, P.a_max,...
                P.t_peak, A_con, b_con,...
                replan_start_tic, P.t_plan) ;
            
            % call fmincon
            P.vdisp('Calling fmincon',7)
            epsilon = 1e-3 ;
            lb = (-5 + epsilon)*ones(3,1) ;
            ub = (5 - epsilon)*ones(3,1) ;
            
            % make initial guess towards x_des
            if vecnorm(x_des) > 0
                initial_guess = 0.25.*make_unit_length(x_des) ;
            else
                initial_guess = zeros(3,1) ;
            end
            
            
            [v_peak,~,exitflag,~] = fmincon(cost, initial_guess, [], [],...
                [], [], lb, ub, cons, P.trajopt_fmincon_options) ;
        end
        
        function [v_peak,exitflag] = trajopt_sample(P,A_con,b_con,v_0,a_0,x_des,replan_start_tic)
            % create sphere at v_0
            N_s = P.N_sample ;
            
            % check if the quadrotor has stopped too many times and reduce
            % the number of samples if so (this can help find solutions
            % faster when stuck around too many obstacles to plan quickly)
            if norm(v_0) < 1e-3
                P.N_stop = P.N_stop + 1 ;
                if P.N_stop > P.N_stop_threshold
                    N_s = 3 ;
                end
            else
                P.N_stop = 0 ;
            end

            S_v = make_v_peak_sphere(P.delta_v_max,N_s,v_0) ;
            
            error_if_out_of_time(replan_start_tic,P.t_plan)
            
            % remove v > v_max
            S_v_log = vecnorm(S_v) <= P.v_max ;
            S_v = S_v(:,S_v_log) ;
            
            error_if_out_of_time(replan_start_tic,P.t_plan)
            
            % figure(3) ; cla ; hold on ; axis equal

            % evaluate constraints
            if ~isempty(A_con)
                C_eval = A_con*S_v + b_con ;
                C_rshp = reshape(C_eval,6,[]) ;
                C_min = min(C_rshp,[],1) + 1e-6;
                C_rshp_2 = reshape(C_min,size(C_eval,1)/6,[]) ;
                C_max = max(C_rshp_2,[],1) ;
                C_log = C_max < 0 ;
                S_v = S_v(:,C_log) ;
            end
            
            
            error_if_out_of_time(replan_start_tic,P.t_plan)
            
            % plot3(S_v(1,:),S_v(2,:),S_v(3,:),'b.') ;
            
            % evaluate cost (note that t_peak is hard-coded in here)
            p_peak = pos_quadrotor_peak(v_0,a_0,S_v) ;
            J_vals = vecnorm(p_peak - x_des) ;
            [~,min_idx] = min(J_vals) ;
            v_peak = S_v(:,min_idx) ;
            
            % FOR DEBUGGING:
            % plot3(v_peak(1),v_peak(2),v_peak(3),'ko','Markersize',12)
            
            error_if_out_of_time(replan_start_tic,P.t_plan)
            
            exitflag = 1 ;
        end
        
        %% plot
        function plot(P,~)
            % plot current plan
            if ~isempty(P.current_plan.Z_des)
                Z_des = P.current_plan.Z_des ;
                
                % get axes
                if P.plot_local_view
                    x_cur = P.current_plan.Z_des(1:3,end) ;
                    x_lo = x_cur - 10 ;
                    x_hi = x_cur + 10;
                    axis_bds = [x_lo' ; x_hi'] ;
                    axis(axis_bds(:)') ;
                    view(2)
                end
                
                % plot current plan
                if check_if_plot_is_available(P,'current_plan')
                    P.plot_data.current_plan.XData = Z_des(1,:) ;
                    P.plot_data.current_plan.YData = Z_des(2,:) ;
                    P.plot_data.current_plan.ZData = Z_des(3,:) ;
                else
                    current_plan_data = plot3(Z_des(1,:),Z_des(2,:),Z_des(3,:),'b--') ;
                    P.plot_data.current_plan = current_plan_data ;
                end
            end
            
            % plot current zono
            if ~isempty(P.info) && ~isempty(P.info.v_peak) && P.plot_zonotope_reach_set_flag
                P.plot_zonotope_reach_set()
            end
            
            % plot obstacle predictions
            if P.plot_obstacle_predictions_flag
                P.plot_obstacle_predictions() ;
            end
            
            % FOR DEBUGGING: plot current obstacles
            % P.plot_current_obstacles()
            
            % plot high-level plan
            if P.plot_HLP_flag
                plot(P.HLP)
            end
        end
        
        function plot_at_time(P,t,~)
            % get info index corresponding to current time
            idx_last = P.get_info_index_at_time(t) ;
            
            if P.plot_desired_trajectory_flag
                P.plot_desired_trajectory(t)
            end
            
            if P.plot_zonotope_reach_set_flag && ~isempty(idx_last)
                % get current reach set
                if ~isempty(P.info.FRS_plot.Faces)
                    F_t = P.info.FRS_plot.Faces{idx_last} ;
                    V_t = P.info.FRS_plot.Vertices{idx_last} ;
                else
                    [v_pk,x_0,v_0,a_0] = P.get_zonotope_reach_set_slice_data(t) ;
                    [F_t,V_t] = P.make_zonotope_reach_set_patch_data(v_pk,x_0,v_0,a_0) ;
                end
                
                if check_if_plot_is_available(P,'FRS_plot')
                    P.plot_data.FRS_plot.Faces = F_t ;
                    P.plot_data.FRS_plot.Vertices = V_t ;
                else
                    patch_data = patch('Faces',F_t,...
                        'Vertices',V_t,...
                        'FaceColor',P.plot_zono_face_color,...
                        'FaceAlpha',P.plot_zono_face_opacity,...
                        'EdgeColor',P.plot_zono_edge_color,...
                        'EdgeAlpha',P.plot_zono_edge_opacity,...
                        'LineWidth',P.plot_zono_edge_width,...
                        'FaceLighting',P.plot_zono_lighting) ;
                    P.plot_data.FRS_plot = patch_data ;
                end
            end
            
            if P.plot_obstacle_predictions_flag
                P.plot_obstacle_predictions(t,idx_last) ;
            end
        end
        
        function plot_obstacle_predictions(P,t,idx_last)
            if nargin < 2
                try
                    t = P.info.t_start_plan(end) ;
                catch
                    t = 0 ;
                end
            end
            
            
            if nargin < 3
                % get info index corresponding to current time
                idx_last = P.get_info_index_at_time(t) ;
            end
            
            % iterate through obstacles and get the dynamic ones
            if ~isempty(P.info.obstacles)
                O = P.info.obstacles{idx_last};
                if ~isempty(O)
                    cnt = 0 ;
                    F_obs = [] ;
                    V_obs = [] ;
                    for idx = 1:length(O)
                        o = O{idx} ;
                        if isa(o,'zonotope_dynamic_obstacle')
                            [F,V] = P.get_dynamic_obs_patch_data(o,t) ;
                            F_obs = [F_obs ; (F + size(V_obs,1))] ;
                            V_obs = [V_obs ; V] ;
                        end
                    end

                    if check_if_plot_is_available(P,'obstacles')
                        P.plot_data.obstacles.Faces = F_obs ;
                        P.plot_data.obstacles.Vertices = V_obs ;
                    else
                        patch_data = patch('Faces',F_obs,'Vertices',V_obs,...
                            'FaceColor',P.plot_obstacle_predictions_face_color,...
                            'FaceAlpha',P.plot_obstacle_predictions_face_opacity,...
                            'EdgeColor',P.plot_obstacle_predictions_edge_color,...
                            'EdgeAlpha',P.plot_obstacle_predictions_edge_opacity,...
                            'LineWidth',P.plot_obstacle_predictions_edge_width,...
                            'FaceLighting',P.plot_obstacle_predictions_lighting) ;
                        P.plot_data.obstacles = patch_data ;
                    end
                end
            end
        end
        
        function plot_current_obstacles(P)
            % P.plot_current_obstacles()
            %
            % Plot a red star at the center of each current obstacle; this
            % is for debugging.
            
            if ~isempty(P.current_obstacles) && size(P.current_obstacles,2) > 2
                p =  [] ;
                for idx = 3:length(P.current_obstacles)
                    p = [p, P.current_obstacles{idx}.center(:)] ;
                end
                if check_if_plot_is_available(P,'obstacles')
                    P.plot_data.obstacles.XData = p(1,:) ;
                    P.plot_data.obstacles.YData = p(2,:) ;
                    P.plot_data.obstacles.ZData = p(3,:) ;
                else
                    obs_data = plot3(p(1,:),p(2,:),p(3,:),'r*') ;
                    P.plot_data.obstacles = obs_data ;
                end
            end
        end
        
        function plot_zonotope_reach_set(P,varargin)
            % P.plot_zonotope_reach_set()
            % P.plot_zonotope_reach_set(t)
            % P.plot_zonotope_reach_set(v_pk,x_0,v_0,a_0)
            % 
            % Plot the zonotope reach set the provided time t, or plot the
            % latest available reach set. If more inputs are provided, plot
            
            switch length(varargin)
                case 0
                    t = P.get_time_of_last_successful_plan() ;
                    [v_pk,x_0,v_0,a_0] = P.get_zonotope_reach_set_slice_data(t) ;
                case 1
                    t = varargin{1} ;
                    [v_pk,x_0,v_0,a_0] = P.get_zonotope_reach_set_slice_data(t) ;
                case 4
                    v_pk = varargin{1} ;
                    x_0 = varargin{2} ;
                    v_0 = varargin{3} ;
                    a_0 = varargin{4} ;
                otherwise
                    error('Incorrect number of inputs!')
            end
            
            [F,V] = P.make_zonotope_reach_set_patch_data(v_pk,x_0,v_0,a_0) ;
            
            P.info.FRS_plot.Faces = [P.info.FRS_plot.Faces, {F}] ;
            P.info.FRS_plot.Vertices = [P.info.FRS_plot.Vertices, {V}] ;
            
            if check_if_plot_is_available(P,'FRS_plot')
                P.plot_data.FRS_plot.Faces = F ;
                P.plot_data.FRS_plot.Vertices = V ;
            else
                patch_data = patch('Faces',F,...
                    'Vertices',V,...
                    'FaceColor',P.plot_zono_face_color,...
                    'FaceAlpha',P.plot_zono_face_opacity,...
                    'EdgeColor',P.plot_zono_edge_color,...
                    'EdgeAlpha',P.plot_zono_edge_opacity,...
                    'LineWidth',P.plot_zono_edge_width) ;
                P.plot_data.FRS_plot = patch_data ;
            end
        end
        
        function plot_zonotope_reach_set_history(P)
            % P.plot_zonotope_reach_set_history()
            %
            % Iterate thru P.info and plot the FRS executed for each time
            
            % get all the plan start times
            t_idxs = P.info.t_start_plan ;
            N_t = length(t_idxs) ;
            
            % preallocate all faces/vertices data
            F_all = [] ;
            V_all = [] ;
            
            % for each start time...
            for idx = 1:N_t
                % get the zonotope FRS slice data
                t = t_idxs(idx) ;
                [v_pk,x_0,v_0,a_0] = P.get_zonotope_reach_set_slice_data(t) ;
                
                % get the duration that was executed of each plan
                if idx < N_t
                    t_end = t_idxs(idx+1) - t_idxs(idx) - P.FRS_time_step ;
                else
                    t_end = P.t_total ;
                end
                
                % make the patch data for this FRS
                [F,V] = P.make_zonotope_reach_set_patch_data(v_pk,x_0,v_0,a_0,...
                    0,t_end) ;
                
                % update F_all and V_all
                F_all = [F_all ; F + size(V_all,1)] ;
                V_all = [V_all ; V] ;
            end
            
            % create plot
            if check_if_plot_is_available(P,'FRS_history')
                P.plot_data.FRS_plot.Faces = F_all ;
                P.plot_data.FRS_plot.Vertices = V_all ;
            else
                patch_data = patch('Faces',F_all,...
                    'Vertices',V_all,...
                    'FaceColor',P.plot_zono_face_color,...
                    'FaceAlpha',P.plot_zono_face_opacity,...
                    'EdgeColor',P.plot_zono_edge_color,...
                    'EdgeAlpha',P.plot_zono_edge_opacity,...
                    'LineWidth',P.plot_zono_edge_width) ;
                P.plot_data.FRS_plot = patch_data ;
            end
        end
        
        
        function plot_desired_trajectory(P,t)
            % P.plot_desired_trajectory(t)
            %
            % Plot the desired trajectory corresponding to the time t, or
            % plot the latest desired trajectory plan if no time is
            % provided.
            
            % get info index for plan
            if nargin < 2
                [~,idx] = P.get_time_of_last_successful_plan() ;
            else
                idx = P.get_info_index_at_time(t) ;
            end
            
            % get plan itself
            if ~isempty(idx)
                Z_des = P.info.Z_des{idx} ;

                % plot current plan
                if check_if_plot_is_available(P,'current_plan')
                    P.plot_data.current_plan.XData = Z_des(1,:) ;
                    P.plot_data.current_plan.YData = Z_des(2,:) ;
                    P.plot_data.current_plan.ZData = Z_des(3,:) ;
                else
                    current_plan_data = plot3(Z_des(1,:),Z_des(2,:),Z_des(3,:),'b--',...
                        'LineWidth',P.plot_desired_trajectory_line_width) ;
                    P.plot_data.current_plan = current_plan_data ;
                end
            end
        end
        
        function [F_out,V_out] = make_zonotope_reach_set_patch_data(P,v_pk,...
                                    x_0,v_0,a_0,t_start,t_end)
            % set the start and end times
            if nargin < 6
                t_start = 0 ;
            end
            if nargin < 7
                t_end = P.t_total ;
            end
            
            % get the FRS for plotting
            FRS_plot = P.FRS ;
            
            % add tracking error
            switch P.tracking_error_type
                case 'constant'
                    FRS_plot = add_tracking_error_to_FRS(FRS_plot, P.tracking_error_zono);
                case 'table'
                    FRS_plot = add_tracking_error_to_FRS(FRS_plot, P.tracking_error_table, v_0);
            end
            
            % filter out the start and end times
            if nargin > 5
                T = P.FRS_time(2:end) ; % this is 2:end because, if there
                                        % are n time intervals in the FRS
                                        % then there are n-1 zonotopes
                T_log = (T >= t_start) & (T <= t_end) ;
                FRS_plot = FRS_plot(T_log) ;
            end
            
            % slice the FRS by k = (k_pk,k_v,k_a)
            FRS_plot = slice_FRS(FRS_plot, [2; 7; 12; 3; 8; 13; 4; 9; 14], [v_0; a_0; v_pk]);
            
            % generate patch info for the FRS
            position_dimensions = [1; 6; 11];
            F_out = [] ;
            V_out = [] ;
            cnt = 0;
            
            for idx = 1:P.plot_zono_skip_idxs:length(FRS_plot)
                Z = FRS_plot{idx}{1}.Z;
                c = Z(position_dimensions, 1) + x_0;
                G = Z(position_dimensions, 2:end);
                lwh = 2*sum(abs(G),2);
                [F,V] = make_cuboid_for_patch(lwh(1),lwh(2),lwh(3),c) ;
                
                switch P.plot_zono_style
                    case 'face'
                        % get the front face and corresponding vertices
                        F = F(4,:) ;
                        V = V(F(:),:) ;
                        F_out = [F_out ; (4*cnt) + (1:4)] ;
                    case 'tube'
                        % keep all faces
                        F_out = [F_out ; (8*cnt) + F] ;
                end
                V_out = [V_out ; V] ;
                cnt = cnt + 1 ;
            end
        end
        
        %% utility
        function [t_last,idx_last] = get_time_of_last_successful_plan(P)
            % t_last = P.get_time_of_last_successful_plan()
            % [t_last, idx_last] = P.get_time_of_last_successful_plan()
            %
            % Get the last time at which a trajectory was found.
            
            t_plan_all = P.info.t_start_plan ;
            t_log = ~isnan(P.info.v_peak(1,:)) ;
            idx_last = find(t_log,1,'last') ;
            t_last = t_plan_all(idx_last) ;
            
            if isempty(t_last)
                t_last = 0 ;
            end
        end
        
        function [idx,t_of_plan] = get_info_index_at_time(P,t)
            % [idx,t_of_plan] = P.get_info_index_at_time()
            % [idx,t_of_plan] = P.get_info_index_at_time(t)
            %
            % Return the index of the P.info structure corresponding to the
            % provided time t, or return the index of the last successful
            % plan.
            
            if nargin < 2
                [t_of_plan,idx] = P.get_time_of_last_successful_plan() ;
            else
                t_log = P.info.t_start_plan <= t ;
                idx = find(t_log,1,'last') ;
                t_of_plan = P.info.t_start_plan(idx) ;
            end
        end
        
        function [v_pk,x_0,v_0,a_0] = get_zonotope_reach_set_slice_data(P,t)
            % [v_pk,x_0,v_0,a_0] = P.get_zonotope_reach_set_slice_data(t)
            %
            % Get the info needed to slice the FRS with respect to a plan
            % at the given time t; if no time is given, then the output is
            % given at the last available time.
            
            % get the time at which to slice the reach set
            t_max = max(P.info.t_start_plan) ;
            if nargin < 2
                t = t_max ;
            end
            t_last = P.get_time_of_last_successful_plan() ;
            t = min([t,t_max,t_last]) ;
            
            % get latest peak speed, accel, initial speed, and position
            % used to create a plan
            [v_pk,x_0,v_0,a_0] = match_trajectories(t,...
                P.info.t_start_plan, P.info.v_peak,...
                P.info.t_start_plan, P.info.x_0,...
                P.info.t_start_plan, P.info.v_0,...
                P.info.t_start_plan, P.info.a_0,...
                'previous') ;
            
            % if v_pk has nans, this means all planning iterations failed,
            % so set v_peak to zero
            if any(isnan(v_pk))
                v_pk = zeros(3,1) ;
            end
        end
    end
end