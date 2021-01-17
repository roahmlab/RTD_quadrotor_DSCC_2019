classdef quadrotor_mueller_spline_planner < planner
% Class: quadrotor_mueller_spline_planner < planner
%
% Implements the trajectory planning method from [1] for quadrotors.
% Modifications are made to enable receding-horizon planning within the
% simulator framework.
%
% [1] Mueller, M., et al., "A Computationally Efficient Motion Primitive
%     for Quadrocopter Trajectory Generation," T-RO 2015
%
% See also: quadrotor_zono_RTD_planner
% quadrotor_richter_spline_planner

    properties
        % high-level planner properties
        lookahead_distance = 1 ;
        HLP_grow_tree_mode = 'new' ; % 'new' or 'seed' or 'keep'
        HLP_timeout = 0.5 ;
        
        % state info
        tracking_error_amount = 0.1 ;
        max_speed = 1 ; % m/s
        max_accel = Inf ; % m/s^2
        
        % estimates for state information
        accel_est = zeros(3,1) ;
        
        % spline optimization info
        previous_plan ;
        reached_goal_flag = false ;
        goal_radius = 0.1 ;
        
    end
    
    methods
        %% constructor
        function P = quadrotor_mueller_spline_planner(varargin)
            % set default HLP
            HLP = quadrotor_RRT_HLP() ;
            
            % make planner
            P@planner('HLP',HLP,'name','Mueller Spline Planner',varargin{:}) ;
        end
        
        %% setup
        function setup(P,agent_info,world_info)
            % set up the buffer based on agent info
            if isempty(P.buffer)
                P.buffer = norm((agent_info.body_dimensions)./2) + ...
                    + P.tracking_error_amount ;
            end
            
            % set the world bounds
            b = P.buffer ;
            P.bounds = world_info.bounds + b.*[1 -1 1 -1 1 -1] ;
            
            % set up lookahead distance based on max speed
            P.lookahead_distance = max([P.t_move*P.max_speed,P.lookahead_distance]) ;
            
            % set up the high-level planner
            if isa(P.HLP,'quadrotor_RRT_HLP')
                P.HLP.setup(agent_info,world_info) ;
                P.HLP.buffer = P.buffer ;
                P.HLP.grow_tree_mode = P.HLP_grow_tree_mode ;
                P.HLP.timeout = P.HLP_timeout ;
                P.HLP.new_node_max_distance_from_agent = agent_info.sensor_radius ;
                P.HLP.goal_as_new_node_rate = 0.2 ;
                P.HLP.new_node_growth_distance = 1 ;
            end
            
            % set up previous plan
            P.previous_plan.T = [] ;
            P.previous_plan.Z = [] ;
            P.reached_goal_flag = false ;
            
            % set up p info
            P.info.T = [] ;
            P.info.Z = [] ;
            P.info.t_start_plan = [] ;
            P.info.waypoint = [] ;
            
            % set up plot data
            P.plot_data.trajectory = [] ;
            P.plot_data.waypoint = [] ;
            P.plot_data.HLP_plan = [] ;
        end
       
        %% replan
        function [T,U,Z] = replan(P,agent_info,world_info)
            P.vdisp('Planning!',5)
            
            % start timer
            start_tic = tic ;
            
            % get agent state
            if isempty(P.previous_plan.T)
                P.vdisp('Using agent''s current state for initial condition',8)
                x_cur = agent_info.position(:,end) ;
                v_cur = agent_info.velocity(:,end) ;
                T_old = [] ;
                Z_old = [] ;
            else
                P.vdisp('Using previous plan for initial condition',8)
                T_old = P.previous_plan.T ;
                Z_old = P.previous_plan.Z ;
                z_move = match_trajectories(P.t_move,T_old,Z_old) ;
                x_cur = z_move(1:3) ;
                v_cur = z_move(4:6) ;
            end
            
            % run the RRT to get a waypoint
            P.vdisp('Running RRT*',7)
            x_des = P.HLP.get_waypoint(agent_info,world_info,P.lookahead_distance) ;
            
            % % FOR DEBUGGING
            % plot_path(x_des,'kp')
            
            % process obstacles
            P.vdisp('Prepping obstacles for collision check',8)
            O = world_info.obstacles ;
            O_static = {} ;
            for idx = 1:length(O)
                o = O{idx} ;
                if isa(o,'zonotope_obstacle')
                    O_static = [O_static, {o}] ;
                end
            end
            b = P.buffer ;
            O_cell = cellfun(@(o) [o.center ; o.body_dimensions(:) + b],...
                O_static,'UniformOutput',false) ;
            
            % generate a bunch of samples of possible points to reach,
            % distributed around the point x_des
            N = 1000 ;
            d_x = [1 ; 1 ; 0.5] ;
            x_lo = x_des - d_x ;
            x_hi = x_des + d_x ;
            X_lo = repmat(x_lo,1,N) ;
            X_hi = repmat(x_hi,1,N) ;
            X_sample = [x_des, rand_range(X_lo,X_hi)] ;
            
            % bound the points by the world boudns
            B = P.bounds ;
            B_lo = repmat([B(1) ; B(3) ; B(5)],1,N+1) ;
            B_hi = repmat([B(2) ; B(4) ; B(6)],1,N+1) ;
            
            X_lo_log = X_sample < B_lo ;
            X_sample(X_lo_log) = B_lo(X_lo_log) ;
            
            X_hi_log = X_sample > B_hi ;
            X_sample(X_hi_log) = B_hi(X_hi_log) ;
            
            % make sure the points in X_sample are unique
            X_sample = unique(X_sample','rows')' ;
            
            % sort X_sample by distance to x_des
            d_des = dist_point_to_points(x_des,X_sample) ;
            [~,idxs] = sort(d_des,'ascend') ;
            X_sample = X_sample(:,idxs) ;
            
            % get initial conditions for spline
            a_cur = P.accel_est ;
            
            % iterate through the points in X_sample, getting a trajectory
            % for each one, and checking if the trajectory is valid (if it
            % is, cool!)
            t_cur = toc(start_tic) ;
            traj_chk = false ;
            while (idx < size(X_sample,2)) && (t_cur <= P.t_plan) && (~traj_chk)
                % get desired change in position
                x_des_idx = X_sample(:,idx) - x_cur ;
                
                % make time horizon
                dt = (4/3)*vecnorm(x_des_idx - x_cur)./P.max_speed ;
                
                % sanity check the time horizon
                if dt == 0
                    dt = P.t_move ;
                end
                
                % generate spline
                [T_temp,~,Z_temp] = generate_spline_desired_position(x_des_idx,...
                    v_cur,a_cur,dt,0.01) ;
                
                % shift trajectory to current location
                    Z_temp(1:3,:) = Z_temp(1:3,:) + repmat(x_cur,1,size(Z_temp,2)) ;
                
                % get position
                X_temp = Z_temp(1:3,:) ;
                
                % % FOR DEBUGGING
                % plot_path(X_temp,'b--')
                % pause(0.01) ;
                
                % make sure the spline doesn't intersect any obstacles
                O_chk = cellfun(@(o) dist_point_to_box(X_temp,o(4),o(5),o(6),o(1:3)) == 0,...
                    O_cell,'UniformOutput',false) ;
                O_chk = ~any(cell2mat(O_chk)) ;
                
                % make sure the spline doesn't leave the bounds
                B = P.bounds ;
                B_chk = all(dist_point_to_box(X_temp,B) == 0) ;
                
                % update trajectory check
                traj_chk = O_chk && B_chk ;
                
                % update time and index
                idx = idx + 1 ;
                t_cur = toc(start_tic) ;
            end
            
            % if a trajectory was found successfully, yay!
            if traj_chk
                T = T_temp ;
                Z = Z_temp ;
            else
                P.vdisp('Traj planning failed',6)
               
                % if no best path exists, just command the quadrotor to
                % hover in place
                if ~isempty(T_old)
                    P.vdisp('Trying to continue old plan',6)
                    
                    % try to use what's left of the previous plan
                    T_log = T_old >= P.t_move ;
                    
                    if any(T_log)
                        T = T_old(T_log) ;
                        if T(1) > P.t_move
                            T = [P.t_move, T] ;
                        end
                        Z = match_trajectories(T,T_old,Z_old) ;
                        T = T - T(1) ;
                    else
                        T = [0 10] ;
                        Z = [repmat(x_cur,1,2) ; zeros(12,2)] ;
                    end
                else
                    P.vdisp('Hovering in place',6)
                    
                    T = [0 10] ;
                    Z = [repmat(x_cur,1,2) ; zeros(12,2)] ;
                end
            end
            
            % sanity check the current plan
            if length(T) < 2
                T = [T, 10] ;
                Z = [Z, Z(:,end)] ;
            end
            
            if T(end) < P.t_move
                T = [T, P.t_move, 2*P.t_move] ;
                Z = [Z, repmat([Z(1:3,end);zeros(12,1)],1,2)] ;
            end
            
            % update previous plan
            P.previous_plan.T = T ;
            P.previous_plan.Z = Z ;
            
            % generate a dummy input
            U = zeros(agent_info.n_inputs,size(T,2)) ;
            
            % update the acceleration and jerk estimates based on P.t_move
            P.vdisp('Updating acceleration and jerk estimates from new plan',9)
            P.accel_est = match_trajectories(P.t_move,T,Z(7:9,:)) ;
            
            % update info
            P.info.T = [P.info.T, {T}] ;
            P.info.Z = [P.info.Z, {Z}] ;
            P.info.t_start_plan = [P.info.t_start_plan, agent_info.time(end)] ;
            P.info.waypoint = [P.info.waypoint, x_des] ;
        end
        
        %% plotting
         function plot(P,~)
            if ~isempty(P.info.T)
                P.plot_at_time(P.info.t_start_plan(end)) ;
            end
        end
        
        function plot_at_time(P,t)
            if nargin < 2
                t = 0 ;
            end
            
            % find the info index at which the latest time applies
            if ~isempty(P.info.T)
                idx = find(P.info.t_start_plan >= t,1,'first') ;
                
                % plot trajectory plan
                if ~isempty(idx)
                    plot_object(P,P.info.Z{idx}(1:3,:),'trajectory','b--') ;
                    
                    % plot waypoint
                    waypoint = P.info.waypoint(:,idx) ;
                    if ~isnan(waypoint)
                        plot_object(P,waypoint,'waypoint','kp') ;
                    end
                end
            end
        end
    end
end