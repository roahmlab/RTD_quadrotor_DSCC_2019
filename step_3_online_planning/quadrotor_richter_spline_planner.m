classdef quadrotor_richter_spline_planner < planner
% Class: quadrotor_richter_spline_planner < planner
%
% Implements the trajectory planning method from [1] for quadrotors.
% Modifications are made to enable receding-horizon planning within the
% simulator framework.
%
% [1] Richter, C., et al., "Polynomial Trajectory Planning for Aggressive
%     Quadrotor Flight in Dense Indoor Environments," IJRR 2016.
%
% See also: quadrotor_zono_RTD_planner
% quadrotor_mueller_spline_planner

    properties
        % high-level planner properties
        lookahead_distance = 1 ;
        HLP_grow_tree_mode = 'seed' ; % 'new' or 'seed' or 'keep'
        HLP_timeout = 0.5 ;
        
        % state info
        tracking_error_amount = 0.1 ;
        max_speed = 5 ; % m/s
        max_accel = Inf ; % m/s^2
        
        % estimates for state information
        accel_est = zeros(3,1) ;
        jerk_est = zeros(3,1) ;
        
        % spline optimization info
        previous_plan ;
        reached_goal_flag = false ;
        goal_radius = 0.1 ;
    end
    
    methods
        %% constructor
        function P = quadrotor_richter_spline_planner(varargin)
            % set default HLP
            HLP = quadrotor_RRT_HLP() ;
            
            % make planner
            P@planner('HLP',HLP,'name','Richter Spline Planner',varargin{:}) ;
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
            P.info.HLP_plan = [] ;
            % P.info.waypoint = [] ;
            
            % set up plot data
            P.plot_data.trajectory = [] ;
            P.plot_data.waypoint = [] ;
            P.plot_data.HLP_plan = [] ;
            
%             % cheat a little bit :)
%             if strcmpi(P.HLP_grow_tree_mode,'seed') || strcmpi(P.HLP_grow_tree_mode,'keep')
%                 P.vdisp('Running RRT in advance',4)
%                 P.HLP.timeout = 3 ;
%                 P.HLP.grow_tree(agent_info,world_info) ;
%                 P.HLP.timeout = P.HLP_timeout ;
%             end
        end
        
        %% replan
        function [T,U,Z] = replan(P,agent_info,world_info)
            P.vdisp('Planning!',5)
            
            % start timer
            %start_tic = tic ;
            
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
            
            % if we haven't planned all the way to the goal, try planning
            % some more...
            if ~P.reached_goal_flag
                % run the RRT
                P.vdisp('Running RRT*',7)
                exit_flag = P.HLP.grow_tree(agent_info,world_info) ;
                
                % get just the non-dynamic obstacles
                P.vdisp('Prepping obstacles for collision check',8)
                O = world_info.obstacles ;
                O_static = {} ;
                for idx = 1:length(O)
                    o = O{idx} ;
                    if isa(o,'zonotope_obstacle')
                        O_static = [O_static, {o}] ;
                    end
                end
                
                % get the representation of the obstacles needed for collision
                % checking
                b = P.buffer ;
                O_cell = cellfun(@(o) [o.center ; o.body_dimensions(:) + b],...
                    O_static,'UniformOutput',false) ;
                
                % pessimism!
                traj_chk = false ;
                
                if exit_flag > 0
                    P.vdisp('RRT* was successful! Making spline traj.',6)
                    best_path = P.HLP.best_path ;
                    [T,Z,d,d_vec,t_vec,traj_chk] = P.make_spline(x_cur,v_cur,best_path,O_cell) ;
                end
                
                if traj_chk
                    P.vdisp('Traj planning successful',6)
                    
                    % check if reached goal
                    if vecnorm(Z(1:3,end) - world_info.goal) < P.goal_radius
                        P.vdisp('Goal reached by planner!',6)
                        P.reached_goal_flag = true ;
                    end
                    
                    if strcmpi(P.HLP_grow_tree_mode,'seed') || strcmpi(P.HLP_grow_tree_mode,'keep')
                        P.vdisp('Shifting trajectory based on RRT* growth method',8)
                        % if the RRT is kept around, toss out all of the
                        % trajectory before x_cur
                        t_start = match_trajectories(d,d_vec,t_vec) ;
                        T_log = T >= t_start ;
                        T_new = T(T_log) ;
                        if T_new(1) > t_start
                            T_new = [t_start, T_new] ;
                        end
                        
                        Z = match_trajectories(T_new,T,Z) ;
                        T = T_new - t_start ;
                    end
                else
                    P.vdisp('Traj planning failed',6)
                    
                    best_path = nan ;
                    x_des = nan ;
                    
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
            else
                % shift the current traj by P.t_move
                P.vdisp('Continuing previous plan!',6)
                
                if ~isempty(P.info.HLP_plan)
                    best_path = P.info.HLP_plan{end} ;
                    x_des = best_path(:,end) ;
                else
                    x_des = x_cur ;
                end
                
                T_log = T_old >= P.t_move ;
                T = T_old(T_log) ;
                
                if any(T_log)
                    if T(1) > P.t_move
                        T = [P.t_move, T] ;
                    end
                    Z = match_trajectories(T,T_old,Z_old) ;
                    T = T - P.t_move ;
                else
                    T = [0, P.t_move, 2*P.t_move] ;
                    Z = [repmat(x_cur,1,3) ; zeros(12,3)] ;
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
            [a_est,j_est] = match_trajectories(P.t_move,T,Z(7:9,:),T,Z(10:12,:)) ;
            P.accel_est = a_est ;
            P.jerk_est = j_est ;
            
            % update info
            P.info.T = [P.info.T, {T}] ;
            P.info.Z = [P.info.Z, {Z}] ;
            P.info.t_start_plan = [P.info.t_start_plan, agent_info.time(end)] ;
            P.info.HLP_plan = [P.info.HLP_plan, {best_path}] ;
            % P.info.waypoint = [P.info.waypoint, {x_des}] ;
        end
        
        %% make spline
        function [T,Z,d,d_vec,t_vec,traj_chk] = make_spline(P,x_cur,v_cur,best_path,O_cell)
            % if there is a best path, use it to generate a polynomial
            % trajectory plan
            best_path_distances = P.HLP.best_path_distance ;
            
            % get the agent's current distance along the path
            d = dist_point_on_polyline(x_cur,best_path) ;
            
            % always try to get all the way to the goal
            d_lkhd = best_path_distances(end) ;
            
            % get the desired points to reach
            current_point_dist = 1 ;
            current_point_speed = P.max_speed ;
            N_tries_allowed = 7 ;
            
            for try_idx = 1:N_tries_allowed
                P.vdisp('Fitting spline to RRT* output',6)
                
                d_vec = 0:current_point_dist:d_lkhd ;
                if d_vec(end) ~= d_lkhd
                    d_vec = [d_vec, d_lkhd] ;
                end
                x_des = match_trajectories(d_vec,best_path_distances,best_path) ;
                
                % FOR DEBUGGING:
                % plot_path(best_path,'-','LineWidth',2,'Color',[0.5 0.5 1])
                
                % FOR DEBUGGING:
                % plot_path(x_des,'rp')
                
                % get spline duration
                t_vec = d_vec / current_point_speed ;
                
                % get the time-optimal spline
                [T,Z,exit_flag] = P.make_single_spline(t_vec,x_des(:,2:end),x_cur,v_cur) ;
                
                if exit_flag == 1
                    P.vdisp('Spline generated successfully!')
                    
                    % shift trajectory to current location
                    Z(1:3,:) = Z(1:3,:) + repmat(x_cur,1,size(Z,2)) ;
                    
                    % get position/velocity/acceleration
                    X = Z(1:3,:) ;
                    % V = Z(4:6,:) ; A = Z(7:9,:) ;
                    
                    % make sure the spline doesn't intersect any obstacles
                    O_chk = cellfun(@(o) dist_point_to_box(X,o(4),o(5),o(6),o(1:3)) == 0,...
                        O_cell,'UniformOutput',false) ;
                    O_chk = ~any(cell2mat(O_chk)) ;
                    
                    % make sure the spline doesn't leave the bounds
                    B = P.bounds ;
                    B_chk = all(dist_point_to_box(X,B) == 0) ;
                    
                    % make sure the velocity and acceleration constraints are
                    % obeyed
                    %V_chk = all(vecnorm(V) <= P.max_speed) ;
                    %A_chk = all(vecnorm(A) <= P.max_accel) ;
                    
                    traj_chk = O_chk && B_chk ;%&& V_chk && A_chk ;
                    if traj_chk
                        P.vdisp('Spline obeys constraints',9)
                        break
                    else
                        P.vdisp('Spline does not obey constraints, trying again',8)
                        current_point_dist = current_point_dist/1.1 ;
                        current_point_speed = current_point_speed./1.1 ;
                    end
                end
            end
        end
        
        function [T,Z,exit_flag] = make_single_spline(P,t_vec,x_des,x_cur,v_cur)
            % get the accel and jerk inputs
            a_cur = P.accel_est ;
            j_cur = P.jerk_est ;
            
            % get spline coeffs with quadprog
            coeffs = get_spline_coeffs(t_vec,x_des,x_cur,v_cur,a_cur,j_cur) ;
            
            % reshape spline coeffs so that the waypoints are in the
            % 'depth' direction of the array
            N_wp = length(t_vec) - 1 ;
            try
                coeffs = reshape(coeffs,7,N_wp,3) ;
                coeffs = permute(coeffs,[1 3 2]) ;

                % get trajectory
                [T,Z] = get_spline_points_multi_waypoint(coeffs,t_vec) ;
                exit_flag = 1 ;
            catch
                T = [] ;
                Z = [] ;
                exit_flag = -1 ;
            end
            
            % sanity check
            if any(isnan(Z(:)))
                exit_flag = -1 ;
            end
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
                    
%                     % plot waypoint
%                     wp = P.info.waypoint{idx} ;
%                     if ~isnan(wp)
%                         plot_object(P,wp,'waypoint','p','Color',[1 0.7 0.5]) ;
%                     end
                    
                    % plot HLP plan
                    HLP_plan = P.info.HLP_plan{idx} ;
                    if ~isnan(HLP_plan)
                        plot_object(P,HLP_plan,'HLP_plan','--','Color',[1 0.7 0.5]) ;
                    end
                end
                
                % Z = P.info.Z{idx} ;
                % if check_if_plot_is_available(P,'trajectory')
                %     P.plot_data.trajectory.XData = Z(1,:) ;
                %     P.plot_data.trajectory.YData = Z(2,:) ;
                %     P.plot_data.trajectory.ZData = Z(3,:) ;
                % else
                %     d = plot_path(Z(1:3,:),'b--') ;
                %     P.plot_data.trajectory = d ;
                % end
            end
        end
    end
end

%% HELPER FUNCTIONS
function H = build_H( n, wps, ~, r, t )
%BUILDH Builds the H matrix for a single segment of the optimization
%problem
% Inputs:
%   n           Order of the polynomials of a trajectory
%   wps         Number of waypoints (not including initial conditions)
%   mu          Constant making the integrand non-dimentional
%   r           Order of the derivative of the position
%   t           Time vector, always starts with 0
%
% Outputs:
%   H           H matrix for the QP problem

% Author:   Andre Phu-Van Nguyen <andre-phu-van.nguyen@polymtl.ca>
% Good reference paper
% Adam Bry's thesis and richter_rss13_workshop.

num_coeffs = n + 1;

% Follow equations 3.30 page 37
H = [];
for wp = 1:wps
    H_r = zeros(num_coeffs);
    t0 =  t(wp);
    tend = t(wp+1);
    for i = 0:n
        for l = 0:n
            if i >= r && l >=r
                cum_mul = 1;
                for m = 0:r-1
                    cum_mul = cum_mul * (i-m) * (l-m);
                end
                H_r(i+1,l+1) = cum_mul;% * (tend-t0)^(i+l-2*r+1);
                H_r(i+1,l+1) = H_r(i+1,l+1) / (i+l-2*r+1);
            else
                H_r(i+1,l+1) = 0;
            end
        end
    end
    
    % "The cost matrix is constructed as block diagonal on Qk" (page 41)
    H_r =  1 ./ ((t(wp+1) - t(wp))^(2*r-1)) .* H_r;
    H_r = rot90(rot90(H_r));
    H = blkdiag(H, H_r);
end

end

function [Aeq, beq] = build_constraints( n, r, constraints, t)
%BUILDCONSTRAINTS Builds the constraints matrix
% Inputs:
%   n           Order of the polynomials of a trajectory
%   r           Order of the derivative of the position
%   constraints r by wps+1 Matrix of constraints
%               Rows are derivatives and columns are waypoints
%               Inf denotes unfixed variable.
%   t           Time vector, always starts with 0
%
% Outputs:
%   H           H matrix for the QP problem

% Author:   Andre Phu-Van Nguyen <andre-phu-van.nguyen@polymtl.ca>
%
% Basically follow equations 16 to 21 of richter_rss13_workshop
n_coeffs = n+1;
s = size(constraints);
n_wps = s(2);
coeffs = get_coefficient_matrix(n, r);
I = rot90(eye(n_coeffs, n_coeffs));     % Just need this for computations

A_0 = [];
b_0 = [];

for wp = 1:n_wps      % For each waypoint including initial conditions
    for der = 1:r       % For each derivative of the polynomial up to r-1
        if constraints(der, wp) ~= Inf    % We have a constraint
            if wp == 1
                % Initial conditions
                % Add only departure constraints
                a = zeros(1, (n_wps-1) * n_coeffs);
                % Set the part of 'a' corresponding to the wp
                % XXX
                int_t = 1 / (t(wp+1) - t(wp))^(der-1); % der-1 to get 0th derivative
                polynomial = coeffs(der, :) .* I(der, :) * int_t;
                % Get start idx of a block of coefficients
                idx = (wp-1) * n_coeffs + 1;
                a(1, idx:idx+n) = polynomial;
                b = constraints(der, wp);
            elseif wp == n_wps
                % Final conditions
                % Add only arrival constraints
                a = zeros(1, (n_wps-1) * n_coeffs);
                % Set the part of 'a' corresponding to the wp
                % XXX
                int_t_next = 1 / (t(wp) - t(wp-1))^(der-1); % t now and t prev
                polynomial = coeffs(der, :) * int_t_next;
                % Get the start idx of the **previous** block.
                idx = (wp-2) * n_coeffs + 1;
                a(1, idx:idx+n) = polynomial;
                b = constraints(der, wp);
            else
                % Middle/waypoint conditions
                a = zeros(2, (n_wps-1) * n_coeffs);
                
                % Add departure constraint
                int_t_next = 1 / (t(wp) - t(wp-1))^(der-1);  % time now and prev
                poly = coeffs(der, :) * int_t_next;
                idx = (wp-2) * n_coeffs + 1;
                a(1, idx:idx+n) = poly;
                
                % Add arrival constraint
                int_t = 1 / (t(wp+1) - t(wp))^(der-1);  % first row is 0th derivative
                poly = coeffs(der, :) .* I(der, :) * int_t;
                idx = (wp-1) * n_coeffs + 1;
                a(2, idx:idx+n) = poly;
                
                b = ones(2, 1);
                b = b .* constraints(der, wp); % both lines equal to this
            end
            A_0 = [A_0; a];
            b_0 = [b_0; b];
        end
    end
end
%
% Aeq = A_0;
% beq = b_0;

% Now build continuity constraints follow equation 3.53 of "Control,
% estimation, and planning algorithms for aggressive flight using onboard
% sensing" By Bry, Adam Parker

% Basically what we will do is add arrival and derparture constraints on
% the same line to enforce continuity between two polynomials
A_t = [];
b_t = [];

for wp = 2:n_wps-1      % XXX for each INTERMEDIATE waypoint
    for der = 1:r+1       % for each dervative including the 0th derivative
        if constraints(der, wp) == Inf
            a = zeros(1, (n_wps-1) * n_coeffs);
            int_t = 1 / (t(wp) - t(wp-1))^(der-1);
            int_t_next = 1 / (t(wp+1) - t(wp))^(der-1);
            
            % from prev wp
            a_prev = coeffs(der, :) * int_t;
            idx_prev = (wp-2) * n_coeffs + 1;
            a(1, idx_prev:idx_prev+n) = a_prev;
            
            % to next wp
            a_next = - coeffs(der, :) .* I(der,:) * int_t_next;
            idx_next = (wp-1) * n_coeffs + 1;
            a(1, idx_next:idx_next+n) = a_next;
            
            b = 0;
            
            A_t = [A_t; a];
            b_t = [b_t; b];
        end
    end
end
% Acont = A_t;
% bcont = b_t;
Aeq = [A_0; A_t];
beq = [b_0; b_t];
end

function [ coefficients ] = get_coefficient_matrix( n, r )
%GETCOEFFICIENTMATRIX Gives a matrix where each line are the coefficients
%of a derivative of a polynomial of order n
% Inputs:
%   n           Order of the polynomials of a trajectory
%   r           Order of the last derivative
% Example:
%   A polynomial of order n = 6 and we want the coefficients up to
%   derivative r = 4. We will get
%
% coefficients =
% 
%      1     1     1     1     1     1     1
%      6     5     4     3     2     1     0
%     30    20    12     6     2     0     0
%    120    60    24     6     0     0     0
%    360   120    24     0     0     0     0

% Author:   Andre Phu-Van Nguyen <andre-phu-van.nguyen@polymtl.ca>

num_coeffs = n + 1;     % +1 to include the c_0 coefficient
num_poly = r + 1;       % +1 to include the 0th derivative

coefficients = zeros(num_poly, num_coeffs);
polynomial = ones(1, num_coeffs);
for i = 1:num_poly
    coefficients(i,1:length(polynomial)) = polynomial;
    polynomial = polyder(polynomial);
end

end

function out = get_spline_coeffs(t_vec,x_des,x_cur,v_cur,a_cur,j_cur)
    
    % specify waypoints relative to origin
    N_wp = size(x_des,2) ;
    x_dif = x_des - repmat(x_cur,1,N_wp)  ;

    % generate waypoints for quadprog (separately in x, y, z)
    w1 = inf(5, N_wp+1) ;
    w2 = inf(5, N_wp+1) ;
    w3 = inf(5, N_wp+1) ;
    
    w1(:,1) = [0 ; v_cur(1) ; a_cur(1) ; j_cur(1) ; 0] ;
    w2(:,1) = [0 ; v_cur(2) ; a_cur(2) ; j_cur(2) ; 0] ;
    w3(:,1) = [0 ; v_cur(3) ; a_cur(3) ; j_cur(3) ; 0] ;
    
    for idx = 2:(N_wp+1)
        w1(:,idx) = [x_dif(1,idx-1) ; inf(4,1)] ;
        w2(:,idx) = [x_dif(2,idx-1) ; inf(4,1)] ;
        w3(:,idx) = [x_dif(3,idx-1) ; inf(4,1)] ;
    end
    
    w1(2:3,end) = 0 ; % specify zero final velocity and acceleration
    w2(2:3,end) = 0 ;
    w3(2:3,end) = 0 ;
    
    % generate the H matrix for the quadratic program
    polynomial_order = 6 ;
    N_wp = size(w1,2) - 1 ;
    position_derivative_order = 4 ;
    H = build_H(polynomial_order, N_wp, 1, position_derivative_order, t_vec) ;
    H = blkdiag(H,H,H) ;

    % generate the constraints matrices
    [A1,b1] = build_constraints(polynomial_order,position_derivative_order,w1,t_vec) ;
    [A2,b2] = build_constraints(polynomial_order,position_derivative_order,w2,t_vec) ;
    [A3,b3] = build_constraints(polynomial_order,position_derivative_order,w3,t_vec) ;
    A = blkdiag(A1,A2,A3) ;
    b = [b1(:); b2(:); b3(:)] ;

    % call quadprog
    options = optimoptions('quadprog', 'Display', 'off',...
        'MaxIterations', 4000);
    
    [out,~] = quadprog(H,[],[],[],A,b,[],[],[],options) ;
end

function [T,X,varargout] = get_spline_points(coeffs,dt,N_t)
% [T,X,X',X'',...] = get_spline_points(coeffs,dt,N_t)
%
% Given an N-by-3 vector of spline coefficients and a duration dt, return
% the time vector T = [0, ..., dt], the spline X = [x0,...,xN], and as many
% derivatives of the spline as the user specifies.
%
% Author: Shreyas Kousik
% Created: 6 Nov 2019
% Updated: 12 Jan 2020

    if nargin < 2
        dt = 1 ;
    end
    
    if nargin < 3
        N_t = 100 ;
    end

    % get splines in x, y, and z
    c1 = coeffs(:,1)' ;
    c2 = coeffs(:,2)' ;
    c3 = coeffs(:,3)' ;

    % get NONDIMENSIONAL time vector
    T_eval = linspace(0,1,N_t) ;

    % evaluate polynomial
    X = [polyval(c1,T_eval) ; polyval(c2,T_eval) ; polyval(c3,T_eval)] ;
    
    % rescale time
    T = linspace(0,dt,N_t) ;
    
    % get derivatives if specified 
    varargout = cell(1,nargout-2) ;
    
    for idx = 1:nargout
        % differentiate polynomial
        c1 = polyder(c1) ;
        c2 = polyder(c2) ;
        c3 = polyder(c3) ;
        
        % evaluate output
        varargout{idx} = [polyval(c1,T_eval) ; polyval(c2,T_eval) ; polyval(c3,T_eval)] ;
    end
end

function [T,Z] = get_spline_points_multi_waypoint(coeffs,t_vec,N_t)
    if nargin < 3
        N_t = 100 ;
    end
    
    % gon' fill these in
    T = [] ;
    Z = [] ;
    
    N_wp = size(coeffs,3) ;
    N_t = ceil(N_t/N_wp) ;
    
    for idx = 1:N_wp
        dt = t_vec(idx+1) - t_vec(idx) ;
        [t,x,v,a,j,s] = get_spline_points(coeffs(:,:,idx),dt,N_t) ;
        Z_idx = [x;v;a;j;s] ;
        
        if idx == 1
            T = t ;
            Z = Z_idx ;
        else
            T = [T, T(end) + t(2:end)] ;
            Z = [Z , Z_idx(:,2:end)] ;
        end
    end
    
%     T = T(2:end) ;
%     Z = Z(:,2:end) ;
end

function out = test_spline_speed_and_accel(coeffs,max_speed,max_accel)
    [~,~,V,A] = get_spline_points(coeffs) ;
    v_test = ~any(vecnorm(V) > max_speed) ;
    a_test = ~any(vecnorm(A) > max_accel) ;
    out = v_test && a_test ;
end