classdef zonotope_dynamic_box_world < zonotope_box_world
properties
    N_dyn = 10 ;
    obs_dyn_min_speed = 0 ; % m/s
    obs_dyn_max_speed = 1 ;
    obs_dyn_size_range = [0.25,5,2,2] ;  % [min,max,mean,std] side length
    obs_dyn_N_points_on_path = 10 ; % max number of points an obstacle can visit
    obs_dyn_position_max_distance = 10 ; % max distance between points
    
    plot_obstacles_over_time_flag = false ;
    plot_obstacles_over_time_face_color = [1 0.7 0.7] ;
    plot_obstacles_over_time_face_opacity = 0.05 ;
    plot_obstacles_over_time_edge_color = [1 0.5 0.5] ;
    plot_obstacles_over_time_edge_opacity = 0.2 ;
    plot_obstacles_over_time_edge_width = 1.5 ;
    plot_obstacles_over_time_lighting = 'flat' ;
end

methods
    %% constructor
    function W = zonotope_dynamic_box_world(varargin)
        W@zonotope_box_world(varargin{:}) ;
        
        W.plot_data.obstacles_over_time = [] ;
    end
    
    %% setup
    function setup(W,~)
        setup@zonotope_box_world(W) ;
        
        % create dynamic obstacles, then create static obstacles using
        % superclass default functions
        for oidx = 1:W.N_dyn
            W.obstacles = [W.obstacles, {W.create_dynamic_obstacle()}] ;
        end
        
        W.N_obstacles = length(W.obstacles) ;
    end
    
    %% reset
    function reset(W)
        % call the superclass method
        reset@zonotope_box_world(W) ;
        
        % reset the obstacle times! this was a nasty ugly bad bad bug...
        W.update_obstacle_times(0) ;
    end
    
    %% create dynamic obstacle
    function add_obstacle(W,varargin)
        % W.add_obstacle(varargin)
        %
        % This method adds an obstacle to the world. There are several ways
        % to use it:
        %
        % To add a randomly-generated obstacle, pass in just one argument,
        % obstacle_type, as 'tall', 'wide', 'long', 'boxy', or 'dyn'
        %   W.add_obstacle(obstacle_type)
        %
        % To create a static obstacle centered at the origin with
        % dimensions l x w x h:
        %   W.add_obstacle(l,w,h)
        %
        % To create a static obstacle centered at a point c in \R^3:
        %   W.add_obstacle(l,w,h,c)
        %
        % To create a dynamic obstacle that follows a position path, which
        % should be a 3-by-N array of positions, with a randomly-generated
        % constant speed:
        %   W.add_obstacle(l,w,h,position)
        %
        % To create a dynamic obstacle that follows a position path at the
        % given scalar speed:
        %   W.add_obstacle(l,w,h,position,speed)
        %
        % To create a dynamic obstacle that follows a trajectory given by a
        % 3-by-N position array and a 1-by-N time array:
        %   W.add_obstacle(l,w,h,position,time)
        
        if length(varargin) <= 3
            % if length(varargin) == 1, then check if it's a 'dyn' type
            if length(varargin) == 1 && strcmpi(varargin{1},'dyn')
                obs = W.create_dynamic_obstacle() ;
                W.update_obstacle_list(obs) ;
            else
                add_obstacle@zonotope_box_world(W,varargin{:})
            end
        elseif length(varargin) == 4
            l = varargin{1} ;
            w = varargin{2} ;
            h = varargin{3} ;
            c = varargin{4} ;
            
            [~,N_cols] = size(c) ;
            
            if N_cols == 1
                % if the 4th argument is only one point, then treat it as
                % the center of a static obstacle
                add_obstacle@zonotope_box_world(W,l,w,h,c) ;
            else
                % otherwise
                spd = rand_range(W.obs_dyn_min_speed,W.obs_dyn_max_speed) ;
                obs = zonotope_dynamic_obstacle(l,w,h,c,spd) ;
                W.update_obstacle_list(obs) ;
            end
        elseif length(varargin) == 5
            obs = zonotope_dynamic_obstacle(varargin{:}) ;
            W.update_obstacle_list(obs) ;
        else
            error('Incorrect number of inputs!')
        end
    end
    
    function obs = create_dynamic_obstacle(W)
        W.vdisp('Creating dynamic obstacle',7)
        
        % create obstacle positions
        p = W.create_obstacle_positions() ;

        % create obstacle size
        dlo = W.obs_dyn_size_range(1) ;
        dhi = W.obs_dyn_size_range(2) ;
        m = W.obs_dyn_size_range(3) ;
        s = W.obs_dyn_size_range(4) ;

        l = rand_range(dlo,dhi,m,s) ;
        w = rand_range(dlo,dhi,m,s) ;
        h = rand_range(dlo,dhi,m,s) ;
        
        % create obstacle speed
        spd = rand_range(W.obs_dyn_min_speed,W.obs_dyn_max_speed) ;

        % make obstacle
        obs = zonotope_dynamic_obstacle(l,w,h,p,spd) ;
    end
    
    function P = create_obstacle_positions(W)
        % create lots of random points in the world
        B = W.bounds ;
        N = 10000 ;
        P = [rand_range(B(1),B(2),[],[],1,N) ;
            rand_range(B(3),B(4),[],[],1,N)
            rand_range(B(5),B(6),[],[],1,N)] ;
        
        % remove any points too close to the start and goal
        d_to_start = dist_point_to_points(W.start,P) ;
        P = P(:,d_to_start > W.buffer_start) ;
        
        d_to_goal = dist_point_to_points(W.goal,P) ;
        P = P(:,d_to_goal > W.buffer_start) ;
        
        % choose some of the points to make the path of the obstacle
        N = W.obs_dyn_N_points_on_path ;
        N_P = size(P,2) ;
        N_sample = min(N_P,N) ;
        idxs = 1:N_sample ;
        idxs = datasample(idxs,N_sample) ;
        P = P(:,idxs) ;
        
        % get the distances between the points
        P_loop = [P, P(:,1)] ;
        dp = vecnorm(diff(P_loop,1,2)) ;
        
        % if any of the distances are greater than the max allowable
        % distance, shrink all the distances
        dp_max = max(dp) ;
        if dp_max > W.obs_dyn_position_max_distance
            dp_scale = W.obs_dyn_position_max_distance ./ dp_max ;
            p_center = mean(P,2) ;
            P = dp_scale.*(P - p_center) + p_center ;
        end
        
        % make sure the positions are unique
        P = unique(P','rows','stable')' ;
    end
    
    %% collision check
    function out = collision_check(W,agent_info,check_full_traj)
        if nargin < 3
            check_full_traj = true ;
        end
        
        out = collision_check@zonotope_box_world(W,agent_info,check_full_traj) ;
        
        W.update_obstacle_times(agent_info.time(end))
    end
    
    function update_obstacle_times(W,t)
       % update obstacle times
        for idx = 1:W.N_obstacles
            o = W.obstacles{idx} ;
            if isa(o,'zonotope_dynamic_obstacle')
                o.update_time_and_position(t) ;
            end
        end 
    end
    
    function out = collision_check_loop(W,V_check,T_check)
        % V_check is a 3*N_V-by-N_T_check array; each column is the check
        % to be done at that time
        out = false ;
        
        % for each time...
        for check_idx = 1:length(T_check)
            t_idx = T_check(check_idx) ;
            
            obs_struct = W.get_obstacle_collision_check_struct(t_idx) ;
            
            V_check_idx = reshape(V_check(:,check_idx),3,[]) ;
            
            out = any(inpolyhedron(obs_struct,V_check_idx')) ;
            
            if out
                break
            end
            
%             % FOR DEBUGGING
%             v = V_check_idx ;
%             plot3(v(1,:),v(2,:),v(3,:),'r.')
%             plot3(V(:,1),V(:,2),V(:,3),'b.')
%             pause(0.01)
        end
    end
    
    function check_struct = get_obstacle_collision_check_struct(W,t_idx)
        % get the obstacle collision check structure at that time
        F = zeros(1,3) ;
        V = [] ;
        for o_idx = 1:W.N_obstacles
            o = W.obstacles{o_idx} ;
            
            if isa(o,'zonotope_dynamic_obstacle')
                o_str = o.get_collision_check_struct(t_idx) ;
                V_idx = o_str.vertices ;
                F_idx = o_str.faces ;
            else
                o_str = o.collision_check_input_struct ;
                V_idx = o_str.vertices ;
                F_idx = o_str.faces ;
            end
            
            V = [V ; V_idx] ;
            F = [F ; F_idx + 8*(o_idx-1)] ;
        end
        
        F = F(2:end,:) ;
        
        check_struct.faces = F ;
        check_struct.vertices = V ;
    end
    
    %% get world info
    function world_info = get_world_info(W,agent_info,~)
        % get world info
        world_info.goal = W.goal ;
        
        % get agent current position and time
        x = agent_info.position(:,end) ;
        t = agent_info.time(end) ;
        
        % return all obstacles that are within the sensor radius at the
        % current time
        O = {} ;
        oidx = 1 ;
        for idx = 1:W.N_obstacles
            o = W.obstacles{idx} ;
            
            % the the obstacle's vertices
            if isa(o,'cuboid_dynamic_obstacle')
                V = o.get_vertices_at_time(t) ;
            else
                V = o.vertices ;
            end
            
            % get the closest point on the obstacle to the current agent
            % position
            x_close = closest_point_on_box(x,V) ;
            
            % if the closest point is within the sensor radius, add that
            % obstacle to the list
            if norm(x_close - x) <= agent_info.sensor_radius
                O{oidx} = o ;
                oidx = oidx + 1 ;
            end
        end
        
        % output
        world_info.obstacles = O ;
    end
    
    %% plotting
    function plot_obstacles_over_time(W,time_vector)
        % W.plot_obstacles_over_time(time_vector)
        %
        % Plot all dynamic obstacles at the times in the provided time
        % vector.
        
        F_obs = [] ;
        V_obs = [] ;
        for idx = 1:length(W.obstacles)
            o = W.obstacles{idx} ;
            if isa(o,'zonotope_dynamic_obstacle')
                [F,V] = W.get_dynamic_obs_patch_data(o,time_vector) ;
                F_obs = [F_obs ; (F + size(V_obs,1))] ;
                V_obs = [V_obs ; V] ;
            end
        end
        
        if check_if_plot_is_available(W,'obstacles_over_time')
            W.plot_data.obstacles_over_time.Faces = F_obs ;
            W.plot_data.obstacles_over_time.Vertices = V_obs ;
        else
            patch_data = patch('Faces',F_obs,'Vertices',V_obs,...
                'FaceColor',W.plot_obstacles_over_time_face_color,...
                'FaceAlpha',W.plot_obstacles_over_time_face_opacity,...
                'EdgeColor',W.plot_obstacles_over_time_edge_color,...
                'EdgeAlpha',W.plot_obstacles_over_time_edge_opacity,...
                'LineWidth',W.plot_obstacles_over_time_edge_width,...
                'FaceLighting',W.plot_obstacles_over_time_lighting) ;
            W.plot_data.obstacles_over_time = patch_data ;
        end
    end
    
    function [F_out,V_out] = get_dynamic_obs_patch_data(W,o,time_vector)
        % get info for plotting obstacle
        F = o.plot_faces ;
        V = o.plot_vertices ;
        N = size(V,1) ;
        
        % set up output
        F_out = [] ;
        V_out = [] ;
        
        cnt = 0 ;
        
        for t_idx = time_vector
            p_t = o.get_position_at_time(t_idx) ;
            V_idx = V + repmat(p_t',N,1) ;
            V_out = [V_out ; V_idx] ;
            F_out = [F_out ; F + N*cnt] ;
            cnt = cnt + 1 ;
        end
    end
end
end