classdef cuboid_dynamic_obstacle < obstacle
    properties
        current_time = 0 ;
        current_position = zeros(3,1) ;
        speed
        body_dimensions
        faces
        vertices
        plot_faces
        plot_vertices
        collision_check_input_struct
    end
    
    methods
        %% constructor
        function O = cuboid_dynamic_obstacle(L,W,H,position,speed_or_time)
            % O = cuboid_dynamic_obstacle(L,W,H,position,speed_or_time)
            %
            % Create a cuboid that travels along the polyline path given by
            % the position argument. If the speed_or_time argument is a 
            % scalar, then the obstacle traverses the path at that scalar
            % speed. If the speed_or_time input is a vector, then the
            % obstacle traverses the path and attains the given positions
            % at the given times.
            
            % make sure the first and last position are the same so the
            % obstacle moves in a loop from position to position
            p0 = position(:,1) ; p1 = position(:,end) ;
            position_got_looped_flag = false ;
            if vecnorm(p0 - p1) > 0
                position = [position, p0] ;
                position_got_looped_flag = true ;
            end
            
            O@obstacle('position',position,'dimension',3) ;
            O.body_dimensions = [L W H] ;
            O.current_position = p0 ;
            
            % create body patch info for plotting
            [F,V] = make_cuboid_for_patch(O.body_dimensions(1),O.body_dimensions(2),O.body_dimensions(3)) ;
            O.faces = F ;
            O.vertices = V ;
            O.plot_faces = F ;
            O.plot_vertices = V ;
            O.plot_data.body = [] ;
            
            % create object for collision check
            K = convhull(V) ;
            O.collision_check_input_struct.faces = K ;
            O.collision_check_input_struct.vertices = V ;
            
            % given position and speed, compute timing
            N_speed_or_time = length(speed_or_time) ;
            if N_speed_or_time == 1
                speed = speed_or_time ;
                O.speed = speed ;
                time = compute_timing_from_position_and_speed(position,speed) ;
            else
                time = speed_or_time ;
                
                if position_got_looped_flag
                    % if the position was turned into a loop, then the
                    % time will be missing an entry, so we'll use the
                    % obstacle's average speed over the entire trajectory
                    % to figure out how long it takes to complete the loop
                    total_distance = dist_polyline_cumulative(position(:,1:end-1)) ;
                    total_time = time(end) ;
                    average_speed = total_distance(end) / total_time ;
                    dist_to_close_loop = dist_point_to_points(position(:,end-1),position(:,end)) ;
                    time_to_close_loop = dist_to_close_loop / average_speed ;
                    time = [time, time(end) + time_to_close_loop] ;
                end
                
                if length(time) ~= size(position,2)
                    error(['The input time and position arrays do not ',...
                        'have the same number of columns!'])
                end
            end
            O.time = time ;
        end
        
        %% make struct for collision check
        function out = get_collision_check_struct(O,t)
            out = O.collision_check_input_struct ;
            out.vertices = O.get_vertices_at_time(t) ;
        end
        
        %% plot
        function plot(O)
            O.plot_at_time(O.current_time) ;
        end
        
        function plot_at_time(O,t)
            % get cuboid for plotting body
            F = O.plot_faces ;
            V = O.plot_vertices ;
            
            % move vertices to current time
            p_t = O.get_position_at_time(t) ;
            V = V + repmat(p_t',8,1) ;
            
            if check_if_plot_is_available(O,'body')
                O.plot_data.body.Faces = F ;
                O.plot_data.body.Vertices = V ;
            else
                patch_data = patch('Faces',F,'Vertices',V,...
                                   'FaceAlpha',O.plot_face_opacity,...
                                   'FaceColor',O.plot_face_color,...
                                   'EdgeColor',O.plot_edge_color,...
                                   'EdgeAlpha',O.plot_edge_opacity) ;
                O.plot_data.body = patch_data ;
            end
        end
        
        %% utility
        function p_t = get_position_at_time(O,t)
            % get the time within O.time
            t = mod(t,O.time(end)) ;
            
            % get the position
            p_t = match_trajectories(t,O.time,O.position) ;
        end
        
        function V_t = get_vertices_at_time(O,t)
            p_t = O.get_position_at_time(t) ;
            V_t = O.vertices ;
            V_t = V_t + repmat(p_t',8,1) ;
        end
        
        function update_time_and_position(O,t)
            O.current_time = t ;
            O.current_position = O.get_position_at_time(t) ;
        end
    end
end