classdef zonotope_human_obstacle < zonotope_dynamic_obstacle
    methods
        %% constructor
        function O = zonotope_human_obstacle(position,speed)
            % set the position trajectory's height at 1 m (the obstacle
            % will be 2m tall)
            position(3,:) = 1 ;
            
            % call superclass
            O@zonotope_dynamic_obstacle(0.4,0.4,2,position,speed) ;
            
            % make head
            % make head 0.2 m in diameter
            [sx,sy,sz] = sphere(20) ;
            [hf, hv] = surf2patch(sx,sy,sz) ;
            hv = 0.2.*hv ;
            
            % make head height at 1.5 m
            hv(:,3) = hv(:,3) + 1.5 ;
            
            % make body
            r = 0.2.*[0, ones(1,12),cos(linspace(0,pi/2,7))] ;
            [x,y,z] = cylinder(r) ;
            [cf,cv] = surf2patch(x,y,z) ;
            
            % stretch body out vertically
            cv(:,3) = 1.3.*cv(:,3) ;
            
            % make one patch set of faces and vertices
            F = [hf ; cf + size(hv,1)] ;
            V = [hv ; cv] ;
            
            % center body at (0,0,0)
            V(:,3) = V(:,3) - (1.7/2) ;
            
            % set plot properties
            O.plot_faces = F ;
            O.plot_vertices = V ;
            O.plot_edge_color = [0.5 0 0] ;
            O.plot_edge_opacity = 0 ;
            
            % set up plot data
            O.plot_data.body = [] ;
        end
        
        %% plotting
        function plot_at_time(O,t)
            % get cuboid for plotting body
            F = O.plot_faces ;
            V = O.plot_vertices ;
            
            % FOR DEBUGGING:
            % F = O.collision_check_input_struct.faces ;
            % V = O.collision_check_input_struct.vertices ;
            
            % move vertices to current time, but leave height as is
            p_t = O.get_position_at_time(t) ;
            V = V + repmat(p_t',size(V,1),1) ;
            
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
    end
end