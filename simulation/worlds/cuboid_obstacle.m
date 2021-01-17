classdef cuboid_obstacle < obstacle
    properties
        center
        body_dimensions
        bounds
        faces
        vertices
        plot_faces
        plot_vertices
        collision_check_input_struct
        collision_check_time_discretization = 0.01 ; % s
    end
    
    methods
        %% constructor
        function O = cuboid_obstacle(L,W,H,C)            
            O@obstacle('position',C,'dimension',3) ;
            O.center = C ; % this mirrors the position field
            O.body_dimensions = [L W H] ;
            
            % generate the cuboid bounds
            B = box_to_bounds(L,W,H,C) ;
            O.bounds = B ;
            
            % create body patch info for plotting
            [F,V] = make_cuboid_for_patch(O.body_dimensions(1),O.body_dimensions(2),O.body_dimensions(3),C) ;
            O.faces = F ;
            O.vertices = V ;
            O.plot_faces = F ;
            O.plot_vertices = V ;
            O.plot_data.body = [] ;
            
            % create object for collision check
            K = convhull(V) ;
            O.collision_check_input_struct.faces = K ;
            O.collision_check_input_struct.vertices = V ;
        end
        
        %% utility
        function out = check_if_point_inside(O,p)
            out = (O.dist_to_point(p) == 0) ;
        end
        
        function d = dist_to_point(O,p)
            % d = C.dist_to_point(p)
            %
            % Return the (positive) distance d from the obstacle to the
            % point; d is 0 if
            % p is in or on C.
            
            d = dist_point_to_box(p,O.bounds) ;
        end
        
        function out = check_if_line_segment_intersects(O,l)
            % Given a line segment l = [l1, l2] \in \R^3, check if it
            % intersects the cuboid by using Smits' algorithm.
            
            % check that no points in the line are inside the obstacle
            out = any(O.check_if_point_inside(l)) ;
            
            if ~out
                B = O.bounds ;
                xlo = B(1) ; xhi = B(2) ;
                ylo = B(3) ; yhi = B(4) ;
                zlo = B(5) ; zhi = B(6) ;
                
                % get the projection of the line onto the three planes
                lx = l(1,:) ;
                ly = l(2,:) ;
                lz = l(3,:) ;
                
                % check if the xy line intersects the xy plane by checking if it straddles
                % one or both of the x boundary lines
                x_check = ((lx(1) <= xlo && lx(2) >= xlo) || (lx(2) <= xlo && lx(1) >= xlo) && ...
                    (lx(1) <= xhi && lx(2) >= xhi) || (lx(2) <= xhi && lx(1) >= xhi)) ;
                y_check = ((ly(1) <= ylo && ly(2) >= ylo) || (ly(2) <= ylo && ly(1) >= ylo) && ...
                    (ly(1) <= yhi && ly(2) >= yhi) || (ly(2) <= yhi && ly(1) >= yhi)) ;
                z_check = ((lz(1) <= zlo && lz(2) >= zlo) || (lz(2) <= zlo && lz(1) >= zlo) && ...
                    (lz(1) <= zhi && lz(2) >= zhi) || (lz(2) <= zhi && lz(1) >= zhi)) ;
                
                out = x_check & y_check & z_check ;
            end
        end
        
        %% plot
        function plot(O)
            % get info for plotting body
            F = O.plot_faces ;
            V = O.plot_vertices;
            
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
        
        function plot_at_time(O,~)
            O.plot ;
        end
    end
end