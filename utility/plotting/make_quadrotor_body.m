function [F1,F2,V] = make_quadrotor_body()
    % make one quarter of the top and bottom of the body
    Vq = [+170   -8   0 ;
          +170   +8   0 ;
           +50   +15   -10 ;
            +15  +50   -10] ;
        
    % rotate copies of Vq
    R = eul2rotm([0 0 pi/2],'XYZ') ;
    V = [Vq ; (R*Vq')' ; (R*R*Vq')' ; (R*R*R*Vq')'] ;
    
    % create top and bottom of body
    N_V = size(V,1) ;
    Vlo = V + repmat([0 0 0],N_V,1) ;
    Vhi = V + repmat([0 0 -20],N_V,1) ;
    Vlo(:,3) = Vlo(:,3) ;
    V = 1e-3*[Vlo ; Vhi] ;
    F1 = [[1:N_V,1] ; [1:N_V,1]+N_V] ;
    
    % create faces to surround body
    F2 = repmat([1 2 18 17],16,1) + repmat((0:15)',1,4) ;
    F2(end,:) = [16 1 17 32] ;
    
    %% plotting
%     % 2D version
%     figure(1) ; cla ; hold on ; axis equal
%     plot(V(:,1),V(:,2),'b')
%     axis equal
% 
%     % 3D Version
%     figure(2) ; cla ; hold on ; axis equal
%     data_1 = patch('Faces',F1,'Vertices',V,'FaceColor',[0.7 0.7 1],'FaceAlpha',0.9) ;
%     data_2 = patch('Faces',F2,'Vertices',V,'FaceColor',[0.7 0.7 1],'FaceAlpha',0.9) ;
end