%% plotting setup
% agent setup
A = quadrotor_agent('plot_view_behind_robot',false) ;
v_0 = [0;1;-0.1] ;
a_0 = zeros(3,1) ;
A.attitude = eul2rotm([0.2 0 -0.1],'XYZ') ;
A.state(A.velocity_indices) = v_0 ;

% coord frame setup
F = rigid_body_agent_SE3(1,1) ;
F.state(F.position_indices) = [0.5;0.5;0.5] ;

% obstacle setup
o = zonotope_obstacle(0.5,1.1,1,[2;0;0]) ;
o.body_plot_face_color = [1 0.5 0.5] ;
o.body_face_opacity = 1 ;

%% plot
% setup
fh = figure(1) ; clf ; hold on ;

% agent plotting setup
A.plot_frame_scale = 0.5 ;
F.plot_frame_scale = 0.6 ;

% obstacle plotting setup
o.body_plot_edge_color = [0 0 0] ;
o.body_edge_opacity = 1 ;

% plotting
plot(A)
plot(F)
plot(o)

% view
campos([ -13.8507  -10.7168   14.9693])
camlight
lightangle(-135,45)
material dull
grid on
axis equal
axis([-0.5 2.75 -2 1 -0.5 1])
set(gca,'FontSize',15,'Projection','Perspective')

% figure size
pos = [   0    8.1806   13.3333    7.1528] ;
set(fh,'Units','Inches','PaperPositionMode','Auto','Position',pos,...
 'PaperSize',[pos(3) pos(4)])