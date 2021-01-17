%% plotting setup
% agent setup
A = quadrotor_agent('plot_view_behind_robot',false) ;
A.plot_frame_scale = 1 ;
v_0 = [0;1;-0.1] ;
a_0 = zeros(3,1) ;
A.attitude = eul2rotm([0.2 0 0],'XYZ') ;
A.state(A.velocity_indices) = v_0 ;

% obstacle setup
o = zonotope_obstacle(0.5,1.1,1,[2;0;0]) ;
o.body_plot_face_color = [1 0.5 0.5] ;
o.body_face_opacity = 1 ;

% planner setup
frs_filename = 'quadrotor_FRS_v7_a10_dt0p02_small_body.mat' ;
FRS = load(frs_filename) ;
TBL = load('quadrotor_tracking_error_table_vmax_5.25.mat') ;
TBL = TBL.tracking_error_table ;
P = quadrotor_zono_RTD_planner(FRS,TBL,'use_fmincon_online_flag',false,...
    'add_noise_to_waypoint',false) ;

% create info for planning
agent_info = A.get_agent_info ;
g = [3;-1;0.1] ;
world_info.goal = g ;
world_info.obstacles{1} = o ;
P.setup(agent_info,world_info) ;

% run planner
[T,U,Z] = P.replan(agent_info,world_info) ;

% create info for K space plot
position_dimensions = [1; 6; 11];
param_dimensions = [4; 9; 14];
IC_dim = [2; 7; 12; 3; 8; 13];
IC = [v_0; a_0];

FRS = FRS.Rcont ;

R_obs = {} ;
for idx = 1:length(FRS)
    FRS_idx = zonotope_slice(FRS{idx}{1}, IC_dim, IC) ;
    R_obs{idx} = compute_unsafe(FRS_idx, {o.zono}, position_dimensions, param_dimensions);
end

%% plot
% setup
fh = figure(1) ; clf ;
P.zono_plot_edge_color = [0 0 1] ;
P.zono_plot_edge_opacity = 0.01 ;
P.zono_plot_face_opacity = 0.05;
o.body_plot_edge_color = [0 0 0] ;
o.body_edge_opacity = 1 ;

% plotting
subplot(1,2,2) ; hold on ;
plot(A)
plot(o)
plot(P)
plot3(g(1),g(2),g(3),'kx','MarkerSize',15)

% lighting
% H = light ;
% H.Position = [1;1;1] ;
% lightangle(45,45)
% material dull

% view
view(3)
grid on
axis([-1 3.5 -0.5 2.75 -1.5 1.5])
axis equal
set(gca,'FontSize',15)

% plot reachable set area
subplot(1,2,1) ; hold on ;
for idx = 1:length(FRS)
    if ~isempty(R_obs{idx}{1})
        fake_c = (R_obs{idx}{1}(1:2, 2) + R_obs{idx}{1}(1:2, 1))/2;
        fake_G = [(R_obs{idx}{1}(1, 2) - R_obs{idx}{1}(1, 1))/2, 0 ;
            0, (R_obs{idx}{1}(2, 2) - R_obs{idx}{1}(2, 1))/2];
        Zproj = zonotope([fake_c, fake_G]);
        Zproj = reduce(Zproj,'girard',3);
        Zpts = polygon(Zproj)';
        p1 = fill(Zpts(:, 1), Zpts(:, 2),'r');
        p1.FaceAlpha = 1;
        p1.FaceColor = [1 0.5 0.5] ;
        p1.EdgeAlpha = 0;
    end
end
axis([0 5 -5 2])
set(gca,'FontSize',15)

% figure size
 pos = [0    9.1806   20.1389    6.1528] ;
 set(fh,'Units','Inches','PaperPositionMode','Auto','Position',pos,...
     'PaperSize',[pos(3) pos(4)])