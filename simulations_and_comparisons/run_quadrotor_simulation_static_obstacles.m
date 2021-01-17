%% user parameters
% quadrotor
v_max = 5 ;
sensor_radius = 12 ;
move_method = 'integrator' ; % 'integrator' or 'direct'

% planner
t_plan = 0.75 ;
t_move = 0.75 ;
tracking_error_type = 'none' ; % 'none' or 'table' or 'constant'
use_fmincon = false ;
% frs_filename = 'quadrotor_FRS_v7_a10_dt0.02.mat' ;
frs_filename = 'temp_FRS.mat' ;
tbl_filename = 'quadrotor_tracking_error_table_dt0.02_vmax_5.25_zonotope.mat' ;

% world
bounds = [0 80 -10 10 0 10] ;
goal_radius = 1.5 ;
N_tall = 20 ;
N_wide = 20 ;
N_long = 0 ;
N_boxy = 20 ;
tall_dims = [0.5 1] ;
wide_dims = [0.25 0.75] ;
boxy_dims = [1 7 3 1] ;
obs_color = [1 0.7 0.7] ;
obs_opacity = 0.5 ;

% simulator
allow_replan_errors = true ;
verbose_level = 10 ;
max_sim_time = 1e6 ;
max_sim_iter = 50 ;

% plotting
line_width = 2 ;
plot_while_running = true ;
animate_after_run = false ;
plot_planning_times = false ;
agent_camera_distance = 3 ; % default is 3
agent_camera_position = [-3;0;1] ; % default is [-3;0;1.5]
agent_camera_view = 'behind' ; % none, behind, above, or onboard
plot_zono_FRS_flag = true ;
plot_zono_style = 'tube' ; % 'face' or 'tube'
save_gif = false ; % note that this will run really slowly at hi-res

%% automated from here
% load FRS
FRS = load(frs_filename) ;
if ~exist('tbl','var')
    disp('Loading tracking error table!')
    tbl = load(tbl_filename) ;
    tbl = tbl.tracking_error_table ;
else
    disp('Table already loaded!')
end

% create agent
A = quadrotor_agent('verbose',verbose_level,...
    'move_method',move_method,...
    'camera_view_style',agent_camera_view,...
    'camera_direction_offset',agent_camera_position,...
    'camera_follow_distance',agent_camera_distance,...
    'sensor_radius',sensor_radius,...
    'integrator_method','ode45') ;

% create world
W = zonotope_box_world('verbose',verbose_level,'N_tall',N_tall,...
    'N_wide',N_wide,'N_boxy',N_boxy,'N_long',N_long,...
    'goal_radius',goal_radius,...
    'obs_tall_size_range',tall_dims,...
    'obs_wide_size_range',wide_dims,...
    'obs_boxy_size_range',boxy_dims,...
    'bounds',bounds,...
    'use_wall_obstacles_flag',true) ;

% create planner
P = quadrotor_zono_RTD_planner(FRS,tbl,'t_move',0.5,'verbose',verbose_level,...
    'v_max',v_max,'plot_local_view',~agent_camera_view,'timeout',t_plan,...
    't_plan',t_plan,'t_move',t_move,...
    'N_sample',11,...
    'add_noise_to_waypoint',false,...
    'tracking_error_type',tracking_error_type,...
    'use_fmincon_online_flag',use_fmincon,...
    'plot_zonotope_reach_set_flag',plot_zono_FRS_flag,...
    'plot_zono_style',plot_zono_style) ;

% set up simulator
S = simulator(A,W,P,'verbose',verbose_level,...
    'allow_replan_errors',allow_replan_errors,...
    'max_sim_time',max_sim_time,'max_sim_iterations',max_sim_iter,...
    'plot_while_running',plot_while_running) ;

%% let it rip
summary = S.run() ;

%% stop agent at end
stop(A)

%% set linewidths
A.plot_data.body.LineWidth = line_width ;
A.plot_data.sides.LineWidth = line_width ;
A.plot_data.rotor.LineWidth = line_width ;
A.plot_data.reference_trajectory.LineWidth = line_width ;
A.plot_data.trajectory.LineWidth = line_width ;

for idx = 1:W.N_obstacles
    o = W.obstacles{idx} ;
    o.plot_data.body.LineWidth = line_width ;
end

%% planning times
if plot_planning_times
    figure(2) ; clf ; hold on ;
    plot(summary.planning_time)
    N = length(summary.planning_time) ;
    plot(summary.total_iterations,[P.t_plan, P.t_plan],'r--')
    xlabel('planning iterations')
    ylabel('planning time [s]')
    set(gca,'FontSize',15)
    ylim([0,P.t_plan+0.1])
end

%% report speed
[v_stats,a_stats] = A.report_speed_and_accel_stats ;

disp(['Speed min/mean/max: ',num2str(v_stats,'%0.2f ')])
disp(['Accel min/mean/max: ',num2str(a_stats,'%0.2f ')])

%% animation
if animate_after_run
    figure(1)
    clf
    hold on
    plot(W)
    axis equal
    plot_at_time(A)
    camlight
    lighting flat
    material dull
    lightangle(10,70)
    
    %% animate
    animate(S,1,1,save_gif)
end