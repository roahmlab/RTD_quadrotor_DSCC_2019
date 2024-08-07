%% user parameters
% quadrotor
v_max = 5 ;
sensor_radius = 12 ;
move_method = 'integrator' ; % 'integrator' or 'direct'

% planner
t_plan = 0.75 ;
t_move = 0.75 ;
tracking_error_type = 'table' ; % 'none' or 'table' or 'constant'
use_fmincon = false ;
frs_filename = 'quadrotor_FRS_v7_a10_dt0.02.mat' ;
tbl_filename = 'quadrotor_tracking_error_table_dt0.02_vmax_5.25_zonotope.mat' ;

% world
bounds = [0 80 -10 10 0 10] ;
goal_radius = 1.5 ;
N_tall = 20 ; % number of "tall" obstacles
N_wide = 20 ; % number of "wide" obstacles
N_long = 0 ; % number of "long" obstacles
N_boxy = 20 ; % number of "boxy" obstacles
tall_dims = [0.5 1] ;
wide_dims = [0.25 0.75] ;
boxy_dims = [1 7 3 1] ;

% plotting
agent_camera_distance = 3 ; % default is 3
agent_camera_position = [-3;0;1] ; % default is [-3;0;1.5]
agent_camera_view = 'behind' ; % none, behind, above, or onboard
plot_zono_FRS_flag = true ;
plot_zono_style = 'tube' ; % 'face' or 'tube'

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

%% let it rip
% get agent and world ready
W.reset() ;
A.reset(W.start) ; 

% get planner ready
agent_info = A.get_agent_info() ;
world_info = W.get_world_info(agent_info,P) ;
P.setup(agent_info,world_info) ;

% call planner
[T_nom,U_nom,Z_nom] = P.replan(agent_info,world_info) ;
A.move(P.t_move,T_nom,U_nom,Z_nom) ;

%% plotting
figure(1) ; cla ; hold on ; axis equal ; axis(W.bounds)

plot(W)
plot(A)
plot(P)

set_plot_linewidths(1.5)