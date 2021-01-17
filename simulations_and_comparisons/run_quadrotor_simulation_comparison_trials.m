%% user parameters
% quadrotor
v_max = 5 ;
sensor_radius = 12 ;

% planner
t_plan = 0.75 ;
t_move = 0.75 ;
use_fmincon = false ;
frs_filename = 'quadrotor_FRS_v7_a10_dt0.02.mat' ;
tbl_filename = 'quadrotor_tracking_error_table_dt0.02_vmax_5.25_zonotope.mat' ;

% world
bounds = [0 80 -10 10 0 10] ;
goal_radius = 1.5 ;
N_tall = 70 ;
N_wide = 20 ;
N_long = 0 ;
N_boxy = 30 ;
tall_dims = [0.5 1] ;
wide_dims = [0.1 0.2] ;
boxy_dims = [1 7 3 1] ;
obs_color = [1 0.7 0.7] ;
obs_opacity = 0.5 ;

% simulation
start_idx = 5 ;
end_idx = 500 ;
verbose_level = 10 ;
max_sim_time = 300 ;
max_sim_iter = 50 ;

% plotting
plot_during_sim = false ;
agent_camera_distance = 3 ; % default is 3
agent_camera_position = [-3;0;1] ; % default is [-3;0;1.5]
plot_agent_view = 'behind' ; % none, behind, above, or onboard
plot_zonotopes = true ;

% file handling
save_file_header = 'trial_20190410_' ;
file_location = '~/MATLAB/quadrotor_RTD/step4_simulation/trial_data/trials_20190410/' ;

%% automated from here
% load FRS (note, need to aldo load tracking error table)
FRS = load(frs_filename) ;
if ~exist('tbl','var')
    tbl = load(tbl_filename) ;
    tbl = tbl.tracking_error_table ;
end

%% run loop
tic
for idx = start_idx:end_idx
    % create agent
    A = quadrotor_agent('verbose',verbose_level,...
        'plot_view_style',plot_agent_view,...
        'camera_direction_offset',agent_camera_position,...
        'camera_follow_dist',agent_camera_distance,...
        'sensor_radius',sensor_radius) ;
    
    % create world
    W = zonotope_box_world('verbose',verbose_level,'N_tall',N_tall,...
        'N_wide',N_wide,'N_boxy',N_boxy,'N_long',N_long,...
        'obs_color',obs_color,...
        'obs_opacity',obs_opacity,...
        'goal_radius',goal_radius,...
        'obs_tall_size_range',tall_dims,...
        'obs_wide_size_range',wide_dims,...
        'obs_boxy_size_range',boxy_dims,...
        'bounds',bounds) ;
    
    % create planners
%     P1 = quadrotor_zono_RTD_planner(FRS,tbl,'t_move',0.5,'verbose',verbose_level,...
%         'v_max',v_max,'timeout',t_plan,...
%         't_plan',t_plan,'t_move',t_move,...
%         'tracking_error_type','none',...
%         'use_fmincon_online_flag',use_fmincon,...
%         'plot_zonotopes',plot_zonotopes,...
%         'name','using no tracking error') ;
    
    P2 = quadrotor_zono_RTD_planner(FRS,tbl,'t_move',0.5,'verbose',verbose_level,...
        'v_max',v_max,'plot_local_view',~plot_agent_view,'timeout',t_plan,...
        't_plan',t_plan,'t_move',t_move,...
        'tracking_error_type','constant',...
        'use_fmincon_online_flag',use_fmincon,...
        'plot_zonotopes',plot_zonotopes,...
        'name','using constant tracking error 0.1 m') ;
    
    P3 = quadrotor_zono_RTD_planner(FRS,tbl,'t_move',0.5,'verbose',verbose_level,...
        'v_max',v_max,'plot_local_view',~plot_agent_view,'timeout',t_plan,...
        't_plan',t_plan,'t_move',t_move,...
        'tracking_error_type','table',...
        'use_fmincon_online_flag',use_fmincon,...
        'plot_zonotopes',plot_zonotopes,...
        'name','using computed tracking error table') ;
    
%     P = {P1 P2 P3} ;
    P = {P2 P3} ;
    
    % set up simulator
    S = simulator(A,W,P,'verbose',verbose_level,'allow_replan_errors',false,...
        'max_sim_time',max_sim_time,'max_sim_iterations',max_sim_iter,...
        'plot_while_running',plot_during_sim) ;
    
    % run simulation
    summary = S.run ;

    % save summary
    filename = [file_location,save_file_header,num2str(idx,'%04.f'),'.mat'] ;
    save(filename,'summary') 
toc
end