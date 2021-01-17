%% NOTE THIS IS HACKED TOGETHER FOR NOW
% The simulator should be able to just take in worlds, and reset the agent
% and planner elegantly, and save summaries on its own... I just haven't
% written that code yet.
%
% Shreyas, 2019 Apr 1, 1:05 AM
%
%% user parameters
% quadrotor
v_max = 5.0 ;
int_dt = 0.005 ;

% planner
t_plan = 0.75 ;
t_move = 0.75 ;
add_tracking_error = false ;
use_fmincon = false ;
frs_filename = 'quadrotor_FRS_10cm_buffer.mat' ; % alt:  'quadrotor_FRS_v0_6_a0_10.mat'

% world
goal_radius = 2 ;
N_tall = 50 ;
N_wide = 10 ;
N_long = 0 ;
N_boxy = 30 ;
tall_dims = [0.25 0.5] ;
wide_dims = [0.25 0.5] ;
obs_color = [1 0.7 0.7] ;
obs_opacity = 0.7 ;

% simulation
start_idx = 1 ;
N_worlds = 1000 ;
verbose_level = 2 ;
max_sim_time = 300 ;
max_sim_iter = 300 ;

% plotting
plot_during_sim = true ;
agent_camera_position = [-3;0;1.5] ; % default is [-3;0;1.5]
plot_agent_view = true ;

% file handling
save_file_header = 'trial_20190402_' ;
file_location = '~/MATLAB/quadrotor_RTD/step4_simulation/trial_data/trials_20190402/' ;

%% automated from here
% load FRS and tracking error table
frs_file = load(frs_filename) ;
tbl_file = load('quadrotor_tracking_error_table_vmax_5.25.mat') ;

FRS = frs_file.Rcont ;
tbl = tbl_file.tracking_error_table ;

%% run loop
tic
for idx = start_idx:(start_idx+N_worlds)
    % create agent
    A = quadrotor_agent('verbose',verbose_level,'set_axes_when_animating',false,...
            'plot_view_behind_robot',plot_agent_view,...
            'integrator_time_discretization',int_dt,...
            'camera_direction_offset',agent_camera_position,...
            'camera_follow_dist',agent_camera_distance) ;

    % create world
    W = zonotope_box_world('verbose',verbose_level,'N_tall',N_tall,...
                        'N_wide',N_wide,'N_boxy',N_boxy,'N_long',N_long,...
                        'obs_color',obs_color,...
                        'obs_opacity',obs_opacity,...
                        'goal_radius',goal_radius,...
                        'obs_tall_size_range',tall_dims,...
                        'obs_wide_size_range',wide_dims) ;

    % create planner
    P = quadrotor_zono_RTD_planner(FRS,tbl,'verbose',verbose_level,...
            'v_max',v_max,'plot_local_view',~plot_agent_view,'timeout',t_plan,...
            't_plan',t_plan,'t_move',t_move,...
            'add_tracking_error',add_tracking_error,...
            'use_fmincon_online_flag',use_fmincon) ;  
                
	% set up simulator
    S = simulator(A,W,P,'verbose',verbose_level,'allow_replan_errors',false,...
            'max_sim_time',max_sim_time,'max_sim_iterations',max_sim_iter,...
            'plot_while_running',plot_during_sim) ;
        
	% run simulation
    summary = S.run() ;
    
    % save summary
    filename = [file_location,save_file_header,num2str(idx,'%03.f'),'.mat'] ;
    save(filename,'summary')    
toc
end