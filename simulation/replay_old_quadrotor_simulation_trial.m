%% user parameters
filename = 'trial_20190410_0262.mat' ;
% filename = 'trial_20190410_0134.mat' ;


obs_color = [1 0.7 0.7] ;
obs_opacity = 0.5 ;

agent_camera_distance = 3 ; % default is 3
agent_camera_position = [-3;0;1] ; % default is [-3;0;1.5]
plot_agent_view = 'behind' ; % none, behind, above, or onboard

verbose_level = 10 ;


%% automated from here
load(filename)

agent_info = summary(1).agent_info ;
bounds = summary(1).bounds ;
obstacles = summary(1).obstacles ;
planner_info = summary(1).planner_info ;
goal = summary(1).goal ;

% create agent
A = quadrotor_agent('verbose',verbose_level,...
    'plot_view_style',plot_agent_view,...
    'camera_direction_offset',agent_camera_position,...
    'camera_follow_dist',agent_camera_distance) ;

% create world
W = zonotope_box_world('verbose',verbose_level,...
    'obs_color',obs_color,...
    'obs_opacity',obs_opacity,...
    'bounds',bounds,...
    'obstacles',obstacles,...
    'goal',goal) ;

% fill in agent state
A.time = agent_info.time ;
A.state = agent_info.state ;
A.attitude = agent_info.attitude ;

%% plot
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
animate(A)