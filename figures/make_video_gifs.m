clear ; clc ; close all ;

%% user parameters
% video_file = 'dyn_video_3' ;
% video_path = '~/Research/videos/quadrotor/simulation/dynamic_obstacles/mat_files/' ;
video_file = 'best_paper_talk_dynamic_obstacles' ;
video_path = '~/Research/videos/quadrotor/simulation/DSCC_2019/mat_files/' ;
save_video_gif = true ;
plot_agent_trajectory = false ;
line_width = 2 ;
framerate = 30 ;
use_black_background = false ;

%% automated from here
load([video_path,video_file,'.mat'])
A.make_plot_input_data()

%% animation setup
fh = figure(1) ; clf ; hold on ; axis equal ; grid on

camlight
lighting flat
material dull
lightangle(10,70)

% plot
S.plot_at_time

% framerate
S.animation_time_discretization = 1/framerate ;

% set initial linewidths
[A,W] = set_linewidths(line_width,A,W) ;

%% behind quadrotor no traj or zonotopes
% initialize plot
clf ; hold on ; axis equal ; grid on
camlight
lighting flat
material dull
lightangle(10,70)
plot_at_time(A)
plot(W)
A.camera_view_style = 'behind' ;
A.camera_direction_offset = [-3;0;1] ;
A.camera_follow_distance = 3 ;
[A,W] = set_linewidths(line_width,A,W) ;
P.plot_zonotope_reach_set_flag = false ;
P.plot_desired_trajectory_flag = false ;
P.plot_desired_trajectory_line_width = 3 ;
W.obstacles{2}.plot_face_opacity = 1 ;
W.obstacles{2}.plot_face_color = [0.5 0.4 0.4] ;
W.obstacles{2}.center = W.obstacles{2}.center + [0;0;0.05] ;
W.obstacles{5}.plot_face_opacity = 0 ;
if use_black_background
    set(gca,'color','k') ;
    W.plot_data.goal_sphere.FaceAlpha = 0.5 ;
else
    set(gca,'color','w') ;
end

% plot
plot_at_time(A)

% animate
S.save_gif_filename = [video_file,'_behind.gif'] ;
S.animate(1,1,save_video_gif) ;
% S.animate(1,1,false) ;

%% behind quadrotor with zonotopes
% initialize plot
clf ; hold on ; axis equal ; grid on
camlight
lighting flat
material dull
lightangle(10,70)
plot_at_time(A)
plot(W)
A.camera_view_style = 'behind' ;
% A.camera_direction_offset = [-3;0;2] ;
% A.camera_follow_dist = 4 ;
A.camera_direction_offset = [-3;0;1] ;
A.camera_follow_distance = 3 ;
[A,W] = set_linewidths(line_width,A,W) ;
P.plot_zonotope_reach_set_flag = true ;
P.plot_zono_face_opacity = 0 ;
P.plot_zono_edge_width = 5 ;
P.plot_desired_trajectory_line_width = 3 ;
W.obstacles{2}.plot_face_opacity = 1 ;
W.obstacles{2}.plot_face_color = [0.5 0.4 0.4] ;
W.obstacles{2}.center = W.obstacles{2}.center + [0;0;0.05] ;
W.obstacles{5}.plot_face_opacity = 0 ;
if use_black_background
    set(gca,'color','k') ;
    W.plot_data.goal_sphere.FaceAlpha = 0.5 ;
else
    set(gca,'color','w') ;
end

% reset for animation
plot_at_time(A)

% animate
S.save_gif_filename = [video_file,'_behind_with_zono.gif'] ;
S.animate(1,1,save_video_gif) ;

%% angled view
for idx = 1:2 % this fixes a weird bug
% initialize plot
clf ; hold on ; axis equal ;
camlight
lighting flat
material dull
lightangle(10,70)
plot(W)
plot_at_time(A)
A.plot_view_style = 'none' ;
[A,W] = set_linewidths(3,A,W) ;
W.obstacles{2}.plot_face_opacity = 0.0 ;
if use_black_background
    set(gca,'color','k') ;
    W.plot_data.goal_sphere.FaceAlpha = 0.5 ;
else
    set(gca,'color','w') ;
end

% plot
plot_at_time(A)
camproj('orthographic')
campos([40  -90  410])
end

% animate
S.save_gif_filename = [video_file,'_angle.gif'] ;
S.animate(1,1,save_video_gif) ;

%% extra animation styles
% %% behind quadrotor with trajectory and zonotopes
% % initialize plot
% clf ; hold on ; axis equal ;
% camlight
% lighting flat
% material dull
% lightangle(10,70)
% plot(A)
% plot(W)
% A.plot_view_style = 'behind' ;
% A.camera_direction_offset = [-3;0;1] ;
% A.camera_follow_dist = 3 ;
% [A,W] = set_linewidths(line_width,A,W) ;
% P.plot_zonotopes = true ;
% 
% % reset for animation
% plot_at_time(A)
% 
% % animate
% S.save_gif_filename = [video_file,'_behind_with_zono_and_traj.gif'] ;
% S.animate(1,1,save_video_gif) ;
% 
% %% above world
% % initialize plot
% clf ; hold on ; axis equal ;
% camlight
% lighting flat
% material dull
% lightangle(10,70)
% plot(W)
% plot(A)
% A.plot_view_style = 'none' ;
% [A,W] = set_linewidths(3,A,W) ;
% P.plot_zonotopes = true ;
% 
% % plot
% plot_at_time(A)
% view(2)
% 
% % animate
% S.save_gif_filename = [video_file,'_above.gif'] ;
% S.animate(1,1,save_video_gif) ;


%% helper functions
function [A,W] = set_linewidths(line_width,A,W)
    A.plot_data.body.LineWidth = line_width ;
    A.plot_data.sides.LineWidth = line_width ;
    A.plot_data.rotor.LineWidth = line_width ;

    try
    A.plot_data.reference_trajectory.LineWidth = line_width ;
    A.plot_data.trajectory.LineWidth = line_width ;
    catch
    end

    for idx = 1:W.N_obstacles
    o = W.obstacles{idx} ;
    o.plot_data.body.LineWidth = line_width ;
    end
end