%% description
% This script tests the ode1_with_SO3 and ode2_with_SO3 functions
%
% Author: Shreyas Kousik
% Created: aeons ago
% Updated: 11 Aug 2020
%
%% user parameters
% motion (input time, velocity, and angular velocity)
T_in = [0 5] ;
v_in = 2*rand(3,2) - 1 ;
O_in = 0.1*rand(3,2) - 0.05 ;

% timing
dt = 0.1 ;

% plotting
plot_flag = true ;

%% automated from here
% create initial state and orientation
y0 = [zeros(3,1) ; O_in(:,1)] ;
R0 = eye(3) ;

% specify the indices for the angular velocity
O_idxs = 4:6 ;

% run integrator
[tout1,yout1,Rout1] = ode1_with_SO3(@(t,y,R) dyn(t,y,R,T_in,v_in,O_in),T_in,y0,R0,O_idxs,dt) ;
[tout2,yout2,Rout2] = ode2_with_SO3(@(t,y,R) dyn(t,y,R,T_in,v_in,O_in),T_in,y0,R0,O_idxs,dt) ;

% time integration
timeit(@() ode1_with_SO3(@(t,y,R) dyn(t,y,R,T_in,v_in,O_in),T_in,y0,R0,O_idxs,dt))
timeit(@() ode2_with_SO3(@(t,y,R) dyn(t,y,R,T_in,v_in,O_in),T_in,y0,R0,O_idxs,dt))

%% plotting
if plot_flag
    figure(1) ; cla ;
    plot_1 = plot_SE3_body_frame(Rout1(:,:,1),'Color',[1 0 0],'LineWidth',2) ;
    plot_2 = plot_SE3_body_frame(Rout2(:,:,1),'Color',[0 0 1],'LineWidth',2) ;
    for idx = 2:length(tout1)
        p1 = yout1(idx,1:3)' ;
        p2 = yout2(idx,1:3)' ;
        plot_1 = plot_SE3_body_frame(Rout1(:,:,idx),p1,'Data',plot_1) ;
        plot_2 = plot_SE3_body_frame(Rout2(:,:,idx),p2,'Data',plot_2) ;
        view(3)
        grid on
        axis equal
        axis(5.*[-1 1 -1 1 -1 1])
        pause(dt)
    end
end

%% dynamics
function y_dot = dyn(t,~,~,T_in,v_in,O_in)
    y_dot = match_trajectories(t,T_in,[v_in; O_in]) ;
    if any(isnan(y_dot))
        y_dot = [v_in(:,end); O_in(:,end)] ;
    end
end