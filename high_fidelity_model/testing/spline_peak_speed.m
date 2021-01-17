%% user parameters
% init conds
v_0    = [0;0;0] ;
a_0    = [0;0;0] ;

% input
v_peak = [2;-1;1] ;

% timing
t_plan  = 0.25 ;
t_peak  = 0.5 ;
t_total = 1.5 ;
dt      = 0.025 ;

%% automated from here
% generate spline
[T,Z,Z_plan] = generate_spline_peak_speed(v_0,a_0,v_peak,t_plan,t_peak,t_total,dt) ;

%% plotting
figure(1) ; clf ;

% position
subplot(1,3,1) ; hold on ;
plot3(Z(1,:),Z(2,:),Z(3,:),'b')
plot3(Z_plan(1),Z_plan(2),Z_plan(3),'ro')
view(3)
title('position')
axis equal
set(gca,'FontSize',15)

% speed
subplot(1,3,2)
plot(T,Z(4,:),'r',T,Z(5,:),'g',T,Z(6,:),'b')
legend('v_x','v_y','v_z')
title('speed')
set(gca,'FontSize',15)

% speed
subplot(1,3,3)
plot(T,Z(7,:),'r',T,Z(8,:),'g',T,Z(9,:),'b')
legend('a_x','a_y','a_z')
title('accel')
set(gca,'FontSize',15)
