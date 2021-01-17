%% user parameters
% init conds
v_0    = [0;0;0] ;
a_0    = [0;0;0] ;

% input
v_peak = [-3;2;1] ;

% timing
t_plan  = 0.5 ;
t_peak  = 1 ;
t_total = 3 ;
dt      = 0.025 ;

% control gains
kx = 20 ;
ky = 20 ;
kz = 20 ;
ki = 1 ;

%% automated from here
% make agent
A = QC_SDRE_agent ;
A.params.k1 = kx ;
A.params.k2 = ky ;
A.params.k3 = kz ;
A.params.ki = ki ;

% generate spline
[T_des,Z_des,Z_plan] = generate_spline_peak_speed(v_0,a_0,v_peak,t_plan,t_peak,t_total,dt) ;

% add braking to end of spline
T_des = [T_des, (T_des(end)+dt):dt:(T_des(end)+4)] ;
Z_des = [Z_des, repmat(Z_des(:,end),1,length(T_des)-length(Z_des))] ;

% create nominal inputs, which are advanced by a bit
U_des = [Z_des(4:6,4:end), zeros(3,3)] ;
% U_des = Z_des(4:6,:) ;

% move agent
A.reset(zeros(A.n_states,1))
tic
A.move(T_des(end),T_des,U_des,Z_des)
toc

% get agent's state output
T_out = A.time ;
Z_out = A.state ;

% reinterpolate spline to match T_des
Z_out = matchTrajectories(T_des,T_out,Z_out) ;
Z_des(:,end) = Z_des(:,end-1) ;

x_out = Z_out(1,:) ;
y_out = Z_out(3,:) ;
z_out = Z_out(5,:) ;

vx_out = Z_out(2,:) ;
vy_out = Z_out(4,:) ;
vz_out = Z_out(6,:) ;

% compute position error
x_des = Z_des(1,:) ;
y_des = Z_des(2,:) ;
z_des = Z_des(3,:) ;

x_err = x_out - x_des ;
y_err = y_out - y_des ;
z_err = z_out - z_des ;

% get desired acceleration
ax_des = Z_des(7,:) ;
ay_des = Z_des(8,:) ;
az_des = Z_des(9,:) ;

%% plotting
% trajectory
figure(1) ; clf ;
hold on ; axis equal ;
plot3(Z_des(1,:),Z_des(2,:),Z_des(3,:),'b--')
plot(A)
plot3(Z_plan(1),Z_plan(2),Z_plan(3),'ro')
view(3)
xlabel('x') ; ylabel('y') ; zlabel('z') ;
title('trajectory')
set(gca,'FontSize',15)


figure(2) ; clf ;
% position error
subplot(1,3,1) ; hold on ;
plot(T_des,x_err,'r',T_des,y_err,'g',T_des,z_err,'b')
legend('x','y','z')
title('position error')
set(gca,'FontSize',15)

% speed
subplot(1,3,2) ; hold on ;
plot(T_des,Z_des(4,:),'r--',T_des,Z_des(5,:),'g--',T_des,Z_des(6,:),'b--')
plot(T_des,vx_out,'r',T_des,vy_out,'g',T_des,vz_out,'b')
legend('v_x','v_y','v_z')
title('speed')
set(gca,'FontSize',15)

% acceleration
subplot(1,3,3)
plot(T_des,Z_des(7,:),'r',T_des,Z_des(8,:),'g',T_des,Z_des(9,:),'b')
legend('a_x','a_y','a_z')
title('accel')
set(gca,'FontSize',15)
