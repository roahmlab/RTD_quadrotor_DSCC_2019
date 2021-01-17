%% user parameters
% QC (quadcopter) initial conditions for spline
p0 = [0;0;0] ; % position in x,y,z (leave this at the origin)
v0 = [0;0;0] ; % speed
a0 = [0;0;0] ; % acceleration

% desired future position (note that this is made to be no more than Dp_max
% from the robot)
pf = [2;1;0] ;
Dp_max = 10 ; % m

% timing
time_horizon = 2 ;
sample_time = 0.05 ;

% control gains
p_xy = 10 ;
p_z = 10 ;
d_xy = 10 ;
d_z = 10 ;
p_rp = 10 ;
p_yaw = 10 ;
p_pq = 10 ;
p_r = 10 ;

% qc properties
min_rotor_speed = 0 ;
max_rotor_speed = 7800 ; % rpm
I_xx = 3.3e-3 ;
I_yy = I_xx ;
I_zz = 6.6e-3 ;
k_F = 6.11e-8 ;
k_M = 1.5e-9 ;

%% automated from here
% create params struct
params.m = 0.5 ;
params.g = 9.81 ;
params.L = 0.2 ;
params.J = diag([I_xx,I_yy,I_zz]) ;
params.J_inv = inv(params.J) ;
params.p_xy = p_xy ;
params.d_xy = d_xy ;
params.p_z = p_z ;
params.d_z = d_z ;
params.p_rp = p_rp ;
params.p_yaw = p_yaw ;
params.p_pq = p_pq ;
params.p_r = p_r ;
params.min_rotor_speed = min_rotor_speed ;
params.max_rotor_speed = max_rotor_speed ;

% shrink pf
Dp_cur = norm(pf - p0) ;
if Dp_cur > Dp_max
    disp('shrinking pf')
    pf = pf*Dp_max./Dp_cur ;
    disp(pf)
end

% create spline
[T_des,~,Z_des] = generate_QC_spline_traj(pf,v0,a0,time_horizon,sample_time) ;
T_des = [T_des, (T_des(end)+sample_time):sample_time:(T_des(end)+3)] ;
Z_des = [Z_des, repmat(Z_des(:,end),1,length(T_des)-length(Z_des))] ;

% call dynamics
x0 = [zeros(6,1);1;zeros(6,1)] ;
tic
[T_out,Y_out] = ode45(@(t,x) QC_dyn_faessler_quat(t,x,T_des,Z_des,params),T_des,x0) ;
toc

T_out = T_out' ;
Y_out = Y_out' ;
                
% reinterpolate spline to match T_in
Z_des = matchTrajectories(T_des,T_des,Z_des) ;
Z_des(:,end) = Z_des(:,end-1) ;

Z_out = matchTrajectories(T_des,T_out,Y_out) ;
T_out = T_des ;

x_out = Z_out(1,:) ;
y_out = Z_out(2,:) ;
z_out = Z_out(3,:) ;

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
figure(1) ; clf ;

% trajectory
subplot(1,4,1) ; hold on ; axis equal ;
plot3(Z_des(1,:),Z_des(2,:),Z_des(3,:),'b--')
plot3(x_out,y_out,z_out)
view(3)
xlabel('x') ; ylabel('y') ; zlabel('z') ;
set(gca,'FontSize',15)

% positional error
subplot(1,4,2) ;
plot(T_des,x_err,'r',T_des,y_err,'g',T_des,z_err,'b') ;
axis([0,T_des(end),-0.2,0.2])
legend('x','y','z')
xlabel('t') ; ylabel('pos. error') ; title('Position')
set(gca,'FontSize',15)

% speed
subplot(1,4,3) ; hold on ;
plot(T_out,Y_out(4,:),'r',T_out,Y_out(5,:),'g',T_out,Y_out(6,:),'b') ;
plot(T_out,Z_des(4,:),'r--',T_out,Z_des(5,:),'g--',T_out,Z_des(6,:),'b--')
axis([0,T_des(end),-3,3])
legend('v_x','v_y','v_z')
xlabel('t') ; ylabel('speed (dashed is desired)') ; title('Speed')
set(gca,'FontSize',15)

% desired acceleration
subplot(1,4,4) ;
plot(T_out,ax_des,'r',T_out,ay_des,'g',T_out,az_des,'b')
axis([0,T_des(end),-4,4])
legend('a_x','a_y','a_z') ;
xlabel('t') ; ylabel('desired accel') ; title('Acceleration')
set(gca,'FontSize',15)