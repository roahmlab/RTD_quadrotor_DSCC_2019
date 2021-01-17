%% user parameters
% QC (quadcopter) initial conditions
p0 = [0;0;0] ; % position in x,y,z (leave this at the origin)
v0 = [1;0;0] ; % speed
a0 = [0;0;0] ; % acceleration

% desired future position (note that this is made to be no more than Dp_max
% from the robot)
pf = [1;1;0] ;

% timing
time_horizon = 4 ; % s
sample_time = 0.05 ; % s

% parameters (from https://arxiv.org/pdf/1304.6765.pdf)
g = 9.81 ;
m = 0.755 ;
J = diag([0.43 0.43 1.02]) ;
d = 0.169 ;
c_rf = 0.0132 ;
k_x = 12.8 ;
k_v = 4.22 ;
k_i = 1.28 ;
k_R = 0.65 ;
k_O = 0.11 ;
k_I = 0.06 ;
c_1 = 3.6 ;
c_2 = 0.8 ;
sg = 1 ;

%% automated from here
% create params struct
params.g = g ;
params.m = m ;
params.J = J ;
params.d = d ;
params.c_rf = c_rf ;
params.k_x = k_x ;
params.k_v = k_v ;
params.k_i = k_i ;
params.k_R = k_R ;
params.k_O = k_O ;
params.k_I = k_I ;
params.c_1 = c_1 ;
params.c_2 = c_2 ;
params.sg = sg ;

% create spline
[T_des,~,Z_des] = generate_QC_spline_traj(pf,v0,a0,time_horizon,sample_time) ;
T_des = [T_des, (T_des(end)+sample_time):sample_time:(T_des(end)+3)] ;
Z_des = [Z_des, repmat(Z_des(:,end),1,length(T_des)-length(Z_des))] ;

% create nominal inputs, which are advanced by 3*sample_time
U_des = [Z_des(4:6,4:end), zeros(3,3)] ;

% call dynamics
R0 = eye(3) ;
O0 = zeros(3,1) ;
x0 = [p0;v0;R0(:);O0;zeros(6,1)] ;
x0(1:6) = x0([1 4 2 5 3 6]) ;
tic
[T_out,Y_out] = ode23t(@(t,x) QC_dyn_SE3(t,x,T_des,U_des,Z_des,params),...
                    T_des,x0) ;
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
hold on ; axis equal ;
plot3(Z_des(1,:),Z_des(2,:),Z_des(3,:),'b--')
plot3(x_out,y_out,z_out)
view(3)
xlabel('x') ; ylabel('y') ; zlabel('z') ;
set(gca,'FontSize',15)

% positional error
figure(2) ; clf ;
subplot(1,3,1) ;
plot(T_des,x_err,'r',T_des,y_err,'g',T_des,z_err,'b') ;
axis([0,T_des(end),-0.2,0.2])
legend('x','y','z')
xlabel('t') ; ylabel('pos. error') ; title('Position')
set(gca,'FontSize',15)

% speed
subplot(1,3,2) ; hold on ;
plot(T_out,Y_out(2,:),'r',T_out,Y_out(4,:),'g',T_out,Y_out(6,:),'b') ;
plot(T_out,Z_des(4,:),'r--',T_out,Z_des(5,:),'g--',T_out,Z_des(6,:),'b--')
axis([0,T_des(end),-3,3])
legend('v_x','v_y','v_z')
xlabel('t') ; ylabel('speed (dashed is desired)') ; title('Speed')
set(gca,'FontSize',15)

% desired acceleration
subplot(1,3,3) ;
plot(T_out,ax_des,'r',T_out,ay_des,'g',T_out,az_des,'b')
axis([0,T_des(end),-4,4])
legend('a_x','a_y','a_z') ;
xlabel('t') ; ylabel('desired accel') ; title('Acceleration')
set(gca,'FontSize',15)