%% user parameters
% QC (quadcopter) initial conditions
p0 = [0;0;0] ; % position in x,y,z (leave this at the origin)
v0 = [0;0;0] ; % speed
a0 = [0;0;0] ; % acceleration

% desired future position (note that this is made to be no more than Dp_max
% from the robot)
pf = [0;0;-3] ;
Dp_max = 3 ; % m

% timing
time_horizon = 3 ; % s
sample_time = 0.05 ; % s

% control gains
k1 = 40 ;
k2 = 40 ;
k3 = 40 ;

%% automated from here
% create params struct
params.g = 9.81 ;
params.m = 0.5 ;
params.l = 0.2 ;
params.Ix = 4.85e-3 ;
params.Iy = params.Ix ;
params.Iz = 8.81e-3 ;
params.JR = 3.36e-5 ;
params.b = 2.92e-6 ;
params.d = 1.12e-7 ;
params.k1 = k1 ;
params.k2 = k2 ;
params.k3 = k3 ;

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

% create nominal inputs, which are advanced by 3*sample_time
U_des = [Z_des(4:6,4:end), zeros(3,3)] ;

% call dynamics
x0 = [p0;v0;zeros(6,1)] ;
x0(1:6) = x0([1 4 2 5 3 6]) ;
tic
[T_out,Y_out] = ode45(@(t,x) QC_dyn_SDRE(t,x,T_des,U_des,params),...
                    T_des,x0) ;
toc             
% testing with integral control (this works poorly because it's PI about
% the position error)
% e0 = U_des(:,1) - v0 ;
% [T_out,Y_out] = ode45(@(t,x) QC_dyn_SDRE_PI(t,x,T_des,U_des,params),...
%                     T_des,[x0;e0]) ;

T_out = T_out' ;
Y_out = Y_out' ;
                
% reinterpolate spline to match T_in
Z_des = matchTrajectories(T_des,T_des,Z_des) ;
Z_des(:,end) = Z_des(:,end-1) ;

Z_out = matchTrajectories(T_des,T_out,Y_out) ;
T_out = T_des ;

x_out = Z_out(1,:) ;
y_out = Z_out(3,:) ;
z_out = Z_out(5,:) ;

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
plot(T_out,Y_out(2,:),'r',T_out,Y_out(4,:),'g',T_out,Y_out(6,:),'b') ;
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