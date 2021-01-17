%% user parameters
% timing
t_total = 7 ;
t_brake = 1 ;
t_sample = 0.1 ;

% traj params
kx = 1 ; % m/s
ky = -0.2 ;
kz = 0.0 ;

% quadcopter parameters
gravity = 9.81 ;
angle_gain = 0.2 ;
thrust_gain = 0.91 ;
Sx_bounds = [-10,10] ; % in degrees (gets converted to rad automatically)
Sy_bounds = [-10,10] ; % in degrees
Tz_bounds = [0, 2*gravity] ; % in m/s^2

% quadcopter initial condition (position and angles are zero)
vx0 = 0.1 ;
vy0 = 0 ;
vz0 = 0 ;
wx0 = 0 ;
wy0 = 0 ;

% PID parameters
kPx = 0.01 ;
kPy = 0.01 ;
kPz = 1 ;

kDx = 0.01 ;
kDy = 0.01 ;
kDz = 0 ;

kIx = 0.01 ;
kIy = 0.001 ;
kIz = 0.1 ;

%% automated from here
% set up traj params object
traj_params.t_total = t_total ;
traj_params.t_brake = t_brake ;
traj_params.t_sample = t_sample ;
traj_params.kx = kx ;
traj_params.ky = ky ;
traj_params.kz = kz ;

% make desired trajectory
[t_des,traj_des] = make_desired_traj_line(traj_params) ;

% set up quadcopter params
QC_params.gravity = gravity ;
QC_params.angle_gain = angle_gain ;
QC_params.thrust_gain = thrust_gain ;
QC_params.Sx_bounds = deg2rad(Sx_bounds) ;
QC_params.Sy_bounds = deg2rad(Sy_bounds) ;
QC_params.Tz_bounds = Tz_bounds ;

% set up PID params
PID_params.kPx = kPx ;
PID_params.kPy = kPy ;
PID_params.kPz = kPz ;
    
PID_params.kDx = kDx ;
PID_params.kDy = kDy ;
PID_params.kDz = kDz ;
    
PID_params.kIx = kIx ;
PID_params.kIy = kIy ;
PID_params.kIz = kIz ;

%% track trajectory
% set up initial condition
z0 = [0, vx0, 0, wx0, 0, vy0, 0, wy0, 0, vz0, 0, 0, 0] ;

% call ode45
[tout,zout] = ode45(@(t,z) quadcopter_dynamics_PID(t,z,t_des,traj_des,QC_params,PID_params),...
                    [0,t_total],z0);

%% plotting
figure(1) ; cla ; axis equal ; hold on ;
xlabel('x') ; ylabel('y') ; zlabel('z') ;

% extract desired traj
px_des = traj_des(1,:) ;
py_des = traj_des(3,:) ;
pz_des = traj_des(5,:) ;

vx_des = traj_des(2,:) ;
vy_des = traj_des(4,:) ;
vz_des = traj_des(6,:) ;

% get QC state
px_traj = zout(:,1) ;
py_traj = zout(:,5) ;
pz_traj = zout(:,9) ;

% plot
plot3(px_des,py_des,pz_des,'b--')
plot3(px_traj,py_traj,pz_traj,'r-')