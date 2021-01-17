%% user parameters
% QC (quadcopter) initial conditions
p0 = [0;0;0] ; % position in x,y,z (leave this at the origin)
v0 = [0;0;0] ; % speed
a0 = [0;0;0] ; % acceleration

% open loop inputs
T_in = [0 1] ;
U_in = 4.48e3* [2 3 ;
                1 1 ;
                1 1 ;
                2 3] ;

% control gains
k_p = 10 ;
k_d = 10 ;
k_ang = 300 ;
k_ang_vel = 30 ;

% qc properties
m = 0.5 ;
L = 0.55/2 ;
I_xx = 3.3e-3 ;
I_yy = I_xx ;
I_zz = 6.6e-3 ;
k_F = 6.11e-8 ;
k_M = 1.5e-9 ;
min_rotor_speed = 0 ;
max_rotor_speed = 9000 ; % rpm

%% automated from here
% create params struct
params.m = m ;
params.g = 9.81 ;
params.L = L ;
params.I = diag([I_xx,I_yy,I_zz]) ;
params.k_F = k_F ;
params.k_M = k_M ;
params.K_p = diag([k_p k_p k_p]) ;
params.K_d = diag([k_d k_d k_d]) ;
params.k_pph = k_ang ;
params.k_dph = k_ang_vel ;
params.k_pth = k_ang ;
params.k_dph = k_ang_vel ;
params.k_pps = k_ang ;
params.k_dps = k_ang_vel ;
params.min_rotor_speed = min_rotor_speed ;
params.max_rotor_speed = max_rotor_speed ;

% call dynamics
x0 = [p0;v0;zeros(9,1)] ;
tic
[T_out,Z_out] = ode45(@(t,x) QC_dyn_GRASP_open_loop(t,x,T_in,U_in,params),...
                    [0,T_in(end)],x0) ;
toc

T_out = T_out' ;
Z_out = Z_out' ;

x_out = Z_out(1,:) ;
y_out = Z_out(2,:) ;
z_out = Z_out(3,:) ;
                
%% plotting
figure(1) ; clf ;

% trajectory
hold on ; axis equal ;
plot3(x_out,y_out,z_out,'b')
view(3)
xlabel('x') ; ylabel('y') ; zlabel('z') ;
set(gca,'FontSize',15)
