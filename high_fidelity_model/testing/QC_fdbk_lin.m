%% user parameters
% inputs: T_input and U_input must have the same number of columns; the
% columns of U_input are linearly interpolated between the times in T_input
% and passed as desired speeds in (x,y,z) to the QC
T_input = [0 1 2 3 4] ;
U_input = [0 0 1 0 0;    % desired speed in x
           0 0 0 0 0 ;    % " y
           0 0 0 0 0] ; % " z

% initial condition
x0 = zeros(12,1) ;

% quadcopter params
L = 0.2 ;
Ix = 4.85e-3 ;
Iy = 4.85e-3 ;
Iz = 8.81e-3 ;

% outer loop control gains
k1 = 5 ;
k2 = 5 ;
k3 = 5 ;

% inner loop control gains
K2 = -80 ;
K3 = -80 ;
K4 = -80 ;
w2 = ((K2/2)^2)*Ix/L ;
w3 = ((K3/2)^2)*Iy/L ;
w4 = ((K4/2)^2)*Iz ;

% input bounds
max_rotor_rpm = 1e5 ; % rpm

%% automated from here
% create params struct
params.g = 9.81 ;
params.m = 0.5 ;
params.l = L ;
params.Ix = Ix ;
params.Iy = Iy ;
params.Iz = Iz ;
params.JR = 3.36e-5 ;
params.b = 2.92e-6 ;
params.d = 1.12e-7 ;
params.k1 = k1 ;
params.k2 = k2 ;
params.k3 = k3 ;
params.K2 = K2 ;
params.K3 = K3 ;
params.K4 = K4 ;
params.w2 = w2 ;
params.w3 = w3 ;
params.w4 = w4 ;
params.max_rotor_speed = max_rotor_rpm * 2*pi/60 ;

% call dynamics
[tout,yout] = ode45(@(t,x) QC_dyn_fdbk_lin(t,x,T_input,U_input,params),...
                    T_input([1 end]),x0) ;
                
%% plotting
figure(1) ; cla ;

% trajectory
subplot(1,2,1) ; axis equal ;
plot3(yout(:,1),yout(:,3),yout(:,5))
xlabel('x') ; ylabel('y') ; zlabel('z') ;

% speed
subplot(1,2,2) ;
plot(tout,yout(:,[2 4 6])) ;
xlabel('t') ; ylabel('speed')