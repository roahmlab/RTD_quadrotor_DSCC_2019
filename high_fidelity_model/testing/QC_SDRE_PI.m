%% user parameters
% inputs: T_input and U_input must have the same number of columns; the
% columns of U_input are linearly interpolated between the times in T_input
% and passed as desired speeds in (x,y,z) to the QC
T_input = [0 1 2 3 4] ;
U_input = [0 1 0 0 0;    % desired speed in x
           0 0 0 0 0 ;    % " y
           0 0 0 0 0] ; % " z

% initial state and error
x0 = zeros(12,1) ;

% control gains
kpx = 5 ;
kpy = 5 ;
kpz = 5 ;
kix = 1 ;
kiy = 1 ;
kiz = 1 ;

% input bounds
max_rotor_rpm = 1e5 ; % rpm

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
params.kpx = kpx ;
params.kpy = kpy ;
params.kpz = kpz ;
params.kix = kix ;
params.kiy = kiy ;
params.kiz = kiz ;
params.max_rotor_speed = max_rotor_rpm * 2*pi/60 ;

% get initial velocity error
e0 = x0([2 4 6]) - U_input(:,1) ;

% call dynamics
[tout,yout] = ode45(@(t,x) QC_dyn_SDRE_PI_and_input_cons(t,x,T_input,U_input,params),...
                    T_input([1 end]),[x0;e0]) ;
                
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