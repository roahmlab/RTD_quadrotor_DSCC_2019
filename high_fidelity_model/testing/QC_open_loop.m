%% user parameters
% inputs
T_input = [0 2] ;
U_input = [6 6;
           0.01 0.01 ;
           0 0 ;
           0 0 ;
           0 0] ;

% initial condition
x0 = zeros(12,1) ;

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

% call dynamics
[tout,yout] = ode45(@(t,x) QC_dyn_open_loop(t,x,T_input,U_input,params),...
                    T_input,x0) ;
                
%% plotting
figure(1) ; cla ; hold on ; axis equal ;
plot3(yout(:,1),yout(:,3),yout(:,5))
xlabel('x') ; ylabel('y') ; zlabel('z') ;