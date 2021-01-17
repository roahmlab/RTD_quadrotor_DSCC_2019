%% user parameters
% initial conditions
x0 = zeros(3,1) ;
v0 = zeros(3,1) ;
R0 = eye(3) ;
O0 = zeros(3,1) ;

% control inputs
T = [0 1 2] ;
f = [10 10 10] ;
M = zeros(3,3) ;

% parameters
m = 1 ;
g = 9.81 ;
J = eye(3) ;

%% automated from here
% create params
params.m = m ;
params.g = g ;
params.J = J ;

% create mass matrix
mass_mat = blkdiag(eye(18),zeros(10)) ;

% set up initial constraints
c1_0 = R0'*R0 - eye(3) ;
c2_0 = det(R0) - 1 ;

% create inputs vector
U = [f; M] ;

% create initial condition vector
y0 = [x0 ; v0 ; R0(:) ; O0(:) ; c1_0(:) ; c2_0] ;

% set up ode options
options = odeset('Mass',mass_mat) ;

% solve ODE
[tout,yout] = ode15s(@(t,y) rigid_body_dyn_dae(t,y,T,U,params),...
                     [0,T(end)], y0, options) ;