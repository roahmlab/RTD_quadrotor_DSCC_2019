%% user parameters
% control inputs
T_in = [0 1 6] ; % s
F_in = [5.4 5.4 5.4] ; % N
M_in = zeros(3) ;

% timing
dt = 0.05 ;
t_move = T_in(end) ;

%% automated from here
% set up quadrotor agent
A = quadrotor_agent() ;
A.integrator_time_discretization = dt ;
A.animation_time_discretization = 2*dt ;

% create open loop inputs
U_in = [F_in ; M_in] ;
Z_in = [] ;

% move quadrotor
A.move(t_move,T_in,U_in,Z_in)

%% plotting
plot(A)
axis equal

% %% animation
% A.animate