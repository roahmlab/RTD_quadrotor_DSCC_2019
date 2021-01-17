%% user parameters
% thrust and moment inputs
T_in = 0:0.1:14 ;
F_in = 9.9.*ones(size(T_in)) ;
M_in = [zeros(size(T_in)) ;
        0.00006.*sin(T_in) ;
        0.002.*ones(1,floor(length(T_in)/2)), -0.0015.*ones(1,ceil(length(T_in)/2))] ;

% flags
plot_flag = true ;

%% automated from here
% create inputs
U_in = [F_in ; M_in] ;

% create agent
A = quadrotor_agent() ;

% move agent
A.move(T_in(end),T_in,U_in) 

%% plotting
figure(1) ; clf ; hold on
plot(A) ;
view(3)
axis equal