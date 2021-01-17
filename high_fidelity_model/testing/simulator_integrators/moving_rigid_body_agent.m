%% user parameters
% rigid body parameters
m = 1 ; % kg
J = eye(3) ; % inertia matrix

% force and moment inputs
T_in = [0 10] ;
F_in = 0.5.*rand(3,2) - 0.25 ;
M_in = 0.5.*rand(3,2) - 0.25 ;

% flags
plot_flag = true ;
compensate_for_gravity = true ;

%% automated from here
% create inputs
if compensate_for_gravity
    F_in = F_in + repmat([0;0;9.81],1,size(F_in,2)) ;
end
U_in = [F_in ; M_in] ;

% create agent
A = rigid_body_agent_SE3(m,J) ;

% move agent
A.move(T_in(end),T_in,U_in) 

%% plotting
figure(1) ; clf ; hold on
plot(A) ;
view(3)
axis equal