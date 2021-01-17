%% user parameters
% initial conditions
p0 = [0;0;0] ; % position in x,y,z (leave this at the origin)
v0 = [2;1;0] ; % speed
a0 = [0;0;0] ; % acceleration

% desired future position (note that this is made to be no more than Dp_max
% from the robot)
pf = [2;0;0] ;

% timing
t_move  = 8 ;
t_plan  = 0.25 ;
t_peak  = 0.5 ;
t_total = 3 ;
dt      = 0.025 ;

%% automated from here
% generate spline
[T_ref,~,Z_ref] = generate_spline_desired_position(pf,v0,a0,t_total,dt) ;
U_ref = zeros(4,length(T_ref)) ;

% add extra time to end of spline
T_ref = [T_ref, T_ref(end) + 0.05, T_ref(end) + 15] ;
U_ref = [U_ref, U_ref(:,end), U_ref(:,end)] ;
Z_ref = [Z_ref, [Z_ref(1:3,end);zeros(12,1)], [Z_ref(1:3,end);zeros(12,1)]] ;

% get agent
A = quadrotor_agent() ;
A.integrator_time_discretization = 0.01 ;
A.LLC = Mellinger_LLC() ;

% try different initial conditon
A.state(A.velocity_indices) = v0 ;

% move agent along spline
A.move(t_move,T_ref,U_ref,Z_ref)

%% plotting
figure(1) ; clf ; hold on ; grid on ; axis equal ;
plot(A)
plot3(Z_ref(1,:),Z_ref(2,:),Z_ref(3,:),'b:')
view(3) ;

%%
A.animate