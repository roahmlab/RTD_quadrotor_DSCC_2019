%% user parameters
% initial conditions
p0 = [1;0;5] ; % position in x,y,z (leave this at the origin)
v0 = [0;0;0] ; % speed
a0 = [0;0;0] ; % acceleration

% desired peak speed
v_peak = [1;0;0] ;

% timing
t_move  = 3 ;
t_plan  = 0.5 ;
t_peak  = 1.0 ;
t_total = 3 ;
dt      = 0.025 ;
dt_int  = 0.01 ;

% plottig
animate_flag = true ;

%% automated from here
% generate spline
[T_ref,Z_ref] = generate_spline_peak_speed(v0,a0,v_peak,t_plan,t_peak,t_total,dt) ;
U_ref = zeros(4,length(T_ref)) ;

% add extra time to end of spline
T_ref = [T_ref, T_ref(end) + 0.05, T_ref(end) + 15] ;
U_ref = [U_ref, U_ref(:,end), U_ref(:,end)] ;
Z_ref = [Z_ref, [Z_ref(1:3,end);zeros(12,1)], [Z_ref(1:3,end);zeros(12,1)]] ;

% shift Z_ref
Z_ref(1:3,:) = Z_ref(1:3,:) + p0 ;

% get agent
A = quadrotor_agent() ;
A.integrator_time_discretization = dt_int ;
A.LLC = Mellinger_LLC() ;

% try different initial conditon
A.state(A.position_indices) = p0 ;
A.state(A.velocity_indices) = v0 ;

% move agent along spline
A.move(t_move,T_ref,U_ref,Z_ref)

%% plotting
figure(1) ; clf ; hold on ; grid on ; axis equal ;
plot(A)
plot3(Z_ref(1,:),Z_ref(2,:),Z_ref(3,:),'b:')
view(3) ;

%%
if animate_flag
    A.animate
end