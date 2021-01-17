%% user parameters
% speed bounds
v_min = 0.5 ;
v_max = 1 ; % m/s
v_rand = true ;

% accel bounds
a_max = 5 ; % m/s^2
a_opp_to_v = false ;

% timing
t_move  = 5 ;
t_plan  = 0.5 ;
t_peak  = 1.0 ;
t_total = 3 ;
t_extra = 15 ;
dt      = 0.025 ;
dt_int  = 0.01 ;

% plotting
animate_flag = false ;

%% automated from here
% make initial conditions
p0 = [0;0;0] ; % position in x,y,z (leave this at the origin)
a0 = [0;0;0] ; % acceleration

% make initial speed
if v_rand
    v_dir = make_random_unit_vector(3) ;
else
    v_dir = [1;0;0] ;
end
v0 = rand_range(v_min,v_max)*v_dir ;
disp(['Initial speed: ',num2str(norm(v0))])

% make peak speed
if a_opp_to_v
    a_dir = -v0./norm(v0) ;
else
    a_dir = make_random_unit_vector(3) ;
end
v_peak = v0 + a_max*t_peak*a_dir ;
if norm(v_peak) > v_max
    disp('Adjusting peak speed')
    v_peak = (v_peak/norm(v_peak))*v_max ;
end

% generate spline
[T_ref,Z_ref] = generate_spline_peak_speed(v0,a0,v_peak,t_plan,t_peak,t_total,dt,t_extra) ;
U_ref = zeros(4,length(T_ref)) ;

% get agent
A = quadrotor_agent() ;
A.integrator_time_discretization = dt_int ;
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
if animate_flag
    A.animate
end