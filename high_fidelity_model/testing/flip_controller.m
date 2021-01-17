%% user parameters
save_gif = false ;

%% make agent
figure(1) ; clf ; grid on ; hold on ; axis equal

A = quadrotor_agent ;
A.camera_view_style = 'none' ;
view(3) ;

%% phase 1: accelerate up
dt = 0.001 ;
t_total = 3 ;

v_cur = A.state(A.velocity_indices,end) ;
a_cur = A.a_est ;
v_peak = [0;1;1] ;
t_move = 2 ;
t_peak = 3 ;
[T_in,Z_in] = generate_spline_peak_speed(v_cur,a_cur,v_peak,0,t_peak,t_total,dt) ;
U_in = zeros(4,length(T_in)) ;

Z_in(1:3,:) = Z_in(1:3,:)  + A.state(A.position_indices,end) ;

A.move(t_move,T_in,U_in,Z_in) ;

plot(A)

%% phase 2: flip
% switch to flip controller
A.LLC = attitude_LLC() ;

t_flip = 0.15 ;

T_flip_in = 0:dt:t_flip ;

flip_rate = 4*pi ;
flip_axis = [1;0;0] ;

A.move(t_flip,T_flip_in,repmat([flip_rate;flip_axis],1,length(T_flip_in)))

plot(A)

%% phase 3: stop
A.LLC = Mellinger_LLC() ;

v_cur = A.state(A.velocity_indices,end) ;
a_cur = A.a_est ;
v_peak = [0;0;0] ;
t_total = 6 ;
t_move = 4 ;
t_peak = 3 ;
[T_in,Z_in] = generate_spline_peak_speed(v_cur,a_cur,v_peak,0,t_peak,t_total,dt) ;
U_in = zeros(4,length(T_in)) ;

Z_in(1:3,:) = Z_in(1:3,:)  + A.state(A.position_indices,end) ;

t_go = 1 ;

A.move(t_go,T_in,U_in,Z_in) ;

plot(A)

%% animate
plot(A)
axis equal
animate(A,save_gif)