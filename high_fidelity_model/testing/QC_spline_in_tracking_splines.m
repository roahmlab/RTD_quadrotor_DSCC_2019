%% user parameters
% QC (quadcopter) initial conditions
p0 = [0;0;0] ; % position in x,y,z (leave this at the origin)
v0 = [1;0;0] ; % speed
a0 = [0;0;0] ; % acceleration

% desired future position
pf = [1;1;0] ;

% timing
time_horizon = 3 ; % s
sample_time = 0.05 ; % s

%% automated from here
% create spline
[T,U,Z] = generate_QC_spline_traj(pf,v0,a0,time_horizon,sample_time) ;

% create initial cond of splinecopter
state_0 = [p0;v0;0;0] ;
[T_out,Z_out] = ode45(@(time,state) QC_dyn_spline_in(time,state,T,Z),...
                    [0,T(end)],state_0) ;
                
Z_out = Z_out' ;

%% plotting
figure(1) ; clf ; hold on ; axis equal ;

% plot spline
plot3(Z(1,:),Z(2,:),Z(3,:),'b--')

% plot traj
plot3(Z_out(1,:),Z_out(2,:),Z_out(3,:),'b')

view(3)