%% user parameters
% QC (quadcopter) initial conditions
p0 = [0;0;0] ; % position in x,y,z (leave this at the origin)
v0 = [0;2;0] ; % speed (note this doesn't make sense to be nonzero since the attitude is originally 0)
a0 = [0;0;0] ; % acceleration

% desired future position (note that this is made to be no more than Dp_max
% from the robot)
pf = [1;0;3] ;
Dp_max = 3 ; % m

% timing
time_horizon = 3 ; % s
sample_time = 0.05 ; % s

% control gains (proportional gains for velocity tracking)
kx = 20 ;
ky = 20 ;
kz = 20 ;
ki = 10 ; % gain on position error (i.e. integral gain on velocity)

%% automated from here
% create agent
A = QC_SDRE_agent ;
A.params.k1 = kx ;
A.params.k2 = ky ;
A.params.k3 = kz ;
A.params.ki = ki ;

% shrink pf
Dp_cur = norm(pf - p0) ;
if Dp_cur > Dp_max 
    disp('shrinking pf')
    pf = pf*Dp_max./Dp_cur ;
    disp(pf)
end

% report angle between pf and v0
s0 = norm(v0) ;
dot_prod = pf'*v0/(norm(pf)*s0) ;
ang = acos(dot_prod) ;
disp(['Angle between pf and v0 is ',num2str(ang)])

% report angle limit for v0
ang_lim = pi - (pi/4)*s0 ;
disp(['Angle limit for v0 is: ',num2str(ang_lim)])

% only run traj if angle limit is satisfied
if ang <= ang_lim || isnan(ang)
    % create spline
    [T_des,~,Z_des] = generate_QC_spline_traj(pf,v0,a0,time_horizon,sample_time) ;
    T_des = [T_des, (T_des(end)+sample_time):sample_time:(T_des(end)+4)] ;
    Z_des = [Z_des, repmat(Z_des(:,end),1,length(T_des)-length(Z_des))] ;

    % create nominal inputs, which are advanced by 3*sample_time
    U_des = [Z_des(4:6,4:end), zeros(3,3)] ;

    % reset agent
    x0 = [p0;v0;zeros(6,1)] ;
    x0(1:6) = x0([1 4 2 5 3 6]) ;
    A.reset(x0) ;

    % move agent
    tic
    A.move(T_des(end),T_des,U_des,Z_des)
    toc

    % get agent's state output
    T_out = A.time ;
    Z_out = A.state ;

    % reinterpolate spline to match T_out
    Z_des = matchTrajectories(T_out,T_des,Z_des) ;

    x_out = Z_out(1,:) ;
    y_out = Z_out(3,:) ;
    z_out = Z_out(5,:) ;

    % compute position error
    x_des = Z_des(1,:) ;
    y_des = Z_des(2,:) ;
    z_des = Z_des(3,:) ;

    x_err = x_out - x_des ;
    y_err = y_out - y_des ;
    z_err = z_out - z_des ;

    % get desired acceleration
    ax_des = Z_des(7,:) ;
    ay_des = Z_des(8,:) ;
    az_des = Z_des(9,:) ;

    %% plotting
    % trajectory
    figure(1) ; clf ;
    hold on ; axis equal ;
    plot3(Z_des(1,:),Z_des(2,:),Z_des(3,:),'b--')
    plot(A)
    view(3)
    xlabel('x') ; ylabel('y') ; zlabel('z') ;
    set(gca,'FontSize',15)

    % positional error
    figure(2) ; clf ;
    subplot(1,3,1) ;
    plot(T_out,x_err,'r',T_out,y_err,'g',T_out,z_err,'b') ;
    axis([0,T_out(end),-0.3,0.3])
    legend('x','y','z')
    xlabel('t') ; ylabel('pos. error') ; title('Position')
    set(gca,'FontSize',15)

    % speed
    subplot(1,3,2) ; hold on ;
    plot(T_out,Z_out(2,:),'r',T_out,Z_out(4,:),'g',T_out,Z_out(6,:),'b') ;
    plot(T_out,Z_des(4,:),'r--',T_out,Z_des(5,:),'g--',T_out,Z_des(6,:),'b--')
    axis([0,T_des(end),-3,3])
    legend('v_x','v_y','v_z')
    xlabel('t') ; ylabel('speed (dashed is desired)') ; title('Speed')
    set(gca,'FontSize',15)

    % desired acceleration
    subplot(1,3,3) ;
    plot(T_out,ax_des,'r',T_out,ay_des,'g',T_out,az_des,'b')
    axis([0,T_des(end),-4,4])
    legend('a_x','a_y','a_z') ;
    xlabel('t') ; ylabel('desired accel') ; title('Acceleration')
    set(gca,'FontSize',15)
else
    disp('Angle exceeds angle limit! Not running sim')
end