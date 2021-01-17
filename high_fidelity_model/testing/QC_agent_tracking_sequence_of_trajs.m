%% user parameters
% timing
t_max = 5 ; % total s for simulation
t_plan = 0.5 ;
t_peak = 1 ;
t_total = 3 ; % total time per trajectory
dt = 0.025 ;

% accel bound
a_max = 1 ; % m/s

% plotting
plot_flag = true ;

%% automated from here
% make agent
A = QC_SDRE_agent ;
v_cur = A.state([2 4 6]) ;
a_cur = zeros(3,1) ;

% setup for plotting
figure(1) ; clf ; hold on ; axis equal ;

%% run agent in loop
iter_total = 1 ;
tic_start = tic ;
t_cur = toc(tic_start) ;

while t_cur < t_max
%% setup 
    % reset agent to where it was at t_plan

    % generate random new peak speed
    a_rand = randRange(0,a_max,[],[],3,1) ;
    if norm(a_rand) > a_max
        a_rand = a_max*a_rand./norm(a_rand) ;
    end
    v_peak = v_cur + a_rand.*t_plan ;
    
    % generate spline
    [T_des,Z_des,Z_plan] = generate_spline_peak_speed(v_cur,a_cur,v_peak,t_plan,t_peak,t_total,dt) ;
    
    % add braking to end of spline
    T_des = [T_des, (T_des(end)+dt):dt:(T_des(end)+4)] ;
    Z_des = [Z_des, repmat(Z_des(:,end),1,length(T_des)-length(Z_des))] ;    

    % create nominal inputs, which are advanced by a bit
    U_des = [Z_des(4:6,4:end), zeros(3,3)] ;
    
    % move agent along spline for t_plan
    A.move(T_des(end),T_des,U_des,Z_des)
    
%% compute position error
    %
    
   
%% plotting
    if plot_flag
        clf ; hold on ; axis equal
        plot3(Z_des(1,:),Z_des(2,:),Z_des(3,:),'b--')
        plot(A)
        view(3) ;
        pause(0.001)
    end
    
%% prep for next iter
    a_cur = Z_plan(7:9) ;
    iter_total = iter_total + 1 ;
    
    t_cur = toc(tic_start) ;
end