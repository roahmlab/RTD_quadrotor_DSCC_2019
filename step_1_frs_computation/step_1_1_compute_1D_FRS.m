%% user parameters
t_plan = 0.5 ;
t_peak = 1.5 ; % time to peak speed (default 1)
t_total = 3 ; % time of total trajectory (default 3)
v_max = 5 ;
a_max = 10 ;
dt = 0.02 ;
t_to_stop = t_total - t_peak; % time from peak to stop
body_side_length = (0.54)/4 ;

save_filename = 'quadrotor_FRS_1D' ;

%% Recompute the dynamics with the current t_peak, t_total
generate_dynamics( t_plan, t_peak, t_total ) ;

%% Compute FRS for the to peak part of trajectory
%set options --------------------------------------------------------------
options.tStart = 0; %start time
options.tFinal = t_peak; %final time
% parameters: [p0; v0; a0; v_peak; t];
options.x0 = [0; 0; 0; 0; 0]; %initial state for simulation
options.R0 = zonotope([options.x0, diag([0, v_max, a_max, v_max, 0])]); %initial state for reachability analysis
initialSet = options.R0;
options.timeStep=dt; %time step size for reachable set computation
options.taylorTerms=5; %number of taylor terms for reachable sets
options.zonotopeOrder= 20; %zonotope order... increase this for more complicated systems.
options.maxError = 1000*ones(5, 1);
options.verbose = 1;

options.uTrans = 0;
options.U = zonotope([0, 0]);

options.advancedLinErrorComp = 0;
options.tensorOrder = 1;
options.reductionInterval = inf;
options.reductionTechnique = 'girard';

%specify continuous dynamics-----------------------------------------------
quad_toPeak = nonlinearSys(5, 1, @dyn_quadrotor_toPeak, options);
%compute reachable set-----------------------------------------------------
tic
Rcont_toPeak = reach(quad_toPeak, options);
tComp = toc;
disp(['computation time of reachable set: ', num2str(tComp)]);

%% Second, compute FRS for the peak to stop part of trajectory
%set options --------------------------------------------------------------
options.tStart = 0; %start time
options.tFinal = t_to_stop; %final time
% parameters: [p; v0; a0; v_peak; t];
p_peak = Rcont_toPeak{end}{1}.Z(1, 1);
options.R0 = Rcont_toPeak{end}{1} - [0; 0; 0; 0; t_peak]; %initial state for reachability analysis
options.x0 = center(options.R0);
options.timeStep=dt; %time step size for reachable set computation
options.taylorTerms=5; %number of taylor terms for reachable sets
options.zonotopeOrder= 20; %zonotope order... increase this for more complicated systems.
options.maxError = 1000*ones(5, 1);
options.verbose = 1;

options.uTrans = 0;
options.U = zonotope([0, 0]);

options.advancedLinErrorComp = 0;
options.tensorOrder = 1;
options.reductionInterval = inf;
options.reductionTechnique = 'girard';

%specify continuous dynamics-----------------------------------------------
quad_toStop = nonlinearSys(5, 1, @dyn_quadrotor_toStop, options);
%compute reachable set-----------------------------------------------------
tic
Rcont_toStop = reach(quad_toStop, options);
tComp = toc;
disp(['computation time of reachable set: ', num2str(tComp)]);
for i = 1:length(Rcont_toStop)
    Rcont_toStop{i}{1} = Rcont_toStop{i}{1} + [0; 0; 0; 0; t_peak];
end

%% Assemble the full FRS
Rcont = [Rcont_toPeak; Rcont_toStop];

%% Add on a box to account for quadrotor size
quadrotor_zono = zonotope([0, body_side_length; zeros(4, 2)]);
for i = 1:length(Rcont)
    Rcont{i}{1} = Rcont{i}{1} + quadrotor_zono;
end

%plot results--------------------------------------------------------------
for plotRun=1
    % plot different projections
    if plotRun==1
        projectedDimensions=[1 2];
    elseif plotRun==2
        projectedDimensions=[3 4];   
    end 
    
    figure(plotRun); clf;
    hold on

    %plot reachable sets 
    for i=1:length(Rcont)
        Zproj = project(Rcont{i}{1},projectedDimensions);
        Zproj = reduce(Zproj,'girard',3);
        Zpts = polygon(Zproj)';
%         plotFilled(Zproj,[1 2],[.8 .8 .8],'EdgeColor','none');
        fill3(ones(size(Zpts, 1), 1)*i*options.timeStep, Zpts(:, 1), Zpts(:, 2),[0.9 0.9 0.9]);
    end
    
    %plot initial set
%     plotFilled(options.R0,projectedDimensions,'b','EdgeColor','k');
    Z = project(initialSet, projectedDimensions);
    Zpts = polygon(Z)';
    fill3(ones(size(Zpts, 1), 1)*0, Zpts(:, 1), Zpts(:, 2), [0.5 0.5 0.5]);

    %label plot
    xlabel('time');
    ylabel(['x_{',num2str(projectedDimensions(1)),'}']);
    zlabel(['x_{',num2str(projectedDimensions(2)),'}']);
    view([30, 40]);
end
%--------------------------------------------------------------------------

% save Rcont
save(save_filename, 'Rcont', 'options','t_peak','t_total');