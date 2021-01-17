function [ ] = pipeline_obstacle_test( )
%PIPELINE_OBSTACLE Given the initial velocity and acceleration of the
%quadrotor, intersect the FRS with the obstacles and generate constraints for the
%unsafe control parameters.

clc;

position_dimensions = [1; 6];
param_dimensions = [4; 9];
IC_dim = [2; 3; 7; 8; 12; 13];
% IC = [-5; 5; -5; 5; -5; 5];
% IC = [1;2;-1;1.5;0;0];
IC = rand_range(-2, 2, [], [], 6, 1);
v_0 = [IC(1);IC(3);IC(5)];


% nObs = length(obstacle);
nObstacles = 10;
for idx = 1:nObstacles
    obsZ = [10*rand(1)-5, 0.1 0; 10*rand(1)-5, 0, 0.1];
    obstacle{idx} = zonotope(obsZ);
end

figure(2); clf; hold on;
for k = 1:length(obstacle)
    Zproj = project(obstacle{k},[1,2]);
    Zproj = reduce(Zproj,'girard',3);
    Zpts = polygon(Zproj)';
    %         plotFilled(Zproj,[1 2],[.8 .8 .8],'EdgeColor','none');
    %             p1 = fill3(ones(size(Zpts, 1), 1)*i*options1.timeStep, Zpts(:, 1), Zpts(:, 2),'r');
    p1 = fill(Zpts(:, 1), Zpts(:, 2), 'r');
    p1.FaceAlpha = 0.05;
    p1.EdgeAlpha = 0.3;
end

% also plot quivers for initial conditions
quiver(0, 0, IC(1), IC(2), 0.2, 'LineWidth', 2, 'Color', 'g');
quiver(0, 0, IC(3), IC(4), 0.2, 'LineWidth', 2, 'Color', 'm');


FRS = load('quadrotor_FRS_v7_a10_dt0.01.mat');
FRS = FRS.Rcont;

%% Get tracking error
tbl_file = load('quadrotor_tracking_error_table_dt0.01_vmax_5.25.mat') ;
% tbl_idx = query_tracking_error_table(tracking_error_table, v_0);
% tbl = tracking_error_table(tbl_idx, :);
tbl = get_tracking_error_for_current_velocity(tbl_file.tracking_error_table, v_0);

%% Compute the unsafe set for each obstacle
A_con = [];
b_con = [];
disp('Constraint creation:');
tic
for idx = 1:length(FRS)
   frs = FRS{idx}{1};
   frs = zonotope_slice(frs, IC_dim, IC);
   
   % add in tracking error
   tracking_c = zeros(size(frs.Z, 1), 1);
   tracking_G = zeros(size(frs.Z, 1), 3);
   tracking_G(1, 1) = (tbl(idx, :).ex_hi - tbl(idx, :).ex_lo)/2;
   tracking_G(6, 2) = (tbl(idx, :).ey_hi - tbl(idx, :).ey_lo)/2;
   tracking_G(11, 3) = (tbl(idx, :).ez_hi - tbl(idx, :).ez_lo)/2;
   tracking_zono = zonotope([tracking_c, tracking_G]);
   
   frs_tracking = frs + tracking_zono;
%    if i == 300
%        disp('Tracking zonotope at last time step:');
%        disp(tracking_zono.Z);
%    end
   
   unsafeK = compute_unsafe(frs_tracking, obstacle, position_dimensions, param_dimensions);
   R_obs{idx} = unsafeK;
   
   % create the constraints
   % unsafeK is a 3x2 matrix, with the lower bound and upper bound of
   % unsafeK in each axis.
   for j = 1:length(unsafeK)
       if ~isempty(unsafeK{j})
%            A = [1 0 0; -1 0 0; 0 1 0; 0 -1 0; 0 0 1; 0 0 -1];
%            b = [-unsafeK(1, 1); unsafeK(1, 2); -unsafeK(2, 1); unsafeK(2, 2); -unsafeK(3, 1); unsafeK(3, 2)];
%            A_con = [A_con; A];
%            b_con = [b_con; b];
           A = [1 0; -1 0; 0 1; 0 -1];
           b = [-unsafeK{j}(1, 1); unsafeK{j}(1, 2); -unsafeK{j}(2, 1); unsafeK{j}(2, 2)];
           A_con = [A_con; A];
           b_con = [b_con; b];
       end
   end
end
toc

% plot unsafe sets new representation
figure(3); clf; hold on;
for idx=1:1:length(FRS)
    for k = 1:length(obstacle)
        if isempty(R_obs{idx}{k})
            continue;
        else
            obs_idx(idx, k) = 1;
        end
        %             Zproj = project(R_obs{i}{k},slice_dim);
        %             Zproj = R_obs{i}{k};
        % lolz hack
        fake_c = (R_obs{idx}{k}(:, 2) + R_obs{idx}{k}(:, 1))/2;
        fake_G = [(R_obs{idx}{k}(1, 2) - R_obs{idx}{k}(1, 1))/2, 0; 0, (R_obs{idx}{k}(2, 2) - R_obs{idx}{k}(2, 1))/2];
        Zproj = zonotope([fake_c, fake_G]);
        Zproj = reduce(Zproj,'girard',3);
        Zpts = polygon(Zproj)';
        %         plotFilled(Zproj,[1 2],[.8 .8 .8],'EdgeColor','none');
        p1 = fill(Zpts(:, 1), Zpts(:, 2),'r');
        p1.FaceAlpha = 0.05;
        p1.EdgeAlpha = 0.3;
    end
end
drawnow();

% [testx, testy] = ginput(1);
% myx = [testx; testy];
% 
% % write fmincon code to try to get as close as possible to the point.
% myfun = @(x) sum((myx - x).^2);

figure(2); hold on;
[testx, testy] = ginput(1);
desx = [testx; testy];
plot(testx, testy, 'k.', 'MarkerSize', 30);
% myfun = @(x) sum(([pos_quadrotor_final(IC(1), IC(2), x(1)); pos_quadrotor_final(IC(3), IC(4), x(2))] - desx).^2);
% myfun = @(x) sum(([pos_quadrotor_plan(IC(1), IC(2), x(1)); pos_quadrotor_plan(IC(3), IC(4), x(2))] - desx).^2);
myfun = @(x) sum(([pos_quadrotor_peak(IC(1), IC(2), x(1)); pos_quadrotor_peak(IC(3), IC(4), x(2))] - desx).^2);
mycon = @(x) obscon(x, A_con, b_con);

epsilon = 1e-3;
disp('fmincon:');
tic
xout = fmincon(myfun, [0;0], [], [], [], [], [-5 + epsilon; -5 + epsilon], [5 - epsilon; 5 - epsilon], mycon);
toc

figure(3); hold on;
plot(xout(1), xout(2), 'k.', 'MarkerSize', 30);


% 
% con_eval = A_con*myx + b_con;
% con_eval = reshape(con_eval, 4, []);
% con_eval = min(con_eval, [], 1);
% 
% inout = max(con_eval);
% if inout >= 0
%     disp('Point is inside unsafe set.');
% else
%     disp('Point is outside unsafe set.');
% end

figure(2);
[~, xout1] = ode45(@dyn_quadrotor_toPeak, [0:0.01:1], [0;IC(1);IC(2);xout(1);0]);
[~, yout1] = ode45(@dyn_quadrotor_toPeak, [0:0.01:1], [0;IC(3);IC(4);xout(2);0]);
[~, xout2] = ode45(@dyn_quadrotor_toStop, [0:0.01:2], [0;IC(1);IC(2);xout(1);0]);
[~, yout2] = ode45(@dyn_quadrotor_toStop, [0:0.01:2], [0;IC(3);IC(4);xout(2);0]);
% [~, xout1] = ode45(@dyn_quadrotor_toPeak, [0:0.01:1], [0;-5;5;myx(1);0]);
% [~, yout1] = ode45(@dyn_quadrotor_toPeak, [0:0.01:1], [0;-5;5;myx(2);0]);
% [~, xout2] = ode45(@dyn_quadrotor_toStop, [0:0.01:2], [0;-5;5;myx(1);0]);
% [~, yout2] = ode45(@dyn_quadrotor_toStop, [0:0.01:2], [0;-5;5;myx(2);0]);
xout_plot = [xout1(:, 1); xout1(end, 1) + xout2(:, 1)];
yout_plot = [yout1(:, 1); yout1(end, 1) + yout2(:, 1)];
plot(xout_plot(:, 1), yout_plot(:, 1), 'r', 'LineWidth', 0.2);
plot(xout_plot(end, 1), yout_plot(end, 1), 'r.', 'MarkerSize', 30);

% also test the position functions
% it works!!
p_plan_x = pos_quadrotor_plan(IC(1), IC(2), xout(1));
p_plan_y = pos_quadrotor_plan(IC(3), IC(4), xout(2));

p_peak_x = pos_quadrotor_peak(IC(1), IC(2), xout(1));
p_peak_y = pos_quadrotor_peak(IC(3), IC(4), xout(2));

p_f_x = pos_quadrotor_final(IC(1), IC(2), xout(1));
p_f_y = pos_quadrotor_final(IC(3), IC(4), xout(2));


plot(p_plan_x, p_plan_y, 'b.', 'MarkerSize', 14);
plot(p_peak_x, p_peak_y, 'b.', 'MarkerSize', 20);
plot(p_f_x, p_f_y, 'b.', 'MarkerSize', 30);



end

function [c, ceq] = obscon(x, A_con, b_con)
    epsilon = 1e-1;
    ceq = [];

    c = A_con*x + b_con;
    c = reshape(c, 4, []);
    c = min(c, [], 1) + epsilon;
end

