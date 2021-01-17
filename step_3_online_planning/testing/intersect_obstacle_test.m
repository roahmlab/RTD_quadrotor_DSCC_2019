% trying out obstacle intersection
% for now, assume the obstacle is a zonotope...

clear all; close all;
ploton = 1;
% position_dimensions = [1, 6];
% slice_dim = [4; 9];
position_dimensions = [6, 11];
slice_dim = [9; 14];
IC_dim = [2; 3; 7; 8; 12; 13];
IC = [-5; 5; -5; 5; -5; 5];

% obsZ = [1 0.2 0; 1 0 0.2];
% obstacle{1} = zonotope(obsZ);
% obsZ = [-1.5 0.1 0; -0.5 0 0.1];
% obstacle{2} = zonotope(obsZ);
% obstacle_Z = get(obstacle, 'Z');
% obstacle_c = obstacle_Z(:, 1);
% obstacle_G = obstacle_Z(:, 2:end);

nObstacles = 10;
for i = 1:nObstacles
    obsZ = [10*rand(1)-5, 0.1 0; 10*rand(1)-5, 0, 0.1];
    obstacle{i} = zonotope(obsZ);
end


% assuming that a_0 and v_0 are known explicitly...
load('quadrotor_FRS_v7_a10_dt0.01.mat');
if ploton
    projectedDimensions=position_dimensions;
    
    figure(1); clf;
    hold on
    
    %plot reachable sets
    for i=1:10:length(Rcont)
        Zproj = project(Rcont{i}{1},projectedDimensions);
        Zproj = reduce(Zproj,'girard',3);
        Zpts = polygon(Zproj)';
        %         plotFilled(Zproj,[1 2],[.8 .8 .8],'EdgeColor','none');
        p1 = fill3(ones(size(Zpts, 1), 1)*i*options1.timeStep, Zpts(:, 1), Zpts(:, 2),[0.9 0.9 0.9]);
        p1.FaceAlpha = 0.3;
        p1.EdgeAlpha = 0.5;
    end
    
    ylabel(['x_{',num2str(projectedDimensions(1)),'}']);
    zlabel(['x_{',num2str(projectedDimensions(2)),'}']);
    
    %plot obsicles
%     for i=1:1:length(Rcont)
    figure(3); hold on;
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
%     end
    
    
    figure(2); clf;
    hold on
    
    %plot parameter sets
%     for i=1:10:length(Rcont)
% %     for i=1:10:120
%         Zproj = project(Rcont{i}{1},slice_dim);
%         Zproj = reduce(Zproj,'girard',3);
%         Zpts = polygon(Zproj)';
%         %         plotFilled(Zproj,[1 2],[.8 .8 .8],'EdgeColor','none');
%         p1 = fill3(ones(size(Zpts, 1), 1)*i*options1.timeStep, Zpts(:, 1), Zpts(:, 2),[0.9 0.9 0.9]);
%         p1.FaceAlpha = 0.3;
%         p1.EdgeAlpha = 0.5;
%     end
    
    ylabel(['x_{',num2str(slice_dim(1)),'}']);
    zlabel(['x_{',num2str(slice_dim(2)),'}']);
end

R_obs = {};
tic
for i = 1:length(Rcont)
    frs = Rcont{i}{1};
    frs = zonotope_slice(frs, IC_dim, IC);
    R_obs{i} = compute_unsafe(frs, obstacle, position_dimensions, slice_dim);
end
toc
    
% R_obs = {};
% tic
% for i = 1:length(Rcont)
%     % just want to find the unsafe set of the pxf and pyf dimensions.
%     % first, split the FRS into uncontrolled vs controlled zonotopes.
%     
%     zono = Rcont{i}{1};
%     Z = get(zono, 'Z');
%     c = Z(:, 1);
%     G = Z(:, 2:end);
%     
%     slice_idx = [];
%     for j = 1:length(slice_dim)
%         myidxs = find(G(slice_dim(j), :) ~= 0);
%         if length(myidxs) ~= 1
%             if length(myidxs) == 0
%                 error('No generator for slice index');
%             else
%                 error('More than one generator for slice index');
%             end
%         end
%         slice_idx(j, 1) = myidxs;
%     end
%     
% %     uncontrolled_c = c;
%     uncontrolled_c = zeros(length(position_dimensions), 1);
%     uncontrolled_G = G;
%     uncontrolled_G(:, slice_idx) = [];
%     uncontrolled_G = uncontrolled_G(position_dimensions, :);
%     uncontrolled_Z = [uncontrolled_c, uncontrolled_G];
%     uncontrolled_zono = zonotope(uncontrolled_Z);
%     % buffer by obstacle
%     % this is ok, since our dynamics are independent of position!
%     for k = 1:length(obstacle)
%         avoid_zono = uncontrolled_zono + obstacle{k};
%         % subtract off current center position
%         avoid_zono = avoid_zono - c(position_dimensions, 1);
%         avoid_int = interval(avoid_zono);
%         
%         unsafe_c = zeros(size(Z, 1), 1);
% %         unsafe_c = c;
%         unsafe_G = zeros(size(Z, 1), length(slice_dim));
%         
%         min_lambda = 0;
%         max_lambda = 0;
%         for j = 1:length(slice_dim)
%             myg = G(:, slice_idx(j));
%             mydim = find(myg(position_dimensions, 1) ~= 0);
%             %         min_lambda = min(1, max(-1, (infimum(avoid_int(mydim))/myg(position_dimensions(mydim), 1))));
%             %         max_lambda = min(1, max(-1, (supremum(avoid_int(mydim))/myg(position_dimensions(mydim), 1))));
%             
%             min_lambda = infimum(avoid_int(mydim))/myg(position_dimensions(mydim), 1);
%             max_lambda = supremum(avoid_int(mydim))/myg(position_dimensions(mydim), 1);
%             
%             if min_lambda > 1 || max_lambda < -1
%                 % no intersection possible.
%                 min_lambda = nan;
%                 max_lambda = nan;
%             else
%                 min_lambda = min(1, max(-1, min_lambda));
%                 max_lambda = min(1, max(-1, max_lambda));
%             end
%             unsafe_c = unsafe_c + (min_lambda + max_lambda)/2*myg;
%             unsafe_G(:, j) = (max_lambda - min_lambda)/2*myg;
%         end
%         %add back on original center of zono
%         unsafe_c = unsafe_c + c;
%         unsafe_Z = [unsafe_c, unsafe_G];
%         if any(any(isnan(unsafe_Z)))
%             R_obs{i}{k} = {};
%         else
%             R_obs{i}{k} = zonotope(unsafe_Z);
%         end
%     end
% end
% toc

if ploton
    figure(2); hold on;
    obs_idx = zeros(length(Rcont), length(obstacle));
    
    %plot unsafe sets
    for i=1:1:length(Rcont)
        for k = 1:length(obstacle)
            if isempty(R_obs{i}{k})
                continue;
            else
                obs_idx(i, k) = 1;
            end
            Zproj = project(R_obs{i}{k},slice_dim);
            Zproj = reduce(Zproj,'girard',3);
            Zpts = polygon(Zproj)';
            %         plotFilled(Zproj,[1 2],[.8 .8 .8],'EdgeColor','none');
            p1 = fill3(ones(size(Zpts, 1), 1)*i*options1.timeStep, Zpts(:, 1), Zpts(:, 2),'r');
            p1.FaceAlpha = 0.05;
            p1.EdgeAlpha = 0.3;
        end
    end
    
    % also plot some random unsafe trajectories
    randk = randsample(find(obs_idx~=0), round(length(obs_idx)/2));
    [I, J] = ind2sub(size(obs_idx), randk);
%     randk = obs_idx;
    for i = 1:length(randk)
        randu = randPoint(project(R_obs{I(i)}{J(i)}, slice_dim));
        my_vpeak_1 = randu(1);
        my_vpeak_2 = randu(2);
        [tout1, xout1] = ode45(@dyn_quadrotor_toPeak, [0:0.01:1], [0;-5;5;my_vpeak_1;0]);
        [tout1, yout1] = ode45(@dyn_quadrotor_toPeak, [0:0.01:1], [0;-5;5;my_vpeak_2;0]);
        [tout1, xout2] = ode45(@dyn_quadrotor_toStop, [0:0.01:2], [0;-5;5;my_vpeak_1;0]);
        [tout1, yout2] = ode45(@dyn_quadrotor_toStop, [0:0.01:2], [0;-5;5;my_vpeak_2;0]);
        xout = [xout1(:, 1); xout1(end, 1) + xout2(:, 1)];
        yout = [yout1(:, 1); yout1(end, 1) + yout2(:, 1)];
        figure(3);
        plot(xout(:, 1), yout(:, 1), 'r', 'LineWidth', 0.2);
    end
end
    
disp('woo');



% woo


