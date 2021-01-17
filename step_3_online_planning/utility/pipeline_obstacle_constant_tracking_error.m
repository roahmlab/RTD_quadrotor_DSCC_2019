function [ A_con, b_con ] = pipeline_obstacle_constant_tracking_error(v_0, a_0, FRS, obstacle, tracking_error_zono)
%PIPELINE_OBSTACLE Given the initial velocity and acceleration of the
%quadrotor, intersect the FRS with the obstacles and generate constraints for the
%unsafe control parameters.
    position_dimensions = [1; 6; 11];
    param_dimensions = [4; 9; 14];
    IC_dim = [2; 7; 12; 3; 8; 13];
    IC = [v_0; a_0];


    %% Compute the unsafe set for each obstacle
    %A_con = [];
    b_con = [];
    for i = 1:length(FRS)
       frs = FRS{i}{1};

       % slice frs by initial condition
       frs = zonotope_slice(frs, IC_dim, IC);

       % add in tracking error
       frs_tracking = frs + tracking_error_zono;

       % compute unsafe control parameters
       unsafeK = compute_unsafe(frs_tracking, obstacle, position_dimensions, param_dimensions);
%        unsafeK = comp?ute_unsafe_parallel(frs_tracking, obstacle, position_dimensions, param_dimensions);
%        R_obs{i} = unsafeK ;

       % create the constraints
       % unsafeK is a 3x2 matrix, with the lower bound and upper bound of
       % unsafeK in each axis.
       for j = 1:length(unsafeK)
           if ~isempty(unsafeK{j})
               %A = [1 0 0; -1 0 0; 0 1 0; 0 -1 0; 0 0 1; 0 0 -1];
               b = [-unsafeK{j}(1, 1); unsafeK{j}(1, 2);
                    -unsafeK{j}(2, 1); unsafeK{j}(2, 2);
                    -unsafeK{j}(3, 1); unsafeK{j}(3, 2)];
               %A_con = [A_con; A];
               b_con = [b_con; b];
           end
       end
    end
    
    A = [1 0 0; -1 0 0; 0 1 0; 0 -1 0; 0 0 1; 0 0 -1] ;
    A_con = repmat(A,size(b_con,1)/6,1) ;

%     % FOR DEBUGGING: plot unsafe sets new representation
%     figure(3); clf; hold on;
%     for idx=1:1:length(FRS)
%         for k = 1:length(obstacle)
%             if isempty(R_obs{idx}{k})
%                 continue;
%             else
%                 obs_idx(idx, k) = 1;
%             end
%             %             Zproj = project(R_obs{i}{k},slice_dim);
%             %             Zproj = R_obs{i}{k};
%             % lolz hack
%             fake_c = (R_obs{idx}{k}(1:2, 2) + R_obs{idx}{k}(1:2, 1))/2;
%             fake_G = [(R_obs{idx}{k}(1, 2) - R_obs{idx}{k}(1, 1))/2, 0 ;
%                       0, (R_obs{idx}{k}(2, 2) - R_obs{idx}{k}(2, 1))/2];
%             Zproj = zonotope([fake_c, fake_G]);
%             Zproj = reduce(Zproj,'girard',3);
%             Zpts = polygon(Zproj)';
%             %         plotFilled(Zproj,[1 2],[.8 .8 .8],'EdgeColor','none');
%             p1 = fill(Zpts(:, 1), Zpts(:, 2),'r');
%             p1.FaceAlpha = 0.05;
%             p1.EdgeAlpha = 0.3;
%         end
%     end
%     drawnow();
end

