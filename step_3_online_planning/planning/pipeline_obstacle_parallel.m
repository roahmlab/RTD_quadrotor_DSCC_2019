function [ A_con, b_con ] = pipeline_obstacle_parallel(v_0, a_0, FRS, obstacle, ~)
%PIPELINE_OBSTACLE Given the initial velocity and acceleration of the
%quadrotor, intersect the FRS with the obstacles and generate constraints for the
%unsafe control parameters.
    position_dimensions = [1; 6; 11];
    param_dimensions = [4; 9; 14];
    IC_dim = [2; 7; 12; 3; 8; 13];
    IC = [v_0; a_0];

    %% Get tracking error
%     if nargin > 4
%         tbl = get_tracking_error_for_current_velocity(tracking_error_table, v_0);
%     end

    %% Compute the unsafe set for each obstacle
    %A_con = [];
    b_con = nan(6*length(obstacle),length(FRS)) ;
    parfor b_col_idx = 1:length(FRS)
       frs = FRS{b_col_idx}{1};

       % slice frs by initial condition
       frs = zonotope_slice(frs, IC_dim, IC);

%        % add in tracking error
%        if nargin > 4
%            tracking_c = zeros(size(frs.Z, 1), 1);
%            tracking_c(1, 1) = (tbl(b_col_idx, :).ex_hi + tbl(b_col_idx, :).ex_lo)/2;
%            tracking_c(2, 1) = (tbl(b_col_idx, :).ey_hi + tbl(b_col_idx, :).ey_lo)/2;
%            tracking_c(3, 1) = (tbl(b_col_idx, :).ez_hi + tbl(b_col_idx, :).ez_lo)/2;
%            tracking_G = zeros(size(frs.Z, 1), 3);
%            tracking_G(1, 1) = (tbl(b_col_idx, :).ex_hi - tbl(b_col_idx, :).ex_lo)/2;
%            tracking_G(6, 2) = (tbl(b_col_idx, :).ey_hi - tbl(b_col_idx, :).ey_lo)/2;
%            tracking_G(11, 3) = (tbl(b_col_idx, :).ez_hi - tbl(b_col_idx, :).ez_lo)/2;
%            tracking_zono = zonotope([tracking_c, tracking_G]);
% 
%            frs_tracking = frs + tracking_zono;
%        else
%            frs_tracking = frs ;
%        end

       % compute unsafe control parameters
%        unsafeK = compute_unsafe(frs_tracking, obstacle, position_dimensions, param_dimensions);
        unsafeK = compute_unsafe(frs, obstacle, position_dimensions, param_dimensions);
       % R_obs{idx1} = unsafeK ;

       % create the constraints
       % unsafeK is a 3x2 matrix, with the lower bound and upper bound of
       % unsafeK in each axis.
       b_temp = nan(6*length(unsafeK),1) ;
       for unsafe_idx = 1:length(unsafeK)
           if ~isempty(unsafeK{unsafe_idx})
               %A = [1 0 0; -1 0 0; 0 1 0; 0 -1 0; 0 0 1; 0 0 -1];
               b_new = [-unsafeK{unsafe_idx}(1, 1); unsafeK{unsafe_idx}(1, 2);
                    -unsafeK{unsafe_idx}(2, 1); unsafeK{unsafe_idx}(2, 2);
                    -unsafeK{unsafe_idx}(3, 1); unsafeK{unsafe_idx}(3, 2)];
               %A_con = [A_con; A];
               b_temp = [b_temp ;b_new] ;
           end
       end
       b_temp = [b_temp ; nan(6*(length(obstacle) - length(unsafeK)),1)] ;
       b_con(:,b_col_idx) = b_temp ;
    end
    
    b_con = b_con(:) ;
    b_con(isnan(b_con)) = [] ;
    
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

