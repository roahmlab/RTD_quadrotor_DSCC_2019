function [R_unsafe] = compute_unsafe_parallel(frs, obs, workspace_dim, param_dim)
% frs is a zonotope representing the FRS at the current time
% obs is a cell containing obstacles represented as zonotopes
% workspace_dim is a vector containing the indices of the workspace
% dimensions in the FRS
% param_dim is a vector containing the indices of the workspace dimensions
% in the FRS.

frs_Z = get(frs, 'Z');
frs_c = frs_Z(:, 1);
frs_G = frs_Z(:, 2:end);

% get the indices of generators corresponding to parameters
frs_param_idx = [];
for i = 1:length(param_dim)
    myidx = find(frs_G(param_dim(i), :) ~= 0);
    switch length(myidx)
        case 0
            error('No generator for param_dim %i', param_dim(i));
        case 1
            % all good
        otherwise
            error('More than one generator for param_dim %i', param_dim(i));
    end
    frs_param_idx(i, 1) = myidx;
end

% separate out the "uncontrolled" part of FRS
frs_uncontrolled_c = zeros(length(workspace_dim), 1);
frs_uncontrolled_G = frs_G(workspace_dim, :);
frs_uncontrolled_G(:, frs_param_idx) = [];
% frs_uncontrolled = zonotope([frs_uncontrolled_c, frs_uncontrolled_G]);

% get unsafe params for each obstacle
R_unsafe = {};
parfor i = 1:length(obs)
    % buffer obstacle by uncontrolled FRS.
    % this is ok, since our dynamics are independent of position!
    buffered_obs_c = frs_uncontrolled_c + obs{i}.Z(:, 1);
    buffered_obs_G = [frs_uncontrolled_G, obs{i}.Z(:, 2:end)];
    % subtract off current center position
    buffered_obs_c = buffered_obs_c - frs_c(workspace_dim, 1);
    % turn buffered obstacle zonotope into an interval.
%     buffered_obs = interval(buffered_obs);
    %determine left and right limit
    delta = sum(abs(buffered_obs_G),2);
    leftLimit = buffered_obs_c - delta;
    rightLimit = buffered_obs_c + delta;
    
%     unsafe_c = zeros(size(frs_Z, 1), 1);
%     unsafe_G = zeros(size(frs_Z, 1), length(param_dim));

    unsafe_c = zeros(length(param_dim), 1);
    unsafe_G = zeros(length(param_dim), length(param_dim));
%     
    for j = 1:length(param_dim)
        myg = frs_G(:, frs_param_idx(j));
        mydim = myg(workspace_dim, 1) ~= 0; % get workspace dim this param is associate with.
        
        lambda_1 = leftLimit(mydim)/myg(workspace_dim(mydim), 1);
        lambda_2 = rightLimit(mydim)/myg(workspace_dim(mydim), 1);
        
        if lambda_1 > 1 || lambda_2 < -1
            % no intersection possible.
            lambda_1 = nan;
            lambda_2 = nan;
        else
            lambda_1 = min(1, max(-1, lambda_1));
            lambda_2 = min(1, max(-1, lambda_2));
        end
        myg_param = frs_G(param_dim, frs_param_idx(j));
        unsafe_c = unsafe_c + (lambda_1 + lambda_2)/2*myg_param;
        unsafe_G(:, j) = (lambda_2 - lambda_1)/2*myg_param;
    end
    %add back on original center of zono
    unsafe_c = unsafe_c + frs_c(param_dim);
    unsafe_Z = [unsafe_c, unsafe_G];
    if any(any(isnan(unsafe_Z)))
        R_unsafe{i} = {};
    else
%                 R_unsafe{i} = zonotope(unsafe_Z);
        for k = 1:length(param_dim)
            plus_g = unsafe_c(k) + unsafe_G(k, k);
            minus_g = unsafe_c(k) - unsafe_G(k, k);
            R_unsafe{i}(k, 1) = min(plus_g, minus_g);
            R_unsafe{i}(k, 2) = max(plus_g, minus_g);
        end
    end
end
end

