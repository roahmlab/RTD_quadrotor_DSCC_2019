function [FRS_sigma] = slice_FRS(FRS, IC_dim, IC)
% slice FRS using zonotope_slice

FRS_sigma = cell(length(FRS), 1);
for i = 1:length(FRS)
    frs = FRS{i}{1};
    
    % slice frs by initial condition
    frs = zonotope_slice(frs, IC_dim, IC);
    FRS_sigma{i}{1} = frs;
end

end

