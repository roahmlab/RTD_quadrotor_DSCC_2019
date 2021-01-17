function [FRS_epsilon] = add_tracking_error_to_FRS(FRS, tracking_error_in, v_0)
% [FRS_epsilon] = add_tracking_error_to_FRS(FRS, tracking_error_in, v_0)
%
% Adds the tracking error to the whole FRS given v_0, if a table is passed
% in, or adds the tracking error zonotope to each FRS zonotope if a
% zonotope is passed in with no v_0.

FRS_epsilon = cell(length(FRS), 1);

if nargin == 3
    tbl = get_tracking_error_for_current_velocity(tracking_error_in, v_0);
    for i = 1:length(FRS)
        frs = FRS{i}{1};

        tracking_c = zeros(size(frs.Z, 1), 1);
        tracking_c(1, 1) = (tbl(i, :).ex_hi + tbl(i, :).ex_lo)/2;
        tracking_c(2, 1) = (tbl(i, :).ey_hi + tbl(i, :).ey_lo)/2;
        tracking_c(3, 1) = (tbl(i, :).ez_hi + tbl(i, :).ez_lo)/2;
        tracking_G = zeros(size(frs.Z, 1), 3);
        tracking_G(1, 1) = (tbl(i, :).ex_hi - tbl(i, :).ex_lo)/2;
        tracking_G(6, 2) = (tbl(i, :).ey_hi - tbl(i, :).ey_lo)/2;
        tracking_G(11, 3) = (tbl(i, :).ez_hi - tbl(i, :).ez_lo)/2;
        tracking_zono = zonotope([tracking_c, tracking_G]);

        frs_tracking = frs + tracking_zono;

        FRS_epsilon{i}{1} = frs_tracking;
    end
elseif nargin == 2
    for i = 1:length(FRS)
        frs = FRS{i}{1};

        frs_tracking = frs + tracking_error_in;

        FRS_epsilon{i}{1} = frs_tracking;
    end
else
    error('Invalid number of input arguments.')
end
