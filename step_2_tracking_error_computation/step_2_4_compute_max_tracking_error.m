% compute a box with side length as the maximum tracking error
if ~exist('tbl','var')
    tbl = load('quadrotor_tracking_error_table_dt0.02_v1.5.mat') ;
    tbl = tbl.tracking_error_table ;
end

ex_max = max(abs(tbl.ex_hi - tbl.ex_lo)) 
ey_max = max(abs(tbl.ey_hi - tbl.ey_lo)) 
ez_max = max(abs(tbl.ez_hi - tbl.ez_lo)) 