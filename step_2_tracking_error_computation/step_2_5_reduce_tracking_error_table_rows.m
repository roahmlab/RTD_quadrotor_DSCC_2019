%% user parameters
% file i/o
filename = 'quadrotor_tracking_error_table_dt0.01_vmax_5.25.mat' ;

% timing
dt_new = 0.02 ;

%% automated from here
% get table info
load(filename) ;
v = bin_center_velocities ;

% get table itself
tbl = tracking_error_table ;

% prep new table
tbl_new = tbl(1,:) ;

% get timing
t_max = max(tbl.t_hi) ;
t_lo_new = 0:dt_new:(t_max - dt_new) ;
t_hi_new = dt_new:dt_new:t_max ;

% for each bin center velocity, get all rows...
N_v = size(v,1) ;

% tic
for idx = 1:N_v
    % get rows for current velocity
    v_idx = v(idx,:) ;
    tbl_idx = get_tracking_error_for_current_velocity(tbl,v_idx(:),true) ;
    
    % for each new time bin, combine the information for the rows of the
    % previous time bin
    for idx2 = 1:length(t_lo_new)
        t_lo_idx = t_lo_new(idx2) ;
        t_hi_idx = t_hi_new(idx2) ;
        
        % find rows that are between the new time bounds
        tbl_rows_log = (tbl_idx.t_lo >= t_lo_idx) & (tbl_idx.t_hi <= t_hi_idx) ;
        tbl_rows_idx = tbl_idx(tbl_rows_log,:) ;
        
        % get new error vals
        ex_lo_new = min(tbl_rows_idx.ex_lo) ;
        ex_hi_new = max(tbl_rows_idx.ex_hi) ;
        ey_lo_new = min(tbl_rows_idx.ey_lo) ;
        ey_hi_new = max(tbl_rows_idx.ey_hi) ;
        ez_lo_new = min(tbl_rows_idx.ez_lo) ;
        ez_hi_new = max(tbl_rows_idx.ez_hi) ;
        
        % create new row
        tbl_row_new = tbl_rows_idx(1,:) ;
        tbl_row_new.t_lo = t_lo_idx ;
        tbl_row_new.t_hi = t_hi_idx ;
        tbl_row_new.ex_lo = ex_lo_new ;
        tbl_row_new.ex_hi = ex_hi_new ;
        tbl_row_new.ey_lo = ey_lo_new ;
        tbl_row_new.ey_hi = ey_hi_new ;
        tbl_row_new.ez_lo = ez_lo_new ;
        tbl_row_new.ez_hi = ez_hi_new ;
        
        % append row to new table
        tbl_new = [tbl_new ; tbl_row_new] ;
    end
    
    disp([num2str(100*idx/N_v),' pct complete'])
    % toc
end

%% save output
tracking_error_table = tbl_new ;
N_bins = size(tbl_new,1) ;
dt = dt_new ;
save('temporary_updated_error_table.mat','tracking_error_table', 'a_max', 'bin_center_velocities', 'dt', 'dv', 'N_bins', 'N_bins_per_dim', 't_max', 'v_max');
