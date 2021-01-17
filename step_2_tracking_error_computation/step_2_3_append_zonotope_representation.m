%% user parameters
% file i/o
filename = 'quadrotor_tracking_error_table_dt0.02_vmax_5.25.mat' ;

% user feedback
display_count = 1000 ; % display timing after this many rows

%% automated from here
% load the tracking error table
load(filename) ;

% number of rows in FRS generators and centers
N_FRS_rows = 15 ;

% get current tracking error table
tbl = tracking_error_table;

% add zonotope rows
tbl.zonotope = cell(size(tbl, 1), 1);

% extra vars
N_rows = size(tbl, 1) ;
zono_cell = cell(N_rows, 1) ;
last_filled_idx = 0 ;

%% generate zonotopes for all the rows of the table
% iterate through each row of the table and make a zonotope for that row,
% represented as a center and generator matrix

start_tic = tic ;
for idx = 1:N_rows
    % make the center
    tracking_c = zeros(N_FRS_rows, 1);
    tracking_c(1, 1) = (tbl(idx, :).ex_hi + tbl(idx, :).ex_lo)/2;
    tracking_c(6, 1) = (tbl(idx, :).ey_hi + tbl(idx, :).ey_lo)/2;
    tracking_c(11, 1) = (tbl(idx, :).ez_hi + tbl(idx, :).ez_lo)/2;
    
    % make the generator
    tracking_G = zeros(N_FRS_rows, 3);
    tracking_G(1, 1) = (tbl(idx, :).ex_hi - tbl(idx, :).ex_lo)/2;
    tracking_G(6, 2) = (tbl(idx, :).ey_hi - tbl(idx, :).ey_lo)/2;
    tracking_G(11, 3) = (tbl(idx, :).ez_hi - tbl(idx, :).ez_lo)/2;
    
    % make the zonotope
    tracking_zono = zonotope([tracking_c, tracking_G]);
    
    % update the current cell array full of 'topes
    zono_cell{idx} = tracking_zono ;
    
    % toc the timer
    disptime = toc(start_tic) ;
    
    % display timing info and commit the current generated zonotopes to the
    % table
    if mod(idx, display_count) == 0
        disp([num2str(idx) ' took ' num2str(disptime) ' seconds.']) ;
%         tbl.zonotope((last_filled_idx+1):idx, 1) = zono_cell((last_filled_idx+1):idx);
        last_filled_idx = idx ;
    elseif idx == N_rows
        disp(['All ',num2str(idx) ' rows took ' num2str(disptime) ' seconds.']) ;
%         tbl.zonotope(idx - mod(N_rows, display_count) + 1:idx, 1) = zono_cell(1:mod(N_rows, display_count), 1);
    end
end
toc(start_tic)

%% fill in zonotopes to tracking error table
disp('Filling in tracking error table with zonotopes')
tic
tbl.zonotope = zono_cell ;
toc

%% test for empty rows
Z = tbl.zonotope ;
E = false(size(Z)) ;
for idx = 1:length(Z)
    E(idx) = isempty(Z{idx}) ;
end

disp(['Number of rows missing zonotopes: ',num2str(sum(E))])

%% update table for saving
tracking_error_table = tbl ;
clear tbl tempcell N_rows N_FRS_rows display_count

%% save result
save([filename(1:end-4),'_zonotope.mat']) ;