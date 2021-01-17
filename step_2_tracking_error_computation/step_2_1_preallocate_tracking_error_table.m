%% user parameters
% velocity and time info
v_max = 1.5 ; % m/s
a_max = 3 ; % m/s^2
t_max = 3 ; % s
v_thresh_pct = 110 ;

% bin specification
N_bins_per_dim = 15 ; % number of bins per dimension
dt = 0.02 ; % seconds spanned per time bin

% file i/o
filename = 'quadrotor_tracking_error_table' ;

%% automated from here
% side length of bin
dv = 2*v_max/N_bins_per_dim ;

% get bin centers in each dimension
vx_range = 0:dv:v_max ;
vy_range = 0:dv:v_max ;
vz_range = -v_max:dv:v_max ;

% get bin centers in time
t_range = (dt/2):dt:t_max ;

% get all bin centers
[vx_c,vy_c,vz_c,t_c] = ndgrid(vx_range,vy_range,vz_range,t_range) ;
vx_c = vx_c(:) ;
vy_c = vy_c(:) ;
vz_c = vz_c(:) ;
t_c = t_c(:) ;

% get original number of bins
N_bins_orig = length(vx_c) ;

% keep all bins that have centers within v_thresh_pct of max speed
v_c = [vx_c,vy_c,vz_c] ;
spd_c = vecnorm(v_c,2,2) ;
spd_c_log = spd_c <= ((v_thresh_pct/100)*v_max) ;
vx_c = vx_c(spd_c_log) ;
vy_c = vy_c(spd_c_log) ;
vz_c = vz_c(spd_c_log) ;
t_c = t_c(spd_c_log) ;

% get unique velocities of bin centers
bin_center_velocities = unique([vx_c,vy_c,vz_c],'rows') ;

% get number of bins
N_bins = length(vx_c) ;

% get bin lower and upper bounds
vx_lo = vx_c - dv/2 ;
vy_lo = vy_c - dv/2 ;
vz_lo = vz_c - dv/2 ;
vx_hi = vx_c + dv/2 ;
vy_hi = vy_c + dv/2 ;
vz_hi = vz_c + dv/2 ;

% get time lower and upper bounds
t_lo = t_c - dt/2 ;
t_hi = t_c + dt/2 ;

% get initial error estimates
ex_lo = zeros(N_bins,1) ;
ey_lo = zeros(N_bins,1) ;
ez_lo = zeros(N_bins,1) ;
ex_hi = zeros(N_bins,1) ;
ey_hi = zeros(N_bins,1) ;
ez_hi = zeros(N_bins,1) ;

% generate table 
tracking_error_table = table(t_lo,t_hi,vx_lo,vx_hi,vy_lo,vy_hi,vz_lo,vz_hi,...
    ex_lo,ex_hi,ey_lo,ey_hi,ez_lo,ez_hi) ;

%% save table
filename = [filename,'_dt',num2str(dt),'_v',num2str(v_max),'.mat'] ;
save(filename,'tracking_error_table','v_max','dv','t_max','a_max',...
    'N_bins_per_dim','dt','N_bins','bin_center_velocities','filename')