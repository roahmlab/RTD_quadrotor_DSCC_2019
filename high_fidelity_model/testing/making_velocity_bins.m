%% user parameters
v_max = 3.75 ; % +/- in x,y,z
N_bins_per_dim = 15 ; % number of bins per dimension, so this number cubed is the total
filename = 'QR_initial_condition_info.mat' ;

%% automated from here
% side length of bin
dv = 2*v_max/N_bins_per_dim ;

% get range of speeds for each dimension
v_range = -v_max:dv:v_max ;

% get vertices of all bins
[vx,vy,vz] = ndgrid(v_range,v_range,v_range) ;
v_vertices_all = [vx(:), vy(:), vz(:)]' ;
length(v_vertices_all)

% get all bins with at least one vertex inside the sphere of radius v_max
D_bin = sqrt(3)*dv; % diagonal length of a single bin
d_bins_log = distPointToPoints([0;0;0],v_vertices_all) <= (v_max+D_bin) ;
v_vertices = v_vertices_all(:,d_bins_log) ;

% save vertices and info
save(filename,'v_vertices','v_max','dv','v_range','N_bins_per_dim')

%% plotting
figure(1) ; clf ; hold on ; axis equal ;
plot3(v_vertices_all(1,:),v_vertices_all(2,:),v_vertices_all(3,:),'.')
plot3(v_vertices(1,:),v_vertices(2,:),v_vertices(3,:),'r.')