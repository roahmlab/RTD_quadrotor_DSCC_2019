%% description
% This script combines three of the 1D FRS for the quadrotor into a single
% 3D FRS.
%
% Authors: Patrick Holmes and Shreyas Kousik
% Updated: 7 Oct 2020 (cleaned all zero generator columns)
%
%% user parameters
load_filename = 'quadrotor_FRS_1D.mat' ;
save_filename = 'quadrotor_FRS.mat' ;

%% automated from here
plot_flag = 1;

FRSx = load(load_filename);
FRSy = load(load_filename);
FRSz = load(load_filename);
Rcont = {};
tic
for i = 1:length(FRSx.Rcont)
    % get x y and z zonotopes
    Z1 = get(FRSx.Rcont{i}{1}, 'Z');
    Z2 = get(FRSy.Rcont{i}{1}, 'Z');
    Z3 = get(FRSz.Rcont{i}{1}, 'Z');
    
    % create zeros of appropriate size
    zero_padding = zeros(size(Z1, 1), size(Z1, 2) - 1);
    
    % concatenate zonotopes
    Z_new = [Z1(:, :), zero_padding, zero_padding ;
        Z2(:, 1), zero_padding, Z2(:, 2:end), zero_padding ;
        Z3(:, 1), zero_padding, zero_padding, Z3(:, 2:end)];
    
    
    % remove any columns that are all zeros
    zero_generators_log = sum(Z_new,1) ;
    Z_new(:,zero_generators_log) = [] ;
    
    % save the new zonotope
    Rcont{i}{1} = zonotope(Z_new);
end
disp('Computation time of FRS 3D intersection:');
toc;



% get timing and options
t_peak = FRSx.t_peak ;
t_total = FRSx.t_total ;
options1 = FRSx.options;
options2 = FRSy.options;
options3 = FRSz.options;

% save FRS
save(save_filename, 'Rcont', 'options1', 'options2', 'options3','t_peak','t_total');

%% plotting
if plot_flag
    % plots FRS in x-y plane as a function of time
    projectedDimensions=[1 6];
    
    figure(1); clf;
    hold on
    
    %plot reachable sets
    for i=1:1:length(Rcont)
        Zproj = project(Rcont{i}{1},projectedDimensions);
        Zproj = reduce(Zproj,'girard',3);
        Zpts = polygon(Zproj)';
        p1 = fill3(ones(size(Zpts, 1), 1)*i*FRSx.options.timeStep, Zpts(:, 1), Zpts(:, 2),[0.9 0.9 0.9]);
        
        p1.FaceColor = [0 1 0];
        p1.FaceAlpha = 0.3;
        p1.EdgeAlpha = 0.5;
    end
    
    ylabel(['x_{',num2str(projectedDimensions(1)),'}']);
    zlabel(['x_{',num2str(projectedDimensions(2)),'}']);
end