%% description
% This script iterates through the quadrotor FRS and cleans it up by
% removing any generators that are all zeros
%
% Author: Shreyas Kousik
% Created: 7 Oct 2020
% Updated: nah
%
%% user parameters
FRS_filename = 'quadrotor_FRS_v7_a10_dt0.02.mat' ;

%% automated from here
% load FRS
load(FRS_filename)

%%
% iterate through Rcont zonotopes and remove all generators with only
% zeros, then combine all generators that are linearly dependent (haven't
% implemented this last part yet)
for idx = 1:length(Rcont)
    Z = Rcont{idx}{1} ;
    G = Z.generators ;
    zero_generators_log = sum(G,1) == 0 ;
    if any(zero_generators_log)
        G(:,zero_generators_log) = [] ;
    else
       disp(['generator ',num2str(idx),' is clean!']) 
    end
    
    % create new zonotope
    c = Z.center ;
    Z_new = zonotope([c,G]) ;
    
    % replace the zonotope
    Rcont{idx}{1} = Z_new ;
end

% save output
save('temp_FRS.mat','Rcont','t_peak','t_total','options1','options2','options3')