%% user parameters
folder_name = '~/MATLAB/quadrotor_RTD/step4_simulation/trial_data/trials_20190410/' ;

%% automated from here
files = dir(folder_name) ;

n_goal = [] ;
n_crash = [] ;
crash_filenames = {} ;

n_trials = 0 ;
c_idx = 1 ;
for idx = 1:length(files)
  % disp(100*idx/length(files))
    fidx = load(files(idx).name) ;
    s = fidx.summary ;

    if isempty(n_goal)
        n_goal = zeros(1,length(s)) ;
        n_crash = zeros(1,length(s)) ;
    end

    for sidx = 1:length(s)
        n_goal(sidx) = n_goal(sidx) + s(sidx).goal_check ;
        n_crash(sidx) = n_crash(sidx) + s(sidx).collision_check ;

    end

    n_trials = n_trials + 1 ;
end

n_goal./n_trials
n_crash./n_trials
