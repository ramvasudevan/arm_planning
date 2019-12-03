%% user parameters
trial_header = 'trial_20191125';
trial_folder = '/Users/pdholmes/Documents/MATLAB/arm_planning/simulator_files/testing/trial_data/';
trial_file_location = sprintf('%s*%s*', trial_folder, trial_header);

%% automated from here
files = dir(trial_file_location) ;

n_goal = [] ;
n_crash = [] ;
crash_filenames = {} ;

n_trials = 0 ;
c_idx = 1 ;
for idx = 1:length(files)
    disp(100*idx/length(files))
    fidx = load([trial_folder files(idx).name]) 
    s = fidx.summary ;
    
    if isempty(n_goal)
        n_goal = zeros(1,length(s)) ;
        n_crash = zeros(1,length(s)) ;
    end
    
    for sidx = 1:length(s)
        n_goal(sidx) = n_goal(sidx) + s(sidx).goal_check ;
        n_crash(sidx) = n_crash(sidx) + s(sidx).crash_check ;
        
        %             if s(1).crash_check
        %                 crash_filenames{c_idx} = files(idx).name ;
        %                 c_idx = c_idx + 1 ;
        %             end
    end
    
    n_trials = n_trials + 1 ;
end

n_goal./n_trials
n_crash./n_trials