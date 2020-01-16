%% description
% This script iterates through the saved armtd trials and summarizes
% information about each (ie crash check, goal check, planning time, etc.)
%
% Author: Patrick Holmes
% Created 06 December 2019

clear; clc;

trial_file_header = 'trial_scene_' ;
trial_file_folder = '/Users/pdholmes/Documents/MATLAB/arm_planning/simulator_files/testing/trial_data/20191205/' ;
trial_file_location = sprintf('%s*%s*', trial_file_folder, trial_file_header);
trial_file_list = dir(trial_file_location);

results = struct();
results.scene = {};
results.crash = [];
results.goal = [];
results.avg_planning_time = [];

for idx = 1:length(trial_file_list)
    trial_filename = trial_file_list(idx).name;
    mytrial = load([trial_file_folder trial_filename]);
    summary = mytrial.summary;
    
    results.scene{end+1, 1} = mytrial.world_filename;
    results.crash(end+1, 1) = summary.crash_check;
    results.goal(end+1, 1) = summary.goal_check;
    results.avg_planning_time(end+1, 1) = mean(summary.planning_time(~isnan(summary.planning_time)));
    
end

disp(struct2table(results));

disp('Crashed?');
disp([num2str(sum(results.crash)) ' / ' num2str(length(results.crash))]);
disp('Reached Goal?');
disp([num2str(sum(results.goal)) ' / ' num2str(length(results.goal))]);
disp('Avg. planning time');
disp(num2str(mean(results.avg_planning_time)));