%% description
% This script iterates through the saved armtd trials and summarizes
% information about each (ie crash check, goal check, planning time, etc.)
%
% Author: Patrick Holmes
% Created 06 December 2019

clear; clc;

trial_file_header = 'trial_scene_' ;
trial_file_folder = 'arm_planning/simulator_files/testing/trial_data/20200127/' ;
trial_file_location = sprintf('%s*%s*', trial_file_folder, trial_file_header);
trial_file_list = dir(trial_file_location);

results = struct();
results.scene = {};
results.crash = [];
results.goal = [];
results.start_to_goal_length = [];
results.normalized_path_length = [];
results.avg_planning_time = [];

goal_radius = pi/30;
dz_goal_radius = vecnorm(pi/30*ones(6, 1));

for idx = 1:length(trial_file_list)
    trial_filename = trial_file_list(idx).name;
    mytrial = load([trial_file_folder trial_filename]);
    summary = mytrial.summary;
    
    results.scene{end+1, 1} = mytrial.world_filename;
    results.crash(end+1, 1) = summary.collision_check;
    results.goal(end+1, 1) = summary.goal_check;
    z = summary.trajectory(summary.agent_info.joint_state_indices, :);
    dz = dist_polyline_cumulative(z);
    dz_startgoal = vecnorm(summary.goal - summary.start);
    results.start_to_goal_length(end+1, 1) = dz_startgoal;
    results.normalized_path_length(end+1, 1) = (dz(end) + dz_goal_radius)/dz_startgoal;
    results.avg_planning_time(end+1, 1) = mean(summary.planning_time(~isnan(summary.planning_time)));
    
end

disp(struct2table(results));

disp('Crashed?');
disp([num2str(sum(results.crash)) ' / ' num2str(length(results.crash))]);
disp('Reached Goal?');
disp([num2str(sum(results.goal)) ' / ' num2str(length(results.goal))]);
disp('Avg. path length (for successes)');
disp(num2str(mean(results.normalized_path_length(results.goal == 1))));
% disp('Avg. planning time (for successes)');
% disp(num2str(mean(results.avg_planning_time(results.goal == 1))));
% disp('Avg. planning time (for failures)');
% disp(num2str(mean(results.avg_planning_time(results.goal == 0))));
disp('Avg. planning time');
disp(num2str(mean(results.avg_planning_time)));


%% compare to chomp
load('chomp_data.mat');
goal_col = find(strcmp(column_info, 'reached_goal'));
time_col = find(strcmp(column_info, 'planning_time_to_goal'));
norm_col = find(strcmp(column_info, 'traj_to_world_dist_ratio'));

disp('');
disp('CHOMP reached goal?');
chomp_succ_idx = find(data(:, goal_col) == 1);
disp([num2str(sum(data(chomp_succ_idx, goal_col))) ' / ' num2str(length(data(:, goal_col)))]);
disp('CHOMP avg. path length (for successes)');
disp(num2str(mean(data(chomp_succ_idx, norm_col))));
% disp('CHOMP avg. planning time (for successes)');
% disp(num2str(mean(data(chomp_succ_idx, time_col))));
disp('CHOMP avg. planning time');
disp(num2str(mean(data(:, time_col))));

