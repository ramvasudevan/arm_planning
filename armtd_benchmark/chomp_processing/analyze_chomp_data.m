%% description
% Reads the .csv files output by post-processing CHOMP/MoveIt log files and
% analyzes the success of CHOMP in trajectory planning on 100
% randomly-generated scenes. Make sure the following folder is on your
% MATLAB path:
%
%   arm_planning > armtd_benchmark > chomp_processing > processed_data
%
% Author: Shreyas Kousik
% Created: 23 Jan 2020
% Updated: 23 Jan 2020
%
%% load the data
data = [] ;
failed_files = [] ;

for scene_idx = 1:10
    scene_str = num2str(scene_idx,'%02d') ;
    
    disp(['Getting data from scene ',scene_str])
    
    for test_idx = 1:10
        test_str = num2str(test_idx,'%02d') ;
        data_filename = ['chomp_test_results_0',scene_str,'_0',test_str,'.csv'] ;
        
        try
            %disp(['Reading file ',data_filename])
            data = [data ; csvread(data_filename) ] ;
        catch
            failed_files = [failed_files, {data_filename}] ;
        end
    end
end

if ~isempty(failed_files)
    disp('---')
    disp('Unable to read the following files:')
    for idx = 1:length(failed_files)
        disp(failed_files{idx})
    end
end

%% calculate stuff about the data
% check number of successful trials
num_success = sum(data(:,4))

% get average solve time
avg_solve_time = mean(data(:,5))