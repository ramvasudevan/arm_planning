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
% Updated: 28 Jan 2020
%
%% user parameters
data_folder = '~/MATLAB/arm_planning/armtd_benchmark/chomp_processing/processed/chomp_test_results_20200127/' ;
world_folder = '~/MATLAB/arm_planning/simulator_files/testing/saved_worlds/20200127/' ;
traj_folder = '~/MATLAB/arm_planning/armtd_benchmark/chomp_processing/processed/chomp_test_results_20200127/chomp_trajectory_output_20200127/' ;
collision_check_chomp_trajectories_flag = true ;
verbosity = 0 ;

scene_idxs = 4:4:40 ; % use 4:4:40 for all scenes
test_idxs = 1:10 ; % use 1:10 for all tests

%% load the data
data = [] ;
failed_files = [] ;
goal_check = [] ;
collision_check = [] ;

for scene_idx = scene_idxs
    scene_str = num2str(scene_idx,'%02d') ;
    
    disp(['Getting data from scene ',scene_str])
    
    for test_idx = test_idxs
        test_str = num2str(test_idx,'%02d') ;
        data_filename = [data_folder,'chomp_test_results_0',scene_str,'_0',test_str,'.csv'] ;
        
%         try
            % read the data output from running MoveIt
            new_data = csvread(data_filename) ;
            
            % get trajectory info
            traj_filename = ['chomp_trajectory_scene_0',scene_str,'_0',test_str,'.txt'] ;
            traj = textread(traj_filename) ;
            traj = traj(:,1:end-1)' ;
            traj_length = dist_polyline_cumulative(traj) ;
            traj_length = traj_length(end) ;
            
            % get world info
            world_filename = [world_folder, 'scene_0',scene_str,'_0',test_str,'.csv'] ;
            [start, goal, obstacles] = load_saved_world(world_filename) ;
            world_traj_length = vecnorm(start - goal) ;
            
            % add traj length to data
            new_data = [scene_idx, test_idx, new_data, traj_length, world_traj_length, traj_length/world_traj_length] ;
            
            % update data
            data = [data ; new_data] ;            
            
            if collision_check_chomp_trajectories_flag
                % make agent
                A = robot_arm_3D_fetch('verbose', verbosity) ;
                
                % fill in agent
                N = size(traj,2) ;
                dt = 0.03 ;
                t = linspace(0,dt*N,N) ;
                A.time = t ;
                A.state = zeros(A.n_states,N) ;
                A.state(A.joint_state_indices,:) = traj ;
                AI = A.get_agent_info ;
                
                % make world
                W = fetch_base_world_static('create_random_obstacles_flag', false,...
                    'include_base_obstacle', true,...
                    'N_obstacles',length(obstacles),'dimension',3,...
                    'workspace_goal_check', 0,...
                    'verbose', verbosity, 'start', start, 'goal', goal,...
                    'obstacles', obstacles) ;
                W.setup(AI) ;
                
                % perform goal and crash checks
                new_goal_check = W.goal_check(AI) ;
                new_collision_check = W.collision_check(AI,true) ;
                
                disp(['scene ',scene_str,' test ',test_str])
                disp(['   goal: ',num2str(new_goal_check)])
                disp(['   collision: ',num2str(new_collision_check)])
                
                if new_collision_check
                    % animate agent
                    figure(1) ; clf ; axis equal ; hold on ; view(3) ; grid on ;
                    plot(W) ;
                    plot(A) ;
                    animate(A) ;
                end
                
                goal_check = [goal_check ; new_goal_check] ;
                collision_check = [collision_check ; new_collision_check] ;
            end
%         catch
%             failed_files = [failed_files, {data_filename}] ;
%         end
    end
end

if ~isempty(failed_files)
    disp('---')
    disp('Unable to read the following files:')
    for idx = 1:length(failed_files)
        disp(failed_files{idx})
    end
end

%% save data
column_info = {'scene_number',...
    'test_number',...
    'reached_start','planning_time_to_start',...
    'time_per_iter',...
    'reached_goal',...
    'planning_time_to_goal',...
    'time_per_iter',...
    'traj_length',...
    'world_dist_start_to_goal',...
    'traj_to_world_dist_ratio'} ;

% save('chomp_data.mat','data','column_info')

%% calculate stuff about the data
% check number of successful trials
num_= sum(data(:,4)) ;

% check number of crashes
sum(goal_check)
sum(collision_check)

% get average solve time
avg_solve_time = mean(data(:,5)) ;