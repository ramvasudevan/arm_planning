classdef robot_arm_RTD_planner < robot_arm_generic_planner
    properties
        t_total = 1 ; % total duration of desired traj
        time_discretization = 0.01 ; % time discretization of desired traj
    end
    methods
        %% constructor
        function P = robot_arm_RTD_planner(varargin)
            P@robot_arm_generic_planner(varargin{:}) ;
        end
        
        %% replan
        function [T,U,Z] = replan(P,agent_info,world_info)
            %% 1. generate cost function
            % generate a waypoint in configuration space
            q_cur = agent_info.state(P.arm_joint_state_indices) ;
            q_goal = P.goal ;
            dir_des = q_goal - q_cur ;
            dir_des = dir_des./norm(dir_des) ;
            q_des = q_cur + P.lookahead_distance.*dir_des ;
            
            % turn waypoint into a cost function
            %%% PATRICK CODE HERE %%%
            
            %% 2. generate constraints
            %% constraint generation
            % get current obstacles
            O = world_info.obstacles ;
            
            % map obstacles to trajectory parameter space
            %%% PATRICK CODE HERE %%%
            
            %% 3. solve for optimal trajectory parameter
            %%% PATRICK CODE HERE %%%
            
            %% 4. generate desired trajectory
            %%% PATRICK MODIFY THE TRAJECTORY Z HERE %%%
            T = 0:P.time_discretization:P.t_total ;
            N_T = length(T) ;
            U = zeros(P.arm_n_states,N_T) ;
            Z = nan(P.arm_n_states,N_T) ;
        end
    end
end