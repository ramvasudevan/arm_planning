classdef robot_arm_generic_planner < planner
    %% properties
    properties
        % arm
        dimension
        arm_n_states
        arm_n_inputs
        arm_n_links_and_joints
        arm_joint_state_limits
        arm_joint_speed_limits
        arm_joint_input_limits
        arm_joint_state_indices
        arm_joint_speed_indices
        
        % world
        start
        goal
        
        % planner
        t_stop = 1 ; % duration of stopping trajectory
        lookahead_distance = 1 ; % by 2-norm in config space
    end
    
    %% methods
    methods
        %% constructor
        function P = robot_arm_generic_planner(varargin)
            P@planner(varargin{:}) ;
            
            % check that t_stop is greater than t_move
            if P.t_stop <= P.t_move
                t_stop_new = P.t_move + 1 ;
                P.vdisp(['Setting stopping time to ',num2str(t_stop_new),' seconds'],3)
                P.t_stop = t_stop_new ;
            end
        end
        
        %% setup
        function setup(P,agent_info,world_info)
            % fill in agent info
            P.vdisp('Getting agent info',9)
            P.dimension = agent_info.dimension ;
            P.bounds = agent_info.reach_limits ;
            P = fill_in_arm_properties(P,agent_info,true) ;
            
            % fill in world info
            P.vdisp('Getting world info',9)
            P.start = world_info.start ;
            P.goal = world_info.goal ;
        end
        
    end
end