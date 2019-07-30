classdef robot_arm_generic_planner < planner
    %% properties
    properties
        % arm
        dimension
        arm_n_states
        arm_n_inputs
        arm_n_joints
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
            % set any joint limits that are +Inf to pi and -Inf to -pi
            joint_state_limits = agent_info.joint_state_limits ;
            joint_limit_infs = isinf(joint_state_limits) ;
            joint_state_limits(1,joint_limit_infs(1,:)) = -pi ;
            joint_state_limits(2,joint_limit_infs(2,:)) = +pi ;
            
            % agent info
            P.vdisp('Getting agent info',9)
            P.dimension = agent_info.dimension ;
            P.bounds = agent_info.reach_limits ;
            P.arm_n_states = agent_info.n_states ;
            P.arm_n_inputs = agent_info.n_inputs ;
            P.arm_n_joints = agent_info.n_links_and_joints ;
            P.arm_joint_state_limits = joint_state_limits ;
            P.arm_joint_speed_limits = agent_info.joint_speed_limits ;
            P.arm_joint_input_limits = agent_info.joint_input_limits ;
            P.arm_joint_state_indices = agent_info.joint_state_indices ;
            P.arm_joint_speed_indices = agent_info.joint_speed_indices ;
            
            % world info
            P.vdisp('Getting world info',9)
            P.start = world_info.start ;
            P.goal = world_info.goal ;
        end
        
    end
end