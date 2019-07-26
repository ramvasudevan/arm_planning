classdef robot_arm_RRT_planner < planner
    properties
        arm_n_joints
        arm_dimension
        arm_joint_state_limits
        arm_joint_speed_limits
        time_discretization = 0.01 ;
    end
    
    methods
        %% constructor
        function P = robot_arm_RRT_planner(varargin)
            P@planner(varargin{:}); 
            
            %%% START NOTE FOR DAPHNA
            % Put any input arguments for the high level planner
            % construction here
            P.HLP = robot_arm_RRT_HLP() ;
            %%% END NOTE FOR DAPHNA
        end
        
        %% setup
        function setup(P,agent_info,~)
            % P.setup(agent_info, world_info)
            %
            % Get info about workspace dimension and joint limits before
            % running any online receding-horizon planning.

            % get the workspace dimension
            P.arm_dimension = agent_info.dimension ;
            
            % get joint limits (set +Inf to pi -Inf to -pi)
            joint_state_limits = I.joint_state_limits ;
            joint_limit_infs = isinf(joint_state_limits) ;
            joint_state_limits(1,joint_limit_infs(1,:)) = -pi ;
            joint_state_limits(2,joint_limit_infs(2,:)) = +pi ;
            
            P.arm_joint_state_limits = joint_state_limits ;
            P.arm_n_joints = size(joint_state_limits,2) ;
            
            % arm joint speed limits
            P.arm_joint_speed_limits = agent_info.joint_speed_limits ;
        end
        
        %% replan
        function[T,U,Z] = replan(P,agent_info,world_info)
            % get agent's current state
            j_idx = agent_info.joint_state_indices ;
            jd_idx = agent_info.joint_speed_indices ;
            z_cur = agent_info.state(:,end) ;
            
            % get current joint speeds
            zd_cur = agent_info.state(:,jd_idx) ;
            
            %%% START NOTE FOR DAPHNA
            % Run the HLP's RRT for the duration P.t_plan here
            %%% END NOTE FOR DAPHNA
            
            % get the RRT's current shortest path
            Z = P.HLP.waypoints ;
            n_Z = size(Z,2) ;
            
            % convert the path into a trajectory
            if n_Z > 0
                %%% START NOTE FOR DAPHNA
                % The
                %%% END NOTE FOR DAPHNA
                
                T = linspace(0,N_z*P.HLP.time_discretization) ;
                
                % iterate through the columns of Z



                % create a zero array of inputs
            else
                % send an empty trajectory, which will force the arm to
                % begin stopping (probably we will want to continue
                % tracking the previously-found trajectory instead of doing
                % this, but this is an ok place to start)
                T = [] ; U = [] ; Z = [] ;
            end
        end
    end
end