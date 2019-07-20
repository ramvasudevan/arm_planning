classdef robot_arm_PID_LLC < low_level_controller
    properties
        % agent properties
        agent_joint_state_indices
        agent_joint_speed_indices
        
        % gains
        K_ff
        K_p
        K_d
        K_i
        
        % position error for integrator term
        position_error_state
    end
    
    methods
        %% constructor
        function LLC = robot_arm_PID_LLC(varargin)
            LLC = parse_args(LLC,varargin{:}) ;
        end
        
        %% setup
        function setup(LLC,agent)
            % call default setup
            setup@low_level_controller(LLC,agent)
            
            % get indices of joint states and speeds
            LLC.agent_joint_state_indices = agent.joint_state_indices ;
            LLC.agent_joint_speed_indices = agent.joint_speed_indices ;
            
            % create default control gains, assuming the agent's states are
            % in the format
            %   (q_1, qdot_1, q_2, qdot_2, ... , q_n, qdot_n),
            % where q_i is the configuration of joint i
            
            if isempty(LLC.K_ff)
                LLC.K_ff = eye(LLC.n_agent_inputs) ;
            end
            
            if isempty(LLC.K_p)
                P = 0.1*[eye(LLC.n_agent_inputs) ; zeros(LLC.n_agent_inputs)] ;
                LLC.K_p = reshape(P,LLC.n_agent_inputs,[]) ;
            end
            
            if isempty(LLC.K_d)
                D = 0.01*[zeros(LLC.n_agent_inputs) ; eye(LLC.n_agent_inputs)] ;
                LLC.K_d = reshape(D,LLC.n_agent_inputs,[]) ;
            end
            
            if isempty(LLC.K_i)
                LLC.K_i = 0.001*eye(LLC.n_agent_inputs) ;
            end
            
            % reset the integrator error
            LLC.position_error_state = zeros(length(LLC.agent_joint_state_indices),1) ;
        end
        
        %% get control inputs
        function u = get_control_inputs(LLC,~,t,z_cur,varargin)
            % u = LLC.get_control_inputs(~,t_cur,z_cur,T_ref,U_ref,Z_ref)
            
            % get reference time and (feedforward) input
            T_ref = varargin{1} ;
            U_ref = varargin{2} ;
            
            % get reference input and state at the current time
            if length(varargin) > 2 && ~isempty(varargin{3})
                Z_ref = varargin{3} ;
                [u_ref,z_ref] = match_trajectories(t,T_ref,U_ref,T_ref,Z_ref) ;
            else
                u_ref = match_trajectories(t,T_ref,U_ref) ;
                z_ref = z_cur ;
            end
            
            % compute control input
            z_cur
            pd_error = z_cur - z_ref ;
            i_error = LLC.position_error_state ;
            u = LLC.K_ff*u_ref - LLC.K_p*pd_error + ...
                - LLC.K_d*pd_error - LLC.K_i*i_error ;
            
            % update integrator error
            LLC.position_error_state = i_error + pd_error(LLC.agent_joint_state_indices) ;
        end
    end
end