% DON'T USE THIS IT IS OLD :) - PATRICK

classdef robot_arm_3D_fetch_hardware < robot_arm_3D_fetch
    %% properties
    properties
        % ROS communication
        ROS_msg;
        ROS_pub;
        ROS_sub;
        
        % 
%         joint_pos_data
        saving_data_flag = false;
    end
    
    %% methods
    methods
        function A = robot_arm_3D_fetch_hardware(varargin)
            
            A@robot_arm_3D_fetch(varargin{:}) ;
            
            A.vdisp('Setting up ROS', 6);
            % for ROS communication
            rosinit('192.168.1.100', 11311);
            A.ROS_sub = rossubscriber('/joint_states',@A.ROS_sub_callback) ;
            [A.ROS_pub, A.ROS_msg] = rospublisher('/fetch/des_states','trajectory_msgs/JointTrajectory') ;
            A.ROS_msg.JointNames = ["shoulder_pan_joint" "shoulder_lift_joint" "upperarm_roll_joint" "elbow_flex_joint"...
                "forearm_roll_joint" "wrist_flex_joint" "wrist_roll_joint"];
            pause(0.1);
            A.saving_data_flag = true;
        end
        
        function A = ROS_sub_callback(A, src, msg)            
            % get the joint states out of the message
            if size(msg.Position,1) == 13 && A.saving_data_flag
                q = msg.Position(7:12) ;
                q_dot = msg.Velocity(7:12);
                
                % get the time stamp
                t_stamp = msg.Header.Stamp ;
                t = t_stamp.Sec + (t_stamp.Nsec*(1e-9)) ;
                
                % save
                % get the reference trajectory up to time t_move
%                 T = 0:A.integrator_time_discretization:t_move ;
%                 [U,Z] = match_trajectories(T,T_ref,U_ref,T_ref,Z_ref) ;
                
                % append the reference trajectory to the agent's
                % current trajectory
                z = [q(1:end-1, 1)'; q_dot(1:end-1, 1)'];
                z = z(:);
                u = zeros(6, 1);
                A.commit_move_data(t,z,t,u) ;
%                 A.joint_pos_data = [A.joint_pos_data, [t;q]] ;
            end
            
            % pozz
%             pause(0.001)
        end
        
        function A = move(A,t_move,T_ref,U_ref,Z_ref)
            
            position = Z_ref(1:2:end, :)';
            velocity = Z_ref(2:2:end, :)';
            position = [position, zeros(size(position, 1), 1)];
            velocity = [velocity, zeros(size(velocity, 1), 1)];
            time_vector = T_ref - T_ref(1);
            %% make array for message
            N_pts = size(position, 1) ;
            
            % HACK FOR DEBUGGING
            position = zeros(size(position));
            velocity = zeros(size(velocity));
            position(:, end) = pi/6;
            
            % preallocate array for joint trajs
            p_array = [] ;
            
            tic
            for idx = 1:N_pts
                % make a joint trajectory point
                p = rosmessage('trajectory_msgs/JointTrajectoryPoint') ;
                
                % fill in the position and velocity
                p.Positions = position(idx,:) ;
                p.Velocities = velocity(idx,:) ;
                
                % fill in the duration
                p.TimeFromStart = rosduration(time_vector(idx)) ;
                
                % fill in p_array
                p_array = [p_array, p] ;
            end
            toc
            
            A.ROS_msg.Points = p_array;
            A.ROS_send_message();
        end
        
        function A = ROS_send_message(A)
            send(A.ROS_pub, A.ROS_msg);
            A.vdisp('Sent trajectory to ROS!!', 6);
        end
        
        function A = ROS_close(A)
            %% finish up the ros stuff
            A.saving_data_flag = false;
            clear A.ROS_sub;
            rosshutdown;
        end
    end
end