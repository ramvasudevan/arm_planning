clear; clc;

rosinit('192.168.1.100', 11311)

%% setup for traj
% create global variable to fill in with joint state data
global joint_pos_data start_saving_data_flag
joint_pos_data = [] ;
start_saving_data_flag = false ;

% create ros subscriber
joint_sub = rossubscriber('/joint_states',@joint_state_callback) ;

[pub,msg] = rospublisher('/fetch/des_states','trajectory_msgs/JointTrajectory') ;

msg.JointNames =["shoulder_pan_joint" "shoulder_lift_joint" "upperarm_roll_joint" "elbow_flex_joint"...
    "forearm_roll_joint" "wrist_flex_joint" "wrist_roll_joint"];

traj = load('test_trajectory_20200108.mat');

msg.Points = traj.p_array ;

pause(1) ; % pause to make sure ROS stuff comes up correctly

%% send message
start_saving_data_flag = true ;
disp('Sending traj!')
traj_start_tic = tic ;
send(pub,msg)

% wait until done
pause(traj.t(end) + 1) ;
toc(traj_start_tic)

%% finish up the ros stuff
clear joint_sub
rosshutdown

joint_pos_data(1,:) = joint_pos_data(1,:) - joint_pos_data(1,1) ;

%% plot
figure(1); clf; hold on;
plot(traj.t, traj.Position(:,6), 'b');
plot(joint_pos_data(1,:),joint_pos_data(end-1,:),'b--')

%% hlepper functions
function joint_state_callback(src,msg)
    global joint_pos_data start_saving_data_flag

    % get the joint states out of the message
    if size(msg.Position,1) > 2 && start_saving_data_flag
        q = msg.Position(end-6:end) ;
        
        % get the time stamp
        t_stamp = msg.Header.Stamp ;
        t = t_stamp.Sec + (t_stamp.Nsec*(1e-9)) ;
        
        % save
        joint_pos_data = [joint_pos_data, [t;q]] ;
    end
    
    % pozz
    pause(0.001)
end