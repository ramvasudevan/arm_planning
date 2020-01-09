clear; clc;
rosinit('192.168.1.100', 11311)
%% shreycode
% create ros subscriber
joint_sub = rossubscriber('/joint_states',@joint_state_callback) ;

% create global variable to fill in with joint state data
global joint_pos_data start_saving_data_flag
joint_pos_data = [] ;
start_saving_data_flag = false ;

%% patricode from here on
[pub,msg] = rospublisher('/fetch/des_states','sensor_msgs/JointState');

msg.Name=["shoulder_pan_joint" "shoulder_lift_joint" "upperarm_roll_joint" "elbow_flex_joint"...
    "forearm_roll_joint" "wrist_flex_joint" "wrist_roll_joint"];
msg.Position = [0 0 0 0 0 0 0];
msg.Velocity = [0 0 0 0 0 0 0];
% msg.Effort = [0 0 0 0 0 0 0];

traj = load('test_trajectory_20200108.mat');

desiredRate = traj.desiredRate;
rate = robotics.Rate(desiredRate);

rate.OverrunAction = 'drop';
reset(rate)

time_test = [];
pos_test = [];
vel_test = [];
acc_test = [];

start_saving_data_flag = true ;

while rate.TotalElapsedTime < (traj.T)
    curr_time = rate.TotalElapsedTime;
    [~, curr_time_idx] = min(abs(traj.t - curr_time));
    msg.Position = traj.Position(curr_time_idx, :);
    msg.Velocity = traj.Velocity(curr_time_idx, :);
%     msg.Effort = traj.Acceleration(curr_time_idx, :);

    time_test(end+1, 1) = curr_time;
    pos_test(end+1, 1) = msg.Position(6);
    vel_test(end+1, 1) = msg.Velocity(6);
%     acc_test(end+1, 1) = msg.Effort(6);

    send(pub,msg);
    waitfor(rate);
end

msg.Position = zeros(1, 7);
msg.Velocity = zeros(1, 7);
% msg.Effort = zeros(1, 7);
send(pub, msg);

clear joint_sub
rosshutdown

joint_pos_data(1,:) = joint_pos_data(1,:) - joint_pos_data(1,1) ;

%%
figure(1); clf; hold on;
plot(time_test, pos_test, 'b');
% plot(time_test, vel_test, 'r');
plot(joint_pos_data(1,:),joint_pos_data(end-1,:),'b--')

%% hlepper functions
function joint_state_callback(src,msg)
    global joint_pos_data start_saving_data_flag

    % get the joint states out of the message
    if size(msg.Position,1) == 13 && start_saving_data_flag
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