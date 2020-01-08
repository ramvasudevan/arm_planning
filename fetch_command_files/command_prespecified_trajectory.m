rosinit('192.168.1.100', 11311)

[pub,msg] = rospublisher('/fetch/des_states','sensor_msgs/JointState');

msg.Name=["shoulder_pan_joint" "shoulder_lift_joint" "upperarm_roll_joint" "elbow_flex_joint"...
    "forearm_roll_joint" "wrist_flex_joint" "wrist_roll_joint"];
msg.Position = [0 0 0 0 0 0 0];
msg.Velocity = [0 0 0 0 0 0 0];

traj = load('test_trajectory_20200108.mat');

desiredRate = traj.desiredRate;
rate = robotics.Rate(desiredRate);

rate.OverrunAction = 'drop';
reset(rate)

time_test = [];
pos_test = [];
vel_test = [];
while rate.TotalElapsedTime < traj.T
    curr_time = rate.TotalElapsedTime;
    [~, curr_time_idx] = min(abs(traj.t - curr_time));
    msg.Position = traj.Position(curr_time_idx, :);
    msg.Velocity = traj.Velocity(curr_time_idx, :);

    time_test(end+1, 1) = curr_time;
    pos_test(end+1, 1) = msg.Position(6);
    vel_test(end+1, 1) = msg.Velocity(6);

    send(pub,msg);
    waitfor(rate);
end

msg.Position = zeros(1, 7);
msg.Velocity = zeros(1, 7);
send(pub, msg);

figure(1); clf; hold on;
plot(time_test, pos_test, 'b');
plot(time_test, vel_test, 'r');
rosshutdown
