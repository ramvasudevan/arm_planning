% rosinit('192.168.1.100')

[pub,msg] = rospublisher('/fetch/des_states','sensor_msgs/JointState');

msg.Name=["shoulder_pan_joint" "shoulder_lift_joint" "upperarm_roll_joint" "elbow_flex_joint"...
    "forearm_roll_joint" "wrist_flex_joint" "wrist_roll_joint"];
msg.Position = [0 0 0 0 0 0 0 ];
msg.Velocity = [0 0 0 0 0 0 0 ];

desiredRate = 10;
rate = robotics.Rate(desiredRate);

rate.OverrunAction = 'drop';
reset(rate)

omega = 1/20;
time_test = [];
pos_test = [];
vel_test = [];
while rate.TotalElapsedTime < 80
    T = rate.TotalElapsedTime;
    msg.Position(6) = sin(2*pi*omega*T);
    msg.Velocity(6) = 2*pi*omega*cos(2*pi*omega*T);
%     msg.Position(6) = 0;
%     msg.Velocity(6) = 0;

    time_test(end+1, 1) = T;
    pos_test(end+1, 1) = msg.Position(6);
    vel_test(end+1, 1) = msg.Velocity(6);

    send(pub,msg);
    waitfor(rate);
end

msg.Velocity = zeros(1, 7);
send(pub, msg);

figure(1); clf; hold on;
plot(time_test, pos_test, 'b');
plot(time_test, vel_test, 'r');
% rosshutdown
