% script for creating prespecified test trajectory
% 20200108 just use a simple sine wave on the last joint for 80 seconds.
clc; clear;

desiredRate = 10; % desired rate (in Hz) of trajectory
T = 80; % total move time;
t = 0:1/desiredRate:80;

Position = zeros(length(t), 7);
Velocity = zeros(length(t), 7);

omega = 1/20; % for now, set angular frequency of sine wave to 1/20
for i = 1:length(t)
   Position(i, 6) = sin(2*pi*omega*t(i));
   Velocity(i, 6) = 2*pi*omega*cos(2*pi*omega*t(i));
end

save('arm_planning/real_robot_files/test_trajectory_20200108.mat');