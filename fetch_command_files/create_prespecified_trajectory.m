% script for creating prespecified test trajectory
% 20200108 just use a simple sine wave on the last joint for 80 seconds.
clc; clear;

desiredRate = 100; % desired rate (in Hz) of trajectory
T = 60; % total move time;
t = 0:1/desiredRate:80;

Position = zeros(length(t), 7);
Velocity = zeros(length(t), 7);

omega = 1/15; % for now, set angular frequency of sine wave to 1/20
for i = 1:length(t)
   Position(i, 4) = 0.5*sin(2*pi*omega*t(i));
   Velocity(i, 4) = 0.5*2*pi*omega*cos(2*pi*omega*t(i));
   
   Position(i, 5) = sin(2*pi*omega*t(i));
   Velocity(i, 5) = 2*pi*omega*cos(2*pi*omega*t(i));
   
   Position(i, 6) = sin(2*pi*omega*t(i));
   Velocity(i, 6) = 2*pi*omega*cos(2*pi*omega*t(i));
end

save('arm_planning/fetch_command_files/test_trajectory_20200108.mat');