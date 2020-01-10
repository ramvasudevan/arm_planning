% script for creating prespecified test trajectory
% 20200108 just use a simple sine wave on the last joint for 80 seconds.
clc; clear;

traj = load('test_trajectory_fromRTD.mat');

% desiredRate = 100; % desired rate (in Hz) of trajectory

T = traj.t(end) ; % total move time;
buffT = 0; % buffer period of zeros
% t = 0:1/desiredRate:T;
t = traj.t;
Position = zeros(length(t), 7);
Velocity = zeros(length(t), 7);
for i = 1:6
   Position(:, i) = traj.x((2*i)-1, :);
   Velocity(:, i) = traj.x((2*i), :);
end

% Position = zeros(length(t), 7);
% Velocity = zeros(length(t), 7);
% Acceleration = zeros(length(t), 7);

% omega = 1/10; % for now, set angular frequency of sine wave to 1/20
% for i = 1:length(t)
%    Position(i, 4) = 0.5*sin(2*pi*omega*t(i));
%    Velocity(i, 4) = 0.5*2*pi*omega*cos(2*pi*omega*t(i));
%    Acceleration(i, 4) = 0.5*(2*pi*omega)^2*(-sin(2*pi*omega*t(i)));
%    
%    Position(i, 5) = sin(2*pi*omega*t(i));
%    Velocity(i, 5) = 2*pi*omega*cos(2*pi*omega*t(i));
%    Acceleration(i, 5) = (2*pi*omega)^2*(-sin(2*pi*omega*t(i)));
%    
%    Position(i, 6) = sin(2*pi*omega*t(i));
%    Velocity(i, 6) = 2*pi*omega*cos(2*pi*omega*t(i));
%    Acceleration(i, 6) = (2*pi*omega)^2*(-sin(2*pi*omega*t(i)));
% 
% end

% add zero velocity buffer to the end
Position = [Position; repmat(Position(end, :), 50, 1)];
Velocity = [Velocity; zeros(50, 7)];

%% make array for message
N_traj = length(Position) ;

% preallocate array for joint trajs
p_array = [] ;

tic
for idx = 1:N_traj
    % make a joint trajectory point
    p = rosmessage('trajectory_msgs/JointTrajectoryPoint') ;
    
    % fill in the position and velocity
    p.Positions = Position(idx,:) ;
    p.Velocities = Velocity(idx,:) ;
    
    % fill in the duration
    p.TimeFromStart = rosduration(t(idx)) ;
    
    % fill in p_array
    p_array = [p_array, p] ;
end
toc

%% save outpoop
% bufft = 0:1/desiredRate:buffT;
% t = [bufft, t + buffT];
% T = T + buffT;
% Position = [zeros(length(bufft), 7); Position];
% Velocity = [zeros(length(bufft), 7); Velocity];

save('arm_planning/fetch_command_files/test_trajectory_20200108.mat');