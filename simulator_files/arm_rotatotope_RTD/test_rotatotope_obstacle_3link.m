clear all; clc;

% code for testing the constraint generation for a 3 link arm
figure(1); clf; hold on; view(3); axis equal;

% set FRS_options
FRS_options = struct();
FRS_options.t_plan = 0.01;
FRS_options.T = 1;
FRS_options.L = 0.33;
FRS_options.buffer_dist = 0;
FRS_options.combs = generate_combinations_upto(200);
FRS_options.maxcombs = 200;

% get current obstacles
obs_center = [0.8; 0.2; -0.2];
obs_width = [0.1];
O{1} = box_obstacle_zonotope('center', obs_center(:), 'side_lengths', [obs_width, obs_width, obs_width]);
plot(O{1});

% get current state of robot
q_0 = 0*ones(6, 1) ;
q_dot_0 = 0.1*ones(6, 1) ;
good_k = -pi/6*ones(6, 1) ;
bad_k = [pi/6 - 0.001; pi/6 - 0.001; pi/12; pi/24; -pi/36; pi/48];

% generate FRS
R = robot_arm_FRS_rotatotope_fetch(q_0, q_dot_0, FRS_options);
R.plot(10, {'b', 'b', 'b'});
R.plot_slice(good_k, 10, {'g', 'g', 'g'});
R.plot_slice(bad_k, 10, {'r', 'r', 'r'});

% map obstacles to trajectory parameter space
R = R.generate_constraints(O);

% test one particular set of constraints
A_con = R.A_con{1}{3}{97};
b_con = R.b_con{1}{3}{97};
k_con = R.k_con{1}{3}{97};
c_param = R.c_k(R.link_joints{3});
g_param = R.g_k(R.link_joints{3});

% for the bad_k
k_param = bad_k(R.link_joints{3});
lambda = c_param + (k_param./g_param);
lambdas = k_con.*lambda;
lambdas(~k_con) = 1;
lambdas = prod(lambdas, 1)';
c_obs = A_con*lambdas - b_con;
c_obs_max = max(c_obs);
bad_c_k = -(c_obs_max);

% for the good_k
k_param = good_k(R.link_joints{3});
lambda = c_param + (k_param./g_param);
lambdas = k_con.*lambda;
lambdas(~k_con) = 1;
lambdas = prod(lambdas, 1)';
c_obs = A_con*lambdas - b_con;
c_obs_max = max(c_obs);
good_c_k = -(c_obs_max);

% display
disp('Good c_k value (should be -0.1518): ');
disp(good_c_k);

disp('Bad c_k value (should be 0.0867): ');
disp(bad_c_k);

%format plot
xlim([-1, 1]);
ylim([-1, 1]);
zlim([-1, 1]);

box on;
xlabel('X');