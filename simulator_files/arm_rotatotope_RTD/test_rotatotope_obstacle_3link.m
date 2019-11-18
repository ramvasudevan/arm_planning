clear all; clc;

% code for testing the constraint generation for a 3 link arm
% figure(1); clf; hold on; view(3); axis equal;

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
obs_center = [0.6; 0.4; 0.7];
obs_width = [0.15];
O{2} = box_obstacle_zonotope('center', obs_center(:), 'side_lengths', [obs_width, obs_width, obs_width]);
obs_center = [0.5; 0.3; -0.5];
obs_width = [0.2];
O{3} = box_obstacle_zonotope('center', obs_center(:), 'side_lengths', [obs_width, obs_width, obs_width]);
obs_center = [0.1; 0.2; -0.1];
obs_width = [0.05];
O{4} = box_obstacle_zonotope('center', obs_center(:), 'side_lengths', [obs_width, obs_width, obs_width]);
obs_center = [-0.5; -0.3; 0.4];
obs_width = [0.1];
O{5} = box_obstacle_zonotope('center', obs_center(:), 'side_lengths', [obs_width, obs_width, obs_width]);

% get current state of robot
q_0 = 0*ones(6, 1) ;
q_dot_0 = 0.1*ones(6, 1) ;
good_k = -pi/6*ones(6, 1) ;
bad_k = [pi/6 - 0.001; pi/6 - 0.001; pi/12; pi/24; -pi/36; pi/48];

% generate FRS
R = robot_arm_FRS_rotatotope_fetch(q_0, q_dot_0, O, FRS_options);

% R.plot(10, {'b', 'b', 'b'});
% R.plot_slice(good_k, 10, {'g', 'g', 'g'});
% R.plot_slice(bad_k, 10, {'r', 'r', 'r'});

% map obstacles to trajectory parameter space
R = R.generate_constraints(O);

% test one particular set of constraints
% default 1, 3, 97

savefig_path = ''; % specify the path where you want to save your figures
for obstacle_id = 1:5
    figure(1); clf;
    good_diff = [];
    bad_diff = [];
    for link_id = 1:1:3
        good_diff{link_id} = [];
        bad_diff{link_id} = [];
        for time_step = 1:100
            A_con = R.A_con{obstacle_id}{link_id}{time_step};
            b_con = R.b_con{obstacle_id}{link_id}{time_step};
            k_con = R.k_con{obstacle_id}{link_id}{time_step};
            mex_A_con = R.mex_A_con{obstacle_id}{link_id}{time_step};
            mex_b_con = R.mex_b_con{obstacle_id}{link_id}{time_step};
            mex_k_con = R.mex_k_con{obstacle_id}{link_id}{time_step};
            c_param = R.c_k(R.link_joints{link_id});
            g_param = R.g_k(R.link_joints{link_id});

            % for the bad_k
            k_param = bad_k(R.link_joints{link_id});
            lambda = c_param + (k_param./g_param);
            lambdas = k_con.*lambda;
            lambdas(~k_con) = 1;
            lambdas = prod(lambdas, 1)';
            c_obs = A_con*lambdas - b_con;
            [c_obs_max,bad_idx] = max(c_obs);
            bad_c_k = (c_obs_max);

            % for the mex_bad_k
            mex_lambdas = mex_k_con.*lambda;
            mex_lambdas(~mex_k_con) = 1;
            mex_lambdas = prod(mex_lambdas, 1)';
            mex_c_obs = mex_A_con*mex_lambdas - mex_b_con;
            [mex_c_obs_max,bad_mex_idx] = max(mex_c_obs);
            mex_bad_c_k = (mex_c_obs_max);

            % for the good_k
            k_param = good_k(R.link_joints{link_id});
            lambda = c_param + (k_param./g_param);
            lambdas = k_con.*lambda;
            lambdas(~k_con) = 1;
            lambdas = prod(lambdas, 1)';
            c_obs = A_con*lambdas - b_con;
            [c_obs_max,good_idx] = max(c_obs);
            good_c_k = (c_obs_max);

            % for the mex_good_k
            mex_lambdas = mex_k_con.*lambda;
            mex_lambdas(~mex_k_con) = 1;
            mex_lambdas = prod(mex_lambdas, 1)';
            mex_c_obs = mex_A_con*mex_lambdas - mex_b_con;
            [mex_c_obs_max,good_mex_idx] = max(mex_c_obs);
            mex_good_c_k = (mex_c_obs_max);

            % display
            %disp('Good c_k value (should be -0.1518): ');

            good_diff{link_id} = [good_diff{link_id}, mex_good_c_k - good_c_k];

            %disp('Bad c_k value (should be 0.0867): ');
            bad_diff{link_id} = [bad_diff{link_id}, mex_bad_c_k - bad_c_k];
        end
        subplot(1,3,link_id);
        plot(1:100,good_diff{link_id},'r.',1:100,bad_diff{link_id},'b.');
        xlabel('time steps');
        ylabel('difference');
        title(['link ', int2str(link_id)]);

    %     disp(mean(good_diff{link_id}))
    %     disp(max(good_diff{link_id}))
    %     disp(mean(bad_diff{link_id}))
    %     disp(max(bad_diff{link_id}))
    end
    suptitle(['difference for obstacle ',int2str(obstacle_id)]);
    saveas(gcf,[savefig_path,int2str(obstacle_id),'.png']);
end


%format plot
% xlim([-1, 1]);
% ylim([-1, 1]);
% zlim([-1, 1]);

% box on;
% xlabel('X');
