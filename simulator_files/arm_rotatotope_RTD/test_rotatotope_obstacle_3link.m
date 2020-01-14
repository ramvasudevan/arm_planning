clc;

% code for testing the constraint generation for a 3 link arm
% figure(1); clf; hold on; view(3); axis equal;

FRS_options = struct();
% FRS_options.position_dimensions = [1;2;3];
% FRS_options.extra_position_dimensions = [4;5];
% FRS_options.IC_dimensions = [6;7];
% FRS_options.param_dimensions = [8;9];
% FRS_options.nLinks = nLinks;
% FRS_options.time_discretization = 0.01;

FRS_options.t_plan = 0.5;
FRS_options.origin_shift = [-0.0326;0;0.7260];
FRS_options.T = 1;
FRS_options.L = 0.33;
FRS_options.buffer_dist = 0.1460;
FRS_options.combs = generate_combinations_upto(200);
FRS_options.maxcombs = 200;

% get current state of robot
q = [0.7885;
    0.7956;
    0.3887;
   -1.4218;
    0.6108;
   -0.8643] ;
q_dot = zeros(6, 1) ;
q_des = [0.6441;
        0.6902;
        0.5426;
       -1.4591;
        0.4469;
       -0.9425];

% generate FRS
R = robot_arm_FRS_rotatotope_fetch(q, q_dot, FRS_options);
                    
% get current obstacles
obs_center = [0.8; 0.2; -0.2];
obs_width = [0.1];
O{1} = box_obstacle_zonotope('center', obs_center(:), 'side_lengths', [obs_width, obs_width, obs_width]);
obs_center = [0.6; 0.4; -0.7];
obs_width = [0.15];
O{2} = box_obstacle_zonotope('center', obs_center(:), 'side_lengths', [obs_width, obs_width, obs_width]);
obs_center = [0.6; -0.4; 0.7];
obs_width = [0.1];
O{3} = box_obstacle_zonotope('center', obs_center(:), 'side_lengths', [obs_width, obs_width, obs_width]);
obs_center = [-0.8; 0.5; 0.7];
obs_width = [0.1];
O{4} = box_obstacle_zonotope('center', obs_center(:), 'side_lengths', [obs_width, obs_width, obs_width]);
obs_center = [0.6; -0.4; -0.7];
obs_width = [0.1];
O{5} = box_obstacle_zonotope('center', obs_center(:), 'side_lengths', [obs_width, obs_width, obs_width]);

R = R.generate_constraints(O);

good_k = -pi/6*ones(6, 1) ;
bad_k = [pi/6 - 0.001; pi/6 - 0.001; pi/12; pi/24; -pi/36; pi/48];

% for i = 1:length(O)
%     plot(O{i});
% end
% R.plot(10);

R_cuda = robot_arm_FRS_rotatotope_fetch_cuda(q, q_dot, q_des, O, bad_k, FRS_options);
eval_out = R_cuda.eval_output;
eval_grad_out = R_cuda.eval_grad_output;
eval_hess_out = R_cuda.eval_hess_output;
mex_res = R_cuda.mex_res;
return;
%% analysis
% link_id = 1;
% time_id = 66;
% rot = R.link_FRS;
% data = [rot{link_id}{time_id}.RZ;
%     [0,any(rot{link_id}{time_id}.k_idx)&rot{link_id}{time_id}.c_idx]];
% [d,id] = sort(vnorm(data(1:3,:)),'descend');
% disp(data(:,id));
% 
% mex_data = [R_cuda.RZ{time_id};
%     any(R_cuda.k_idx{time_id})&R_cuda.c_idx{time_id}];
% [d,id] = sort(vnorm(mex_data(1:3,:)),'descend');
% disp(mex_data(:,id));

[c, ceq, gradc, gradceq] = eval_constraints(R, bad_k, q, q_dot);
plot(c,'r.');hold on;plot(eval_out,'b.')
return;

% test one particular set of constraints
% default 1, 3, 97

for obstacle_id = 1:length(O)
    figure(obstacle_id); clf;
    good_diff = [];
    bad_diff = [];
    for link_id = 1:1:3
        good_diff{link_id} = [];
        bad_diff{link_id} = [];
        for time_step = 1:100
            con_id = ((obstacle_id-1) * R.n_links + link_id-1) * R.n_time_steps + time_step;
%             A_con = R.A_con{obstacle_id}{link_id}{time_step};
%             b_con = R.b_con{obstacle_id}{link_id}{time_step};
%             k_con = R.k_con{obstacle_id}{link_id}{time_step};
%             mex_A_con = R_cuda.A_con{obstacle_id}{link_id}{time_step};
%             mex_d_con = R_cuda.d_con{obstacle_id}{link_id}{time_step};
%             mex_delta_con = R_cuda.delta_con{obstacle_id}{link_id}{time_step};
%             mex_k_con = R_cuda.k_con{obstacle_id}{link_id}{time_step};
%             c_param = R.c_k(R.link_joints{link_id});
%             g_param = R.g_k(R.link_joints{link_id});

            % for the bad_k
%             k_param = bad_k(R.link_joints{link_id});
%             lambda = c_param + (k_param./g_param);
%             lambdas = double(k_con).*lambda;
%             lambdas(~k_con) = 1;
%             lambdas = prod(lambdas, 1)';
%             c_obs =  A_con*lambdas - b_con;
%             [c_obs_max,bad_idx] = max(c_obs);
%             bad_c_k = -(c_obs_max);
            bad_c_k = c(con_id);

            % for the mex_bad_k
%             k_param = bad_k(R.link_joints{link_id});
%             lambda = c_param + (k_param./g_param);
%             mex_lambdas = mex_k_con.*lambda;
%             mex_lambdas(~mex_k_con) = 1;
%             mex_lambdas = prod(mex_lambdas, 1)';
%             mex_c_obs = mex_A_con*mex_lambdas - mex_d_con - mex_delta_con;
%             mex_c_obs = [mex_c_obs; -mex_A_con*mex_lambdas + mex_d_con - mex_delta_con];
%             [mex_c_obs_max,bad_mex_idx] = max(mex_c_obs);
%             mex_bad_c_k = -(mex_c_obs_max);
            
            mex_bad_c_k = eval_out(con_id);

            % for the good_k
%             k_param = good_k(R.link_joints{link_id});
%             lambda = c_param + (k_param./g_param);
%             lambdas = k_con.*lambda;
%             lambdas(~k_con) = 1;
%             lambdas = prod(lambdas, 1)';
%             c_obs = A_con*lambdas - b_con;
%             [c_obs_max,good_idx] = max(c_obs);
%             good_c_k = -(c_obs_max);

            % for the mex_good_k
%             mex_lambdas = mex_k_con.*lambda;
%             mex_lambdas(~mex_k_con) = 1;
%             mex_lambdas = prod(mex_lambdas, 1)';
%             mex_c_obs = mex_A_con*mex_lambdas - mex_b_con;
%             [mex_c_obs_max,good_mex_idx] = max(mex_c_obs);
%             mex_good_c_k = -(mex_c_obs_max);

            % display
            %disp('Good c_k value (should be -0.1518): ');

%             good_diff{link_id} = [good_diff{link_id}, mex_good_c_k - good_c_k];

            %disp('Bad c_k value (should be 0.0867): ');
            bad_diff{link_id} = [bad_diff{link_id}, mex_bad_c_k - bad_c_k];
        end
        max(abs(bad_diff{link_id}))
        subplot(1,3,link_id);
        plot(1:1:100,bad_diff{link_id},'b.');
        
        xlabel('time steps');
        ylabel('difference');
        title(['link ', int2str(link_id)]);

    %     disp(mean(good_diff{link_id}))
    %     disp(max(good_diff{link_id}))
    %     disp(mean(bad_diff{link_id}))
    %     disp(max(bad_diff{link_id}))
    end
    suptitle(['difference for obstacle ',int2str(obstacle_id)]);
end


%format plot
% xlim([-1, 1]);
% ylim([-1, 1]);
% zlim([-1, 1]);

% box on;
% xlabel('X');