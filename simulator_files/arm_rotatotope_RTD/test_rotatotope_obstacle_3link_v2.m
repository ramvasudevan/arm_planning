close all; clc;

teston = 1;
% code for testing the constraint generation for a 3 link arm
figure(1); clf; hold on; axis equal;

% set FRS_options
FRS_options = struct();
FRS_options.t_plan = 0.01;
FRS_options.T = 1;
FRS_options.L = 0.33;
FRS_options.buffer_dist = 0.1460;
FRS_options.combs = generate_combinations_upto(200);
FRS_options.maxcombs = 200;
FRS_options.origin_shift = [-0.03265; 0; 0.72601];


% get current obstacles
% obs_center = [-0.03265 ;0; 0.72601];
% obs_width = [0.06];
% O{1} = box_obstacle_zonotope('center', obs_center(:), 'side_lengths', [obs_width, obs_width, obs_width]);
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
obs_patch = plotFilled(O{1}.zono, [1, 3], 'r');
obs_patch.FaceAlpha = 0.2;
% plot(O{1});
% O = {};

% get current state of robot
% q_0 = 0*ones(6, 1) ;
q_0 = [0; 0; 0; pi/2 + 5*pi/48; 0; pi/2 + 5*pi/48];
% q_0 = [pi/6; pi/4; pi/6; -pi/4; pi/6; pi/4];
q_dot_0 = 0*ones(6, 1) ;
q_des = [0.6441;
        0.6902;
        0.5426;
       -1.4591;
        0.4469;
       -0.9425];
% good_k = -pi/6*ones(6, 1) ;
% bad_k = pi/6*ones(6, 1) - 0.001 ;
% bad_k = [pi/6 - 0.001; pi/6 - 0.001; pi/12; pi/24; -pi/36; pi/48];
bad_k = [pi/6 - 0.001; -pi/6 + 0.001; pi/6 - 0.001; pi/6 - 0.001; -pi/6 + 0.001; pi/6 - 0.001];
% bad_k = [pi/6 - 0.001; -pi/6 + 0.001 ; pi/6 - 0.001 ; -pi/6 + 0.001; pi/6 - 0.001; pi/6 - 0.001];
% bad_k = -pi/6*ones(6, 1) + 0.001;
% good_k = [0; 0; 0; -pi/6 + 0.001; 0; -pi/6 + 0.001];
% bad_k = [0; 0; 0; pi/6 - 0.001; 0; pi/6 - 0.001];


% generate FRS
R = robot_arm_FRS_rotatotope_fetch(q_0, q_dot_0, FRS_options);
% R.plot(99, {'b', 'b', 'b'});
% R.plot_slice(good_k, 10, {'g', 'g', 'g'});
% R.plot_slice(bad_k, 10, {'r', 'r', 'r'});
% R.plot_slice_gensIncluded(bad_k, 10, {'r', 'r', 'r'});

% map obstacles to trajectory parameter space
% R = R.generate_constraints(O);
R = R.generate_polytope_normals(O);

% cuda FRS
R_cuda = robot_arm_FRS_rotatotope_fetch_cuda(q_0, q_dot_0, q_des, O, bad_k, FRS_options);
[eval_out,eval_grad_out] = R.evaluate_sliced_constraints(bad_k, O);

mex_eval_out = R_cuda.eval_output;
mex_eval_grad_out = R_cuda.eval_grad_output;
mex_res = R_cuda.mex_res
return;
% figure(1);
% plot(eval_out,'r.');hold on;plot(mex_eval_out(1:(end-100)),'b.');
% legend('patrick','bohao');
% figure(2);
% plot(eval_out-mex_eval_out(1:(end-100)));

% link_id = 3;
% time_id = 79;
% rot = R.link_FRS;
% data = [rot{link_id}{time_id}.RZ;
%     [0,rot{link_id}{time_id}.c_idx];
%     [zeros(link_id*2,1),rot{link_id}{time_id}.k_idx];
%     [zeros(link_id*2,1),rot{link_id}{time_id}.C_idx];];
% [d,id] = sort(vnorm(data(1:3,:)),'descend');
% disp(data(:,id));
% 
% mex_data = [R_cuda.RZ{time_id};
%     double(R_cuda.c_idx{time_id});
%     double(R_cuda.k_idx{time_id})-1];
% [d,id] = sort(vnorm(mex_data(1:3,:)),'descend');
% disp(mex_data(:,id));

% generate self intersection constraints
% R = R.generate_self_intersection_constraints();

% grid the k_4, k_6 constraint space
if teston
    idx1 = 1;
    idx2 = 3;
%     for idx2 = 1:100
            idx3 = 100;
        
%         A_con = R.A_con{idx1}{idx2}{idx3};
%         b_con = R.b_con{idx1}{idx2}{idx3};
%         k_con = R.k_con{idx1}{idx2}{idx3};
%         
        c_k = R.c_k;
        g_k = R.g_k;
        
        figure(2); clf; hold on;
        lims = [-g_k(4) -g_k(4) g_k(4) g_k(4) -g_k(4); -g_k(6) g_k(6) g_k(6) -g_k(6) -g_k(6)];
        plot(lims(1, :)', lims(2, :)', 'k--', 'LineWidth', 4);
        
        myk = linspace(-g_k(4), g_k(4), 20);
        [Xk, Yk] = meshgrid(myk, myk);
        Zk = zeros(length(myk),length(myk));
        Zk_cuda = zeros(length(myk),length(myk));
        for i = 1:length(myk)
            for j = 1:length(myk)
                K = [0;0;0;Xk(i, j); 0;Yk(i, j)];
%                 lambda = (K - c_k)./g_k;
%                 lambdas_prod = k_con.*lambda;
                
                % dumb way to do this... want to multiply rows of lambdas
                % together, replacing zeros with ones
%                 lambdas_prod = lambdas_orig;
%                 lambdas_prod(~k_con) = 1;
%                 lambdas_prod = prod(lambdas_prod, 1)';
%                 lambdas_prod(~any(lambdas_orig)) = 0; % set lambdas corresponding to all zero columns equal to zero
                
%                 Zk = A_con*lambdas_prod - b_con;
                h = R.evaluate_sliced_constraints(K, O);
                Zk(i,j) = max(h);
%                 Zk = -Zk;
%                 disp(Zk);
                %             Zk = -Zk; % hmm i'm off by this extra negative sign somewhere
                if Zk(i,j) >= 0
                    plot(Xk(i, j), Yk(i, j), 'bx', 'MarkerSize', 10, 'LineWidth', 6);
%                     pause;
                end
                
                
                R_cuda = robot_arm_FRS_rotatotope_fetch_cuda(q_0, q_dot_0, q_des, O, K, FRS_options);
                mex_eval_out = R_cuda.eval_output(1:(end-100));
                Zk_cuda(i,j) = max(mex_eval_out);
                
                if Zk_cuda(i,j) >= 0
                    plot(Xk(i, j), Yk(i, j), 'r.', 'MarkerSize', 10, 'LineWidth', 6);
%                     pause;
                end
                
            end
%         end
    end
    
    xlabel('k_4', 'FontSize', 20);
    ylabel('k_6', 'FontSize', 20);
    plot(good_k(4), good_k(6), 'g.', 'MarkerSize', 20);
    plot(bad_k(4), bad_k(6), 'r.', 'MarkerSize', 20);
    
    disp('Click a point!');
    [p1, p2] = ginput(1);
    plot(p1, p2, 'kx', 'MarkerSize', 16, 'LineWidth', 6);
    
    figure(1);
    slice_pt = [0;0;0;p1;0;p2];
%     patch1 = plotFilled(R.link_FRS{1}{100}.slice(slice_pt(1:2)), [1, 3], 'k');
%     patch1.FaceAlpha = 0.2;
    myzono = zonotope(R.link_FRS{3}{100}.slice(slice_pt));
    patch2 = plotFilled(myzono, [1, 3], 'k');
    patch2.FaceAlpha = 0.2;
    
    figure(3); clf; view(3); axis equal; hold on;
    R.plot_slice([0;0;0;p1;0;p2], 99, {'k', 'k', 'k'});
    plot(O{1});
    
end

    
    


% test one particular set of constraints
% A_con = R.A_con{1}{3}{97};
% b_con = R.b_con{1}{3}{97};
% k_con = R.k_con{1}{3}{97};
% c_param = R.c_k(R.link_joints{3});
% g_param = R.g_k(R.link_joints{3});
% 
% % for the bad_k
% k_param = bad_k(R.link_joints{3});
% lambda = c_param + (k_param./g_param);
% lambdas = k_con.*lambda;
% lambdas(~k_con) = 1;
% lambdas = prod(lambdas, 1)';
% c_obs = A_con*lambdas - b_con;
% c_obs_max = max(c_obs);
% bad_c_k = -(c_obs_max);
% 
% % for the good_k
% k_param = good_k(R.link_joints{3});
% lambda = c_param + (k_param./g_param);
% lambdas = k_con.*lambda;
% lambdas(~k_con) = 1;
% lambdas = prod(lambdas, 1)';
% c_obs = A_con*lambdas - b_con;
% c_obs_max = max(c_obs);
% good_c_k = -(c_obs_max);
% 
% % display
% disp('Good c_k value (should be -0.1518): ');
% disp(good_c_k);
% 
% disp('Bad c_k value (should be 0.0867): ');
% disp(bad_c_k);
% 
% %format plot
% xlim([-1, 1]);
% ylim([-1, 1]);
% zlim([-1, 1]);
% 
% box on;
% xlabel('X');