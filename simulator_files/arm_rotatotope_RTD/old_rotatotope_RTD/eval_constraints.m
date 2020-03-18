        function [c, ceq, gradc, gradceq] = eval_constraints(R, k_opt, q_0, q_dot_0)
            epsilon = 1e-3;
            ceq = [];
            gradceq = [];
            
            c = [];
            gradc = [];
            %%% Joint limit constraints:
%             [q_min, q_max, q_dot_min, q_dot_max, grad_q_min, grad_q_max, grad_q_dot_min, grad_q_dot_max] = compute_max_min_states(q_0, q_dot_0, k_opt);
%             c_joint = [];
%             c_joint = [c_joint; P.arm_joint_state_limits(1, :)' - q_min];
%             c_joint = [c_joint; -P.arm_joint_state_limits(2, :)' + q_max];
%             c_joint = [c_joint; P.arm_joint_speed_limits(1, :)' - q_dot_min];
%             c_joint = [c_joint; -P.arm_joint_speed_limits(2, :)' + q_dot_max];
% 
%             grad_c_joint = [-grad_q_min, grad_q_max, -grad_q_dot_min, grad_q_dot_max];
%            
%             c = [c; c_joint];
%             gradc = [gradc, grad_c_joint];
                        
            %%% Obstacle constraint generation:
            for i = 1:length(R.A_con) % for each obstacle
                for j = 1:length(R.A_con{i}) % for each link
                    idx = find(~cellfun('isempty', R.A_con{i}{j}));
                    k_param = k_opt(R.link_joints{j});
                    c_param = R.c_k(R.link_joints{j});
                    g_param = R.g_k(R.link_joints{j});
%                     lambda = c_param + (k_param./g_param);
                    lambda = (k_param - c_param)./g_param;
                    for k = 1:length(idx) % for each time step
                        lambdas_prod = double(R.k_con{i}{j}{idx(k)}).*lambda;
                        lambdas_prod(~R.k_con{i}{j}{idx(k)}) = 1;
                        lambdas_prod = prod(lambdas_prod, 1)';
                        
                        c_obs = R.A_con{i}{j}{idx(k)}*lambdas_prod - R.b_con{i}{j}{idx(k)};
                        c_obs_max = max(c_obs);
                        c_k = -(c_obs_max - epsilon);
                        c = [c; c_k];
                        
                        % specify gradients
                        % this is going to be really gross... but basically
                        % the gradient will depend on the row of A being
                        % multiplied by lambda, as well as which k's the
                        % lambdas depend on. 
                        if nargout > 2
                            maxidx = find(c_obs == c_obs_max);
                            k_con_temp = R.k_con{i}{j}{idx(k)}';
                            lambdas_grad = double(k_con_temp);
                            cols = 1:length(lambda);
                            for l = 1:length(lambda)
                                lambdas_grad_temp = double(k_con_temp(:, l));
                                lambdas_grad_temp = lambdas_grad_temp*lambda(l);
                                lambdas_grad_temp(~k_con_temp(:, l)) = 1;
                                lambdas_grad(:, cols ~= l) = lambdas_grad_temp.*lambdas_grad(:, cols ~= l);
                            end
                            if length(maxidx) > 1
%                                 disp('ahhh');
                                tempgradc = R.A_con{i}{j}{idx(k)}(maxidx, :)*lambdas_grad;
                                gradc = [gradc, [(-max(tempgradc)')./g_param; zeros(length(k_opt) - length(lambda), 1)]];
                            else
                                gradc = [gradc, [(-(R.A_con{i}{j}{idx(k)}(maxidx, :)*lambdas_grad)')./g_param; zeros(length(k_opt) - length(lambda), 1)]];
                            end
                        end
                    end
                end
                
%                 error_if_out_of_time(P.trajopt_start_tic,P.t_plan)
            end
            
            %%% Self-intersection constraint generation:
            for i = 1:length(R.A_con_self) % for each pair of joints that can intersect
                idx = find(~cellfun('isempty', R.A_con_self{i}));
                for j = 1:length(idx) % for each (nonempty) time step
                    k_param = k_opt;
                    c_param = R.c_k;
                    g_param = R.g_k;
                    
                    lambda = (k_param - c_param)./g_param;
                    
                    % dumb way to do this... want to multiply rows of lambdas
                    % together, replacing zeros with ones
                    lambdas_prod = double(R.k_con_self{i}{idx(j)}).*lambda;
                    lambdas_prod(~R.k_con_self{i}{idx(j)}) = 1;
                    lambdas_prod = prod(lambdas_prod, 1)';
                    
                    c_obs = R.A_con_self{i}{idx(j)}*lambdas_prod - R.b_con_self{i}{idx(j)};
                    c_obs_max = max(c_obs);
                    c_k = -(c_obs_max - epsilon);
                    c = [c; c_k];
                    
                    % specify gradients
                    % this is going to be really gross... but basically
                    % the gradient will depend on the row of A being
                    % multiplied by lambda, as well as which k's the
                    % lambdas depend on.
                    if nargout > 2
                        maxidx = find(c_obs == c_obs_max);
                        k_con_temp = R.k_con_self{i}{idx(j)}';
                        lambdas_grad = double(k_con_temp);
                        cols = 1:length(lambda);
                        for l = 1:length(lambda)
                            lambdas_grad_temp = double(k_con_temp(:, l));
                            lambdas_grad_temp = lambdas_grad_temp*lambda(l);
                            lambdas_grad_temp(~k_con_temp(:, l)) = 1;
                            lambdas_grad(:, cols ~= l) = lambdas_grad_temp.*lambdas_grad(:, cols ~= l);
                        end
                        if length(maxidx) > 1
                            tempgradc = R.A_con_self{i}{idx(j)}(maxidx, :)*lambdas_grad;
                            gradc = [gradc, (-max(tempgradc)')./g_param];
                        else
                            gradc = [gradc, (-(R.A_con_self{i}{idx(j)}(maxidx, :)*lambdas_grad)')./g_param];
                        end
                    end
                end
            end
                    
            
        end
        
        