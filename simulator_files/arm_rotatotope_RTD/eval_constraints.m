function [c, ceq, gradc, gradceq, moremax] = eval_constraints(R, k_opt)
            %epsilon = 1e-3;
            epsilon = 0;
            ceq = [];
            gradceq = [];
            
            c = [];
            gradc = [];
            moremax = [];
            %%% Obstacle constraint generation:
            for i = 1:length(R.A_con) % for each obstacle
                for j = 1:length(R.A_con{i}) % for each link
                    idx = find(~cellfun('isempty', R.A_con{i}{j}));
                    k_param = k_opt(R.link_joints{j});
                    c_param = R.c_k(R.link_joints{j});
                    g_param = R.g_k(R.link_joints{j});
                    lambda = c_param + (k_param./g_param);
                    for k = 1:length(idx) % for each time step
                        lambdas = R.k_con{i}{j}{idx(k)}.*lambda;
                        lambdas(~R.k_con{i}{j}{idx(k)}) = 1;
                        lambdas = prod(lambdas, 1)';
                        
                        c_obs = R.A_con{i}{j}{idx(k)}*lambdas - R.b_con{i}{j}{idx(k)};
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
                            lambdas_grad = k_con_temp;
                            cols = 1:length(lambda);
                            for l = 1:length(lambda)
                                lambdas_grad_temp = k_con_temp(:, l);
                                lambdas_grad_temp = lambdas_grad_temp*lambda(l);
                                lambdas_grad_temp(~k_con_temp(:, l)) = 1;
                                lambdas_grad(:, cols ~= l) = lambdas_grad_temp.*lambdas_grad(:, cols ~= l);
                            end
                            if length(maxidx) > 1
                                moremax = [moremax,((i-1) * length(R.A_con{i}) + j-1) * length(idx) + k];
                                tempgradc = R.A_con{i}{j}{idx(k)}(maxidx, :)*lambdas_grad;
                                gradc = [gradc, [(-max(tempgradc)')./g_param; zeros(length(k_opt) - length(lambda), 1)]];
                            else
                                gradc = [gradc, [(-(R.A_con{i}{j}{idx(k)}(maxidx, :)*lambdas_grad)')./g_param; zeros(length(k_opt) - length(lambda), 1)]];
                            end
                        end
                        
                    end
                end
            end
            
        end