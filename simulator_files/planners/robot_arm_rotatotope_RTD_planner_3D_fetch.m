classdef robot_arm_rotatotope_RTD_planner_3D_fetch < robot_arm_generic_planner
    properties
        t_total = 1 ; % total duration of desired traj
        time_discretization = 0.01 ; % time discretization of desired traj
        FRS_options struct;
        
        % for current FRS
        R ;
        
        % from previous time step
        Z_prev = [];
        R_prev ;
        q_0_prev = [];
        q_dot_0_prev = [];
        k_opt_prev = [];
        
        iter = 0;
        first_iter_pause = 1;
       
    end
    methods
        function P = robot_arm_rotatotope_RTD_planner_3D_fetch(varargin)
            t_move = 0.5;
            lookahead_distance = 0.3;
%             HLP = robot_arm_RRT_HLP('make_new_tree_every_iteration_flag',true) ;
%             HLP = robot_arm_PRM_HLP( ) ;
            HLP = robot_arm_straight_line_HLP( );
            P@robot_arm_generic_planner('lookahead_distance', lookahead_distance, 't_move', t_move, 'HLP', HLP, ...
                varargin{2:end}) ;
            P.FRS_options = varargin{1};
%             P.FRS_options.combs = generate_combinations_upto(200);
%             P.FRS_options.maxcombs = 200;
        end
        
        function [T,U,Z] = replan(P,agent_info,world_info)
            %%% 1. generate cost function
            % generate a waypoint in configuration space
            if P.first_iter_pause && P.iter == 0
               pause; 
            end
            P.iter = P.iter + 1;
            planning_time = tic;

            q_des = P.HLP.get_waypoint(agent_info,world_info,P.lookahead_distance) ;
                        
            %%% 2. generate constraints
            % get current obstacles
            O = world_info.obstacles ;
            
            % get current state of robot
            q_0 = agent_info.state(P.arm_joint_state_indices, end) ;
            q_dot_0 = agent_info.state(P.arm_joint_speed_indices, end) ;
            
            % generate FRS
            P.R = robot_arm_FRS_rotatotope_fetch(q_0, q_dot_0, P.FRS_options);
            
            % map obstacles to trajectory parameter space
            P.R = P.R.generate_constraints(O);
%             [~, k_lim, k_unsafe_A, k_unsafe_b] = compute_unsafe_parameters_3D(P.R, P.phi_dot_0, O, P.FRS_options);
            
            %%% 3. solve for optimal trajectory parameter
            [k_opt, trajopt_failed] = P.trajopt(q_0, q_dot_0, q_des);
            
            %%% 4. generate desired trajectory
            if ~trajopt_failed
                P.vdisp('New trajectory found!',3);
                
                [T, U, Z] = P.generate_trajectory(q_0, q_dot_0, k_opt);
                
                toc(planning_time);
             
                P.R_prev = P.R;
                P.Z_prev = Z;
                
                P.q_0_prev = q_0;
                P.q_dot_0_prev = q_dot_0;
                P.k_opt_prev = k_opt;
            else % if no safe trajectory parameter found:
                % generate a braking trajectory
                P.vdisp('Unable to find new trajectory!',3)
                T = 0:P.time_discretization:P.t_total ;
                T_plan = T(1:floor(length(T)/2));
                %                 T_brake = T(floor(length(T)/2)+1:end);
                N_T = length(T) ;
                U = zeros(P.arm_n_inputs,N_T) ;
                if ~isnan(P.q_0_prev) % if prev trajectory was not a braking traj
                    %                     Z = P.generate_trajectory(T, P.q_0_prev, P.q_dot_0_prev, P.k_opt_prev);
                    %                     Z_brake = Z(:, floor(length(T)/2)+1:end);
                    Z_brake = P.Z_prev(:, floor(length(T)/2)+1:end);
                    z = [Z_brake(1:2:end, end)'; zeros(length(q_0), 1)'];
                    Z = [Z_brake, ones(size(Z_brake, 1), length(T_plan)).*z(:)];
                    P.q_0_prev = nan;
                    P.q_dot_0_prev = nan;
                    P.k_opt_prev = nan;
                else
                    % command to stay in place
                    z = [q_0'; zeros(length(q_0), 1)'];
                    Z = ones(length(q_0) + length(q_dot_0), length(T)).*z(:);
                    P.q_0_prev = nan;
                    P.q_dot_0_prev = nan;
                    P.k_opt_prev = nan;
                end
                toc(planning_time);
            end
            %             toc;
        end
        
        function [k_opt, trajopt_failed] = trajopt(P, q_0, q_dot_0, q_des)
            % use fmincon to optimize the cost subject to constraints
            P.vdisp('Running trajopt',3)
            
            cost_func = @(k) P.eval_cost(k, q_0, q_dot_0, q_des);
            constraint_func = @(k) P.eval_constraint(k);
            
            % generate upper and lower bounds
%             lb = [];
%             ub = [];
%             for i = 1:size(k_lim{end}.V, 1)
%                 lb = [lb; min(k_lim{end}.V(i, :))];
%                 ub = [ub; max(k_lim{end}.V(i, :))];
%             end
            lb = P.R.c_k - P.R.g_k;
            ub = P.R.c_k + P.R.g_k;
            
            initial_guess = (lb + ub)/2;
           
%             options = optimoptions('fmincon','SpecifyConstraintGradient',true, 'Algorithm', 'interior-point');
%             options = optimoptions('fmincon','SpecifyConstraintGradient',true);
            options = optimoptions('fmincon');
            [k_opt, ~, exitflag, ~] = fmincon(cost_func, initial_guess, [], [], [], [], lb, ub, constraint_func, options) ;
            
            if exitflag <= 0
%                 error('planner:trajOpt', 'Solver did not converge.');
                trajopt_failed = true;
            else
                trajopt_failed = false;
            end
        end
        
        function [cost] = eval_cost(P, k, q_0, q_dot_0, q_des)
            % generate a simple cost fucntion
           q_plan = compute_q_plan(P, q_0, q_dot_0, k);
           cost = sum((q_plan - q_des).^2);
        end
        
        function [c, ceq] = eval_constraint(P, k_opt)
            epsilon = 1e-3;
            ceq = [];
%             gradceq = [];
            
            c = [];
%             gradc = [];
            %%% Obstacle constraint generation:
            for i = 1:length(P.R.A_con) % for each obstacle
                for j = 1:length(P.R.A_con{i}) % for each link
                    idx = find(~cellfun('isempty', P.R.A_con{i}{j}));
                    k_param = k_opt(P.R.link_joints{j});
                    c_param = P.R.c_k(P.R.link_joints{j});
                    g_param = P.R.g_k(P.R.link_joints{j});
                    lambda = c_param + (k_param./g_param);
                    for k = 1:length(idx) % for each time step
                        lambdas = P.R.k_con{i}{j}{idx(k)}.*lambda;
                        lambdas(~lambdas) = 1;
                        lambdas = prod(lambdas, 1)';
                        
                        c_obs = P.R.A_con{i}{j}{idx(k)}*lambdas - P.R.b_con{i}{j}{idx(k)};
                        c_obs_max = max(c_obs);
                        c_k = -(c_obs_max - epsilon);
                        c = [c; c_k];
                        
                        % specify gradients
                        % PATRICK need to fill this out.
%                         if nargout > 2
%                             maxidx = find(c_obs == c_obs_max);
%                             if length(maxidx) > 1
% %                                 disp('ahhh');
%                                 tempgradc = k_unsafe_A{i}{j}{idx(k)}(maxidx, :);
%                                 gradc = [gradc, -max(tempgradc)'];
%                             else
%                                 gradc = [gradc, -k_unsafe_A{i}{j}{idx(k)}(maxidx, :)'];
%                             end
%                         end
                        
                    end
                end
            end
            
        end
        
        function [T, U, Z] = generate_trajectory(P, q_0, q_dot_0, k_opt)
            T = 0:P.time_discretization:P.t_total ;
            T_plan = T(1:floor(length(T)/2)+1) ;
            T_brake = T(floor(length(T)/2)+1:end) ;
            
            t_to_stop = T_brake(end) - T_brake(1);
            U = zeros(P.arm_n_inputs,length(T));
            
            q_to_peak = q_0 + q_dot_0.*T_plan + (1/2)*k_opt.*T_plan.^2;
            q_dot_to_peak = q_dot_0 + k_opt.*T_plan;
            
            q_peak = q_to_peak(:, end);
            q_dot_peak = q_dot_to_peak(:, end);
            
            T_brake = T_brake - T_brake(1);
            q_to_stop = q_peak + q_dot_peak.*T_brake + (1/2)*((0 - q_dot_peak)./t_to_stop).*T_brake.^2;
            q_dot_to_stop = q_dot_peak + ((0 - q_dot_peak)./t_to_stop).*T_brake;
            
            % remove overlap
            q_to_stop(:, 1) = [];
            q_dot_to_stop(:, 1) = [];
            
            Z = zeros(length(q_0)*2, length(T));
            for i = 1:size(q_to_peak, 1)
               Z(2*i-1:2*i, :) =  [q_to_peak(i, :), q_to_stop(i, :); q_dot_to_peak(i, :), q_dot_to_stop(i, :)];
            end
        end
        
        function [q_plan] = compute_q_plan(P, q_0, q_dot_0, k)
            % returns the configuration at t_plan given initial state and
            % chosen acceleration k
            
            q_plan = q_0 + q_dot_0*P.t_plan + (1/2)*k*P.t_plan^2;
        end
    end
end