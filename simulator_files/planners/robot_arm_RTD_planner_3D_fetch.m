classdef robot_arm_RTD_planner_3D_fetch < robot_arm_generic_planner
    properties
        t_total = 1 ; % total duration of desired traj
        time_discretization = 0.01 ; % time discretization of desired traj
        q_0_prev = [];
        q_dot_0_prev = [];
        k_opt_prev = [];
        FRS_options struct;
    end
    methods
        %% constructor
        function P = robot_arm_RTD_planner_3D_fetch(varargin)
            t_move = 0.5;
            lookahead_distance = 1;
            HLP = robot_arm_RRT_HLP('make_new_tree_every_iteration_flag',true) ;
%             HLP = robot_arm_PRM_HLP( ) ;
            P@robot_arm_generic_planner('lookahead_distance', lookahead_distance, 't_move', t_move, 'HLP', HLP, ...
                varargin{2:end}) ;
            P.FRS_options = varargin{1};
        end
        
        %% replan
        function [T,U,Z] = replan(P,agent_info,world_info)
            %% 1. generate cost function
            % generate a waypoint in configuration space
            tic;
%             q_cur = agent_info.state(P.arm_joint_state_indices, end) ;
%             q_goal = P.goal ;
%             dir_des = q_goal - q_cur ;
%             dir_des = dir_des./norm(dir_des) ;
%             q_des = q_cur + P.lookahead_distance.*dir_des ;
            q_des = P.HLP.get_waypoint(agent_info,world_info,P.lookahead_distance) ;
                        
            %% 2. generate constraints
            % get current obstacles
            O = world_info.obstacles ;
            
            % map obstacles to trajectory parameter space
            q_0 = agent_info.state(P.arm_joint_state_indices, end) ;
            q_dot_0 = agent_info.state(P.arm_joint_speed_indices, end) ;
            
            [~, k_lim, k_unsafe_A, k_unsafe_b] = compute_unsafe_parameters(q_0, q_dot_0, O, P.FRS_options);
            %% plot unsafe parameters:
            % link 1 and 2 unsafe;
%             figure(2); clf ; hold on; 
            
%             uA = k_lim{2}.A;
%             uB = k_lim{2}.b;
%             
%             upoly = mptPolytope(uA, uB);
%             plot(upoly)
%             
%             for i = 1:length(k_unsafe_A)
%                 for j = 1:length(k_unsafe_A{i})
%                     for k = 1:length(k_unsafe_A{i}{j})
%                         if ~isempty(k_unsafe_A{i}{j}{k})
%                             A = [k_unsafe_A{i}{j}{k}; uA];
%                             B = [k_unsafe_b{i}{j}{k}; uB];
%                             mypoly = mptPolytope(A, B);
%                             try
%                                 V = vertices(mypoly);
%                                 V = get(V, 'V')';
%                                 K = convhull(V);
%                                 pu = patch(V(K, 1), V(K, 2), 'r');
%                                 pu.FaceAlpha = 0.1;
%                             catch
%                                 warning('Unsafe polytope plotting did not work');
%                             end
%                         end
%                     end
%                 end
%             end
%             
%             title('Unsafe control parameters', 'FontSize', 24);
%             axis equal; axis square;
%             
%             xlabel('u_1', 'FontSize', 24);
%             ylabel('u_2', 'FontSize', 24);
            %% 3. solve for optimal trajectory parameter
            try
                k_opt = P.trajopt(k_lim, k_unsafe_A, k_unsafe_b, q_0, q_dot_0, q_des);
                P.vdisp('New trajectory found!',3)
                %% 4. generate desired trajectory
                T = 0:P.time_discretization:P.t_total ;
                N_T = length(T) ;
                U = zeros(P.arm_n_inputs,N_T) ;
                Z = P.generate_trajectory(T, q_0, q_dot_0, k_opt);
                P.q_0_prev = q_0;
                P.q_dot_0_prev = q_dot_0;
                P.k_opt_prev = k_opt;
                
                %% plot k_opt and reachable sets
%                 plot(k_opt(1), k_opt(2), 'k.', 'MarkerSize', 20);                
%                 figure(1); hold on;
%                 for i = 1:length(links{1}.FRS)
%                     Z1 = zonotope_slice(links{1}.FRS{i}, links{1}.info.param_dimensions, k_opt(1));
%                     p = plotFilled(Z1, [1, 2], 'g');
%                     p.FaceAlpha = 0.04;
%                     p.EdgeAlpha = 0.4;
%                 end
%                 
%                 for i = 1:length(links{2}.FRS)
%                     Z2 = zonotope_slice(links{2}.FRS{i}, links{2}.info.param_dimensions, k_opt);
%                     %    Z = project(Rcont2{i}{1}, [1, 2]);
%                     p = plotFilled(Z2, [1, 2], 'g');
%                     p.FaceAlpha = 0.04;
%                     p.EdgeAlpha = 0.4;
%                 end
%                 pause;
            %% if no safe trajectory parameter found:
            catch
                % generate a braking trajectory
                P.vdisp('Unable to find new trajectory!',3)
                T = 0:P.time_discretization:P.t_total ;
                T_plan = T(1:floor(length(T)/2));
                T_brake = T(floor(length(T)/2)+1:end);
                N_T = length(T) ;
                U = zeros(P.arm_n_inputs,N_T) ;
                if ~isnan(P.q_0_prev) % if prev trajectory was not a braking traj
                    Z = P.generate_trajectory(T, P.q_0_prev, P.q_dot_0_prev, P.k_opt_prev);
                    Z_brake = Z(:, floor(length(T)/2)+1:end);
%                     myones = ones(1, length(T_plan));
%                     myzeros = zeros(1, length(T_plan));
%                     Z = [Z_brake, [myones*Z_brake(1, end); myzeros; myones*Z_brake(3, end); myzeros; myones*Z_brake(5, end); myzeros]];
                    z = [Z_brake(1:2:end, end)'; zeros(length(q_0), 1)'];
                    Z = [Z_brake, ones(size(Z, 1), length(T_plan)).*z(:)];
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
            end
            toc;
        end
        
        function [Z] = generate_trajectory(P, T, q_0, q_dot_0, q_dot_pk)
            % will have to rewrite when moving to 3D.
            Z_plan = [];
            T_plan = T(1:floor(length(T)/2));
            Z_brake = [];
            T_brake = T(floor(length(T)/2)+1:end);
            
            z_0 = cumsum(q_0);
            z_dot_0 = cumsum(q_dot_0);
            
            for i = 1:length(q_0)
                % arm trajectory
                [tout, xout] = ode45(@arm_dyn_toPeak_ODE, T_plan, [z_0(i); z_dot_0(i); q_dot_pk(i)]);
                Z_plan(2*(i-1)+1, :) = xout(:, 1)';
                Z_plan(2*i, :) = z_dot_0(i) + ((q_dot_pk(i) - (z_dot_0(i)))/P.t_plan).*T_plan;
                
                [tout, xout] = ode45(@arm_dyn_toStop_ODE, T_brake, [Z_plan(2*(i-1)+1, end); z_dot_0(i); q_dot_pk(i)]);
                Z_brake(2*(i-1)+1, :) = xout(:, 1)';
                Z_brake(2*i, :) = q_dot_pk(i) + ((0 - q_dot_pk(i))/(P.t_total - P.t_plan)).*(T_brake - P.t_plan);
                
                % now in terms of z... put in terms of q
                for j = 1:i-1
                    Z_plan(2*(i-1)+1, :) = Z_plan(2*(i-1)+1, :) - Z_plan(2*(j-1)+1, :);
                    Z_plan(2*i, :) = Z_plan(2*i, :) - Z_plan(2*j, :);
                    Z_brake(2*(i-1)+1, :) = Z_brake(2*(i-1)+1, :) - Z_brake(2*(j-1)+1, :);
                    Z_brake(2*i, :) = Z_brake(2*i, :) - Z_brake(2*j, :);
                end
            end
            
            Z = [Z_plan, Z_brake];
        end
        
%                 function [Z] = generate_trajectory_old(P, T, q_0, q_dot_0, q_dot_pk)
%             % ah... q_dot_pk (aka k_opt) is in terms of theta_1_dot_pk,
%             % theta_1_dot_pk + theta_2_dot_pk...
%             % so when specifying trajectory for q2 and q2dot, have to
%             % subtract off q1 and q1dot
%             Z_plan = [];
%             T_plan = T(1:floor(length(T)/2));
%             Z_brake = [];
%             T_brake = T(floor(length(T)/2)+1:end);
%             
%             % first arm trajectory
%             [tout, xout] = ode45(@arm_dyn_toPeak_ODE, T_plan, [q_0(1); q_dot_0(1); q_dot_pk(1)]);
%             Z_plan(1, :) = xout(:, 1)';
%             Z_plan(2, :) = q_dot_0(1) + ((q_dot_pk(1) - q_dot_0(1))/P.t_plan).*T_plan;
%             
%             [tout, xout] = ode45(@arm_dyn_toStop_ODE, T_brake, [Z_plan(1, end); q_dot_0(1); q_dot_pk(1)]);
%             Z_brake(1, :) = xout(:, 1)';
%             Z_brake(2, :) = q_dot_pk(1) + ((0 - q_dot_pk(1))/(P.t_total - P.t_plan)).*(T_brake - P.t_plan);
%             
%             % second arm trajectory
%             [tout, xout] = ode45(@arm_dyn_toPeak_ODE, T_plan, [q_0(1) + q_0(2); q_dot_0(1) + q_dot_0(2); q_dot_pk(2)]);
%             Z_plan(3, :) = xout(:, 1)';
%             Z_plan(4, :) = q_dot_0(1) + q_dot_0(2) + ((q_dot_pk(2) - (q_dot_0(1) + q_dot_0(2)) )/P.t_plan).*T_plan;
%             
%             [tout, xout] = ode45(@arm_dyn_toStop_ODE, T_brake, [Z_plan(3, end); q_dot_0(1) + q_dot_0(2); q_dot_pk(2)]);
%             Z_brake(3, :) = xout(:, 1)';
%             Z_brake(4, :) = q_dot_pk(2) + ((0 - q_dot_pk(2))/(P.t_total - P.t_plan)).*(T_brake - P.t_plan);
%             
% 
%             % third arm trajectory
%             [tout, xout] = ode45(@arm_dyn_toPeak_ODE, T_plan, [q_0(1) + q_0(2) + q_0(3); q_dot_0(1) + q_dot_0(2) + q_dot_0(3); q_dot_pk(3)]);
%             Z_plan(5, :) = xout(:, 1)';
%             Z_plan(6, :) = q_dot_0(1) + q_dot_0(2) + q_dot_0(3) + ((q_dot_pk(3) - (q_dot_0(1) + q_dot_0(2) + q_dot_0(3)) )/P.t_plan).*T_plan;
%             
%             [tout, xout] = ode45(@arm_dyn_toStop_ODE, T_brake, [Z_plan(5, end); q_dot_0(1) + q_dot_0(2) + q_dot_0(3); q_dot_pk(3)]);
%             Z_brake(5, :) = xout(:, 1)';
%             Z_brake(6, :) = q_dot_pk(3) + ((0 - q_dot_pk(3))/(P.t_total - P.t_plan)).*(T_brake - P.t_plan);
%             
%             % subtract from second arm
%             Z_plan(3, :) = Z_plan(3, :) - Z_plan(1, :); % subtract q1
%             Z_plan(4, :) = Z_plan(4, :) - Z_plan(2, :); % subtract q1_dot
%             Z_brake(3, :) = Z_brake(3, :) - Z_brake(1, :); % subtract q1
%             Z_brake(4, :) = Z_brake(4, :) - Z_brake(2, :); % subtract q1_dot
%             
%             % subtract from third arm
%             Z_plan(5, :) = Z_plan(5, :) - Z_plan(1, :) - Z_plan(3, :); % subtract q1 and q2
%             Z_plan(6, :) = Z_plan(6, :) - Z_plan(2, :) - Z_plan(4, :); % subtract q1_dot and q2_dot
%             Z_brake(5, :) = Z_brake(5, :) - Z_brake(1, :) - Z_brake(3, :); % subtract q1 and q2
%             Z_brake(6, :) = Z_brake(6, :) - Z_brake(2, :) - Z_brake(4, :); % subtract q1_dot and q2_dot
%             
% %             for i = 1:length(q_0)
% %                 [tout, xout] = ode45(@arm_dyn_toPeak_ODE, T_plan, [q_0(i); q_dot_0(i); q_dot_pk(i)]);
% %                 Z_plan(2*i-1, :) = xout(:, 1)';
% %                 Z_plan(2*i, :) = q_dot_0(i) + ((q_dot_pk(i) - q_dot_0(i))/P.t_plan).*T_plan;
% %             end
% %             for i = 1:length(q_0)
% %                 [tout, xout] = ode45(@arm_dyn_toStop_ODE, T_brake, [Z_plan(2*i-1, end); q_dot_0(i); q_dot_pk(i)]);
% %                 Z_brake(2*i-1, :) = xout(:, 1)';
% %                 Z_brake(2*i, :) = q_dot_pk(i) + ((0 - q_dot_pk(i))/(P.t_total - P.t_plan)).*(T_brake - P.t_plan);
% %             end
%             Z = [Z_plan, Z_brake];
%         end
             
        function [q_plan] = compute_q_plan(P, q_0, q_dot_0, q_dot_pk)
            % returns the configuration at t_plan given initial state and
            % chosen peak velocity
            q_plan = zeros(length(q_0), 1);
            for i = 1:length(q_0)
                if i == 1
                    q_plan(i, 1) = q_0(i) + q_dot_0(i)*P.t_plan + ((q_dot_pk(i) - q_dot_0(i))/2)*P.t_plan;
                else
                    q_plan(i, 1) = q_0(i) + q_dot_0(i)*P.t_plan + ((q_dot_pk(i) - q_dot_pk(i-1) - q_dot_0(i))/2)*P.t_plan;
                end
            end
        end
        
        function [cost] = eval_cost(P, k, q_0, q_dot_0, q_des)
           q_plan = compute_q_plan(P, q_0, q_dot_0, k);
           cost = sum((q_plan - q_des).^2);
        end
        
        function [c, ceq] = eval_constraint(P, k_opt, k_unsafe_A, k_unsafe_b)
            epsilon = 1e-3;
            ceq = [];
            
            c_unsafe = [];
            %% Obstacle constraint checking:
            for i = 1:length(k_unsafe_A) % for each link
                for j = 1:length(k_unsafe_A{i}) % for each obstacle
                    for k = 1:length(k_unsafe_A{i}{j}) % for each time step
                        if ~isempty(k_unsafe_A{i}{j}{k})
                            myc = k_unsafe_A{i}{j}{k}*k_opt - k_unsafe_b{i}{j}{k};
                            myc = -(max(myc) - epsilon);
                            c_unsafe = [c_unsafe; myc];
                        end
                    end
                end
            end
            
            c = [c_unsafe];
        end
        
        function [k_opt] = trajopt(P, k_lim, k_unsafe_A, k_unsafe_b, q_0, q_dot_0, q_des)
            % use fmincon to optimize the cost subject to constraints
            P.vdisp('Running trajopt',3)
            cost_func = @(k) P.eval_cost(k, q_0, q_dot_0, q_des);
            constraint_func = @(k) P.eval_constraint(k, k_unsafe_A, k_unsafe_b);
            
            % generate upper and lower bounds
            lb = [];
            ub = [];
            for i = 1:length(k_lim)
                lb = [lb; min(k_lim{end}.V(i, :))];
                ub = [ub; max(k_lim{end}.V(i, :))];
            end
            
            initial_guess = (lb + ub)/2;
           
            [k_opt, ~, exitflag, ~] = fmincon(cost_func, initial_guess, [], [], [], [], lb, ub, constraint_func) ;
            
            if exitflag <= 0
                error('Solver did not converge.');
            end
        end
    end
end