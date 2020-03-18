classdef robot_arm_RTD_planner_2link2d < robot_arm_generic_planner
    properties
        t_total = 1 ; % total duration of desired traj
        time_discretization = 0.01 ; % time discretization of desired traj
        q_0_prev = [];
        q_dot_0_prev = [];
        k_opt_prev = [0;0];
    end
    methods
        %% constructor
        function P = robot_arm_RTD_planner_2link2d(varargin)
            t_move = 0.5 ;
            lookahead_distance = 0.3 ;
            HLP = robot_arm_RRT_HLP('make_new_graph_every_iteration_flag',true) ;
            P@robot_arm_generic_planner('lookahead_distance',lookahead_distance,...
                't_move', t_move, 'HLP', HLP,...
                varargin{:}) ;
        end
        
        %% replan
        function [T,U,Z] = replan(P,agent_info,world_info)
            %% 1. generate cost function
            % % generate a waypoint in configuration space
            tic ;
            % q_cur = agent_info.state(P.arm_joint_state_indices, end) ;
            % q_goal = P.goal ;
            % dir_des = q_goal - q_cur ;
            % dir_des = dir_des./norm(dir_des) ;
            % q_des = q_cur + P.lookahead_distance.*dir_des ;
            q_des = P.HLP.get_waypoint(agent_info,world_info,P.lookahead_distance) ;
            
            % turn waypoint into a cost function
            %%% PATRICK CODE HERE %%%
            
            %% 2. generate constraints
            %% constraint generation
            % get current obstacles
            O = world_info.obstacles ;
            
            %%% MALICIOUS HACK DISCARD FIRST OBSTACLE
            %             O = O(2:end);
            
            % map obstacles to trajectory parameter space
            %%% PATRICK CODE HERE %%%
            q_0 = agent_info.state(P.arm_joint_state_indices, end) ;
            q_dot_0 = agent_info.state(P.arm_joint_speed_indices, end) ;
            
            [links, k_lim, k_unsafe_A, k_unsafe_b] = compute_unsafe_parameters_2link2d(q_0, q_dot_0, O);
            %% 3. solve for optimal trajectory parameter
            %%% PATRICK CODE HERE %%%
            %%% plot unsafe parameters:
            %% link 1 and 2 unsafe;
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
            try
                k_opt = P.trajopt(k_lim, k_unsafe_A, k_unsafe_b, q_0, q_dot_0, q_des);
                P.vdisp('New trajectory found!',3)
                %% 4. generate desired trajectory
                %%% PATRICK MODIFY THE TRAJECTORY Z HERE %%%
                T = 0:P.time_discretization:P.t_total ;
                N_T = length(T) ;
                U = zeros(P.arm_n_inputs,N_T) ;
                %                 Z = nan(P.arm_n_states,N_T) ;
                Z = P.generate_trajectory(T, q_0, q_dot_0, k_opt);
                P.q_0_prev = q_0;
                P.q_dot_0_prev = q_dot_0;
                P.k_opt_prev = k_opt;
                
                %                 plot(k_opt(1), k_opt(2), 'k.', 'MarkerSize', 20);
                
                %%% plot the reachable sets?
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
                %
                %
                %                 pause;
                
            catch
                % try to use the old trajectory
                P.vdisp('Unable to find new trajectory!',3)
                %%% PATRICK ADD CODE HERE TO USE BRAKING TRAJECTORY
                T = 0:P.time_discretization:P.t_total ;
                T_plan = T(1:floor(length(T)/2));
                T_brake = T(floor(length(T)/2)+1:end);
                N_T = length(T) ;
                U = zeros(P.arm_n_inputs,N_T) ;
                %                 Z = nan(P.arm_n_states,N_T) ;
                if ~isnan(P.q_0_prev) % if prev trajectory was not a braking traj
                    Z = P.generate_trajectory(T, P.q_0_prev, P.q_dot_0_prev, P.k_opt_prev);
                    Z_brake = Z(:, floor(length(T)/2)+1:end);
                    myones = ones(1, length(T_plan));
                    myzeros = zeros(1, length(T_plan));
                    Z = [Z_brake, [myones*Z_brake(1, end); myzeros; myones*Z_brake(3, end); myzeros]];
                    P.q_0_prev = nan;
                    P.q_dot_0_prev = nan;
                    P.k_opt_prev = nan;
                else
                    % command to stay in place
                    Z = ones(4, length(T)).*[q_0(1); 0; q_0(2); 0];
                    P.q_0_prev = nan;
                    P.q_dot_0_prev = nan;
                    P.k_opt_prev = nan;
                end
                %                 pause;
            end
            
            toc;
            
        end
        
        function [Z] = generate_trajectory(P, T, q_0, q_dot_0, q_dot_pk)
            % ah... q_dot_pk (aka k_opt) is in terms of theta_1_dot_pk,
            % theta_1_dot_pk + theta_2_dot_pk...
            % so when specifying trajectory for q2 and q2dot, have to
            % subtract off q1 and q1dot
            Z_plan = [];
            T_plan = T(1:floor(length(T)/2));
            Z_brake = [];
            T_brake = T(floor(length(T)/2)+1:end);
            
            % first arm trajectory
            [tout, xout] = ode45(@arm_dyn_toPeak_ODE, T_plan, [q_0(1); q_dot_0(1); q_dot_pk(1)]);
            Z_plan(1, :) = xout(:, 1)';
            Z_plan(2, :) = q_dot_0(1) + ((q_dot_pk(1) - q_dot_0(1))/P.t_plan).*T_plan;
            
            [tout, xout] = ode45(@arm_dyn_toStop_ODE, T_brake, [Z_plan(1, end); q_dot_0(1); q_dot_pk(1)]);
            Z_brake(1, :) = xout(:, 1)';
            Z_brake(2, :) = q_dot_pk(1) + ((0 - q_dot_pk(1))/(P.t_total - P.t_plan)).*(T_brake - P.t_plan);
            
            % second arm trajectory
            [tout, xout] = ode45(@arm_dyn_toPeak_ODE, T_plan, [q_0(1) + q_0(2); q_dot_0(1) + q_dot_0(2); q_dot_pk(2)]);
            Z_plan(3, :) = xout(:, 1)';
            Z_plan(4, :) = q_dot_0(1) + q_dot_0(2) + ((q_dot_pk(2) - (q_dot_0(1) + q_dot_0(2)) )/P.t_plan).*T_plan;
            
            [tout, xout] = ode45(@arm_dyn_toStop_ODE, T_brake, [Z_plan(3, end); q_dot_0(1) + q_dot_0(2); q_dot_pk(2)]);
            Z_brake(3, :) = xout(:, 1)';
            Z_brake(4, :) = q_dot_pk(2) + ((0 - q_dot_pk(2))/(P.t_total - P.t_plan)).*(T_brake - P.t_plan);
            
            
            Z_plan(3, :) = Z_plan(3, :) - Z_plan(1, :); % subtract q1
            Z_plan(4, :) = Z_plan(4, :) - Z_plan(2, :); % subtract q1_dot
            Z_brake(3, :) = Z_brake(3, :) - Z_brake(1, :); % subtract q1
            Z_brake(4, :) = Z_brake(4, :) - Z_brake(2, :); % subtract q1_dot
            
            %             for i = 1:length(q_0)
            %                 [tout, xout] = ode45(@arm_dyn_toPeak_ODE, T_plan, [q_0(i); q_dot_0(i); q_dot_pk(i)]);
            %                 Z_plan(2*i-1, :) = xout(:, 1)';
            %                 Z_plan(2*i, :) = q_dot_0(i) + ((q_dot_pk(i) - q_dot_0(i))/P.t_plan).*T_plan;
            %             end
            %             for i = 1:length(q_0)
            %                 [tout, xout] = ode45(@arm_dyn_toStop_ODE, T_brake, [Z_plan(2*i-1, end); q_dot_0(i); q_dot_pk(i)]);
            %                 Z_brake(2*i-1, :) = xout(:, 1)';
            %                 Z_brake(2*i, :) = q_dot_pk(i) + ((0 - q_dot_pk(i))/(P.t_total - P.t_plan)).*(T_brake - P.t_plan);
            %             end
            Z = [Z_plan, Z_brake];
        end
        
        function [q_plan] = compute_q_plan(P, q_0, q_dot_0, q_dot_pk)
            % returns the configuration at t_plan given initial state and
            % chosen peak velocity
            % for now only for 2 link arm...
            q_plan(1,1) = q_0(1) + q_dot_0(1)*P.t_plan + ((q_dot_pk(1) - q_dot_0(1))/2)*P.t_plan;
            q_plan(2,1) = q_0(2) + q_dot_0(2)*P.t_plan + ((q_dot_pk(2) - q_dot_pk(1) - q_dot_0(2))/2)*P.t_plan;
        end
        
        function [cost] = eval_cost(P, k, q_0, q_dot_0, q_des)
            q_plan = compute_q_plan(P, q_0, q_dot_0, k);
            cost = sum((q_plan - q_des).^2);
            %             cost = 0.75*(q_plan(1)-q_des(1))^2 + 0.25*(q_plan(2)-q_des(2))^2;
        end
        
        function [c, ceq] = eval_constraint(P, k_opt, k_unsafe_A, k_unsafe_b)
            epsilon = 1e-3;
            ceq = [];
            
            c_unsafe = [];
            %%% PATRICK ADD IN OBSTACLE CONSTRAINT CHECKING
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
            
            lb = [min(k_lim{2}.V(1, :)); min(k_lim{2}.V(2, :))];
            ub = [max(k_lim{2}.V(1, :)); max(k_lim{2}.V(2, :))];
            
            initial_guess = (lb + ub)/2;
            
            [k_opt, ~, exitflag, ~] = fmincon(cost_func, initial_guess, [], [], [], [], lb, ub, constraint_func) ;
            
            if exitflag <= 0
                error('Solver did not converge.');
            end
        end
    end
end