classdef robot_arm_RTD_planner_3D_1link < robot_arm_generic_planner
    properties
        t_total = 1 ; % total duration of desired traj
        time_discretization = 0.01 ; % time discretization of desired traj
        FRS_options struct;
        
        % for current FRS
        R = {};
        phi_dot_0 = [];
        
        % from previous time step
        Z_prev = [];
        R_prev = {};
        phi_dot_0_prev = [];
        q_0_prev = [];
        q_dot_0_prev = [];
        k_opt_prev = [];
        
        iter = 0;
        first_iter_pause = 1;
    end
    methods
        %% constructor
        function P = robot_arm_RTD_planner_3D_1link(varargin)
            t_move = 0.5;
            lookahead_distance = 0.2;
            HLP = robot_arm_RRT_HLP('make_new_tree_every_iteration_flag',true) ;
%             HLP = robot_arm_PRM_HLP( ) ;
%             HLP = robot_arm_straight_line_HLP( );
            P@robot_arm_generic_planner('lookahead_distance', lookahead_distance, 't_move', t_move, 'HLP', HLP, ...
                varargin{2:end}) ;
            P.FRS_options = varargin{1};
        end
        
        %% replan
        function [T,U,Z] = replan(P,agent_info,world_info)
            %% 1. generate cost function
            % generate a waypoint in configuration space
            if P.first_iter_pause && P.iter == 0
               pause; 
            end
            P.iter = P.iter + 1;
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
            
            % get current state of robot
            q_0 = agent_info.state(P.arm_joint_state_indices, end) ;
            q_dot_0 = agent_info.state(P.arm_joint_speed_indices, end) ;
            
            % determine rotation matrices and phi_dot_0 for zonotope RTD
            %%% PATRICK CODE FOR PHI_DOT_0 ALMOST CERTAINLY WRONG
            %%% EDIT FIXED it seems pretty good now...
            [P.R, P.phi_dot_0] = P.get_RTD_IC(agent_info, q_0, q_dot_0);
            
%             quiver3(0, 0, 0, P.R{1}(1, 1), P.R{1}(2, 1), P.R{1}(3, 1), 'b', 'LineWidth', 10);
            quiver3(0, 0, 0, P.R{1}(1, 2), P.R{1}(2, 2), P.R{1}(3, 2), 'm');
            quiver3(0, 0, 0, P.R{1}(1, 3), P.R{1}(2, 3), P.R{1}(3, 3), 'r');
%             pause;

            
            % map obstacles to trajectory parameter space
            [links, k_lim, k_unsafe_A, k_unsafe_b] = compute_unsafe_parameters_3D(P.R, P.phi_dot_0, O, P.FRS_options);
            %% plot unsafe parameters:
            % link 1 and 2 unsafe;
%             figure(2); clf ; hold on; 
%             
%             uA = k_lim{1}.A;
%             uB = k_lim{1}.b;
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
%                 Z = P.generate_trajectory(T, q_0, q_dot_0, k_opt);
                X = generate_trajectory_from_k(P.R, P.phi_dot_0, k_opt, P.FRS_options);
                Q = get_fetch_q_from_traj_1link(X, q_0);
                Q_dot = (diff(Q')./P.time_discretization)';
                Q_dot(:, end+1) = Q_dot(:, end);
                
                Z = [Q(1, :); Q_dot(1, :); Q(2, :); Q_dot(2, :)];
                
                P.R_prev = P.R;
                P.phi_dot_0_prev = P.phi_dot_0;
                P.Z_prev = Z;
                
                P.q_0_prev = q_0;
                P.q_dot_0_prev = q_dot_0;
                P.k_opt_prev = k_opt;
                
                %% plot k_opt and reachable sets
%                 plot(k_opt(1), k_opt(2), 'k.', 'MarkerSize', 20);
%                 figure(1); hold on;
%                 for i = 1:length(links)
%                     for j = 1:2:length(links{i}.FRS)
%                         myZ = zonotope_slice(links{i}.FRS{j}, links{i}.info.param_dimensions, k_opt);
%                         %         Z = links{i}.FRS{j};
%                         V = vertices(project(myZ, [1, 2, 3]));
%                         shp = alphaShape(V(1, :)', V(2, :)', V(3, :)', inf);
%                         p = plot(shp);
%                         p.FaceAlpha = 0;
%                         p.EdgeAlpha = 0.15;
%                         p.EdgeColor = 'b';
%                     end
%                 end
%                 disp(P.phi_dot_0);
%                 pause;

            %% if no safe trajectory parameter found:
            catch
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
            end
            toc;
        end
        
        function [R, phi_dot_0] = get_RTD_IC(P, agent_info, q_0, q_dot_0)
            % for now assuming a 1 link arm.. update for fetch
            curr_R = agent_info.get_link_rotations_and_translations(q_0);
            new_x = curr_R{2}(:, 1);
            
            % get the angular velocity vector from joint speeds
            omega = curr_R{1}*[0; 0; q_dot_0(1)] + curr_R{2}*[0; q_dot_0(2); 0];
            
            % curr_R{2} has the x axis aligned correctly with the first
            % link... we only care about the projection of omega onto the
            % plane perpendicular to the body fixed x-axis, because we
            % don't care about rotations about the x-axis.
            omega_parallel = dot(omega, new_x)*new_x;
            omega_perp = omega - omega_parallel;
            
            
            phi_dot_0(1, 1) = sqrt(sum(omega_perp.^2));
            if phi_dot_0(1, 1) == 0
                new_z = curr_R{2}(:, 3);
            else
                new_z = omega_perp./phi_dot_0(1);
            end
            new_y = cross(new_z, new_x);
            R{1} = [new_x, new_y, new_z];
        end
        
        function [k_opt] = trajopt(P, k_lim, k_unsafe_A, k_unsafe_b, q_0, q_dot_0, q_des)
            % use fmincon to optimize the cost subject to constraints
            P.vdisp('Running trajopt',3)
            cost_func = @(k) P.eval_cost(k, q_0, q_dot_0, q_des);
            constraint_func = @(k) P.eval_constraint(k, k_unsafe_A, k_unsafe_b);
            
            % generate upper and lower bounds
            lb = [];
            ub = [];
            for i = 1:size(k_lim{end}.V, 1)
                lb = [lb; min(k_lim{end}.V(i, :))];
                ub = [ub; max(k_lim{end}.V(i, :))];
            end
            
            initial_guess = (lb + ub)/2;
           
            [k_opt, ~, exitflag, ~] = fmincon(cost_func, initial_guess, [], [], [], [], lb, ub, constraint_func) ;
            
            if exitflag <= 0
                error('Solver did not converge.');
            end
        end
        
                
        function [cost] = eval_cost(P, k, q_0, q_dot_0, q_des)
            % generate a simple cost fucntion
%            q_plan = compute_q_plan(P, q_0, q_dot_0, k);
%            cost = sum((q_plan - q_des).^2);

            x_plan = P.compute_x_plan(q_0, q_dot_0, k);
            
%             myR = make_orientation(q_des(1), 3)*make_orientation(q_des(2), 2);
%             x_des = myR*[0.33; 0; 0];

            myR = make_orientation(P.goal(1), 3)*make_orientation(P.goal(2), 2);
            x_des = myR*[0.33; 0; 0];
            
            cost = sum((x_plan - x_des).^2);
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
        
        function [x_plan] = compute_x_plan(P, q_0, q_dot_0, k)
            % returns the configuration at t_plan given initial state and
            % chosen peak velocity
            
            X = generate_trajectory_from_k(P.R, P.phi_dot_0, k, P.FRS_options);
            x_plan = X(:, 50);
%             q_plan = get_fetch_q_from_traj_1link(x_plan, q_0);
        end
        
%         function [q_plan] = compute_q_plan(P, q_0, q_dot_0, k)
%             % returns the configuration at t_plan given initial state and
%             % chosen peak velocity
%             
%             X = generate_trajectory_from_k(P.R, P.phi_dot_0, k, P.FRS_options);
%             x_plan = X(:, 50);
%             q_plan = get_fetch_q_from_traj_1link(x_plan, q_0);
%         end
    end
end