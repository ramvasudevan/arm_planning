classdef robot_arm_RTD_planner_3D_fetch < robot_arm_generic_planner
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
        function P = robot_arm_RTD_planner_3D_fetch(varargin)
            t_move = 0.5;
            lookahead_distance = 0.1;
%             HLP = robot_arm_RRT_HLP('make_new_tree_every_iteration_flag',true) ;
%             HLP = robot_arm_PRM_HLP( ) ;
            HLP = robot_arm_straight_line_HLP( );
            P@robot_arm_generic_planner('lookahead_distance', lookahead_distance, 't_move', t_move, 'HLP', HLP, ...
                varargin{2:end}) ;
            P.FRS_options = varargin{1};
            P.FRS_options.combs = generate_combinations_upto(200);
            P.FRS_options.maxcombs = 200;
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
%             q_des = P.HLP.get_waypoint(agent_info,world_info,P.lookahead_distance) ;

            x_goal = P.q_to_x(P.goal);
            x_cur = P.q_to_x(agent_info.state(P.arm_joint_state_indices, end));
            dir_des = x_goal - x_cur;
            dir_des = dir_des./norm(dir_des);
            x_des = x_cur + P.lookahead_distance.*dir_des;
                        
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
%             quiver3(0, 0, 0, P.R{1}(1, 2), P.R{1}(2, 2), P.R{1}(3, 2), 'm');
%             quiver3(0, 0, 0, P.R{1}(1, 3), P.R{1}(2, 3), P.R{1}(3, 3), 'r');
%             pause;

            

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
            try
                % map obstacles to trajectory parameter space
                [~, k_lim, k_unsafe_A, k_unsafe_b] = compute_unsafe_parameters_3D(P.R, P.phi_dot_0, O, P.FRS_options);
            
                %% 3. solve for optimal trajectory parameter
                k_opt = P.trajopt(k_lim, k_unsafe_A, k_unsafe_b, q_0, q_dot_0, x_des);
                P.vdisp('New trajectory found!',3)
                %% 4. generate desired trajectory
                T = 0:P.time_discretization:P.t_total ;
                N_T = length(T) ;
                U = zeros(P.arm_n_inputs,N_T) ;
%                 Z = P.generate_trajectory(T, q_0, q_dot_0, k_opt);
                X = generate_trajectory_from_k(P.R, P.phi_dot_0, k_opt, P.FRS_options);
                Q = get_fetch_q_from_traj(X, q_0);
%                 Q = zeros(6, length(T));
                Q_dot = (diff(Q')./P.time_discretization)';
                Q_dot(:, end+1) = Q_dot(:, end);
                
                Z = [];
                for i = 1:size(Q, 1)
                    Z = [Z; Q(i, :); Q_dot(i, :)];
                end
                
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
%                     for j = 1:10:length(links{i}.FRS)
%                         myZ = zonotope_slice(links{i}.FRS{j}, links{i}.info.param_dimensions, k_opt(1:2*i, 1));
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
            catch ME
                switch ME.identifier
                    case 'zonoRTD:slicePoint'
                        P.vdisp(' %%%%% IC outside bounds of zonotopes... executing braking maneuver %%%%%', 3);
                    case 'planner:trajOpt'
                        P.vdisp(' %%%%% Solver did not converge to feasible point... executing braking maneuver %%%%%', 3);
                    otherwise
                        error(ME.message);
                end
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
            % edit updated 8/22/2019... not sure it's correct
            curr_R = agent_info.get_link_rotations_and_translations(q_0);
            omega = zeros(3, 1);
            
            for i = 1:length(curr_R)/2
                new_x = curr_R{2*i}(:, 1);
                
                % get the angular velocity vector from joint speeds
                if i == 1
%                     omega = curr_R{2*(i-1)+1}*[0; 0; q_dot_0(2*(i-1)+1)] + curr_R{2*i}*[0; q_dot_0(2*i); 0];
                    omega = curr_R{1}*[0; 0; q_dot_0(1)] + curr_R{2}*[0; q_dot_0(2); 0];
                elseif i == 2
                    omega = omega + curr_R{3}*[q_dot_0(3); 0; 0] + curr_R{4}*[0; q_dot_0(4); 0];
                elseif i == 3
                    omega = omega + curr_R{5}*[q_dot_0(5); 0; 0] + curr_R{6}*[0; q_dot_0(6); 0];
                end
                    
                % curr_R{2} has the x axis aligned correctly with the first
                % link... we only care about the projection of omega onto the
                % plane perpendicular to the body fixed x-axis, because we
                % don't care about rotations about the x-axis.
                omega_parallel = dot(omega, new_x)*new_x;
                omega_perp = omega - omega_parallel;
                
                phi_dot_0(i, 1) = sqrt(sum(omega_perp.^2));
                if phi_dot_0(i, 1) == 0
                    new_z = curr_R{2*i}(:, 3);
                else
                    new_z = omega_perp./phi_dot_0(i);
                end
                new_y = cross(new_z, new_x);
                R{i} = [new_x, new_y, new_z];
            end
        end
        
        function [k_opt] = trajopt(P, k_lim, k_unsafe_A, k_unsafe_b, q_0, q_dot_0, x_des)
            % use fmincon to optimize the cost subject to constraints
            P.vdisp('Running trajopt',3)
            cost_func = @(k) P.eval_cost(k, q_0, q_dot_0, x_des);
            constraint_func = @(k) P.eval_constraint(k, k_unsafe_A, k_unsafe_b);
            
            % generate upper and lower bounds
            lb = [];
            ub = [];
            for i = 1:size(k_lim{end}.V, 1)
                lb = [lb; min(k_lim{end}.V(i, :))];
                ub = [ub; max(k_lim{end}.V(i, :))];
            end
            
            initial_guess = (lb + ub)/2;
           
            options = optimoptions('fmincon','SpecifyConstraintGradient',true);
            [k_opt, ~, exitflag, ~] = fmincon(cost_func, initial_guess, [], [], [], [], lb, ub, constraint_func, options) ;
            
            if exitflag <= 0
                error('planner:trajOpt', 'Solver did not converge.');
            end
        end
        
                
        function [cost] = eval_cost(P, k, q_0, q_dot_0, x_des)
            % generate a simple cost fucntion
%            q_plan = compute_q_plan(P, q_0, q_dot_0, k);
%            cost = sum((q_plan - q_des).^2);

            x_plan = P.compute_x_plan(q_0, q_dot_0, k);
            
%             q_des = P.goal;
% 
%             myR = make_orientation(q_des(1), 3)*make_orientation(q_des(2), 2);
%             x_des_1 = myR*[0.33; 0; 0];
%             myR2 = make_orientation(q_des(1), 3)*make_orientation(q_des(2), 2)*make_orientation(q_des(3), 1)*make_orientation(q_des(4), 2);
%             x_des_2 = x_des_1 + myR2*[0.33; 0; 0];
%             myR3 = make_orientation(q_des(1), 3)*make_orientation(q_des(2), 2)*make_orientation(q_des(3), 1)*make_orientation(q_des(4), 2)*make_orientation(q_des(5), 1)*make_orientation(q_des(6), 2);
%             x_des_3 = x_des_2 + myR3*[0.33; 0; 0];
            
%             x_des = [x_des_1; x_des_2; x_des_3];
%             x_plan = x_plan(7:9, 1);
%             x_des = x_des_3;
            
            cost = sum((x_plan - x_des).^2);
        end
        
        function [c, ceq, gradc, gradceq] = eval_constraint(P, k_opt, k_unsafe_A, k_unsafe_b)
            epsilon = 1e-3;
            ceq = [];
            gradceq = [];
            
            c = [];
            gradc = [];
            %% Obstacle constraint generation:
            for i = 1:length(k_unsafe_A) % for each link
                for j = 1:length(k_unsafe_A{i}) % for each obstacle
                    idx = find(~cellfun('isempty', k_unsafe_A{i}{j}));
%                     for k = 1:length(k_unsafe_A{i}{j}) % for each time step
                    for k = 1:length(idx)
                        c_obs = k_unsafe_A{i}{j}{idx(k)}*k_opt - k_unsafe_b{i}{j}{idx(k)};
                        c_obs_max = max(c_obs);
                        c_k = -(c_obs_max - epsilon);
                        c = [c; c_k];
                        
                        % specify gradients
                        if nargout > 2
                            maxidx = find(c_obs == c_obs_max);
                            if length(maxidx) > 1
%                                 disp('ahhh');
                                tempgradc = k_unsafe_A{i}{j}{idx(k)}(maxidx, :);
                                gradc = [gradc, -max(tempgradc)'];
                            else
                                gradc = [gradc, -k_unsafe_A{i}{j}{idx(k)}(maxidx, :)'];
                            end
                        end
                    end
                end
            end
            
        end
        
        function [x_plan] = compute_x_plan(P, q_0, q_dot_0, k)
            % returns the configuration at t_plan given initial state and
            % chosen peak velocity
            
%             X = generate_trajectory_from_k(P.R, P.phi_dot_0, k, P.FRS_options);
%             x_plan = X(:, 50);
            
            x_plan = generate_x_plan_from_k(P.R, P.phi_dot_0, k, P.FRS_options);
            
%             error_val = sum((x_plan - x_plan_2).^2);
%             if  error_val > 1e-4
%                 warning('patrick computed x_plan wrong');
%             end
            
%             q_plan = get_fetch_q_from_traj_1link(x_plan, q_0);
        end
        
        function [q_plan] = compute_q_plan(P, q_0, q_dot_0, k)
            % returns the configuration at t_plan given initial state and
            % chosen peak velocity
            
            X = generate_trajectory_from_k(P.R, P.phi_dot_0, k, P.FRS_options);
            x_plan = X(:, 50);
            q_plan = get_fetch_q_from_traj(x_plan, q_0);
        end
        
        function [X] = q_to_x(P, q)
            myR = make_orientation(q(1), 3)*make_orientation(q(2), 2);
            x_1 = myR*[0.33; 0; 0];
            myR2 = make_orientation(q(1), 3)*make_orientation(q(2), 2)*make_orientation(q(3), 1)*make_orientation(q(4), 2);
            x_2 = x_1 + myR2*[0.33; 0; 0];
            myR3 = make_orientation(q(1), 3)*make_orientation(q(2), 2)*make_orientation(q(3), 1)*make_orientation(q(4), 2)*make_orientation(q(5), 1)*make_orientation(q(6), 2);
            x_3 = x_2 + myR3*[0.33; 0; 0];
            
            X = [x_1; x_2; x_3];
        end
    end
end