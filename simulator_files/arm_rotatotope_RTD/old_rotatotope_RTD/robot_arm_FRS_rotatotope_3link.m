classdef robot_arm_FRS_rotatotope_3link
    %robot_arm_FRS_rotatotope_fetch holds onto a cell of rotatotopes
    %describing the FRS of each joint
    %   this class is defined specifically for the fetch robot, and is
    %   mostly a wrapper to facilitate working with the rotatotope.m class.
    
    properties
        rot_axes = [3;2;1;2;1;2]; % order of rotation axes on fetch
        link_joints = {[1;2], [1;2;3;4], [1;2;3;4;5;6]}; % joints that each link depends on
        link_zonotopes = {zonotope([0.33/2, 0.33/2; 0, 0; 0, 0]); zonotope([0.33/2, 0.33/2; 0, 0; 0, 0]); zonotope([0.33/2, 0.33/2; 0, 0; 0, 0])};
        link_EE_zonotopes = {zonotope([0.33; 0; 0]); zonotope([0.33; 0; 0]); zonotope([0.33; 0; 0])};
        link_self_intersection = {[1;3]} % cell array of links that could intersect (joint limits cover the rest)
        n_links = 3;
        n_time_steps = 0;
        dim = 3;
        FRS_path = 'FRS_trig/';
        FRS_key = [];
        
        q = zeros(6, 1);
        q_dot = zeros(6, 1);
        
        link_rotatotopes = {};
        link_EE_rotatotopes = {};
        link_FRS = {}
        
        FRS_options = {};
        
        A_con = {};
        b_con = {};
        k_con = {};
        
        A_con_self = {};
        b_con_self = {};
        k_con_self = {};
        
        c_k = [];
        g_k = [];
    end
    
    methods
        function obj = robot_arm_FRS_rotatotope_3link(q, q_dot, FRS_options)
            %robot_arm_FRS_rotatotope_fetch constructs an FRS for the full arm
            % based on rotatotopes. this class is specific to the Fetch,
            % and will create an FRS using default link lengths and
            % rotation axis order.
            % the constructor just requires the current state and velocity
            % of the robot.
            
            obj.q = q;
            obj.q_dot = q_dot;
            
            FRSkeytmp = load([obj.FRS_path, '0key.mat']); % fix this
            obj.FRS_key = FRSkeytmp.c_IC;
                        
            if ~exist('FRS_options', 'var')
                obj.FRS_options.combs = generate_combinations_upto(200);
                obj.FRS_options.maxcombs = 200;
                obj.FRS_options.buffer_dist = 0;
                obj.FRS_options.origin_shift = zeros(3, 1);
            else
                obj.FRS_options = FRS_options;
            end
            
            obj = obj.create_FRS();
            
            % all rotatotopes defined for same parameter set:
            obj.c_k = obj.link_rotatotopes{end}{end}.c_k;
            obj.g_k = obj.link_rotatotopes{end}{end}.g_k;
            
            % generate k vertices for each link
            for i = 1:obj.n_links
                cubeV = cubelet_ND(length(obj.link_joints{i}))';
                obj.FRS_options.kV{i} = (cubeV.*obj.g_k(obj.link_joints{i})) + obj.c_k(obj.link_joints{i});
                obj.FRS_options.kV_lambda{i} = cubeV;
            end
        end
        
        function obj = create_FRS(obj)
            % this function loads the appropriate trig FRS's given q_dot,
            % rotates them by q, then constructs and stacks rotatotopes at
            % each time step
            trig_FRS = cell(length(obj.q_dot), 1);
            for i = 1:length(obj.q_dot)
                [~, closest_idx] = min(abs(obj.q_dot(i) - obj.FRS_key));
                filename = sprintf('%strig_FRS_%0.3f.mat', obj.FRS_path, obj.FRS_key(closest_idx));
                trig_FRS_load_tmp = load(filename);
                trig_FRS_load{i} = trig_FRS_load_tmp.Rcont;
                A = [cos(obj.q(i)), -sin(obj.q(i)), 0, 0, 0; sin(obj.q(i)), cos(obj.q(i)), 0, 0, 0;...
                    0, 0, 1, 0, 0; 0, 0, 0, 1, 0; 0, 0, 0, 0, 1];
                for j = 1:length(trig_FRS_load{i})
                    trig_FRS{j}{i} = A*zonotope_slice(trig_FRS_load{i}{j}{1}, 4, obj.q_dot(i));
                end
            end
            obj.n_time_steps = length(trig_FRS);
            
            % construct a bunch of rotatotopes
            for i = 1:obj.n_links
               for j = 1:obj.n_time_steps
                   obj.link_rotatotopes{i}{j} = rotatotope(obj.rot_axes(obj.link_joints{i}), trig_FRS{j}(obj.link_joints{i}), obj.link_zonotopes{i});
               end
            end
            for i = 1:obj.n_links - 1
                for j = 1:obj.n_time_steps
                    obj.link_EE_rotatotopes{i}{j} = rotatotope(obj.rot_axes(obj.link_joints{i}), trig_FRS{j}(obj.link_joints{i}), obj.link_EE_zonotopes{i});
                end
            end
            
            % stack
            obj.link_FRS = obj.link_rotatotopes;
            for i = 2:obj.n_links
                for j = 1:obj.n_time_steps
                    for k = 1:i-1
                        obj.link_FRS{i}{j} = obj.link_FRS{i}{j}.stack(obj.link_EE_rotatotopes{k}{j});
                    end
                end
            end
            
            % SHIFT ORIGINS:
            for i = 1:obj.n_links
                for j = 1:obj.n_time_steps
                    obj.link_FRS{i}{j}.Rc = obj.link_FRS{i}{j}.Rc + obj.FRS_options.origin_shift;
                    obj.link_FRS{i}{j}.RZ(:, 1) = obj.link_FRS{i}{j}.RZ(:, 1) + obj.FRS_options.origin_shift;
                end
            end
        end
        
        function [] = plot(obj, rate, colors)
            if ~exist('colors', 'var')
                colors = {'b', 'r', 'm'};
            end
            if ~exist('rate', 'var')
                rate = 1;
            end
            for i = 1:length(obj.link_FRS)
                for j = 1:rate:length(obj.link_FRS{i})
                    obj.link_FRS{i}{j}.plot(colors{i});
                end
            end
            
        end
        
        function [] = plot_slice(obj, k, rate, colors)
            if ~exist('colors', 'var')
                colors = {'b', 'r', 'm'};
            end
            if ~exist('rate', 'var')
                rate = 1;
            end
            for i = 1:length(obj.link_FRS)
                for j = 1:rate:length(obj.link_FRS{i})
                    obj.link_FRS{i}{j}.plot_slice(k(obj.link_joints{i}), colors{i});
                end
            end
            
        end
        
        function [] = plot_slice_gensIncluded(obj, k, rate, colors)
            if ~exist('colors', 'var')
                colors = {'b', 'r', 'm'};
            end
            if ~exist('rate', 'var')
                rate = 1;
            end
            for i = 1:length(obj.link_FRS)
                for j = 1:rate:length(obj.link_FRS{i})
                    obj.link_FRS{i}{j}.plot_slice_gensIncluded(k(obj.link_joints{i}), colors{i});
                end
            end
            
        end
        
        function [obj] = generate_constraints(obj, obstacles)
            for i = 1:length(obstacles)
                for j = 1:length(obj.link_FRS)
                    for k = 1:length(obj.link_FRS{j})
                        if (j == 1 && isprop(obstacles{i}, 'is_base_obstacle') && obstacles{i}.is_base_obstacle) % skip base obstacles for first link
                            obj.A_con{i}{j}{k} = [];
                            obj.b_con{i}{j}{k} = [];
                            obj.k_con{i}{j}{k} = [];
                        else
                            [obj.A_con{i}{j}{k}, obj.b_con{i}{j}{k}, obj.k_con{i}{j}{k}] = obj.link_FRS{j}{k}.generate_constraints(obstacles{i}.zono.Z, obj.FRS_options, j);
                        end
                    end
                end
            end
        end
        
        function [obj] = generate_self_intersection_constraints(obj)
            % takes in pairs of links that could intersect and generates
           % Acon and bcon, as well as the combinations of k_idxs (kcon) these
           % constraints depend on.
           for i = 1:length(obj.link_self_intersection) % for each pair of links
                for j = 1:length(obj.link_FRS{1}) % for each time step
                    R1 = obj.link_FRS{obj.link_self_intersection{i}(1)}{j};
                    R2 = obj.link_FRS{obj.link_self_intersection{i}(2)}{j};
                    
                    nGen_1 = size(R1.Rg, 2);
                    [~, kc_col_1] = find(any(R1.k_idx) & R1.c_idx);
                    frs_k_ind_G_1 = R1.Rg;
                    frs_k_ind_G_1(:, kc_col_1) = [];
                    frs_k_dep_G_1 = R1.Rg(:, kc_col_1);
                    
                    nGen_2 = size(R2.Rg, 2);
                    [~, kc_col_2] = find(any(R2.k_idx) & R2.c_idx);
                    frs_k_ind_G_2 = R2.Rg;
                    frs_k_ind_G_2(:, kc_col_2) = [];
                    frs_k_dep_G_2 = R2.Rg(:, kc_col_2);
                    
                    gen_concat = [frs_k_ind_G_1, frs_k_ind_G_2, 2*obj.FRS_options.buffer_dist/2*eye(3)]; % add buffer distance accounting for 2 links
                    gen_concat(:, ~any(gen_concat)) = []; % delete zero columns
                    gen_zono = [(R1.Rc - R2.Rc), gen_concat]; % centered at R1.Rc - R2.Rc
                    
                    [A_poly, b_poly] = polytope_PH(gen_zono, obj.FRS_options);
                    
                    % the difference in "sliced" points should be outside
                    % this zono:
                    k_dep_pt = [-frs_k_dep_G_1, frs_k_dep_G_2];
                    A_con = A_poly*k_dep_pt;
                    b_con = b_poly;
                    k_con = [[R1.k_idx(:, kc_col_1); zeros(size(R2.k_idx, 1) - size(R1.k_idx, 1), length(kc_col_1))], R2.k_idx(:, kc_col_2)];
                    
                    % add a test here that throws out unnecessary constraints.
                    % ( not entirely sure this is still valid!! )
                    intersection_possible = 0;
                    for k = 1:size(obj.FRS_options.kV_lambda{end}, 2)
                        lambdas_orig = k_con.*obj.FRS_options.kV_lambda{end}(:, k);
                        lambdas_prod = lambdas_orig;
                        lambdas_prod(~lambdas_prod) = 1;
                        lambdas_prod = prod(lambdas_prod, 1)';
                        lambdas_prod(~any(lambdas_orig)) = 0; % set lambdas corresponding to all zero columns equal to zero
                        
                        kVc = A_con*lambdas_prod - b_con;
                        test_kV = max(kVc);
                        if test_kV <= 0
                            intersection_possible = 1;
                        end
                    end
                    if ~intersection_possible
                        A_con = []; b_con = []; k_con = [];
                    end
                    
                    obj.A_con_self{i}{j} = A_con;
                    obj.b_con_self{i}{j} = b_con;
                    obj.k_con_self{i}{j} = k_con;
                    
                end
            end
            
            
        end
    end
end