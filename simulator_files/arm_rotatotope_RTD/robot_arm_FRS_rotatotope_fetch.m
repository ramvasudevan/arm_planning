classdef robot_arm_FRS_rotatotope_fetch
    %robot_arm_FRS_rotatotope_fetch holds onto a cell of rotatotopes
    %describing the FRS of each joint
    %   this class is defined specifically for the fetch robot, and is
    %   mostly a wrapper to facilitate working with the rotatotope.m class.
    
    properties
        rot_axes = [3;2;1;2;1;2]; % order of rotation axes on fetch
        link_joints = {[1;2], [1;2;3;4], [1;2;3;4;5;6]}; % joints that each link depends on
        link_zonotopes = {zonotope([0.33/2, 0.33/2; 0, 0; 0, 0]); zonotope([0.33/2, 0.33/2; 0, 0; 0, 0]); zonotope([0.33/2, 0.33/2; 0, 0; 0, 0])};
        link_EE_zonotopes = {zonotope([0.33; 0; 0]); zonotope([0.33; 0; 0]); zonotope([0.33; 0; 0])};
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
        
        c_k = [];
        g_k = [];
        
        % ===== mex properties =====
        obstacles = [];
        k_opt = [];
        
        mex_RZ = [];
        mex_c_idx = [];
        mex_k_idx = [];
        
        mex_A_con = [];
        mex_b_con = [];
        mex_k_con = [];
        
        deb_RZ = [];
        deb_c_idx = [];
        deb_k_idx = [];
        
        eval_output = [];
        eval_grad_output = []
        % ===== mex properties =====
    end
    
    methods
        function obj = robot_arm_FRS_rotatotope_fetch(q, q_dot, obstacles, k_opt, FRS_options)
            %robot_arm_FRS_rotatotope_fetch constructs an FRS for the full arm
            % based on rotatotopes. this class is specific to the Fetch,
            % and will create an FRS using default link lengths and
            % rotation axis order.
            % the constructor just requires the current state and velocity
            % of the robot.
            
            obj.q = q;
            obj.q_dot = q_dot;
            obj.obstacles = obstacles;
            obj.k_opt = k_opt;
            
            FRSkeytmp = load([obj.FRS_path, '0key.mat']); % fix this
            obj.FRS_key = FRSkeytmp.c_IC;
            
            obj = obj.create_FRS();
            
            if ~exist('FRS_options', 'var')
                obj.FRS_options.combs = generate_combinations_upto(200);
                obj.FRS_options.maxcombs = 200;
                obj.FRS_options.buffer_dist = 0;
            else
                obj.FRS_options = FRS_options;
            end
            
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
            fprintf("ROTATOTOPE CONSTRUCT ORIGINAL TIME\n")
            tic
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
            toc
            
            %% ===== mex =====
            fprintf("MEX TIME\n");
            tic;
            % SUOPPOSE ALL THE INPUT HAVE THE SAME SIZE !!!
            mexin_R = [];
            mexin_Z = [];
            mexin_EE = [];
            for i = 1:obj.n_links
                mexin_Z = [mexin_Z, obj.link_zonotopes{i}.Z];
            end
            for i = 1:obj.n_links-1
                mexin_EE = [mexin_EE, obj.link_EE_zonotopes{i}.Z];
            end

            for i = 1:length(obj.link_joints{end})
                for j = 1:obj.n_time_steps
                    mexin_R = [mexin_R, trig_FRS{j}{i}.Z(:,1:10)];
                end
            end
            
            % promise that the sizes of obstacles are the same, <= 10
            mexin_OZ = [];
            for i = 1:length(obj.obstacles)
                mexin_OZ = [mexin_OZ, obj.obstacles{i}.zono.Z];
            end
            
            %[obj.mex_A_con, obj.mex_b_con, obj.mex_k_con] = rotatotope_mex(obj.n_links, obj.n_time_steps, mexin_R, mexin_Z, mexin_EE, length(obj.obstacles), mexin_OZ);
            [obj.eval_output,obj.eval_grad_output] = rotatotope_mex(obj.n_links, obj.n_time_steps, mexin_R, mexin_Z, mexin_EE, length(obj.obstacles), mexin_OZ, obj.k_opt);
            
            toc;
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
        
        function [obj] = generate_constraints(obj, obstacles)
            fprintf("GENERATE CONSTRAINTS ORIGINAL TIME\n");
            tic;
            for i = 1:length(obstacles)
                for j = 1:length(obj.link_FRS)
                    for k = 1:length(obj.link_FRS{j})
                        [obj.A_con{i}{j}{k}, obj.b_con{i}{j}{k}, obj.k_con{i}{j}{k}] = obj.link_FRS{j}{k}.generate_constraints(obstacles{i}.zono.Z, obj.FRS_options, j);
                    end
                end
            end
            toc;
        end
        
    end
end