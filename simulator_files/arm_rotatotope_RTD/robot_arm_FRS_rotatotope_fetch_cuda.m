classdef robot_arm_FRS_rotatotope_fetch_cuda
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
        
        q;
        q_dot;
        q_des;
        
        FRS_options = {};
        
        obstacles = [];
        k_opt = [];
        
        RZ = [];
        c_idx = [];
        k_idx = [];
        
        A_con = [];
        d_con = [];
        delta_con = [];
        k_con = [];
        
        A_con_self = [];
        d_con_self = [];
        delta_con_self = [];
        k_con_self = [];
        
        eval_output = [];
        eval_grad_output = [];
        eval_hess_output = [];
        
        mex_res = [];
    end
    
    methods
        function obj = robot_arm_FRS_rotatotope_fetch_cuda(q, q_dot, q_des, obstacles, k_opt, FRS_options)
            %robot_arm_FRS_rotatotope_fetch constructs an FRS for the full arm
            % based on rotatotopes. this class is specific to the Fetch,
            % and will create an FRS using default link lengths and
            % rotation axis order.
            % the constructor just requires the current state and velocity
            % of the robot.
            
            obj.q = q;
            obj.q_dot = q_dot;
            obj.q_des = q_des;
            obj.obstacles = obstacles;
            obj.k_opt = k_opt;
            
            FRSkeytmp = load([obj.FRS_path, '0key.mat']); % fix this
            obj.FRS_key = FRSkeytmp.c_IC;
            
            if ~exist('FRS_options', 'var')
                obj.FRS_options.combs = generate_combinations_upto(200);
                obj.FRS_options.maxcombs = 200;
                obj.FRS_options.buffer_dist = 0;
            else
                obj.FRS_options = FRS_options;
            end
            
            % create mex FRS
            obj = obj.create_mex_FRS();
        end
        
        function obj = create_mex_FRS(obj)
            % this function loads the appropriate trig FRS's given q_dot,
            % rotates them by q, then constructs and stacks rotatotopes at
            % each time step
            %trig_FRS = cell(length(obj.q_dot), 1);
            mexin_R = [];
            for i = 1:length(obj.q_dot)
                [~, closest_idx] = min(abs(obj.q_dot(i) - obj.FRS_key));
                filename = sprintf('%strig_FRS_%0.3f.mat', obj.FRS_path, obj.FRS_key(closest_idx));
                trig_FRS_load_tmp = load(filename);
                trig_FRS_load{i} = trig_FRS_load_tmp.Rcont;
                A = [cos(obj.q(i)), -sin(obj.q(i)), 0, 0, 0; sin(obj.q(i)), cos(obj.q(i)), 0, 0, 0;...
                    0, 0, 1, 0, 0; 0, 0, 0, 1, 0; 0, 0, 0, 0, 1];
                for j = 1:length(trig_FRS_load{i})
                    trig_FRS = A*zonotope_slice(trig_FRS_load{i}{j}{1}, 4, obj.q_dot(i));
                    mexin_R = [mexin_R, trig_FRS.Z(:,1:10)];
                end
            end
            obj.n_time_steps = length(trig_FRS);
            
            fprintf("cuda time\n");
            tic;
            
            % promise that the sizes of obstacles are the same, <= 10
            mexin_OZ = [];
            for i = 1:length(obj.obstacles)
                mexin_OZ = [mexin_OZ, obj.obstacles{i}.zono.Z]; 
            end
           
            %[obj.mex_res, obj.RZ, obj.c_idx, obj.k_idx, obj.A_con, obj.d_con, obj.delta_con, obj.k_con, obj.eval_output, obj.eval_grad_output, obj.eval_hess_output, obj.A_con_self, obj.d_con_self, obj.delta_con_self, obj.k_con_self] = rotatotope_mex(mexin_R, length(obj.obstacles), mexin_OZ, obj.k_opt, obj.q, obj.q_dot, obj.q_des);
            obj.mex_res = rotatotope_mex(mexin_R, length(obj.obstacles), mexin_OZ, obj.k_opt, obj.q, obj.q_dot, obj.q_des);
            toc;
        end
    end
end