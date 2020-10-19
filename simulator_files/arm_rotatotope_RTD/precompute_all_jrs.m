function [] = precompute_all_jrs()

realmin('double');
realmin('single');

%~~~~~~~~~ define hyperparameters for our particular parameterization:
dim = 11;
t_plan = 0.5;
t_total = 1;
dt = 0.01;

generate_all_dynamics(t_plan, t_total);

%~~~~~~~~~ params for desired trajectory zonotopes
n_JRS = 401; % separate initial velocity space (K^v_i) into 401 smaller intervals
c_kvi = linspace(-pi, pi, n_JRS); % centers of initial velocity subintervals
c_kai = 0; % acceleration parameter 
delta_kvi = (c_kvi(2) - c_kvi(1))/2; % subinterval is c_kvi +- delta_kvi

%~~~~~~~~~ params for error trajectory zonotopes
n_eJRS = 35;

% centers for error zonotopes
c_e0 = linspace(-pi/12, pi/12, n_eJRS); 
c_de0 = c_kvi(201-floor(n_eJRS/2):201+floor(n_eJRS/2));

% width of generators for error zonotopes
delta_e0 = c_e0(2) - c_e0(1);
delta_de0 = c_de0(2) - c_de0(1);

% create folder to save precomputed JRSs
if ~exist('rotatotopes/all_joint_reachable_sets', 'dir')
    mkdir('rotatotopes/all_joint_reachable_sets');
end

% save vector of initial velocity subinterval centers
save('rotatotopes/all_joint_reachable_sets/c_kvi.mat', 'c_kvi');

% save vector of initial position error subinterval centers
save('rotatotopes/all_joint_reachable_sets/c_e0.mat', 'c_e0');

% save vector of initial velocity error subinterval centers
save('rotatotopes/all_joint_reachable_sets/c_de0.mat', 'c_de0');

% set options for reachability analysis:
options.timeStep = dt;
options.taylorTerms=5; % number of taylor terms for reachable sets
options.zonotopeOrder= 2; % zonotope order... increase this for more complicated systems.
options.maxError = 1000*ones(dim, 1); % our zonotopes shouldn't be "splitting", so this term doesn't matter for now
options.verbose = 0;
options.uTrans = 0; % we won't be using any inputs, as traj. params specify trajectories
options.U = zonotope([0, 0]);
options.advancedLinErrorComp = 0;
options.tensorOrder = 1;
options.reductionInterval = inf;
options.reductionTechnique = 'girard';

for j = 1:n_JRS % compute JRS for each velocity subinterval
    options.tStart = 0; % start time
    options.tFinal = t_plan; % end time for these dynamics
    
    %~~~~~~~~~ desired trajectory params
    th_des_0 = 0; % desired initial position
    th_des_dot_0 = c_kvi(j); % desired initial velocity generator
    delta_kai = max(pi/24, abs(c_kvi(j)/3)); % desired acc generator
    
    for k = 1:n_eJRS
        for p = 1:n_eJRS
            tic;
                    
            %~~~~~~~~~ centers
            % desired 
            c_cq_des = 1; 
            c_sq_des = 0; 

            % error = th - th_des
            c_cq_e = cos(c_e0(k)); %error
            c_sq_e = sin(c_e0(k));
            
            % actual th = error + th_des
            c_cq = cos(th_des_0 + c_e0(k)); 
            c_sq = sin(th_des_0 + c_e0(k));

            % initial center
            options.x0 = [c_cq_des; c_sq_des; c_kai; c_kvi(j); c_cq_e; c_sq_e; c_e0(k); c_de0(p); c_cq; c_sq; 0];
            
            % initial zonotope =  center + generators
            options.R0 = zonotope([options.x0,... 
                                  [0; 0; delta_kai; 0; 0; 0; 0; 0; 0; 0; 0],...
                                  [0; 0; 0; delta_kvi; 0; 0; 0; 0; 0; 0; 0],...
                                  [0; 0; 0; 0; 0; 0; delta_e0; 0; 0; 0; 0],...
                                  [0; 0; 0; 0; 0; 0; 0; delta_de0; 0; 0; 0]]);

            % create system for reachability analysis (1 dummy input)
            sys = nonlinearSys(dim, 1, @all_dyn_zero_to_t_plan, options);

            % compute JRS over t \in [0, t_plan]
            JRS_zero_to_t_plan = reach(sys, options);

            % braking dynamics
            t_plan_slice = zonotope_slice(JRS_zero_to_t_plan{end}{1}, 11, t_plan);
            options.R0 = t_plan_slice;
            options.tStart = t_plan;
            options.tFinal = t_total;

            % create system for reachability analysis (1 dummy input)
            sys = nonlinearSys(dim, 1, @all_dyn_t_plan_to_t_total, options);

            % compute JRS over t \in [t_plan, t_total]
            JRS_t_plan_to_t_total = reach(sys, options);

            % TODO: Think about making this an input
%             dims = [1 2 3 4 9 10];
%             for g = 1:length(JRS_zero_to_t_plan)
%                 B = JRS_zero_to_t_plan{g}{1};
%                 B = project(B, [1 2 3 4 9 10]);
%                 Z = reduce_zono(B.Z, 5, [3,4]);
%                 Z = single(Z);
%                 JRS_zero_to_t_plan{g}{1} = zonotope(Z);
%             end
% 
%             for g = 1:length(JRS_t_plan_to_t_total)
%                 B = JRS_t_plan_to_t_total{g}{1};
%                 B = project(B, [1 2 3 4 9 10]);
%                 Z = reduce_zono(B.Z, 5, [3,4]);
%                 Z = single(Z);
%                 JRS_t_plan_to_t_total{g}{1} = zonotope(Z);
%             end

            % concatenate the full JRS
            JRS = [JRS_zero_to_t_plan; JRS_t_plan_to_t_total];

            %~~~~~~~~ save JRS
            current_c_kvi = c_kvi(j);
            current_c_e0 = c_e0(k);
            current_c_de0 = c_de0(p);      
            filename = sprintf('rotatotopes/all_joint_reachable_sets/JRS_%0.3f_%0.3f_%0.3f.mat',...
                                current_c_kvi, current_c_e0, current_c_de0);
            % save(filename, 'JRS', 'options', 't_plan', 't_total', 'current_c_kvi');
            save(filename, 'JRS');
            % display:
            fprintf('Init vel: %0.2f, Pos err: %0.2f, Vel err: %0.2f, ', current_c_kvi, current_c_e0, current_c_de0);
            toc;
         end
    end           
end
