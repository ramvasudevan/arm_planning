% create arm FRS's with braking maneuver included.
% discretize along initial angular velocity, so that the correct FRSs for
% that velocity can generated on the fly.

% updated 08072019 for 3D FRS's
% update 08092019 trying to remove first chunk of link;

clear all; clc;

fulllinkon = 1;

l1 = 0.33; % approximately correct for fetch links

t_plan = 0.5;
t_total = 1;
g_K = [pi/12; pi/12]; % generate FRS over a pi/6 rad/s sec peak speed interval

generate_arm_dynamics_3D(t_plan, t_total);
dt = 0.01;
dim = 10;

nBins = 401;
% binSize = (pi - -pi)/nBins;
% c_theta_dot_0 = 0 + binSize/2:binSize:pi/2;
% c_theta_dot_0 = -pi:binSize:pi;
c_theta_dot_0 = linspace(-pi, pi, nBins);
binSize = c_theta_dot_0(2) - c_theta_dot_0(1);
g_theta_dot_0 = binSize/2;

c_IC = [c_theta_dot_0;zeros(size(c_theta_dot_0))]; % for now, initial angular velocity of 0 for each DOF
g_IC = [binSize; pi/200]; % small range of initial angular velocity

%% save
if ~exist('FRS_3D', 'dir')
    mkdir('FRS_3D')
end
save('FRS_3D/0key.mat', 'c_IC');

for i = 1:size(c_IC, 2)
    % Compute FRS of link and FRS of end effector
    %set options
    options.tStart = 0;
    options.tFinal = t_plan;
    a1 = 1*l1/2;
    a2 = l1 - a1;
    options.x0 = zeros(dim, 1) + [a1; 0; 0; 0; 1; c_IC(1, i); c_IC(2, i); c_IC(1, i); c_IC(2, i); 0];
    options.R0 = zonotope([options.x0, diag([a2, 0, 0, 0, 0, g_IC(1), g_IC(2), g_K(1), g_K(2), 0])]); % theta_dot between -pi/400 and pi/400
    options.R0_orig = options.R0;
    options.timeStep = dt;
    options.taylorTerms=5; %number of taylor terms for reachable sets
    options.zonotopeOrder= 1.5; %zonotope order... increase this for more complicated systems.
    options.maxError = 1000*ones(dim, 1);
    options.verbose = 1;
    
    options.uTrans = 0;
    options.U = zonotope([0, 0]);
    
    options.advancedLinErrorComp = 0;
    options.tensorOrder = 1;
    options.reductionInterval = inf;
    options.reductionTechnique = 'girard';
    
    if fulllinkon
        %specify 1st link toPeak dynamics------------------------------------------
        sys_l1 = nonlinearSys(dim, 1, @arm_dyn_toPeak_3D, options);
        %compute reachable set-----------------------------------------------------
        tic
        Rcont_toPeak = reach(sys_l1, options);
        tComp = toc;
        
        % then use the computed FRS as the initial FRS for the braking dynamics.
        options.R0 = Rcont_toPeak{end}{1};
        % however, slice the zonotope right at the t_plan time:
        options.R0 = zonotope_slice(options.R0, [dim], t_plan);
        options.tStart = t_plan;
        options.tFinal = t_total;
        %specify 1st link toPeak dynamics------------------------------------------
        sys_l1 = nonlinearSys(dim, 1, @arm_dyn_toStop_3D, options);
        %compute reachable set-----------------------------------------------------
        tic
        Rcont_toStop = reach(sys_l1, options);
        tComp = toc;
        
        % concatenate full FRS
        Rcont = [Rcont_toPeak; Rcont_toStop];
    end
    
    %% now, do the same thing but just for the end effector:
    % Compute FRS of link and FRS of end effector
    %set options
    options.tStart = 0;
    options.tFinal = t_plan;
    options.x0 = zeros(dim, 1) + [l1; 0; 0; 0; 1; c_IC(1, i); c_IC(2, i); c_IC(1, i); c_IC(2, i); 0];
    options.R0 = zonotope([options.x0, diag([0, 0, 0, 0, 0, g_IC(1), g_IC(2), g_K(1), g_K(2), 0])]); % theta_dot between -pi/400 and pi/400
    options.R0EE_orig = options.R0;
    
    %specify 1st link toPeak dynamics------------------------------------------
    sys_l1 = nonlinearSys(dim, 1, @arm_dyn_toPeak_3D, options);
    %compute reachable set-----------------------------------------------------
    tic
    RcontEE_toPeak = reach(sys_l1, options);
    tComp = toc;
    
    % then use the computed FRS as the initial FRS for the braking dynamics.
    options.R0 = RcontEE_toPeak{end}{1};
    % however, slice the zonotope right at the t_plan time:
    options.R0 = zonotope_slice(options.R0, [dim], t_plan);
    options.tStart = t_plan;
    options.tFinal = t_total;
    %specify 1st link toPeak dynamics------------------------------------------
    sys_l1 = nonlinearSys(dim, 1, @arm_dyn_toStop_3D, options);
    %compute reachable set-----------------------------------------------------
    tic
    RcontEE_toStop = reach(sys_l1, options);
    tComp = toc;
    
    % concatenate full FRS
    RcontEE = [RcontEE_toPeak; RcontEE_toStop];
    
    filename = sprintf('FRS_3D/arm_FRS_%0.3f_%0.3f.mat', c_IC(1, i), c_IC(2, i));
    my_c_IC = c_IC(:, i);
    my_g_IC = g_IC;
    if fulllinkon
        save(filename, 'Rcont', 'RcontEE', 'options', 'l1', 't_plan', 't_total', 'my_c_IC', 'my_g_IC');
    else
        save(filename, 'RcontEE', 'options', 'l1', 't_plan', 't_total', 'my_c_IC', 'my_g_IC');
    end
    
end