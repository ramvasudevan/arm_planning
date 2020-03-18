% create arm FRS's with braking maneuver included.
% discretize along initial angular velocity, so that the correct FRSs for
% that velocity can generated on the fly.

clear all; clc;

l1 = 1;

t_plan = 0.5;
t_total = 1;
g_theta_dot_pk = pi/12; % generate FRS over a pi/6 rad/s sec peak speed interval

generate_arm_wBraking_dynamics(t_plan, t_total);
dt = 0.01;
dim = 5;

nBins = 401;
% binSize = (pi - -pi)/nBins;
% c_theta_dot_0 = 0 + binSize/2:binSize:pi/2;
% c_theta_dot_0 = -pi:binSize:pi;
c_theta_dot_0 = linspace(-pi, pi, nBins);
binSize = c_theta_dot_0(2) - c_theta_dot_0(1);
g_theta_dot_0 = binSize/2;

%% save
if ~exist('FRS', 'dir')
    mkdir('FRS')
end
save('FRS/0key.mat', 'c_theta_dot_0');

for i = 1:length(c_theta_dot_0)
    % Compute FRS of link and FRS of end effector
    %set options
    options.tStart = 0;
    options.tFinal = t_plan;
    options.x0 = zeros(dim, 1) + [l1/2; 0; c_theta_dot_0(i); c_theta_dot_0(i); 0];
    options.R0 = zonotope([options.x0, diag([l1/2, 0, g_theta_dot_0, g_theta_dot_pk, 0])]); % theta_dot between -pi/400 and pi/400
    options.timeStep = dt;
    options.taylorTerms=5; %number of taylor terms for reachable sets
    options.zonotopeOrder= 2; %zonotope order... increase this for more complicated systems.
    options.maxError = 1000*ones(dim, 1);
    options.verbose = 1;
    
    options.uTrans = 0;
    options.U = zonotope([0, 0]);
    
    options.advancedLinErrorComp = 0;
    options.tensorOrder = 1;
    options.reductionInterval = inf;
    options.reductionTechnique = 'girard';
    
    %specify 1st link toPeak dynamics------------------------------------------
    sys_l1 = nonlinearSys(dim, 1, @arm_dyn_toPeak, options);
    %compute reachable set-----------------------------------------------------
    tic
    Rcont_toPeak = reach(sys_l1, options);
    tComp = toc;
    
    % then use the computed FRS as the initial FRS for the braking dynamics.
    options.R0 = Rcont_toPeak{end}{1};
    % however, slice the zonotope right at the t_plan time:
    options.R0 = zonotope_slice(options.R0, [5], t_plan);
    options.tStart = t_plan;
    options.tFinal = t_total;
    %specify 1st link toPeak dynamics------------------------------------------
    sys_l1 = nonlinearSys(dim, 1, @arm_dyn_toStop, options);
    %compute reachable set-----------------------------------------------------
    tic
    Rcont_toStop = reach(sys_l1, options);
    tComp = toc;
    
    % concatenate full FRS
    Rcont = [Rcont_toPeak; Rcont_toStop];
    
    %% now, do the same thing but just for the end effector:
    % Compute FRS of link and FRS of end effector
    %set options
    options.tStart = 0;
    options.tFinal = t_plan;
    options.x0 = zeros(dim, 1) + [l1; 0; c_theta_dot_0(i); c_theta_dot_0(i); 0];
    options.R0 = zonotope([options.x0, diag([0, 0, g_theta_dot_0, g_theta_dot_pk, 0])]); % theta_dot between -pi/200 and pi/200
    
    %specify 1st link toPeak dynamics------------------------------------------
    sys_l1 = nonlinearSys(dim, 1, @arm_dyn_toPeak, options);
    %compute reachable set-----------------------------------------------------
    tic
    RcontEE_toPeak = reach(sys_l1, options);
    tComp = toc;
    
    % then use the computed FRS as the initial FRS for the braking dynamics.
    options.R0 = RcontEE_toPeak{end}{1};
    % however, slice the zonotope right at the t_plan time:
    options.R0 = zonotope_slice(options.R0, [5], t_plan);
    options.tStart = t_plan;
    options.tFinal = t_total;
    %specify 1st link toPeak dynamics------------------------------------------
    sys_l1 = nonlinearSys(dim, 1, @arm_dyn_toStop, options);
    %compute reachable set-----------------------------------------------------
    tic
    RcontEE_toStop = reach(sys_l1, options);
    tComp = toc;
    
    % concatenate full FRS
    RcontEE = [RcontEE_toPeak; RcontEE_toStop];
    
    filename = sprintf('FRS/arm_FRS_%0.3f.mat', c_theta_dot_0(i));
    my_c_theta_dot_0 = c_theta_dot_0(i);
    my_g_theta_dot_0 = c_theta_dot_0(i);
    save(filename, 'Rcont', 'RcontEE', 'options', 'l1', 't_plan', 't_total', 'my_c_theta_dot_0', 'my_g_theta_dot_0');
    
end