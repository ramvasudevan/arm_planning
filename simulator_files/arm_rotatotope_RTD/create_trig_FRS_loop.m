% playing around with matZonotopes...

clear all; clc;
figure(1); clf; hold on; axis equal;

dim = 5;

t_plan = 0.5;
t_total = 1;
dt = 0.01;

% c_IC = pi/2;
% g = pi/6;
L = 0.33;

generate_trig_dynamics(t_plan, t_total)

nBins = 401;
c_IC = linspace(-pi, pi, nBins);
binSize = c_IC(2) - c_IC(1);
g_IC = binSize/2;

% save
if ~exist('FRS_trig', 'dir')
    mkdir('FRS_trig')
end
save('FRS_trig/0key.mat', 'c_IC');

for i = 1:length(c_IC)
    options.tStart = 0;
    options.tFinal = t_plan;
    
    options.x0 = [1;0;0;c_IC(i);0];
%     const g:
%     options.R0 = zonotope([options.x0, diag([0, 0, g, g_IC, 0])]); 
    % change g depdending on IC:
    options.R0 = zonotope([options.x0, diag([0, 0, max(pi/24, abs(c_IC(i)/3)), g_IC, 0])]);
    
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
    
    sys = nonlinearSys(dim, 1, @trig_dyn_toPeak, options);
    
    %compute reachable set-----------------------------------------------------
    tic
    Rcont_toPeak = reach(sys, options);
    tComp = toc;
    
    % then use the computed FRS as the initial FRS for the braking dynamics.
    options.R0 = Rcont_toPeak{end}{1};
    % however, slice the zonotope right at the t_plan time:
    options.R0 = zonotope_slice(options.R0, [dim], t_plan);
    options.tStart = t_plan;
    options.tFinal = t_total;
    %specify 1st link to stop dynamics------------------------------------------
    sys = nonlinearSys(dim, 1, @trig_dyn_toStop, options);
    %compute reachable set-----------------------------------------------------
    tic
    Rcont_toStop = reach(sys, options);
    tComp = toc;
    
    % concatenate full FRS
    Rcont = [Rcont_toPeak; Rcont_toStop];
    
    my_c_IC = c_IC(i);
    filename = sprintf('FRS_trig/trig_FRS_%0.3f.mat', my_c_IC);
    save(filename, 'Rcont', 'options', 'L', 't_plan', 't_total', 'my_c_IC');
end
