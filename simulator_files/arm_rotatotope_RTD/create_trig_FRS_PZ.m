%%% description
% this code uses CORA 2018 to generate the "joint reachable sets" of
% cosines and sines that are parameterized by a parameter k
% in this case, we're solving for reachable sines and cosines over a range
% of initial velocities, and saving many reach sets corresponding to each
% range of velocities. we are also varying the set K from which k is drawn,
% depending on the set of initial velocities.

clear; clc;
% figure(1); clf; hold on; axis equal;

dim = 5; % dim 1 cosine, dim 2 sine, dim 3 K, dim 4 initial vel., dim 5 time

t_plan = 0.5;
t_total = 1;
dt = 0.01;

L = 0.33;

generate_trig_dynamics(t_plan, t_total) % generates the dynamics parameterized by k

% nBins = 401; % separate initial velocity space into 401 smaller intervals
% c_IC = linspace(-pi, pi, nBins); % initial velocity in [-pi, pi]
% binSize = c_IC(2) - c_IC(1);
% g_IC = binSize/2;

nBins = 401;
c_IC = linspace(-pi, pi, nBins);
binSize = c_IC(2) - c_IC(1);
g_IC = binSize / 2;

% save
if ~exist('FRS_trig_PZ', 'dir')
    mkdir('FRS_trig_PZ')
end
save('FRS_trig_PZ/0key.mat', 'c_IC');

for i = 139:length(c_IC) % we're going to loop over all velocity intervals
    disp([num2str(i), '/', num2str(length(c_IC))])
    
    params.tStart = 0;
    params.tFinal = t_plan;
    
    params.x0 = [1;0;0;c_IC(i);0];
%     const g:
%     options.R0 = zonotope([options.x0, diag([0, 0, g, g_IC, 0])]); 
    % change g depdending on IC:
%     options.R0 = zonotope([options.x0, diag([0, 0, max(pi/24, abs(c_IC(i)/3)), g_IC, 0])]);
    
    % q = q_dot_0 * t + 0.5 * q_ddot * t^2
    params.R0 = polyZonotope(params.x0, [0 0; 0 0; max(pi/24, abs(c_IC(i)/3)) 0; 0 g_IC; 0 0], [], sparse([1, 0; 0, 1]));
    
    polyZono.maxDepGenOrder = 50;
    polyZono.maxPolyZonoRatio = 0.1;
    polyZono.restructureTechnique = 'reducePca';
    
    options.timeStep = dt;
    options.taylorTerms = 15; %number of taylor terms for reachable sets
    options.zonotopeOrder = 40; %zonotope order... increase this for more complicated systems.
    options.intermediateOrder = 40;
    options.errorOrder = 40;
    options.maxError = 1000*ones(dim, 1);
    options.maxError_x = options.maxError;
    options.maxError_y = 5000;
    options.polyZono = polyZono;
    options.verbose = true;
    
    options.uTrans = 0;
    options.U = zonotope([0, 0]);
    
    options.alg = 'poly';
    options.tensorOrder = 3;
    options.reductionInterval = inf;
    options.reductionTechnique = 'girard';
    
    sys = nonlinearSys(@trig_dyn_toPeak, dim, 1);
    
    %compute reachable set-----------------------------------------------------
    tic
    Rcont_toPeak = reach(sys, params, options);
    tComp = toc;
    
    % then use the computed FRS as the initial FRS for the braking dynamics.
    params.R0 = Rcont_toPeak.timePoint.set{end};
    % however, slice the zonotope right at the t_plan time:
%     options.R0 = zonotope_slice(options.R0, [dim], t_plan);
%     params.R0 = sliceFRS(options.R0, [dim], t_plan);
    params.tStart = t_plan;
    params.tFinal = t_total;
    %specify the "to stop" aka braking dynamics----------------------------
    sys = nonlinearSys(@trig_dyn_toStop, dim, 1);
    %compute reachable set-------------------------------------------------
    tic
    Rcont_toStop = reach(sys, params, options);
    tComp = toc;
    
    % concatenate full FRS
    Rcont = [Rcont_toPeak.timeInterval.set; Rcont_toStop.timeInterval.set];
    
    for j = 1:length(Rcont)
        Rcont{j} = reduceFactorsFull(Rcont{j});
    end
    
    % save this FRS
    my_c_IC = c_IC(i);
    filename = sprintf('FRS_trig_PZ/trig_FRS_%0.3f.mat', my_c_IC);
    save(filename, 'Rcont', 'options', 'L', 't_plan', 't_total', 'my_c_IC');
end
