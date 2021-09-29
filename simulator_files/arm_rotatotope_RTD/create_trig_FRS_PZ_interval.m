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

for i = 1:length(c_IC) % we're going to loop over all velocity intervals
    params.tStart = 0;
    params.tFinal = t_plan;
    
    params.x0 = [1;0;0;c_IC(i);0];
    
    % q = q_dot_0 * t + 0.5 * q_ddot * t^2
    params.R0 = polyZonotope(params.x0, [0 0; 0 0; max(pi/24, abs(c_IC(i)/3)) 0; 0 g_IC; 0 0], [], sparse([1, 0; 0, 1]));
    
    g_k = max(pi/24, abs(c_IC(i)/3));
    c_t = (dt / 2) : dt : (t_plan - dt / 2);
    g_t = dt / 2;
    etg = (g_IC + g_k * c_t) * g_t + 0.5 * g_k * g_t^2;
    Etg = sin(etg);
    
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
    Rcont = [Rcont_toPeak; Rcont_toStop];
    
    % save this FRS
    my_c_IC = c_IC(i);
    filename = sprintf('FRS_trig_PZ/trig_FRS_%0.3f.mat', my_c_IC);
    save(filename, 'Rcont', 'options', 'L', 't_plan', 't_total', 'my_c_IC');
end
