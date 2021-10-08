%%% description
% this code uses CORA 2021 to generate the "joint reachable sets" of
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
dt = 0.005;

L = 0.33;

% generate_trig_dynamics(t_plan, t_total) % generates the dynamics parameterized by k

nBins = 801;
c_IC = linspace(-pi, pi, nBins);
binSize = c_IC(2) - c_IC(1);
g_IC = binSize / 2;

% save
if ~exist('FRS_trig_PZ_improved', 'dir')
    mkdir('FRS_trig_PZ_improved')
end
save('FRS_trig_PZ_improved/0key.mat', 'c_IC');

polyZono.maxDepGenOrder = 50;
polyZono.maxPolyZonoRatio = 0.1;
polyZono.restructureTechnique = 'reducePca';

options.timeStep = dt;
options.taylorTerms = 15; %number of taylor terms for reachable sets
options.zonotopeOrder = 40; %zonotope order... increase this for more complicated systems.
options.intermediateOrder = 40;
options.errorOrder = 40;
options.maxError = 1000*ones(dim, 1);
options.polyZono = polyZono;
options.verbose = true;

options.alg = 'poly';
options.tensorOrder = 3;
options.reductionInterval = inf;
options.reductionTechnique = 'girard';

for i = 1:nBins % we're going to loop over all velocity intervals
    disp([num2str(i), '/', num2str(length(c_IC))])
    
    % change g depdending on IC:
    g_k = max(pi/24, abs(c_IC(i)/3));
    
    Rcont = cell(t_total/dt,1);
    inc = 1;
    
    for tStart = 0:dt:(t_plan - dt)
        params.tStart = tStart;
        params.tFinal = tStart + dt;

        x0 = [1;0;0;c_IC(i);0];

        % q = q_dot_0 * t + 0.5 * q_ddot * t^2
        if tStart > 0
            q_dot_0 = c_IC(i) + interval(-g_IC, g_IC);
            q_ddot = polyZonotope(0, g_k, [], sparse(1));
            q_t = q_dot_0 * tStart + 0.5 * q_ddot * tStart^2;
            cos_q_t = cos(q_t);
            sin_q_t = sin(q_t);
            cossinqt = exactCartProd(cos_q_t, sin_q_t); % [cos(q(t)); sin(q(t))]
        else
            cossinqt = [1;0];
        end
        kICt = polyZonotope([0; c_IC(i); tStart], [g_k; 0; 0], [0; g_IC; 0], sparse(1)); % [k; IC; t];
        params.R0 = exactCartProd(cossinqt, kICt);

        sys = nonlinearSys(@trig_dyn_toPeak, dim, 1);

        %compute reachable set-----------------------------------------------------
        Rcont_toPeak = reach(sys, params, options);

        % concatenate full FRS
        Rcont{inc} = reduceFactorsFull(Rcont_toPeak.timeInterval.set{1}, 1);
        inc = inc + 1;
    end
    
    for tStart = t_plan:dt:(t_total - dt)
        params.tStart = tStart;
        params.tFinal = tStart + dt;
        
        q_dot_0 = c_IC(i) + interval(-g_IC, g_IC);
        q_ddot = polyZonotope(0, g_k, [], sparse(1));
        q_t = (-0.25 + 2 * tStart - tStart^2) * q_dot_0 + (-0.25 + tStart - 0.5 * tStart^2) * q_ddot;
        
        cos_q_t = cos(q_t);
        sin_q_t = sin(q_t);
        cossinqt = exactCartProd(cos_q_t, sin_q_t); % [cos(q(t)); sin(q(t))]
        
        kICt = polyZonotope([0; c_IC(i); tStart], [g_k; 0; 0], [0; g_IC; 0], sparse(1)); % [k; IC; t];
        params.R0 = exactCartProd(cossinqt, kICt);
        
        %specify the "to stop" aka braking dynamics----------------------------
        sys = nonlinearSys(@trig_dyn_toStop, dim, 1);
        %compute reachable set-------------------------------------------------
        Rcont_toStop = reach(sys, params, options);
        
        % concatenate full FRS
        Rcont{inc} = reduceFactorsFull(Rcont_toStop.timeInterval.set{1}, 1);
        inc = inc + 1;
    end
    
    for j = 1:length(Rcont)
        Rcont{j} = getDim(Rcont{j}, [1:2]);
        Rcont{j}.Grest = diag(sum(abs(Rcont{j}.Grest),2));
    end
    
    % save this FRS
    my_c_IC = c_IC(i);
    filename = sprintf('FRS_trig_PZ_improved/trig_FRS_%0.3f.mat', my_c_IC);
    save(filename, 'Rcont', 'options', 'L', 't_plan', 't_total', 'my_c_IC', 'g_k');
end
