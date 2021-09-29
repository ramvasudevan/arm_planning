clear; clc;

t_plan = 0.5;
t_total = 1;
dt = 0.01;

c_IC = 187/200*pi;

PZ_reachset = load('FRS_trig_PZ/trig_FRS_2.937.mat');
OZ_reachset = load('FRS_trig/trig_FRS_2.937.mat');

figure; hold on;

factor = 1.8*rand - 0.9;

for i = [1:5:length(PZ_reachset.Rcont(1).timeInterval.set),length(PZ_reachset.Rcont(1).timeInterval.set)]
%     PZ_sliceRes = sliceFRS(scaleFRS(PZ_reachset.Rcont(1).timeInterval.set{i}, 2, -1, 1), factor);
%     PZ_sliceRes = sliceFRS(PZ_reachset.Rcont(1).timeInterval.set{i}, [factor;0]);
%     plot((PZ_sliceRes), [1,2], 'Color', [1,0,0], 'Filled', false);
    
    OZ_sliceRes = zonotope_slice(OZ_reachset.Rcont(1).timeInterval.set{i}, 3, factor);
    plot(OZ_sliceRes, [1,2], 'Color', [0,0,1], 'Filled', false);
end

g_IC = pi/400;
g_k = max(pi/24, abs(c_IC/3));
c_t = (dt / 2) : 5*dt : (t_plan - dt / 2);
g_t = dt / 2;

for i = c_t
    slicedTraj = c_IC * i + 0.5 * factor * i^2;
    etg = (g_IC + g_k * i) * g_t + 0.5 * g_k * g_t^2;
    Etg = sin(etg);
    PZ_interval = zonotope([cos(slicedTraj); sin(slicedTraj)], Etg * eye(2));
    plot(PZ_interval, [1,2], 'Color', [1,0,0], 'Filled', false);
end

% for i = [1:5:length(PZ_reachset.Rcont(2).timeInterval.set),length(PZ_reachset.Rcont(2).timeInterval.set)]
%     PZ_sliceRes = sliceFRS(PZ_reachset.Rcont(2).timeInterval.set{i}, [factor;0]);
%     plot((PZ_sliceRes), [1,2], 'Color', [1,0,0], 'Filled', false);
%     
%     OZ_sliceRes = zonotope_slice(OZ_reachset.Rcont(2).timeInterval.set{i}, 3, factor);
%     plot(OZ_sliceRes, [1,2], 'Color', [0,0,1], 'Filled', false);
% end

axis equal
% axis([0,1.5,0,1.5]);