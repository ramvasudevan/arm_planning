clear; clc;

t_plan = 0.5;
t_total = 1;
dt = 0.01;

c_IC = 187/200*pi;
g_IC = pi/400;
g_k = max(pi/24, abs(c_IC/3));
g_t = dt / 2;

PZ_reachset = load('FRS_trig_PZ/trig_FRS_2.937.mat');
OZ_reachset = load('FRS_trig/trig_FRS_2.937.mat');

figure; hold on;

factor = 1.8*rand - 0.9;

%% to peak
% plot OZ and PZ
for i = [1:5:50]
%     PZ_sliceRes = sliceFRS(PZ_reachset.Rcont(1).timeInterval.set{i}, [factor;0]);
%     plot((PZ_sliceRes), [1,2], 'Color', [0,1,0], 'Filled', false);
    
    OZ_sliceRes = zonotope_slice(OZ_reachset.Rcont{i}, 3, factor);
    plot(OZ_sliceRes, [1,2], 'Color', [0,0,1], 'Filled', false);
end

% plot PZ interval
c_t = (dt / 2) : 5*dt : (t_plan - dt / 2);

for i = c_t
    slicedTraj = c_IC * i + 0.5 * factor * i^2;
    etg = (interval(c_IC - g_IC, c_IC + g_IC) + interval(-g_k, g_k) * i) * interval(-g_t, g_t) + 0.5 * interval(-g_k, g_k) * interval(-g_t, g_t)^2;
    Etg = sin(etg);
    PZ_interval = zonotope([cos(slicedTraj + Etg.center); sin(slicedTraj + Etg.center)], Etg.rad * eye(2));
    plot(PZ_interval, [1,2], 'Color', [1,0,0], 'Filled', false);
end

% plot real trajectories
for i = c_t
    samplet = (i - g_t) : 1e-4 : (i + g_t);
    traj = c_IC * samplet + 0.5 * factor * samplet.^2;
    plot(cos(traj), sin(traj), 'k');
end

%% braking
% plot OZ and PZ
for i = [51:5:100]
%     PZ_sliceRes = sliceFRS(PZ_reachset.Rcont(2).timeInterval.set{i - 50}, [factor;0]);
%     plot((PZ_sliceRes), [1,2], 'Color', [0,1,0], 'Filled', false);
    
    OZ_sliceRes = zonotope_slice(OZ_reachset.Rcont{i}, 3, factor);
    plot(OZ_sliceRes, [1,2], 'Color', [0,0,1], 'Filled', false);
end

c_t = (t_plan + dt / 2) : 5*dt : (t_total - dt / 2);

% plot PZ interval
for i = c_t
    slicedTraj = -0.25 * c_IC - 0.25 * factor + (2 * c_IC + factor) * i - 0.5 * (2 * c_IC + factor) * i^2;
    etg = (2 * g_IC + g_k) * ((1 - i) * g_t - 0.5 * g_t^2);
    Etg = sin(etg);
    PZ_interval = zonotope([cos(slicedTraj); sin(slicedTraj)], Etg * eye(2));
    plot(PZ_interval, [1,2], 'Color', [1,0,0], 'Filled', false);
end

% plot real trajectories
for i = c_t
    samplet = (i - g_t) : 1e-4 : (i + g_t);
    traj = -0.25 * c_IC - 0.25 * factor + (2 * c_IC + factor) * samplet - 0.5 * (2 * c_IC + factor) * samplet.^2;
    plot(cos(traj), sin(traj), 'k');
end

axis equal
% axis([0,1.5,0,1.5]);