clear; clc;

t_plan = 0.5;
t_total = 1;
dt = 0.005;

PZ_reachset_imp = load('FRS_trig_PZ_improved/trig_FRS_-2.875.mat');
PZ_reachset = load('FRS_trig_PZ/trig_FRS_-2.875.mat');
OZ_reachset = load('FRS_trig/trig_FRS_-2.875.mat');

c_IC = PZ_reachset.my_c_IC;
g_IC = pi/400;
g_k = max(pi/24, abs(c_IC/3));
g_t = dt / 2;

figure; hold on; axis equal;

% slice factor
factor = 2 * rand - 1;
OZ_factor = g_k * factor;

plot_dt = 5;

%% plot OZ and PZ
for i = 1:plot_dt:length(PZ_reachset.Rcont)
    PZ_inp_sliceRes = sliceFRS(PZ_reachset_imp.Rcont{i}, factor);
    plot(interval(PZ_inp_sliceRes), [1,2], 'Color', [0,1,0], 'Filled', false);
    
    PZ_sliceRes = sliceFRS(PZ_reachset.Rcont{i}, factor);
    plot(interval(PZ_sliceRes), [1,2], 'Color', [1,0,0], 'Filled', false);
    
%     OZ_sliceRes = zonotope_slice(OZ_reachset.Rcont{i}, 3, factor);
%     plot(OZ_sliceRes, [1,2], 'Color', [0,0,1], 'Filled', false);

%     plot(PZ_reachset.Rcont{i}, [1,2], 'Color', [0,1,0], 'Filled', false);
%     plot(OZ_reachset.Rcont{i}, [1,2], 'Color', [0,0,1], 'Filled', false);
end

%% plot real trajectories
% to peak
c_t = (dt / 2) : plot_dt*dt : (t_plan - dt / 2);

for i = c_t
    samplet = (i - g_t) : 1e-5 : (i + g_t);
    traj = c_IC * samplet + 0.5 * OZ_factor * samplet.^2;
    plot(cos(traj), sin(traj), 'k');
end

% braking
c_t = (t_plan + dt / 2) : plot_dt*dt : (t_total - dt / 2);

for i = c_t
    samplet = (i - g_t) : 1e-5 : (i + g_t);
    traj = -0.25 * c_IC - 0.25 * OZ_factor + (2 * c_IC + OZ_factor) * samplet - 0.5 * (2 * c_IC + OZ_factor) * samplet.^2;
    plot(cos(traj), sin(traj), 'k');
end

