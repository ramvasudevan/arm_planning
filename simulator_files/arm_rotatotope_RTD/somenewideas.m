clear; clc;

%% initialization
if ~exist('newideas','dir')
    mkdir('newideas');
end

syms p0 v0 t tmid te k 'real'

dt = 1 / 128;

global k_order;
global t_order;

k_order = 2;
t_order = 1;

%% compute reachable sets
% stage 1
traj_1 = p0 + v0 * t + 0.5 * k * t.^2;

cos_traj_1 = cos(traj_1);
cos_traj_1_poly = expandkandt(cos_traj_1, 'cos_traj_1');
sin_traj_1 = sin(traj_1);
sin_traj_1_poly = expandkandt(sin_traj_1, 'sin_traj_1');

% stage 2
traj_2 = p0 - 0.25 * v0 - 0.25 * k + (2 * v0 + k) * t - 0.5 * (2 * v0 + k) * t.^2;

cos_traj_2 = cos(traj_2);
cos_traj_2_poly = expandkandt(cos_traj_2, 'cos_traj_2');
sin_traj_2 = sin(traj_2);
sin_traj_2_poly = expandkandt(sin_traj_2, 'sin_traj_2');

%% validation
figure; hold on;

p0_v = 0;
v0_v = pi;
k_range = interval(-v0_v/3,v0_v/3);
k_v = randPoint(k_range);

cos_traj_1_poly = subs(cos_traj_1_poly, [p0;v0;k], [p0_v;v0_v;k_v]);
sin_traj_1_poly = subs(sin_traj_1_poly, [p0;v0;k], [p0_v;v0_v;k_v]);

for i = 1:5:64
    tmid_v = (i - 0.5) * dt;
    t_range = tmid_v + interval(-dt/2,dt/2);
%     cos_error_bound = cos_traj_1_d_k_LagrangeRemainder(k_range, p0_v, t_range, v0_v) * k_range^k_order + cos_traj_1_d_t_LagrangeRemainder(k_range, p0_v, t_range, v0_v) * interval(-dt/2,dt/2)^t_order;
%     sin_error_bound = sin_traj_1_d_k_LagrangeRemainder(k_range, p0_v, t_range, v0_v) * k_range^k_order + sin_traj_1_d_t_LagrangeRemainder(k_range, p0_v, t_range, v0_v) * interval(-dt/2,dt/2)^t_order;   
    cos_error_bound = cos_traj_1_d_k_LagrangeRemainder(k_range, p0_v, tmid_v, v0_v) * k_range^k_order;
    sin_error_bound = sin_traj_1_d_k_LagrangeRemainder(k_range, p0_v, tmid_v, v0_v) * k_range^k_order;
    cos_center = double(subs(cos_traj_1_poly, t, tmid_v));
    sin_center = double(subs(sin_traj_1_poly, t, tmid_v));
    cossin_reachable_set = [cos_center; sin_center] + [cos_error_bound;sin_error_bound];
    plot(cossin_reachable_set, [1,2], 'Color', [1,0,0], 'Filled', false);

    % compare with real trajectories
%     samplet = (tmid_v - dt/2) : 1e-5 : (tmid_v + dt/2);
    samplet =  tmid_v;
    traj = p0_v + v0_v * samplet + 0.5 * k_v * samplet.^2;
    plot(cos(traj), sin(traj), 'k*');
end


% % braking
% c_t = (t_plan + dt / 2) : plot_dt*dt : (t_total - dt / 2);
% 
% for i = c_t
%     samplet = (i - g_t) : 1e-5 : (i + g_t);
%     traj = -0.25 * c_IC - 0.25 * OZ_factor + (2 * c_IC + OZ_factor) * samplet - 0.5 * (2 * c_IC + OZ_factor) * samplet.^2;
%     plot(cos(traj), sin(traj), 'k');
% end


function t_poly = expandkandt(traj, filename)
    global k_order;
    global t_order;

    syms k t tmid te 'real'

    % perform Taylor expansion at k = 0
    % g(k,x) = g(0,x) + g'(0,x) * k + g''(0,x) * k^2 / 2 + ... + g^(n)(0,x) * k^n / n!
    % where x is other variables
    traj_d_k = getHighOrderDerivatives(traj, k, k_order);
    
    % export Lagrange remainder of k and use interval arithmetic to bound it
    matlabFunction(traj_d_k(end) / factorial(k_order), 'File', ['newideas/',filename,'_d_k_LagrangeRemainder.m']);

    traj_d_k = subs(traj_d_k, k, 0);
    
    % use Taylor terms to rewrite the expression as a polynomial with respect to k
    traj_1_k_poly = 0;
    for i = 1:k_order
        traj_1_k_poly = traj_1_k_poly + traj_d_k(i) * k^(i-1) / factorial(i);
    end
    
    % perform the same procedure over t to save it from cos or sin
    traj_d_t = getHighOrderDerivatives(traj_1_k_poly, t, t_order);
    
    % export Lagrange remainder of k and use interval arithmetic to bound it
    matlabFunction(traj_d_t(end) / factorial(t_order), 'File', ['newideas/',filename,'_d_t_LagrangeRemainder.m']);

    traj_d_t = subs(traj_d_t, t, tmid);
    
    % use Taylor terms to rewrite the expression as a polynomial with respect to t
    t_poly = 0;
    for i = 1:t_order
        t_poly = t_poly + traj_d_t(i) * te^(i-1) / factorial(i);
    end

    t_poly = traj_1_k_poly;
end

function derivatives = getHighOrderDerivatives(expr, var, order)
    derivatives = sym(zeros(1,order+1));
    derivatives(1) = expr;
    for i = 1:order
        derivatives(i+1) = diff(derivatives(i), var);
    end
end