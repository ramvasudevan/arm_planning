function [] = generate_all_dynamics(t_plan,t_total)

if ~exist('rotatotopes/dynamics', 'dir')
   mkdir('rotatotopes/dynamics'); 
end

% define symbolic variable
syms cq sq real; % actual
syms cqi sqi kai kvi t real; % desired
syms ce se t real; % error
syms e0 de de0 ddee real;
syms tdummy udummy real; % CORA will require these arguments, but we won't use them.

%~~~~~~~~~~~acceleration dynamics over [0, t_plan]
% desired dynamics
q_i_dot = kvi + kai*t;
dcqi = -sqi * q_i_dot;
dsqi = cqi * q_i_dot;
dkai = 0;
dkvi = 0;

% computed torque controller gains (MLS 4.5)
eigval = -32;
Kd = eigval^2;
Kp = -2*eigval;

% solution to the error ODE
x_err = expm([0 1; -Kd -Kp] * t);

% error dynamics
de = x_err(2,1) * e0 + x_err(2,2) * de0;
dce = -se*de;
dse = ce*de;
d_e0 = 0;
d_de0 = 0;

% actual dynamics
q_dot = q_i_dot + de;
dcq = -sq * (q_dot);
dsq = cq * (q_dot);

% time
dt = 1;

x = [cqi; sqi; kai; kvi; ce; se; e0; de0; cq; sq; t];
dx = [dcqi; dsqi; dkai; dkvi; dce; dse; d_e0; d_de0; dcq; dsq; dt];
all_dyn_zero_to_t_plan = matlabFunction(dx, 'File', 'rotatotopes/dynamics/all_dyn_zero_to_t_plan', 'vars', {tdummy, x, udummy});

%~~~~~~~~~~~braking dynamics over [t_plan , t_total]
t_to_stop = t_total - t_plan;
q_i_dot_pk = kvi + kai*t_plan;
braking_acceleration = (0 - q_i_dot_pk)/t_to_stop; % brake to 0 velocity from q_i_dot_pk in t_to_stop seconds

% desired dyamics
q_i_dot = q_i_dot_pk + braking_acceleration*(t - t_plan);
dcqi = -sqi*q_i_dot;
dsqi = cqi*q_i_dot;

% error initial conditions at peak
e0_pk = subs(x_err(1,1), t_plan) * e0 + subs(x_err(1,2),t_plan) * de0;
de0_pk = subs(x_err(2,1),t_plan) * e0 + subs(x_err(2,2),t_plan) * de0;

% error dynamics 
de = x_err(2,1) * e0_pk + x_err(2,2) * de0_pk;
dce = -se*de;
dse = ce*de;
d_e0 = 0;
d_de0 = 0;

% actual dynamics
q_dot = q_i_dot + de;
dcq = -sq * (q_dot);
dsq = cq * (q_dot);

dx = [dcqi; dsqi; dkai; dkvi; dce; dse; d_e0; d_de0; dcq; dsq; dt];
all_dyn_t_plan_to_t_total = matlabFunction(dx, 'File', 'rotatotopes/dynamics/all_dyn_t_plan_to_t_total', 'vars', {tdummy, x, udummy});
end
