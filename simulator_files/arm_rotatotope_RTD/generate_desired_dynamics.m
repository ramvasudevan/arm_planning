function [] = generate_desired_dynamics(t_plan,t_total)

if ~exist('rotatotopes/dynamics', 'dir')
   mkdir('rotatotopes/dynamics'); 
end

% define symbolic variable
syms cqi sqi kai kvi t real; % desired
syms tdummy udummy real; % CORA will require these arguments, but we won't use them.

%~~~~~~~~~~~acceleration dynamics over [0, t_plan]
% desired dynamics
q_i_dot = kvi + kai*t;
dcqi = -sqi * q_i_dot;
dsqi = cqi * q_i_dot;
dkai = 0;
dkvi = 0;

% time
dt = 1;

x = [cqi; sqi; kai; kvi; t];
dx = [dcqi; dsqi; dkai; dkvi; dt];
desired_dyn_zero_to_t_plan = matlabFunction(dx, 'File', 'rotatotopes/dynamics/desired_dyn_zero_to_t_plan', 'vars', {tdummy, x, udummy});

%~~~~~~~~~~~braking dynamics over [t_plan , t_total]
t_to_stop = t_total - t_plan;
q_i_dot_pk = kvi + kai*t_plan;
braking_acceleration = (0 - q_i_dot_pk)/t_to_stop; % brake to 0 velocity from q_i_dot_pk in t_to_stop seconds

% desired dyamics
q_i_dot = q_i_dot_pk + braking_acceleration*(t - t_plan);
dcqi = -sqi*q_i_dot;
dsqi = cqi*q_i_dot;

dx = [dcqi; dsqi; dkai; dkvi; dt];
desired_dyn_t_plan_to_t_total = matlabFunction(dx, 'File', 'rotatotopes/dynamics/desired_dyn_t_plan_to_t_total', 'vars', {tdummy, x, udummy});
end
