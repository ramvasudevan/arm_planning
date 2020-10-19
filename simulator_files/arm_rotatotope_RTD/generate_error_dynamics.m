function [] = generate_error_dynamics(t_plan,t_total)

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

% time
dt = 1;

x = [ce; se; e0; de0;t];
dx = [dce; dse; d_e0; d_de0; dt];
error_dyn_zero_to_t_plan = matlabFunction(dx, 'File', 'rotatotopes/dynamics/error_dyn_zero_to_t_plan', 'vars', {tdummy, x, udummy});

%~~~~~~~~~~~braking dynamics over [t_plan , t_total]
% error initial conditions at peak
e0_pk = subs(x_err(1,1), t_plan) * e0 + subs(x_err(1,2),t_plan) * de0;
de0_pk = subs(x_err(2,1),t_plan) * e0 + subs(x_err(2,2),t_plan) * de0;

% error dynamics 
de = x_err(2,1) * e0_pk + x_err(2,2) * de0_pk;
dce = -se*de;
dse = ce*de;
d_e0 = 0;
d_de0 = 0;



dx = [dce; dse; d_e0; d_de0; dt];
error_dyn_t_plan_to_t_total = matlabFunction(dx, 'File', 'rotatotopes/dynamics/error_dyn_t_plan_to_t_total', 'vars', {tdummy, x, udummy});
end
