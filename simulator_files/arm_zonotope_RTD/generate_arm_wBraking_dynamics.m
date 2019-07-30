function [] = generate_arm_wBraking_dynamics(t_plan,t_total)
% constant acceleration to peak speed from initial speed over [0, tplan]
% constant deceleration to zero speed from peak speed over [tplan, t_total]

% assume that t_peak = t_plan
t_to_stop = t_total - t_plan; % time from peak speed to stopping
theta_dot_f = 0; % final speed = 0

syms x y theta_dot_0 theta_dot_pk t real

z = [x; y; theta_dot_0; theta_dot_pk; t];

theta_dot = theta_dot_0 + ((theta_dot_pk - theta_dot_0)/t_plan)*t;

dx = -y*theta_dot;
dy = x*theta_dot;
% dtheta_dot = (theta_dot_pk - theta_dot_0)/tplan;
dtheta_dot_0 = 0;
dtheta_dot_pk = 0;
dt = 1;

dz = [dx; dy; dtheta_dot_0; dtheta_dot_pk; dt];

syms tdummy udummy real;
matlabFunction(dz, 'File', 'arm_dyn_toPeak', 'vars', {tdummy z udummy});

theta_dot = theta_dot_pk + (theta_dot_f - theta_dot_pk)/(t_to_stop)*(t - t_plan);

dx = -y*theta_dot;
dy = x*theta_dot;
% dtheta_dot = -(theta_dot_pk)/(tplan - t_total);
dtheta_dot_0 = 0;
dtheta_dot_pk = 0;
dt = 1;

dz = [dx; dy; dtheta_dot_0; dtheta_dot_pk; dt];

syms tdummy udummy real;
matlabFunction(dz, 'File', 'arm_dyn_toStop', 'vars', {tdummy z udummy});

end

