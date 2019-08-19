function [] = generate_arm_dynamics_3D(t_plan, t_total)
% constant acceleration to peak speed from initial speed over [0, tplan]
% constant deceleration to zero speed from peak speed over [tplan, t_total]

t_to_stop = t_total - t_plan; % time from peak speed to stopping
phi_0 = 0; % align with x-axis
theta_0 = pi/2; % ==> z_0 = 0
phi_dot_f = 0; % final speed = 0;
theta_dot_f = 0; % final speed = 0

syms x y z sph cph z1 phi_dot_0 phi_dot_pk theta_dot_0 theta_dot_pk t real

w = [x; y; z; sph; cph; phi_dot_0; theta_dot_0; phi_dot_pk; theta_dot_pk; t];

% to peak speed dynamics

phi_ddot = ((phi_dot_pk - phi_dot_0)/t_plan);
theta_ddot = ((theta_dot_pk - theta_dot_0)/t_plan);
phi_dot = phi_dot_0 + phi_ddot*t;
theta_dot = theta_dot_0 + theta_ddot*t;

%%% dynamics option 1
% dx = (z*x)/(sqrt(x^2 + y^2))*theta_dot - y*phi_dot;
% dy = (z*y)/(sqrt(x^2 + y^2))*theta_dot + x*phi_dot;
% dz = -sqrt(x^2 + y^2)*theta_dot;

%%% dynamics option 2
% r = sqrt(x^2 + y^2 + z^2);
% phi = phi_0 + phi_dot_0*t + 1/2*phi_ddot*t^2;
% theta = theta_0 + theta_dot_0*t + 1/2*theta_ddot*t^2;
% 
% phi_pk = phi_0 + phi_dot_0*t_plan + 1/2*phi_ddot*t_plan^2;
% theta_pk = theta_0 + theta_dot_0*t_plan + 1/2*theta_ddot*t_plan^2;
% 
% dx = r*cos(theta)*cos(phi)*theta_dot - r*sin(theta)*sin(phi)*phi_dot;
% dy = r*cos(theta)*sin(phi)*theta_dot + r*sin(theta)*cos(phi)*phi_dot;
% dz = -r*sin(theta)*theta_dot;

%%% dynamics option 3 WRONG
% dx = (z*x)/(sqrt(1 - z^2))*theta_dot - y*phi_dot;
% dy = (z*y)/(sqrt(1 - z^2))*theta_dot + x*phi_dot;
% dz = -sqrt(1 - z^2)*theta_dot;

%%% dynamics option 4.. keep phi around.
% dx = z*cos(phi)*theta_dot - y*phi_dot;
% dy = z*sin(phi)*theta_dot + x*phi_dot;
% dz = -sqrt(x^2 + y^2)*theta_dot;
% dphi = phi_dot_0 + phi_ddot*t;

%%% dynamics options 5... use highD system
% dx1 = -y1*(theta_dot - phi_dot);
% dy1 = x1*(theta_dot - phi_dot);
% 
% dx2 = -y2*(theta_dot + phi_dot);
% dy2 = x2*(theta_dot + phi_dot);
% 
% dz1 = -z*theta_dot;
% 
% dx = dx1 + dx2;
% dy = -dy1 + dy2;
% dz = z1*theta_dot;

%%% dynamics options 6... just use trig variables.
dx = -z*cph*theta_dot - y*phi_dot;
dy = -z*sph*theta_dot + x*phi_dot;
% dz = (x/cph)*theta_dot;
% dz = sqrt(x^2 + y^2)*theta_dot;
dz = (x*cph + y*sph)*theta_dot;

dsph = cph*phi_dot;
dcph = -sph*phi_dot;
% dz1 = -z*theta_dot;

d_phi_dot_0 = 0;
d_theta_dot_0 = 0;
d_phi_dot_pk = 0;
d_theta_dot_pk = 0;
dt = 1;


% dw = [dx; dy; dz; dx1; dy1; dx2; dy2; dz1; d_phi_dot_0; d_theta_dot_0; d_phi_dot_pk; d_theta_dot_pk; dt];
dw = [dx; dy; dz; dsph; dcph; d_phi_dot_0; d_theta_dot_0; d_phi_dot_pk; d_theta_dot_pk; dt];

syms tdummy udummy real;
matlabFunction(dw, 'File', 'arm_dyn_toPeak_3D', 'vars', {tdummy, w, udummy});

% now braking dynamics

phi_ddot = (phi_dot_f - phi_dot_pk)/(t_to_stop);
theta_ddot = (theta_dot_f - theta_dot_pk)/(t_to_stop);
phi_dot = phi_dot_pk + phi_ddot*(t - t_plan);
theta_dot = theta_dot_pk + theta_ddot*(t - t_plan);

%%% dynamics option 1
% dx = (z*x)/(sqrt(x^2 + y^2))*theta_dot - y*phi_dot;
% dy = (z*y)/(sqrt(x^2 + y^2))*theta_dot + x*phi_dot;
% dz = -sqrt(x^2 + y^2)*theta_dot;

%%% dynamics option 2
% r = sqrt(x^2 + y^2 + z^2);
% phi = phi_pk + phi_dot_pk*(t - t_plan) + 1/2*phi_ddot*(t - t_plan)^2;
% theta = theta_pk + theta_dot_pk*(t - t_plan) + 1/2*theta_ddot*(t - t_plan)^2;

%%% dynamics option 3 WRONG
% dx = (z*x)/(sqrt(1 - z^2))*theta_dot - y*phi_dot;
% dy = (z*y)/(sqrt(1 - z^2))*theta_dot + x*phi_dot;
% dz = -sqrt(1 - z^2)*theta_dot;

%%% dynamics options 5... use highD system
% dx1 = -y1*(theta_dot - phi_dot);
% dy1 = x1*(theta_dot - phi_dot);
% 
% dx2 = -y2*(theta_dot + phi_dot);
% dy2 = x2*(theta_dot + phi_dot);
% 
% dz1 = -z*theta_dot;
% 
% dx = dx1 + dx2;
% dy = -dy1 + dy2;
% dz = z1*theta_dot;

%%% dynamics options 6... just use trig variables.
dx = -z*cph*theta_dot - y*phi_dot;
dy = -z*sph*theta_dot + x*phi_dot;
% dz = (x/cph)*theta_dot;
% dz = sqrt(x^2 + y^2)*theta_dot;
dz = (x*cph + y*sph)*theta_dot;

dsph = cph*phi_dot;
dcph = -sph*phi_dot;
% dz1 = -z*theta_dot;

d_phi_dot_0 = 0;
d_theta_dot_0 = 0;
d_phi_dot_pk = 0;
d_theta_dot_pk = 0;
dt = 1;

% dw = [dx; dy; dz; dx1; dy1; dx2; dy2; dz1; d_phi_dot_0; d_theta_dot_0; d_phi_dot_pk; d_theta_dot_pk; dt];
dw = [dx; dy; dz; dsph; dcph; d_phi_dot_0; d_theta_dot_0; d_phi_dot_pk; d_theta_dot_pk; dt];

syms tdummy udummy real;
matlabFunction(dw, 'File', 'arm_dyn_toStop_3D', 'vars', {tdummy, w, udummy});

%% now create dynamics for ODE45...
clearvars -except t_plan t_total

t_to_stop = t_total - t_plan; % time from peak speed to stopping
phi_0 = 0; % align with x-axis
theta_0 = pi/2; % ==> z_0 = 0
phi_dot_f = 0; % final speed = 0;
theta_dot_f = 0; % final speed = 0

syms x y z phi_dot_0 phi_dot_pk theta_dot_0 theta_dot_pk t real;

w = [x; y; z; phi_dot_0; theta_dot_0; phi_dot_pk; theta_dot_pk];

% to peak speed dynamics

phi_ddot = ((phi_dot_pk - phi_dot_0)/t_plan);
theta_ddot = ((theta_dot_pk - theta_dot_0)/t_plan);
phi_dot = phi_dot_0 + phi_ddot*t;
theta_dot = theta_dot_0 + theta_ddot*t;

%%% dynamics option 1
dx = -(z*x)/(sqrt(x^2 + y^2))*theta_dot - y*phi_dot;
dy = -(z*y)/(sqrt(x^2 + y^2))*theta_dot + x*phi_dot;
dz = sqrt(x^2 + y^2)*theta_dot;

d_phi_dot_0 = 0;
d_theta_dot_0 = 0;
d_phi_dot_pk = 0;
d_theta_dot_pk = 0;
dt = 1;

dw = [dx; dy; dz; d_phi_dot_0; d_theta_dot_0; d_phi_dot_pk; d_theta_dot_pk];
matlabFunction(dw, 'File', 'arm_dyn_toPeak_3D_ODE', 'vars', {t, w});

% now braking dynamics
phi_ddot = (phi_dot_f - phi_dot_pk)/(t_to_stop);
theta_ddot = (theta_dot_f - theta_dot_pk)/(t_to_stop);
phi_dot = phi_dot_pk + phi_ddot*(t - t_plan);
theta_dot = theta_dot_pk + theta_ddot*(t - t_plan);

%%% dynamics option 1
dx = -(z*x)/(sqrt(x^2 + y^2))*theta_dot - y*phi_dot;
dy = -(z*y)/(sqrt(x^2 + y^2))*theta_dot + x*phi_dot;
dz = sqrt(x^2 + y^2)*theta_dot;

d_phi_dot_0 = 0;
d_theta_dot_0 = 0;
d_phi_dot_pk = 0;
d_theta_dot_pk = 0;
dt = 1;

dw = [dx; dy; dz; d_phi_dot_0; d_theta_dot_0; d_phi_dot_pk; d_theta_dot_pk];
matlabFunction(dw, 'File', 'arm_dyn_toStop_3D_ODE', 'vars', {t, w});


end

