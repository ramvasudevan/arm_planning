% function [ ] = generate_trig_dynamics(t_plan, t_total)

% constant acceleration to peak speed from initial speed over [0, tplan]
% constant deceleration to zero speed from peak speed over [tplan, t_total]

t_plan = 0.5;
t_total = 1;

t_to_stop = t_total - t_plan;
q_dot_f = 0; % final speed = 0;

syms cq sq q_dot_0 q_ddot t real;

x = [cq; sq; q_ddot; q_dot_0; t];

q_dot = q_dot_0 + q_ddot*t;

dcq = -sq*q_dot;
dsq = cq*q_dot;
d_q_dot_0 = 0;
d_q_ddot = 0;
dt = 1;

dx = [dcq; dsq; d_q_ddot; d_q_dot_0; dt];

syms udummy real;
% matlabFunction(dx, 'File', 'trig_dyn_toPeak', 'vars', {x, udummy});

% now braking dynamics
q_dot_pk = q_dot_0 + q_ddot*t_plan;
q_ddot_brake = (q_dot_f - q_dot_pk)/t_to_stop;
q_dot = q_dot_pk + q_ddot_brake*(t - t_plan);

dcq = -sq*q_dot;
dsq = cq*q_dot;
d_q_dot_0 = 0;
d_q_ddot = 0;
dt = 1;

dx = [dcq; dsq; d_q_ddot; d_q_dot_0; dt];

% matlabFunction(dx, 'File', 'trig_dyn_toStop', 'vars', {x, udummy});

% end

