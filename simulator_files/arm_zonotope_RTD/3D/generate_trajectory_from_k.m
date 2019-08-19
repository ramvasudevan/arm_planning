function [X] = generate_trajectory_from_k(R_0, phi_dot_0, k, options)
%GENERATE_TRAJECTORY_FROM_K given initial rotation matrices, initial
%angular velocities phi_dot_0, and control parameters k, generate joint
%position trajectories for the ends of each link.

k1 = k(1:2:end);
k2 = k(2:2:end);
theta_dot_0 = zeros(size(phi_dot_0)); % assume no velocity in theta to start

% for a 3 link arm, R is a 3x1 cell, phi_dot_0 is a 3x1 vector, k is a 6x1
% vector.

% phi_dot_0(i) is the initial angular velocity about the "z" axis specified by
% R.
% k1(i) is the peak angular velocity about the "z" axis specified by R
% k2(i) is the peak angular velocity about a body fixed "y" axis

% t_to_peak = 0+options.timeStep/2:options.timeStep:options.tPlan;
% t_to_stop = options.tPlan + options.timeStep/2:options.timeStep:options.T;

t_to_peak = 0:options.timeStep:options.tPlan;
t_to_stop = options.tPlan:options.timeStep:options.T;

% t_all = [t_to_peak, t_to_stop];

% phi_ddot_to_peak = ((k1 - phi_dot_0)/options.tPlan);
% phi_ddot_to_stop = ((0 - k1)/(options.T - options.tPlan));
% theta_ddot_to_peak = ((k2 - theta_dot_0)/options.tPlan);
% theta_ddot_to_stop = ((0 - k2)/(options.T - options.tPlan));
% 
% phi_to_peak = 0 + phi_dot_0.*t_to_peak + 1/2*phi_ddot_to_peak.*t_to_peak.^2;
% phi_to_stop = phi_to_peak(:, end) + k1.*(t_to_stop - options.tPlan) + 1/2*phi_ddot_to_stop.*(t_to_stop - options.tPlan).^2;
% theta_to_peak = 0 + theta_dot_0*t_to_peak + 1/2*theta_ddot_to_peak.*t_to_peak.^2;
% theta_to_stop = theta_to_peak(:, end) + k2.*(t_to_stop - options.tPlan) + 1/2*theta_ddot_to_stop.*(t_to_stop - options.tPlan).^2;
% 
% phi = [phi_to_peak, phi_to_stop];  % angle about "z" axis
% theta = [theta_to_peak, theta_to_stop];  % angle about body fixed "y"
% 
% X = zeros(3*size(phi, 1), size(phi, 2));

for i = 1:options.nLinks
    [toutpeak, xoutpeak] = ode45(@arm_dyn_toPeak_3D_ODE, [t_to_peak], [options.L; 0; 0; phi_dot_0(i); 0; k1(i); k2(i)]);
    [toutstop, xoutstop] = ode45(@arm_dyn_toStop_3D_ODE, [t_to_stop], [xoutpeak(end, 1:3)'; phi_dot_0(i); 0; k1(i); k2(i)]);
    tout{i} = [toutpeak; toutstop(2:end)];
    xout{i} = [xoutpeak; xoutstop(2:end, :)];
end
% [tout2peak, xout2peak] = ode45(@arm_dyn_toPeak_3D_ODE, [t_to_peak], [options.L; 0; 0; phi_dot_0(2); 0; k1(2); k2(2)]);
% [tout2stop, xout2stop] = ode45(@arm_dyn_toStop_3D_ODE, [t_to_stop], [xout2peak(end, 1:3)'; phi_dot_0(2); 0; k1(2); k2(2)]);
% 
% [tout3peak, xout3peak] = ode45(@arm_dyn_toPeak_3D_ODE, [t_to_peak], [options.L; 0; 0; phi_dot_0(3); 0; k1(3); k2(3)]);
% [tout3stop, xout3stop] = ode45(@arm_dyn_toStop_3D_ODE, [t_to_stop], [xout3peak(end, 1:3)'; phi_dot_0(3); 0; k1(3); k2(3)]);

% for i = 1:length(tout{1})
    for j = 1:options.nLinks
%         R{j} = R_0{j}*make_orientation(theta(j, i), 2)*make_orientation(phi(j, i), 3); % ?? theta cuz i switched around in dynamics
        if j == 1
%             X(3*(j-1)+1:3*j, i) = R{j}*[options.L; 0; 0];
            X(3*(j-1)+1:3*j, :) = R_0{j}*xout{j}(:, 1:3)';
        else
%             X(3*(j-1)+1:3*j, :) = R{j}*[options.L; 0; 0] + X(3*(j-2)+1:3*(j-1), i);
            X(3*(j-1)+1:3*j, :) = R_0{j}*xout{j}(:, 1:3)' + X(3*(j-2)+1:3*(j-1), :);
        end
    end
    
% end

end

