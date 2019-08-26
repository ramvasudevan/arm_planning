function [Q] = get_fetch_q_from_traj(X, q_0)
% Given the joint location trajectories through workspace, and the length
% of each link, compute the fetch generalize coord trajectories
% Assuming each link is 0.33 m for now.

if ~exist('q_0', 'var')
    q_0 = zeros(2*(size(X, 1)/3), 1);
end

X0 = X(:, 1);
Q = zeros(2*(size(X, 1)/3), size(X, 2));

% options = optimoptions('fsolve');
options = optimoptions('fsolve', 'Display', 'none', 'Algorithm', 'levenberg-marquardt');
% options = optimoptions('fsolve', 'Display', 'iter', 'Algorithm', 'levenberg-marquardt', 'SpecifyObjectiveGradient', true);
% options = optimoptions('fsolve','SpecifyObjectiveGradient',true);
% options = optimoptions('fsolve','SpecifyObjectiveGradient',true, 'Algorithm', 'levenberg-marquardt');

% tic
qj1 = fsolve(@(q) joint1_IK(X0(1:3, 1), q), q_0(1:2, 1), options);
qj2 = fsolve(@(q) joint2_IK(X0(4:6, 1), qj1, q), q_0(3:4, 1), options);
qj3 = fsolve(@(q) joint3_IK(X0(7:9, 1), [qj1;qj2], q), q_0(5:6, 1), options);

Q(:, 1) = [qj1; qj2; qj3];

for i = 2:size(X, 2)
    qj1 = fsolve(@(q) joint1_IK(X(1:3, i), q), Q(1:2, i-1), options);
    qj2 = fsolve(@(q) joint2_IK(X(4:6, i), qj1, q), Q(3:4, i-1), options);
    qj3 = fsolve(@(q) joint3_IK(X(7:9, i), [qj1;qj2], q), Q(5:6, i-1), options);
    
    Q(:, i) = [qj1; qj2; qj3];
end
% toc

end

