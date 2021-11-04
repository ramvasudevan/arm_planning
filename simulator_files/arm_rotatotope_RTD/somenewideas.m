clear; clc;

syms p0 v0 t k 'real';

% stage 1
traj_1 = p0 + v0 * t + 0.5 * k * t.^2;

cos_traj_1 = cos(traj_1);

% maximum degree over t is 2


% stage 2
traj_2 = p0 - 0.25 * v0 - 0.25 * k + (2 * v0 + k) * t - 0.5 * (2 * v0 + k) * t.^2;

function derivatives = getHighOrderDerivatives(expr, var)
    order = 2;
    derivatives = sym(zeros(1,order+1));
    derivatives(1) = expr;
    for i = 1:order
        derivatives(i+1) = diff(derivatives(i), var);
    end
end