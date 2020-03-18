function [ dist, coeffs ] = check_inZonotope( Z, pt, pt_dim)
% CHECK_INZONOTOPE LP approach to determine if point is in or out of zonotope.

if ~exist('pt_dim', 'var')
    pt_dim = 1:length(pt)';
end

c = Z.Z(pt_dim, 1);
G = Z.Z(pt_dim, 2:end);
dim = size(pt, 1);
ngen = size(G, 2);

if size(pt, 2) ~= 1
    error('Specify the point to check as a column vector');
elseif dim ~= size(G, 1)
    error('Point dimension does not equal generator dimension.');
elseif ngen < dim
    warning('Fewer generators than dimension of point.');
end

% get vector to pt from center.
y = pt - c;

% run linear program to determine if in or out
% f = [1 zeros(1, ngen)];
f = [1 sparse(1, ngen)];
Aeq = sparse([sparse(dim+1, 1), [sparse(1, ngen); G]]);
A1 = -1*ones(2*ngen, 1);
A2 = speye(ngen);
A2 = sparse([A2; -A2]);
A = sparse([A1, A2]);
beq = [0; y];
% b = zeros(size(A, 1), 1);
b = sparse(size(A, 1), 1);

% options = optimoptions(@linprog, 'Display', 'off');

% x = linprog_gurobi(f, A, b, Aeq, beq);
x = linprog(f, A, b, Aeq, beq);

% if dist > 1, the point lies outside the zonotope.
dist = x(1);
coeffs = x(2:end);

end