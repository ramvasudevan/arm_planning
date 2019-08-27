function [flag] = check_intersection_LP(G, pt)
% uses an LP to check if pt is contained with the the set defined by
% generator matrix G

dim = size(pt, 1);
ngen = size(G, 2);

if size(G, 1) ~= dim
   error('G and pt are not the same dimension'); 
end
if size(pt, 2) > 1
   error('Specify pt as a single column vector');
end

% run linear program to determine if in or out
f = [1 sparse(1, ngen)];
Aeq = sparse([sparse(dim+1, 1), [sparse(1, ngen); G]]);
A1 = -1*ones(2*ngen, 1);
A2 = speye(ngen);
A2 = sparse([A2; -A2]);
A = sparse([A1, A2]);
beq = [0; pt];
b = sparse(size(A, 1), 1);

% options = optimoptions(@linprog, 'Display', 'off');

% x = linprog_gurobi(f, A, b, Aeq, beq);
x = linprog(f, A, b, Aeq, beq);

% if dist > 1, the point lies outside the zonotope.
% dist = x(1);
% coeffs = x(2:end);

if x(1) <= 1
    flag = true;
else
    flag = false;
end

end

