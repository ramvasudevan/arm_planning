function [k_lim] = get_parameter_limits(link)
% just gives the vertices of the box describing the parameter limits in the
% FRS, as well as the constraint form

param_dim = link.info.param_dimensions;
nParams = length(param_dim);
for i = 1:nParams
    c(i, 1) = link.FRS{1}.Z(param_dim(i), 1);
    g(i, 1) = sum(abs(link.FRS{1}.Z(param_dim(i), 2:end)));
end

for i = 1:nParams
    klb(i, 1) = c(i) - g(i);
    kub(i, 1) = c(i) + g(i);
end

k_lim.A = zeros(2*nParams, nParams);
k_lim.b = zeros(2*nParams, 1);
for i = 1:nParams
   k_lim.A(2*i-1:2*i, i) = [-1; 1];
   k_lim.b(2*i-1:2*i, 1) = [-klb(i); kub(i)];
end
% 
% u1c = link.FRS{1}.Z(param_dim(1), 1);
% u1g = sum(abs(link2{1}{1}.Z(param_dim(1), 2:end)));
% u2c = link2{1}{1}.Z(param_dim(2), 1);
% u2g = sum(abs(link2{1}{1}.Z(param_dim(2), 2:end)));
% u1lb = u1c - u1g;
% u1ub = u1c + u1g;
% u2lb = u2c - u2g;
% u2ub = u2c + u2g;
% 
% uA = [-1 0; 1 0; 0 -1; 0 1];
% uB = [-u1lb; u1ub; -u2lb; u2ub];

% and create input vertices

cubeV = cubelet_ND(nParams)';
k_lim.V = (cubeV.*g) + c;

% ux = [u1c - u1g, u1c - u1g, u1c + u1g, u1c + u1g];
% uy = [u2c - u2g, u2c + u2g, u2c + u2g, u2c - u2g];
% uv = [ux; uy];
    
end