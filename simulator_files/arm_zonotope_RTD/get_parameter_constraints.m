function [k_unsafe_A, k_unsafe_b] = get_parameter_constraints(link, k_lim, obstacle)
%GET_PARAMETER_CONSTRAINTS takes in the FRS of a link, the parameter limits,
% and an obstacle to generate hyperplane constraints describing unsafe
% parameters.

param_dim = link.info.param_dimensions;
position_dim = link.info.position_dimensions;
nParams = length(param_dim);

k_unsafe_A = cell(length(link.FRS), 1);
k_unsafe_b = cell(length(link.FRS), 1);

tic
for i = 1:length(link.FRS)
    frs = link.FRS{i};
    frs_Z = get(frs, 'Z');
    frs_c = frs_Z(:, 1);
    frs_G = frs_Z(:, 2:end);
    nGen = size(frs_G, 2);
    [~, param_col] = find(frs_G(param_dim, :) ~= 0);
    
    frs_k_ind_G = frs_G;
    frs_k_ind_G(:, param_col) = [];
    frs_k_ind_G_pos = frs_k_ind_G(position_dim, :);
    
    % overapproximate with a square...could speed computation at expense of
    % accuracy:
    %   frs_k_ind_G_pos = [sum(abs(frs_k_ind_G_pos(1, :))), 0; 0, sum(abs(frs_k_ind_G_pos(2, :)))];
    
    frs_k_dep_G = frs_G(:, param_col);
    frs_k_dep_G_pos = frs_G(position_dim, param_col);
    
    % buffer obstacle by k-independent generators
    buff_obstacle = obstacle + zonotope([zeros(size(position_dim)), frs_k_ind_G_pos]);
    
    % use mptPoly to get H-rep of obstacle
    obs_poly = mptPolytope(buff_obstacle - frs_c(position_dim));
    A_poly = get(obs_poly, 'A');
    b_poly = get(obs_poly, 'b');
    
    % get hyperplanes defining polytope of unsafe k
    my_k_unsafe_A = zeros(size(A_poly, 1), nParams);
    my_k_unsafe_b = zeros(size(A_poly, 1), 1);
    for j = 1:size(A_poly, 1)
        for k = 1:nParams
            frs_k_dep_G_param(1,k) = frs_k_dep_G(param_dim(k), k);
        end
        AG = A_poly(j, :)*frs_k_dep_G_pos./frs_k_dep_G_param;
        my_k_unsafe_A(j, :) = AG;
        my_k_unsafe_b(j, 1) = b_poly(j) + AG*frs_c(param_dim);
    end
    
    test_kV = max(my_k_unsafe_A*k_lim.V - repmat(my_k_unsafe_b, 1, size(k_lim.V, 2)));
    if any(test_kV <= 0)
        % a k vertex is inside polytope, keep it.
        k_unsafe_A{i} = my_k_unsafe_A;
        k_unsafe_b{i} = my_k_unsafe_b;
    end
    
    %% old code when computing A and b of obstacle by hand...
%     Zpts = polygon(buff_obstacle)' - frs_c(position_dim)'; % this line could be sped up a lot by avoiding the "polygon" function
%     K = convhull(Zpts);
%     Zpts = Zpts(K, :);
%     
%     % get hyperplanes defining polytope of unsafe k
%     my_k_unsafe_A = zeros(size(Zpts, 1) -1, 2);
%     my_k_unsafe_b = zeros(size(Zpts, 1) -1, 1);
%     for j = 1:size(Zpts, 1)-1
%         x1 = Zpts(j, :)';
%         x2 = Zpts(j+1, :)';
%         Atmp = [0 -1; 1 0]*(x1 - x2); % hmmm will have to rewrite this for 3D points!!
%         A_poly = (Atmp./norm(Atmp))';
%         b_poly = A_poly*x1;
%         
%         AG = A_poly*frs_k_dep_G_pos./([frs_k_dep_G(param_dim(1), 1), frs_k_dep_G(param_dim(2), 2)]);
%         my_k_unsafe_A(j, :) = AG;
%         %             k_unsafe_b{p}{i}(j, 1) = b_poly - AG*frs_c(param_dim);
%         my_k_unsafe_b(j, 1) = b_poly + AG*frs_c(param_dim);
%     end
%     
%     test_kV = max(my_k_unsafe_A*k_lim.V - repmat(my_k_unsafe_b, 1, 4));
%     if any(test_kV <= 0)
%         % a k vertex is inside polytope, keep it.
%         k_unsafe_A{i} = my_k_unsafe_A;
%         k_unsafe_b{i} = my_k_unsafe_b;
%     end
    
end
disp('Time of constraint generation:');
toc
end

