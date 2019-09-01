function [k_unsafe_A, k_unsafe_b] = get_parameter_constraints(link, k_lim, obstacle, options)
%GET_PARAMETER_CONSTRAINTS takes in the FRS of a link, the parameter limits,
% and an obstacle to generate hyperplane constraints describing unsafe
% parameters.

param_dim = link.info.param_dimensions;
position_dim = link.info.position_dimensions;
nParams = length(param_dim);

k_unsafe_A = cell(length(link.FRS), 1);
k_unsafe_b = cell(length(link.FRS), 1);

V = k_lim.V;

% tic
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
    
    % rearrange frs_k_dep_G so that parameters occur in descending order
    [row_k, col_k] = find(frs_k_dep_G(param_dim, :) ~= 0);
    [~, Ik] = sort(row_k);
    frs_k_dep_G = frs_k_dep_G(:, col_k(Ik));
    
    frs_k_dep_G_pos = frs_k_dep_G(position_dim, :);
%     for k = 1:nParams
%         frs_k_dep_G_param(1,k) = frs_k_dep_G(param_dim(k), k);
%     end
    frs_k_dep_G_param = diag(frs_k_dep_G(param_dim, 1:nParams))';
    
    % buffer obstacle by k-independent generators
%     buff_obstacle = obstacle + zonotope([zeros(length(position_dim),1), frs_k_ind_G_pos]);
    buff_obstacle = [obstacle(:, 1) - frs_c(position_dim), obstacle(:, 2:end), frs_k_ind_G_pos];
    
    % use an LP to check if intersection is possible: edit: jk this is
    % really slow... trying least squares with conservative intersection
    % estimation... also doesn't seem to help with speed.
%     intersectionPossible = check_intersection_LP([frs_G(position_dim, :), buff_obstacle(:, 2:end)], buff_obstacle(:, 1));

%     testG = [frs_G(position_dim, :), buff_obstacle(:, 2:end)];
%     testG(:, ~any(testG)) = [];
%     lambda = (testG)\buff_obstacle(:, 1);
%     intersectionPossible = all(abs(lambda) < 1);

    intersectionPossible = 1;
    
    if intersectionPossible
        % use mptPoly to get H-rep of obstacle
        [A_poly, b_poly] = polytope_PH(buff_obstacle, options);

        my_k_unsafe_A = A_poly*frs_k_dep_G_pos./frs_k_dep_G_param;
        my_k_unsafe_b = b_poly + my_k_unsafe_A*frs_c(param_dim);
        
%         k_unsafe_A{i} = [my_k_unsafe_A, zeros(size(my_k_unsafe_A, 1), link.info.max_params - nParams)];
%         k_unsafe_b{i} = my_k_unsafe_b;

        kVc = my_k_unsafe_A*V - my_k_unsafe_b;
        test_kV = max(kVc);
        if any(test_kV <= 0)
            % a k vertex is inside polytope, keep it.
            k_unsafe_A{i} = [my_k_unsafe_A, zeros(size(my_k_unsafe_A, 1), link.info.max_params - nParams)];
            k_unsafe_b{i} = my_k_unsafe_b;
        end
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
% disp('Time of constraint generation:');
% toc
end

