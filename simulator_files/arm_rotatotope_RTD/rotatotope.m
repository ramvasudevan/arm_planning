classdef rotatotope
    %ROTATOTOPE rotating a set by a set of rotation matrices
    %   this class provides some useful functionality for rotating a
    %   zonotope by a set of rotation matrices.
    %
    %   argument 1) a vector of the rotation axes (ie 1, 2, or 3 for rotating about the
    %   i, j, or k axes in 3D)
    %   argument 2) a cell of zonotopes whose first two dimensions correspond to the
    %   cos and sin values of the desired rotation. a 3rd dimension should
    %   correspond to a trajectory parameter k.
    %   argument 3) the set to be rotated (should be a zonotope)
    %
    %   rotatotope will perform the rotations as if multiplying rotation
    %   matrices together, and will infer whether the set is in 2D or 3D 
    
    properties
        R = {}; % cell of zonotopes representing rotation matrices
        rot_axes = []; % axes about which we are rotating
        Z; % zonotope representing the set to rotate
        dim double;
        red_order = 10; % desired order of reduced zonotope
        
        % hold on to dimensions of first zonotope
        pos_dim = [1, 2];
        k_dim = [3];
        
        % helpful for slicing along k later on
        % C, G, are center and generator of matrix zonotope representing
        % rotation matrices
        % c, g, are center and generator of zonotope representing set.
        
        C = {};
        G = {};
        G_k = {}; % k dependent generator matrices
        c = [];
        g = [];
        c_k = []; % center of k interval
        g_k = []; % generator of k interval

        Rc = []; % resulting zonotope center
        Rg = []; % resulting zonotope generator matrix
        RZ = []; % resulting zonotope [c, g]
        
%         c_idx = false(0); % keep track of which generators are multiplied by zonotope C
%         C_idx = false(0); % keep track of which generators are multiplied by mat zonotopes C's
%         k_idx = false(0); % keep track of which generators depend on which k
        c_idx = []; % keep track of which generators are multiplied by zonotope C
        C_idx = []; % keep track of which generators are multiplied by mat zonotopes C's
        k_idx = []; % keep track of which generators depend on which k
        
        % key: 1 means generator was multiplied by (c, corresponding C or
        % k). 0 means generator was not multiplied by (c, corresponding C
        % or k). nan (for stacking) means generator has no dependence on
        % that (corresponding C or k).
    end
    
    methods
        function obj = rotatotope(varargin)
            %ROTATOTOPE
            if nargin == 3
                % parse the input arguments:
                obj.rot_axes = varargin{1};
                obj.R = varargin{2};
                if isa(varargin{3}, 'zonotope')
                    obj.Z = varargin{end};
                else
                    error('Specify the set to rotate as a zonotope');
                end
            else
                error('Specify 3 arguments to rotatotope');
            end
            
            Z = obj.Z.Z;
            obj.dim = size(Z, 1);
            obj.c = Z(:, 1);
            obj.g = Z(:, 2:end);
            
            obj = obj.generate_matrices();
            obj = obj.multiply();
        end
        
        function obj = generate_matrices(obj)
            for idx = 1:length(obj.R)
                Z = obj.R{idx}.Z;
                
                c = Z(:, 1);
                G = Z(:, 2:end);
                G(:, ~any(G)) = []; % delete zero columns of G
                
                % extract k information
                obj.c_k(idx, 1) = c(obj.k_dim);
                k_col = find(G(obj.k_dim, :) ~= 0);
                if length(k_col) == 1
                    obj.g_k(idx, 1) = G(obj.k_dim, k_col);
                else
                    error('More than one generator for k-dimensions');
                end
                
                % create rotation matrices from cos and sin information
                obj.C{idx, 1} = obj.make_orientation(c(1, 1), c(2, 1), idx, 1);
                
                G_k = G(:, k_col); % k-dependent generator
                for i = 1:size(G_k, 2)
                    obj.G_k{idx, 1}{i, 1} = obj.make_orientation(G_k(1, i), G_k(2, i), idx, 0);
                end
                
                G(:, k_col) = [];
                for i = 1:size(G, 2)
                    obj.G{idx, 1}{i, 1} = obj.make_orientation(G(1, i), G(2, i), idx, 0);
                end
            end
        end
        
        function A = make_orientation(obj, c, s, idx, center_flag)
            switch obj.dim
                case 2
                    if obj.rot_axes(idx) ~= 3
                        error('In 2D, can only rotate about axis 3 (Z-axis)');
                    end
                    A = [c, -s; s, c];
                case 3
                    if center_flag
                        offset = 1;
                    else
                        offset = 0;
                    end
                    switch obj.rot_axes(idx)
                        case 1
                            A = [offset, 0, 0; 0, c, -s; 0, s, c];
                        case 2
                            A = [c, 0, s; 0, offset, 0; -s, 0, c];
                        case 3
                            A = [c, -s, 0; s, c, 0; 0, 0, offset];
                    end
                otherwise
                    error('Computed an invalid dimension from Z');
            end
        end
        
        function obj = multiply(obj)
            % apply the set of rotations and hold on to the resulting set
            % if we have R1*R2*R3*Z, first compute R3*Z, then R2*(R3*Z),
            % etc.
            
            % keep track of the indices that the generators depend on.
            % also keep track of whether slicing will evaluate to a point
            % or to a generator.

            RZ = [obj.c, obj.g];
                                    
            c_idx = zeros(1, size(RZ, 2));
            c_idx(1) = 1;
            
            C_idx = [];
            k_idx = [];
        
            for idx = length(obj.R):-1:1
                
                c_idx_new = [];
                k_idx_new = [];
                C_idx_new = [];
                RZ_new = [];
                
                % multiply C matrix by center and generators
                RZ_new = obj.C{idx}*RZ; % new center is C*c
                c_idx_new = [c_idx_new, c_idx];
                k_idx_new = [k_idx_new, [zeros(size(c_idx)); k_idx]];
                C_idx_new = [C_idx_new, [ones(size(c_idx)); C_idx]];

                % multiply k dep matrices by center and generators
                for i = 1:length(obj.G_k{idx})
                    RZ_new(:, (end+1):(end+size(RZ, 2))) = obj.G_k{idx}{i}*RZ;
                    c_idx_new = [c_idx_new, c_idx];
                    k_idx_new = [k_idx_new, [ones(size(c_idx)); k_idx]];
                    C_idx_new = [C_idx_new, [zeros(size(c_idx)); C_idx]];
                end

                % multiply non k dep matrices by center and generators
                for i = 1:length(obj.G{idx})
                    RZ_new(:, (end+1):(end+size(RZ, 2))) = obj.G{idx}{i}*RZ;
                    c_idx_new = [c_idx_new, c_idx];
                    k_idx_new = [k_idx_new, [zeros(size(c_idx)); k_idx]];
                    C_idx_new = [C_idx_new, [zeros(size(c_idx)); C_idx]];
                end
                
                % reduce number of generators
                [RZ, c_idx, k_idx, C_idx] = obj.reduce(RZ_new, c_idx_new, k_idx_new, C_idx_new);
            end
            
            % store info
            obj.RZ = RZ;
            obj.Rc = RZ(:, 1);
            obj.Rg = RZ(:, 2:end);
            obj.c_idx = c_idx(1, 2:end);
            obj.k_idx = k_idx(:, 2:end);
            obj.C_idx = C_idx(:, 2:end);
        end
        
        function [] = plot(obj, color)
            if ~exist('color', 'var')
                color = 'b';
            end
            Z = zonotope([obj.RZ]);
            switch obj.dim
                case 2
                    p = plotFilled(Z, [1, 2], 'b');
                    p.FaceAlpha = 0.1;
                case 3
                    Z = reduce(Z, 'girard', 4);
                    V = vertices(project(Z, [1, 2, 3]));
                    shp = alphaShape(V(1, :)', V(2, :)', V(3, :)', inf);
                    p = plot(shp);
                    p.FaceAlpha = 0;
                    p.EdgeAlpha = 0.15;
                    p.EdgeColor = color;
            end
        end
        
        function [] = plot_slice(obj, k, color)
            if ~exist('color', 'var')
                color = 'b';
            end
            Z = slice(obj, k);
            Z = zonotope(Z);
            switch obj.dim
                case 2
                    p = plotFilled(Z, [1, 2], color);
                    p.FaceAlpha = 0.1;
                case 3
%                     Z = reduce(Z, 'girard', 4);
                    V = vertices(project(Z, [1, 2, 3]));
                    shp = alphaShape(V(1, :)', V(2, :)', V(3, :)', inf);
                    p = plot(shp);
                    p.FaceAlpha = 0;
                    p.EdgeAlpha = 0.15;
                    p.EdgeColor = color;
            end
        end
        
%         function [] = plot_slice_gensIncluded(obj, k, color)
%             if ~exist('color', 'var')
%                 color = 'b';
%             end
%             Z = slice_gensIncluded(obj, k);
%             switch obj.dim
%                 case 2
%                     p = plotFilled(Z, [1, 2], color);
%                     p.FaceAlpha = 0.1;
%                 case 3
%                     Z = reduce(Z, 'girard', 4);
%                     V = vertices(project(Z, [1, 2, 3]));
%                     shp = alphaShape(V(1, :)', V(2, :)', V(3, :)', inf);
%                     p = plot(shp);
%                     p.FaceAlpha = 0;
%                     p.EdgeAlpha = 0.15;
%                     p.EdgeColor = color;
%             end
%         end
        
        function [Z, lambda, c, g_sliced, slice_to_pt_idx] = slice(obj, k)
            % this slicing function slices generators that don't slice
            % to a point
            % take in a value for k, slice along dimension
            if length(k) ~= length(obj.c_k)
                error('Slice point not correct dimension');
            end
            g_sliced = obj.Rg;
            c = obj.Rc;
            lambda = zeros(length(k), 1);
            for i = 1:length(k)
                if abs(k(i) - obj.c_k(i)) > obj.g_k(i)
                    error('Slice point is out of bounds');
                end
                lambda(i) = (k(i) - obj.c_k(i))/obj.g_k(i);
                
                g_sliced(:, obj.k_idx(i, :) == 1) = g_sliced(:, obj.k_idx(i, :) == 1)*lambda(i); % slice gens
            end
            
            slice_to_pt_idx = all(obj.k_idx ~= 0 | obj.C_idx ~= 0, 1) & obj.c_idx == 1;
            % take the k dep gens that slice to points... add to center
            c_out = c + sum(g_sliced(:, slice_to_pt_idx), 2);
            g_out = g_sliced(:, ~slice_to_pt_idx);
            
            Z = [c_out, g_out];
        end
        
%         function [Z] = slice(obj, k)
%             % PATRICK EDIT 20200121 this function is incorrect
%             % this slicing function only slices generators that slice to a
%             % point
%             % take in a value for k, slice along dimension
%             if length(k) ~= length(obj.c_k)
%                 error('Slice point not correct dimension');
%             end
%             g = obj.Rg;
%             c = obj.Rc;
%             for i = 1:length(k)
%                 if abs(k(i) - obj.c_k(i)) > obj.g_k(i)
%                     error('Slice point is out of bounds');
%                 end
%                 lambda = (k(i) - obj.c_k(i))/obj.g_k(i);
%                 
%                 slice_idx =  obj.k_idx(i, :) & obj.c_idx; % only gens that slice to c
%                 g(:, slice_idx)  = g(:, slice_idx)*lambda; % slice gens
%             end
%             
%             % take the k dep gens that slice to points... add to center
%             c = c + sum(g(:, all(obj.k_idx, 1) & obj.c_idx), 2);
%             g(:, all(obj.k_idx, 1) & obj.c_idx) = [];
%             
%             Z = zonotope([c, g]);
%         end
        
        function [Z_new, c_idx_new, k_idx_new, C_idx_new] = reduce(obj, Z, c_idx, k_idx, C_idx)
            % based off of the "reduceGirard.m" function included in CORA            
            c = Z(:, 1);
            G = Z(:, 2:end);
            
            c_idx_temp = c_idx(:, 2:end);
            k_idx_temp = k_idx(:, 2:end);
            C_idx_temp = C_idx(:, 2:end);
            
            Gunred = [];
            Gred = [];
            
            if ~isempty(G)
                
                %determine dimension of zonotope
                dim = length(G(:,1));
                
                %only reduce if zonotope order is greater than the desired order
                if length(G(1,:))>dim*obj.red_order
                    
                    %compute metric of generators
%                     h = vnorm(G,1,1)-vnorm(G,1,inf);
                    h = vnorm(G, 1, 2);
                    % sort generators according to metric
                    [~,indices] = sort(h);
                    
                    %number of generators that are not reduced
                    nUnreduced = floor(dim*(obj.red_order-1));
                    %number of generators that are reduced
                    nReduced = length(G(1,:))-nUnreduced;
                    
                    %pick generators that are reduced
                    Gred = G(:,indices(1:nReduced));
                    %unreduced generators
                    Gunred = G(:,indices((nReduced+1):end));
                    
                    % box remaining generators
                    d=sum(abs(Gred),2);
                    %build box Gbox from interval hull vector d
                    Gbox=diag(d);
                    
                    Z_new = [c, Gunred, Gbox];
                    c_idx_new = [c_idx(:, 1), c_idx_temp(:, indices((nReduced+1):end)), zeros(1, size(Gbox, 2))];
                    k_idx_new = [k_idx(:, 1), k_idx_temp(:, indices((nReduced+1):end)), zeros(size(k_idx, 1), size(Gbox, 2))];
                    C_idx_new = [C_idx(:, 1), C_idx_temp(:, indices((nReduced+1):end)), zeros(size(C_idx, 1), size(Gbox, 2))];
                else
                    Gunred = G;
                    Z_new = [c, Gunred];
                    c_idx_new = [c_idx(:, 1), c_idx_temp];
                    k_idx_new = [k_idx(:, 1), k_idx_temp];
                    C_idx_new = [C_idx(:, 1), C_idx_temp];
                end
            end
        end
             
        function obj = stack(obj, base)
           % stack this rotatotope on top of the rotatotope "base"
           obj.Rc = obj.Rc + base.Rc;
           obj.Rg = [obj.Rg, base.Rg];
           obj.RZ = [obj.Rc, obj.Rg];
           
           obj.c_idx = [obj.c_idx, base.c_idx];
           
           % if k_idx not same size, add zeros
           k_rows_1 = size(obj.k_idx, 1);
           [k_rows_2, k_cols_2] = size(base.k_idx);
           
           % if C_idx not same size, add zeros
           C_rows_1 = size(obj.C_idx, 1);
           [C_rows_2, C_cols_2] = size(base.C_idx);
           
           obj.k_idx = [obj.k_idx, [base.k_idx; nan(k_rows_1 - k_rows_2, k_cols_2)]];
           obj.C_idx = [obj.C_idx, [base.C_idx; nan(C_rows_1 - C_rows_2, C_cols_2)]];
        end
        
%         function [A_con, b_con, k_con] = generate_constraints(obj, obstacle, options, link_number)
%            % takes in an obstacle (Z matrix of zonotope) and generates linear constraint matrices
%            % Acon and bcon, as well as the combinations of k_idxs (kcon) these
%            % constraints depend on.
%            
%            nGen = size(obj.Rg, 2);
%            [~, kc_col] = find(any(obj.k_idx) & obj.c_idx);
%            
%            frs_k_ind_G = obj.Rg;
%            frs_k_ind_G(:, kc_col) = [];
%            
%            frs_k_dep_G = obj.Rg(:, kc_col);
%            
%            % buffer the obstacle by k-independent generators, as well as
%            % buffer_dist specified by planner:
%            buff_obstacle_c = [obstacle(:, 1) - obj.Rc];
%            buff_obstacle_G = [obstacle(:, 2:end), frs_k_ind_G, options.buffer_dist/2*eye(3)];
%            buff_obstacle_G(:, ~any(buff_obstacle_G)) = []; % delete zero columns of G
%            buff_obstacle = [buff_obstacle_c, buff_obstacle_G];
%            
%            [A_poly, b_poly] = polytope_PH(buff_obstacle, options);
%            
%            A_con = A_poly*frs_k_dep_G;
%            b_con = b_poly;
%            k_con = obj.k_idx(:, kc_col);
%            
%            % add a test here that throws out unnecessary constraints.
%            % ( not entirely sure this is still valid!! )
% %            intersection_possible = 0;
% %            for i = 1:size(options.kV_lambda{link_number}, 2)
% %                lambdas = double(k_con).*options.kV_lambda{link_number}(:, i);
% %                lambdas(~k_con) = 1;
% %                lambdas = prod(lambdas, 1)';
% %                
% %                kVc = A_con*lambdas - b_con;
% %                test_kV = max(kVc);
% %                if test_kV <= 0
% %                    intersection_possible = 1;                  
% %                end
% %            end
% %            if ~intersection_possible
% %               A_con = []; b_con = []; k_con = []; 
% %            end
%         end

        function [A] = generate_polytope_normals(obj, obstacle, options)
            % takes in an obstacle (Z matrix of zonotope) and generates
            % normal vectors to the FRS zono buffered by the obstacle
            
            buff_zono_G = [obj.RZ(:, 2:end), obstacle(:, 2:end)];
%             buff_zono_G(:, ~any(buff_zono_G)) = []; % delete zero columns
            buff_zono_c = obj.RZ(:, 1);
            buff_zono = [buff_zono_c, buff_zono_G];
            
            [~, b, A] = polytope_PH(buff_zono, options); % A are polytope normals
            x = obstacle(:, 1); % test if obs center is in zonotope.~=
            
            % add a test here that throws out unnecessary constraints.
            h = [A; -A]*x - b;
            if max(h) > 0
                % intersection not possible
                A = [];
            end
        end
        
        function [h, grad_h] = evaluate_sliced_constraints(obj, k, obs_Z, A)
            epsilon = 1e-6;
            myk = k(1:length(obj.c_k));
            
            % first, slice by k. g_sliced has been multiplied through by
            % lambda. c should be the same as obj.Rc
            [~, lambda, c, g_sliced, slice_to_pt_idx] = obj.slice(myk);
            c_poly = c + sum(g_sliced(:, slice_to_pt_idx), 2) - obs_Z(:, 1); % shift so that obstacle is zero centered!
            g_poly = [g_sliced(:, ~slice_to_pt_idx), obs_Z(:, 2:end)];
            
            % take dot product with normal vectors to construct Pb
            deltaD = sum(abs((A*g_poly)'))';
            d = A*c_poly;
            Pb = [d+deltaD; -d+deltaD];
            PA = [A; -A];
%             Pb_sign = [ones(size(A, 1)); -1*ones(size(A, 1))];
            
            % evaluate constraints
            h_obs = -Pb; % equivalent to A*[0;0;0] - b
            h_obs_max = max(h_obs);
            h = -(h_obs_max - epsilon);
            
            % evaluate constraint gradients
            max_idx = find(h_obs == h_obs_max);
            if length(max_idx) > 1
%                 disp('AHHHH');
                a = PA(max_idx, :);
%                 a = unique(a, 'rows');
            else
                a = PA(max_idx, :);
            end

            
            grad_h = zeros(length(myk), 1);
            for i = 1:length(myk)
                for j = 1:size(g_sliced, 2)
                    if obj.k_idx(i, j) == 1% if gen was multiplied by this lambda_i(k_i)
                        if lambda(i) == 0 % shit... essentially have to reslice in this case because of a 0/0 when trying to divide by lambda.
%                             error('ahhh!')
                            g_crap = obj.Rg;
                            for poop = 1:length(lambda)
                                if ~(poop == i)
                                    g_crap(:, obj.k_idx(poop, :) == 1) = g_crap(:, obj.k_idx(poop, :) == 1)*lambda(poop); % reslice gens
                                end
                            end
                            if slice_to_pt_idx(j) % this component is in d
                                grad_h(i) = grad_h(i) + min(a*g_crap(:, j)*(1/obj.g_k(i)));
                            else % this component is in deltaD
                                grad_h(i) = grad_h(i) + min(abs(a*g_crap(:, j)*(1/obj.g_k(i)))); % same as above, but with absolute value
                            end
                        else
                            if slice_to_pt_idx(j) % this component is in d
                                grad_h(i) = grad_h(i) + min(a*g_sliced(:, j)*(1/obj.g_k(i))*(1/lambda(i)));
                            else % this component is in deltaD
                                grad_h(i) = grad_h(i) + sign(lambda(i))*min(abs(a*g_sliced(:, j)*(1/obj.g_k(i))*(1/lambda(i)))); % same as above, but with absolute value
                            end
                        end
%                         disp(grad_h);
%                         pause
                    end
                end
            end
            grad_h = [grad_h; zeros(length(k) - length(myk), 1)];
%             disp(k);
            
%             grad_h = -grad_h; % important!!!
            
            
            
%                         
            %%%% OLD BELOW THIS LINE
%             Z = obj.link_FRS{j}{k}.slice(k_opt(obj.link_joints{j}));
%             c = Z(:, 1) - obs_Z(:, 1); % shift so that obstacle is zero centered!
%             G = [Z(:, 2:end), obs_Z(:, 2:end)];
%             
%             deltaD = sum(abs((obj.A{i}{j}{k}*G)'))';
%             
%             d = obj.A{i}{j}{k}*c;
%             
%             Pb = [d+deltaD; -d+deltaD];
%             
%             h_obs = -Pb; % equivalent to A*[0;0;0] - b
%             h_obs_max = max(h_obs);
%             h_tmp = -(h_obs_max - epsilon);
%             
%             h = [h; h_tmp];
%             grad_h = [grad_h, []];
%             
        end
        
    end
end

