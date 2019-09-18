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
        red_order = 10;
        
        % hold on to dimensions of first zonotope
        pos_dim = [1, 2];
        k_dim = [3];
        
        % helpful for slicing along k later on
        % C, G, are center and generator of matzonotope representing
        % rotation matrices
        % c, g, are center and generator of zonotope representing set.
        
        C = {};
        G = {};
        G_k = {};
        c = [];
        g = [];
        c_k = [];
        g_k = [];

        Rc = []; % resulting zonotope center
        Rg = []; % resulting zonotope generator matrix
        RZ = []; % resulting zonotope [c, g]
        
        c_idx = false(0); % keep track of which generators get sliced to a point
        k_idx = false(0); % keep track of which generators depend on which k
    end
    
    methods
        function obj = rotatotope(varargin)
            %ROTATOTOPE
%             if nargin >= 3
%                 % parse the input arguments:
%                 for idx = 1:2:length(varargin)-1
%                     % check validity
%                     if isa(varargin{idx}, 'zonotope')
%                         obj.R{end+1, 1} = varargin{idx};
%                     else
%                         error('Specify zonotopes representing sin and cos of a rotation');
%                     end
%                     obj.rot_axes(end+1, 1) = varargin{idx+1};
%                 end
%                 if isa(varargin{end}, 'zonotope')
%                     obj.Z = varargin{end};
%                 else
%                     error('Specify the set to rotate as a zonotope');
%                 end
%             else
%                 error('Specify at least 3 arguments to rotatotope');
%             end

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
                
                obj.C{idx, 1} = obj.make_orientation(c(1, 1), c(2, 1), idx, 1);
                
                G_k = G(:, k_col);
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
            
            % pat's notes: to implement, try keeping track of
            % the indices that the generators depend on. also keep track of
            % whether slicing will evaluate to a point or to a generator.

            RZ = [obj.c, obj.g];
                                    
            c_idx = false(1, size(RZ, 2));
            c_idx(1) = true;
            k_idx = false(0);
        
            for idx = length(obj.R):-1:1
                
                c_idx_new = false(0);
                k_idx_new = false(0);
                RZ_new = [];
                
                RZ_new = obj.C{idx}*RZ; % new center is C*c
                c_idx_new = [c_idx_new, c_idx];
                k_idx_new = [k_idx_new, [false(size(c_idx)); k_idx]];

                % multiply k dep matrices by generators
                for i = 1:length(obj.G_k{idx})
                    RZ_new(:, (end+1):(end+size(RZ, 2))) = obj.G_k{idx}{i}*RZ;
                    c_idx_new = [c_idx_new, c_idx];
                    k_idx_new = [k_idx_new, [true(size(c_idx)); k_idx]];
                end

                % multiply non k dep matrices by generators
                for i = 1:length(obj.G{idx})
                    RZ_new(:, (end+1):(end+size(RZ, 2))) = obj.G{idx}{i}*RZ;
                    c_idx_new = [c_idx_new, c_idx];
                    k_idx_new = [k_idx_new, [false(size(c_idx)); k_idx]];
                end
                
                [RZ, c_idx, k_idx] = obj.reduce(RZ_new, c_idx_new, k_idx_new);
                
%                 RZ = RZ_new;
%                 c_idx = c_idx_new;
%                 k_idx = k_idx_new;
                
%                 % reduce the size of the zonotope:
%                 c_idx_temp = c_idx_new(1, 2:end);
%                 k_idx_temp = k_idx_new(:, 2:end);
%                 
%                 Rc = RZ(:, 1);
%                 Rg = RZ(:, 2:end);
%                 
%                 gc_red_idx = ~any(k_idx_temp, 1) & any(c_idx_temp, 1);
%                 gnoc_red_idx = ~any(k_idx_temp, 1) & ~any(c_idx_temp, 1);
%                 
%                 % reduction
%                 Rgk = Rg(:, any(k_idx_temp, 1));
%                 Rgc_red = obj.reduce(Rg(:, gc_red_idx), obj.red_order);
%                 Rgnoc_red = obj.reduce(Rg(:, gnoc_red_idx), obj.red_order);
%                                 
%                 % update RZ, c_idx, k_idx
%                 k_rows = size(k_idx_temp, 1);
%                 Rgc_red_cols = size(Rgc_red, 2);
%                 Rgnoc_red_cols = size(Rgnoc_red, 2);
%                 RZ = [Rc, Rgk, Rgc_red, Rgnoc_red];
%                 c_idx = [c_idx(1), c_idx_temp(:, any(k_idx_temp, 1)), true(1, Rgc_red_cols), false(1, Rgnoc_red_cols)];
%                 k_idx = [k_idx(:, 1), k_idx_temp(:, any(k_idx_temp, 1)), false(k_rows, Rgc_red_cols), false(k_rows, Rgnoc_red_cols)];
%                 
            end
            
            obj.RZ = RZ;
            obj.Rc = RZ(:, 1);
            obj.Rg = RZ(:, 2:end);
            obj.c_idx = c_idx(1, 2:end);
            obj.k_idx = k_idx(:, 2:end);
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
            switch obj.dim
                case 2
                    p = plotFilled(Z, [1, 2], color);
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
        
        function [Z] = slice_gensIncluded(obj, k)
            % take in a value for k, slice along dimension
            if length(k) ~= length(obj.c_k)
                error('Slice point not correct dimension');
            end
            g = obj.Rg;
            c = obj.Rc;
%             for i = length(k):-1:1
            for i = 1:length(k)
                if abs(k(i) - obj.c_k(i)) > obj.g_k(i)
                    error('Slice point is out of bounds');
                end
                lambda = (k(i) - obj.c_k(i))/obj.g_k(i);
                
                g(:, obj.k_idx(i, :))  = g(:, obj.k_idx(i, :))*lambda; % slice gens
            end
            
            % take the k dep gens that slice to points... add to center
            c = c + sum(g(:, any(obj.k_idx, 1) & obj.c_idx), 2);
            g(:, any(obj.k_idx, 1) & obj.c_idx) = [];
            
            Z = zonotope([c, g]);
        end
        
        function [Z] = slice(obj, k)
            % take in a value for k, slice along dimension
            if length(k) ~= length(obj.c_k)
                error('Slice point not correct dimension');
            end
            g = obj.Rg;
            c = obj.Rc;
%             for i = length(k):-1:1
            for i = 1:length(k)
                if abs(k(i) - obj.c_k(i)) > obj.g_k(i)
                    error('Slice point is out of bounds');
                end
                lambda = (k(i) - obj.c_k(i))/obj.g_k(i);
                
                slice_idx =  obj.k_idx(i, :) & obj.c_idx; % only gens that slice to c
                g(:, slice_idx)  = g(:, slice_idx)*lambda; % slice gens
            end
            
            % take the k dep gens that slice to points... add to center
            c = c + sum(g(:, any(obj.k_idx, 1) & obj.c_idx), 2);
            g(:, any(obj.k_idx, 1) & obj.c_idx) = [];
            
            Z = zonotope([c, g]);
        end
        
        function [Z_new, c_idx_new, k_idx_new] = reduce(obj, Z, c_idx, k_idx)
            % based off of the "reduceGirard" function included in CORA
            %default values
            
            c = Z(:, 1);
            G = Z(:, 2:end);
            
            c_idx_temp = c_idx(:, 2:end);
            k_idx_temp = k_idx(:, 2:end);
            
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
                    c_idx_new = [c_idx(:, 1), c_idx_temp(:, indices((nReduced+1):end)), false(1, size(Gbox, 2))];
                    k_idx_new = [k_idx(:, 1), k_idx_temp(:, indices((nReduced+1):end)), false(size(k_idx, 1), size(Gbox, 2))];
                else
                    Gunred = G;
                    Z_new = [c, Gunred];
                    c_idx_new = [c_idx(:, 1), c_idx_temp];
                    k_idx_new = [k_idx(:, 1), k_idx_temp];
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
           
           obj.k_idx = [obj.k_idx, [base.k_idx; false(k_rows_1 - k_rows_2, k_cols_2)]];
        end
        
        function [A_con, b_con, k_con] = generate_constraints(obj, obstacle, options, link_number)
           % takes in an obstacle (Z matrix of zonotope) and generates linear constraint matrices
           % Acon and bcon, as well as the combinations of k_idxs (kcon) these
           % constraints depend on.
           
           nGen = size(obj.Rg, 2);
           [~, kc_col] = find(any(obj.k_idx) & obj.c_idx);
           
           frs_k_ind_G = obj.Rg;
           frs_k_ind_G(:, kc_col) = [];
           
           frs_k_dep_G = obj.Rg(:, kc_col);
           
           buff_obstacle = [obstacle(:, 1) - obj.Rc, obstacle(:, 2:end), frs_k_ind_G];
           
           [A_poly, b_poly] = polytope_PH(buff_obstacle, options);
           
           A_con = A_poly*frs_k_dep_G;
           b_con = b_poly;
           k_con = obj.k_idx(:, kc_col);
           
           % add a test here that throws out unnecessary constraints.
           % ( not entirely sure this is still valid!! )
           intersection_possible = 0;
           for i = 1:size(options.kV_lambda{link_number}, 2)
               lambdas = k_con.*options.kV_lambda{link_number}(:, i);
               lambdas(~lambdas) = 1;
               lambdas = prod(lambdas, 1)';
               
               kVc = A_con*lambdas - b_con;
               test_kV = max(kVc);
               if test_kV <= 0
                   intersection_possible = 1;                  
               end
           end
           if ~intersection_possible
              A_con = []; b_con = []; k_con = []; 
           end
        end
    end
end

