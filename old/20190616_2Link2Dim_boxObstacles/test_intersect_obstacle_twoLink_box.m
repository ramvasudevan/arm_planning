% note: you will have to download the CORA 2018 toolbox before using this
% code: https://tumcps.github.io/CORA/

% want to generate set of unsafe link unit vectors given an obstacle
clear all; clc;

position_dim_x = [1];
param_dim_x = [3; 5];
position_dim_y = [2];
param_dim_y = [4; 6];

ploton = 1;

nObstacles = 1;
lObs = 0.05;
for i = 1:nObstacles
    % obstacles are squares with side length 0.05, randomly centered in env.
    % hold onto the corners of the obstacles.
    c = 2*rand(2,1) - 1;
    obstacle{i} = (c + [-lObs, -lObs, lObs, lObs; -lObs, lObs lObs, -lObs])';
end

load('arm_FRS_twoLink');
if ploton
    projectedDimensions=[1,2];
    figure(1); clf; axis equal;
    hold on
    %plot reachable sets
    for i=1:10:length(Rcont)
        Zproj = project(Rcont{i}{1},projectedDimensions);
        Zproj = reduce(Zproj,'girard',3);
        Zpts = polygon(Zproj)';
        p1 = fill(Zpts(:, 1), Zpts(:, 2),[0.9 0.9 0.9]);
        p1.FaceAlpha = 0;
        p1.EdgeAlpha = 0.2;
    end
    
    for k = 1:length(obstacle)
        K = convhull(obstacle{k});
        Zpts = obstacle{k}(K, :);
        p1 = fill(Zpts(:, 1), Zpts(:, 2), 'r');
        p1.FaceAlpha = 0.1;
        p1.EdgeAlpha = 0.3;
    end
    
    xlabel(['x_{',num2str(projectedDimensions(1)),'}']);
    ylabel(['x_{',num2str(projectedDimensions(2)),'}']);
end

for k = 1:length(obstacle)
   % for each obstacle, generate half plane constraints in x and y...
   A_box{1} = 1;
   A_box{2} = -1;
   b_box_x{k}{1} = max(obstacle{k}(:, 1));
   b_box_x{k}{2} = -min(obstacle{k}(:, 1));
   b_box_y{k}{1} = max(obstacle{k}(:, 2));
   b_box_y{k}{2} = -min(obstacle{k}(:, 2));
end

V = cubelet_ND(3); % will need 3D cubes for clipSH algorithm
E = table2array(graph(hypercube(3)).Edges);

tic
for k = 1:length(obstacle)
    for p = 1:2
        if p == 1
            position_dim = position_dim_x;
            param_dim = param_dim_x;
            b_box = b_box_x{k};
        elseif p == 2
            position_dim = position_dim_y;
            param_dim = param_dim_y;
            b_box = b_box_y{k};
        end
        for i = 1:length(Rcont)
            % first, buffer obstacle by k-independent generators
            % edit: for now, disregard k-independent generators
            %     obsm = obs_minus;
            %     obsp = obs_plus;
            
            frs = Rcont{i}{1};
            frs_Z = get(frs, 'Z');
            frs_c = frs_Z(:, 1);
            frs_G = frs_Z(:, 2:end);
            nGen = size(frs_G, 2);
            [~, param_col] = find(frs_G(param_dim, :) ~= 0);
            
            frs_k_ind_G = frs_G;
            frs_k_ind_G(:, param_col) = [];
            frs_k_ind_G_pos = frs_k_ind_G(position_dim, :);
            frs_k_ind_G_pos = [sum(frs_k_ind_G_pos(1, :))];
            %         frs_k_ind_G_pos = [sum(frs_k_ind_G_pos(1, :)), 0; 0, sum(frs_k_ind_G_pos(2, :))];
            frs_k_dep_G = frs_G(:, param_col);
            frs_k_dep_G_pos = frs_G(position_dim, param_col);
            
            WOO_G = [frs_k_ind_G_pos, frs_k_dep_G_pos];
            
            Vout = V;
            Eout = E;
            for j = 1:length(A_box)
                if ~isempty(Vout)
                    [Vout, Eout] = clipSH(Vout, Eout, A_box{j}*WOO_G, b_box{j});
                else
                    break
                end
            end
            if ~isempty(Vout)
                my_k_vertices{k}{p}{i} = frs_k_dep_G(param_dim, :)*Vout(:, 2:end)';
                my_k_edges{k}{p}{i} = Eout;
            else
                my_k_vertices{k}{p}{i} = [];
                my_k_edges{k}{p}{i} = [];
            end
            
        end
    end
end
toc

% unsafe x1-x2 choices
figure(2); clf; hold on;
title('unsafe x1 - x2 params');
xlabel('x1');
ylabel('x2');
xlim([-1, 1]); ylim([-1, 1]);
for k = 1:length(obstacle)
    for i = 1:10:length(my_k_vertices{k}{1})
        if ~isempty(my_k_vertices{k}{1}{i}) && ~isempty(my_k_vertices{k}{2}{i})
            myX1 = my_k_vertices{k}{1}{i}(1, :)';
            myX2 = my_k_vertices{k}{1}{i}(2, :)';
            try
                woo = convhull(myX1, myX2, 'simplify', true);
            catch
                disp('Convex hull (plot) did not work');
                woo = 1:length(myX1);
            end
            p = patch(myX1(woo), myX2(woo), 'r');
            p.FaceAlpha = 0.01;
        end
    end
end

% plot a circle
% theta = linspace(0, 2*pi, 1000);
% x = cos(theta); y = sin(theta);
% plot(x, y, 'k', 'LineWidth', 2);
% axis equal;

% unsafe y1-y2 choices
figure(3); clf; hold on;
title('unsafe y1 - y2 params');
xlabel('y1');
ylabel('y2');
xlim([-1, 1]); ylim([-1, 1]);
Y1pts = [];
Y2pts = [];
for k = 1:length(obstacle)
    for i = 1:10:length(my_k_vertices{k}{1})
        if ~isempty(my_k_vertices{k}{1}{i}) && ~isempty(my_k_vertices{k}{2}{i})
            myY1 = my_k_vertices{k}{2}{i}(1, :)';
            myY2 = my_k_vertices{k}{2}{i}(2, :)';
            try
                woo = convhull(myY1, myY2, 'simplify', true);
            catch
                disp('Convex hull (plot) did not work');
                woo = 1:length(myY1);
            end
            p = patch(myY1(woo), myY2(woo), 'r');
            p.FaceAlpha = 0.01;
        end
    end
end

% plot a circle
% theta = linspace(0, 2*pi, 1000);
% x = cos(theta); y = sin(theta);
% plot(x, y, 'k', 'LineWidth', 2);
% axis equal;

% plot some sample arm configurations showing its unsafe

for k = 1:length(obstacle)
    for i = 1:10:length(my_k_vertices{k}{1})
        if ~isempty(my_k_vertices{k}{1}{i}) && ~isempty(my_k_vertices{k}{2}{i})
            % get random x1 x2 sample:
            Ax = my_k_vertices{k}{1}{i}';
            myx = Ax(randperm(size(Ax, 1), 1), :);
            % get random y1 y2 sample:
            Ay = my_k_vertices{k}{2}{i}';
            myy = Ay(randperm(size(Ay, 1), 1), :);
            % plot
            figure(2); plot(myx(1), myx(2), 'b.', 'MarkerSize', 20);
            figure(3); plot(myy(1), myy(2), 'b.', 'MarkerSize', 20);
            
            figure(1);
            plot([0;l1*myx(1);l1*myx(1) + l2*myx(2)], [0;l1*myy(1);l1*myy(1) + l2*myy(2)], 'b:', 'LineWidth', 2);
            
        end
    end
end
