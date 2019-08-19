% just want to plot the FRS to see if it looks right.

clc; clear all;

plot2D = false;
plotSim = true;
plot3D = true;

load('FRS_3D/0key')

% theta_dot_0 = pi/2;
% [~, closest_idx] = min(abs(theta_dot_0 - c_theta_dot_0));
% filename = sprintf('FRS/arm_FRS_%0.3f.mat', c_theta_dot_0(closest_idx));
% 
% load(filename);

load('/Users/pdholmes/Documents/MATLAB/armPlanning_local/20190813_arm_zonotope_RTD_3D/FRS_3D/arm_FRS_0.393_0.000.mat')

phi_dot_0 = pi/8;
theta_dot_0 = 0;
phi_dot_pk = pi/8 + pi/12 - 0.001;
theta_dot_pk = 0 - pi/12  + 0.001;

figure(1); clf; hold on; axis equal;
figure(2); clf; hold on; axis equal;
figure(3); clf; hold on; axis equal;
figure(4); clf; hold on; axis equal;

% slice_pt = [phi_dot_0; theta_dot_0; phi_dot_pk; theta_dot_pk];
% slice_dim = [4;5;6;7];

% slice_pt = [phi_dot_0; theta_dot_0];
% slice_dim = [4;5];
% 

slice_pt = [phi_dot_0; theta_dot_0; phi_dot_pk; theta_dot_pk];
slice_dim = [6;7;8;9];

% slice_pt = [phi_dot_0; theta_dot_0];
% slice_dim = [7;8];

if plot3D
    if plotSim
        R0 = zonotope_slice(options.R0_orig, slice_dim, slice_pt);
        for i = 1:100
            z = randPoint(R0);
            z(4:5) = [];
            z(end) = [];
            [tout, xout] = ode45(@arm_dyn_toPeak_3D_ODE, [0:0.01:0.5], z);
            [tout2, xout2] = ode45(@arm_dyn_toStop_3D_ODE, [0.5:0.01:1], xout(end, :)');
            figure(1); hold on;
            plot3(xout(:, 1), xout(:, 2), xout(:, 3), 'k-');
            plot3(xout2(:, 1), xout2(:, 2), xout2(:, 3), 'k-');
            figure(4); hold on;
            plot3(xout2(end, 1), xout2(end, 2), xout2(end, 3), 'k.', 'MarkerSize', 20);
        end
    end
    
    for i = 1:5:50
        figure(1); view(3);
        Z = zonotope_slice(Rcont{i}{1}, slice_dim, slice_pt);
        V = vertices(project(Z, [1, 2, 3]));
        shp = alphaShape(V(1, :)', V(2, :)', V(3, :)', inf);
        p = plot(shp);
        p.FaceAlpha = 0;
        p.EdgeAlpha = 0.15;
        p.EdgeColor = 'b';
    end
    
    for i = 51:5:100
        figure(1); view(3);
        Z = zonotope_slice(Rcont{i}{1}, slice_dim, slice_pt);
        V = vertices(project(Z, [1, 2, 3]));
        shp = alphaShape(V(1, :)', V(2, :)', V(3, :)', inf);
        p = plot(shp);
        p.FaceAlpha = 0;
        p.EdgeAlpha = 0.15;
        p.EdgeColor = 'r';
    end
%     for i = 1:1:50
%         figure(1); view(3);
%         Z = zonotope_slice(Rcont{i}{1}, slice_dim, slice_pt);
%         Z = project(Z, [1,2,3]);
%         Z = interval(Z); % overapproximate with an interval
%         lb = infimum(Z);
%         ub = supremum(Z);
%         c = mid(Z);
%         [F, V] = make_cuboid_for_patch(ub(1) - lb(1), ub(2) - lb(2), ub(3) - lb(3), c);
%         %     V = vertices(Z);
%         %     V = get(V, 'V')';
%         %     s = surf2patch(V(:, 1), V(:, 2), V(:, 3));
%         %     p = patch(s);
%         %     p = patch(V(:, 1), V(:, 2), V(:, 3), 'b');
%         p = patch('Faces', F, 'Vertices', V);
%         %     p = plotFilled(Z, [1, 2], 'b');
%         p.FaceAlpha = 0.02;
%         p.FaceColor = 'b';
%         p.EdgeColor = 'k';
%         p.EdgeAlpha = 0.2;
%         %     pause;
%     end
%     
%     for i = 51:1:100
%         figure(1); view(3);
%         Z = zonotope_slice(Rcont{i}{1}, slice_dim, slice_pt);
%         Z = project(Z, [1,2,3]);
%         Z = interval(Z); % overapproximate with an interval
%         lb = infimum(Z);
%         ub = supremum(Z);
%         c = mid(Z);
%         [F, V] = make_cuboid_for_patch(ub(1) - lb(1), ub(2) - lb(2), ub(3) - lb(3), c);
%         %     V = vertices(Z);
%         %     V = get(V, 'V')';
%         %     s = surf2patch(V(:, 1), V(:, 2), V(:, 3));
%         %     p = patch(s);
%         %     p = patch(V(:, 1), V(:, 2), V(:, 3), 'b');
%         p = patch('Faces', F, 'Vertices', V);
%         %     p = plotFilled(Z, [1, 2], 'b');
%         p.FaceAlpha = 0.02;
%         p.FaceColor = 'r';
%         p.EdgeColor = 'k';
%         p.EdgeAlpha = 0.2;
%         %     pause;
%     end
    
end


if plot2D
    if plotSim
        R0 = zonotope_slice(options.R0_orig, slice_dim, slice_pt);
        for i = 1:100
            z = randPoint(R0);
            z(4:5) = [];
            z(end) = [];
            [tout, xout] = ode45(@arm_dyn_toPeak_3D_ODE, [0:0.01:0.5], z);
            [tout2, xout2] = ode45(@arm_dyn_toStop_3D_ODE, [0.5:0.01:1], xout(end, :)');
            figure(1); hold on;
            plot(xout(:, 1), xout(:, 2), 'k-');
            plot(xout2(:, 1), xout2(:, 2), 'k-');
            figure(2); hold on;
            plot(xout(:, 2), xout(:, 3), 'k-');
            plot(xout2(:, 2), xout2(:, 3), 'k-');
            figure(3); hold on;
            plot(xout(:, 1), xout(:, 3), 'k-');
            plot(xout2(:, 1), xout2(:, 3), 'k-');
        end
    end
    % 1-2 proj.
    for i = 1:1:50
        Z = zonotope_slice(Rcont{i}{1}, slice_dim, slice_pt);
        figure(1); hold on;
        p = plotFilled(Z, [1, 2], 'b');
        p.FaceAlpha = 0.02;
        p.FaceColor = 'none';
        p.EdgeColor = 'k';
        p.EdgeAlpha = 0.2;
        %     pause;
    end
    for i = 51:1:100
        Z = zonotope_slice(Rcont{i}{1}, slice_dim, slice_pt);
        figure(1); hold on;
        p = plotFilled(Z, [1, 2], 'r');
        p.FaceAlpha = 0.01;
        p.FaceColor = 'r';
        p.EdgeColor = 'k';
        p.EdgeAlpha = 0.2;
        %     pause;
    end
    % 2-3 proj.
    for i = 1:1:50
        Z = zonotope_slice(Rcont{i}{1}, slice_dim, slice_pt);
        figure(2); hold on;
        p = plotFilled(Z, [2, 3], 'b');
        p.FaceAlpha = 0.02;
        p.FaceColor = 'none';
        p.EdgeColor = 'k';
        p.EdgeAlpha = 0.2;
%             pause;
    end
    for i = 51:1:100
        Z = zonotope_slice(Rcont{i}{1}, slice_dim, slice_pt);
        figure(2); hold on;
        p = plotFilled(Z, [2, 3], 'r');
        p.FaceAlpha = 0.01;
        p.FaceColor = 'r';
        p.EdgeColor = 'k';
        p.EdgeAlpha = 0.2;
        %     pause;
    end
        % 2-3 proj.
    for i = 1:1:50
        Z = zonotope_slice(Rcont{i}{1}, slice_dim, slice_pt);
        figure(3); hold on;
        p = plotFilled(Z, [1, 3], 'b');
        p.FaceAlpha = 0.02;
        p.FaceColor = 'none';
        p.EdgeColor = 'k';
        p.EdgeAlpha = 0.2;
        %     pause;
    end
    for i = 51:1:100
        Z = zonotope_slice(Rcont{i}{1}, slice_dim, slice_pt);
        figure(3); hold on;
        p = plotFilled(Z, [1, 3], 'r');
        p.FaceAlpha = 0.01;
        p.FaceColor = 'r';
        p.EdgeColor = 'k';
        p.EdgeAlpha = 0.2;
        %     pause;
    end
end