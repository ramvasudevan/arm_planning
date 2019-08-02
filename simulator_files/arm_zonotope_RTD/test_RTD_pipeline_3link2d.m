clear all; clc;

theta_0 = [0; 0; 0];
theta_dot_0 = [0; 0; 0];
t_plan = 0.5;

figure(1); clf; subplot(1,2,1); hold on; axis equal;
xlim([-3, 3]); ylim([-3, 3]);
xlabel('X', 'FontSize', 24);
ylabel('Y', 'FontSize', 24);
title('Robot workspace', 'FontSize', 24);

ee1 = [cos(theta_0(1)); sin(theta_0(1))];
ee2 = [ee1(1) + cos(theta_0(1) + theta_0(2)); ee1(2) + sin(theta_0(1) + theta_0(2))];
ee3 = [ee2(1) + cos(theta_0(1) + theta_0(2) + theta_0(3)); ee2(2) + sin(theta_0(1) + theta_0(2) + theta_0(3))];
plot([0; ee1(1); ee2(1); ee3(1)], [0; ee1(2); ee2(2); ee3(2)], 'k-', 'LineWidth', 2);

%% create obstacle dun dun dunn nn

obs_center = [0; 2.5];
% obs_center = [-2; 0.5];
obs_width = [0.1];
xobs = [obs_center(1) - obs_width, obs_center(1) - obs_width, obs_center(1) + obs_width, obs_center(1) + obs_width];
yobs = [obs_center(2) - obs_width, obs_center(2) + obs_width, obs_center(2) + obs_width, obs_center(2) - obs_width];
% xobs = [-1.1 -1.1 -0.9 -0.9];
% yobs = [0.3 0.5 0.5 0.3];

pobs = patch(xobs, yobs, 'r');
pobs.FaceAlpha = 0.2;

% obstacles{1} = [xobs;yobs]';

obstacles{1}.zono = zonotope([obs_center, [obs_width, 0; 0, obs_width]]);
pobs = plotFilled(obstacles{1}.zono, [1, 2], 'r');
pobs.FaceAlpha = 0.2;

while true
    [links, k_lim, k_unsafe_A, k_unsafe_b] = compute_unsafe_parameters_3link2d(theta_0, theta_dot_0, obstacles);
    
    %% link 1 and 2 unsafe;
    figure(1); subplot(1, 2, 2) ; hold on; view(3);
    % xlim([-1, 1]); ylim([-1, 1]);
    
    uA = k_lim{3}.A;
    uB = k_lim{3}.b;
    
    upoly = mptPolytope(uA, uB);
    V = vertices(upoly)
    V = get(V, 'V');
    K = convhulln(V);
    pu = fill3(V(K, 1), V(K, 2), V(K, 3), 'b');
    
    
%     plot(upoly, [1,2,3])
 
    for i = 1:length(k_unsafe_A)
        for j = 1:length(k_unsafe_A{i})
            for k = 1:length(k_unsafe_A{i}{j})
                if ~isempty(k_unsafe_A{i}{j}{k})
                    A = [k_unsafe_A{i}{j}{k}; uA];
                    B = [k_unsafe_b{i}{j}{k}; uB];
                    mypoly = mptPolytope(A, B);
                    try
                        V = vertices(mypoly);
                        V = get(V, 'V')';
                        K = convhull(V);
                        pu = patch(V(K, 1), V(K, 2), 'r');
                        pu.FaceAlpha = 0.1;
                    catch
                        warning('Unsafe polytope plotting did not work');
                    end
                end
            end
        end
    end
    
    title('Unsafe control parameters', 'FontSize', 24);
    axis equal; axis square;
    
    xlabel('u_1', 'FontSize', 24);
    ylabel('u_2', 'FontSize', 24);
    
    %% choose a point in figure 2 for slice pt, plot FRS
    slice_pt = ginput(1)';
    plot(slice_pt(1), slice_pt(2), 'kx', 'MarkerSize', 20, 'LineWidth', 6);
    
    figure(1); subplot(1, 2, 1); hold on;
    for i = 1:length(links{1}.FRS)
        Z = zonotope_slice(links{1}.FRS{i}, links{1}.info.param_dimensions, slice_pt(1));
        p = plotFilled(Z, [1, 2], 'g');
        p.FaceAlpha = 0.04;
        p.EdgeAlpha = 0.4;
    end
    
    for i = 1:length(links{2}.FRS)
        Z = zonotope_slice(links{2}.FRS{i}, links{2}.info.param_dimensions, slice_pt);
        %    Z = project(Rcont2{i}{1}, [1, 2]);
        p = plotFilled(Z, [1, 2], 'g');
        p.FaceAlpha = 0.04;
        p.EdgeAlpha = 0.4;
    end
    
    theta_0(1) = theta_0(1) + theta_dot_0(1)*t_plan + ((slice_pt(1) - theta_dot_0(1))/2)*t_plan;
    theta_0(2) = theta_0(2) + theta_dot_0(2)*t_plan + ((slice_pt(2) - slice_pt(1) - theta_dot_0(2))/2)*t_plan;
    
    theta_dot_0(1) = slice_pt(1);
    theta_dot_0(2) = slice_pt(2) - slice_pt(1);
    
    ee1 = [cos(theta_0(1)); sin(theta_0(1))];
    ee2 = [ee1(1) + cos(theta_0(1) + theta_0(2)); ee1(2) + sin(theta_0(1) + theta_0(2))];
    plot([0; ee1(1); ee2(1)], [0; ee1(2); ee2(2)], 'k-', 'LineWidth', 2);
end