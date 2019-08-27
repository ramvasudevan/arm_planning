%% testing the obstacle intersection for the 3D FRS

clear all; clc;
figure(1); clf; hold on; axis equal;

% where's your obstacle?
obs_center = [cos(pi/4)*0.33; sin(pi/4)*0.33; 0.08];
obs_width = [0.1];

O = box_obstacle_zonotope('center', obs_center(:), 'side_lengths', [obs_width, obs_width, obs_width]);
plot(O);

obstacles{1} = O;

%% set up arm

R{1} = eye(3);
% R{2} = make_orientation(pi/2, 1);
% R{3} = eye(3);

% phi_dot_0 = zeros(3, 1);
phi_dot_0 = pi/2;

options = struct();
options.nLinks = 1;
options.position_dimensions = [1;2;3];
options.extra_position_dimensions = [4;5];
options.IC_dimensions = [6;7];
options.param_dimensions = [8;9];
options.timeStep = 0.01;
options.t_stop = 1;
options.t_plan = 0.5;
options.L = 0.33; % in meters
options.buffer_dist = 0;

plot3(0,0,0,'k.', 'MarkerSize', 50);

% links = stack_FRS_3D(R, phi_dot_0, options);
% for i = 1:length(links)
%     for j = 1:10:length(links{i}.FRS)
% %         Z = zonotope_slice(links{i}.FRS{j}, links{i}.info.param_dimensions, k(1:2*i, 1));
%         Z = links{i}.FRS{j};
%         V = vertices(project(Z, [1, 2, 3]));
%         shp = alphaShape(V(1, :)', V(2, :)', V(3, :)', inf);
%         p = plot(shp);
%         p.FaceAlpha = 0;
%         p.EdgeAlpha = 0.15;
%         p.EdgeColor = 'b';
%     end
% end

%% compute unsafe parameters
[links, k_lim, k_unsafe_A, k_unsafe_b] = compute_unsafe_parameters_3D(R, phi_dot_0, obstacles, options);

%% plot projection of unsafe k onto traj params of first link...
% show that a safe choice of 1st link params can not hit the obs.

figure(2); clf; hold on;
uA = k_lim{1}.A;
uB = k_lim{1}.b;

upoly = mptPolytope(uA, uB);
plot(upoly);

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

%% choose a point in figure 2 for slice pt, plot FRS
slice_pt = ginput(1)';
plot(slice_pt(1), slice_pt(2), 'kx', 'MarkerSize', 20, 'LineWidth', 6);

figure(1); hold on;

for i = 1:length(links)
    for j = 1:2:length(links{i}.FRS)
        Z = zonotope_slice(links{i}.FRS{j}, links{i}.info.param_dimensions, slice_pt);
%         Z = links{i}.FRS{j};
        V = vertices(project(Z, [1, 2, 3]));
        shp = alphaShape(V(1, :)', V(2, :)', V(3, :)', inf);
        p = plot(shp);
        p.FaceAlpha = 0;
        p.EdgeAlpha = 0.15;
        p.EdgeColor = 'b';
    end
end