% trying out the stack_FRS_3D function

clear all; clc;

R = cell(3, 1);
% R{1} = eye(3);
% R{2} = make_orientation(-pi/4, 2);
% R{3} = make_orientation(-pi/2, 2);
R{1} = make_orientation(rand(1)*2*pi, 3)*make_orientation(rand(1)*2*pi, 2)*make_orientation(rand(1)*2*pi, 1);
R{2} = make_orientation(rand(1)*2*pi, 3)*make_orientation(rand(1)*2*pi, 2)*make_orientation(rand(1)*2*pi, 1);
R{3} = make_orientation(rand(1)*2*pi, 3)*make_orientation(rand(1)*2*pi, 2)*make_orientation(rand(1)*2*pi, 1);


phi_dot_0 = zeros(3, 1);
phi_dot_0(1) = rand(1)*2*pi - pi;
phi_dot_0(2) = rand(1)*2*pi - pi;
phi_dot_0(3) = rand(1)*2*pi - pi;
% phi_dot_0(1) = pi;
% phi_dot_0(2) = pi;
% phi_dot_0(3) = -pi;

options = struct();
options.nLinks = 3;
options.position_dimensions = [1;2;3];
options.extra_position_dimensions = [4;5];
options.IC_dimensions = [6;7];
options.param_dimensions = [8;9];
options.timeStep = 0.01;
options.T = 1;
options.tPlan = 0.5;
options.L = 0.33; % in meters

links = stack_FRS_3D(R, phi_dot_0, options);

% now slice links and plot
k = [phi_dot_0(1)+pi/12-pi/200;...
    pi/12-pi/200;...
    phi_dot_0(2)+pi/12-pi/200;...
    pi/12-pi/200;...
    phi_dot_0(3)+pi/12-pi/200;...
    pi/12-pi/200];

% k = [phi_dot_0(1);...
%     pi/12;...
%     phi_dot_0(2);...
%     pi/12;...
%     phi_dot_0(3);...
%     pi/12];
% 
% k = [phi_dot_0(1);...
%     0;...
%     phi_dot_0(2);...
%     0;...
%     phi_dot_0(3);...
%     0];

% k{1} = [phi_dot_0(1)+pi/12-0.001;...
%     pi/12-0.001];
% 
% k{2} = [k{1};...
%     phi_dot_0(2)+pi/12-0.001;...
%     pi/12-0.001];
% 
% k{3} = [k{2};...
%     phi_dot_0(3)+pi/12-0.001;...
%     pi/12-0.001];


figure(1); clf; hold on; axis equal;
plot3(0,0,0,'k.', 'MarkerSize', 50);

% for i = 1:length(links);
%     for j = 1:10:length(links{i}.FRS)
% %         Z = zonotope_slice(links{i}.FRS{j}, links{i}.info.param_dimensions, k{i});
%         Z = links{i}.FRS{j};
%         V = vertices(project(Z, [1, 2, 3]));
%         shp = alphaShape(V(1, :)', V(2, :)', V(3, :)', inf);
%         p = plot(shp);
%         p.FaceAlpha = 0;
%         p.EdgeAlpha = 0.15;
%         p.EdgeColor = 'b';
%     end
% end

% sliced link
for i = 1:length(links)
    for j = 1:10:length(links{i}.FRS)
        Z = zonotope_slice(links{i}.FRS{j}, links{i}.info.param_dimensions, k(1:2*i, 1));
%         Z = links{i}.FRS{j};
        V = vertices(project(Z, [1, 2, 3]));
        shp = alphaShape(V(1, :)', V(2, :)', V(3, :)', inf);
        p = plot(shp);
        p.FaceAlpha = 0;
        p.EdgeAlpha = 0.15;
        p.EdgeColor = 'b';
    end
end

%% generate trajectory
X = generate_trajectory_from_k(R, phi_dot_0, k, options);

for j = 1:(size(X, 1)/3)
   plot3(X(3*(j-1)+1, :)',  X(3*(j-1)+2, :)', X(3*(j-1)+3, :)', 'k-', 'LineWidth', 4);
end

%% test that the trajectory is actually completely contained within zonotope:
flag = 0;
maxdist = 0;
maxdist_idx = [0;0];
tic
for j = 1:(size(X, 1)/3)
    for i = 1:size(X, 2) - 1
        Z = zonotope_slice(links{j}.FRS{i}, links{j}.info.param_dimensions, k(1:2*j, 1));
%         myZ = project(Z, [1, 2, 3]);
        myZ = Z;
        [dist, ~] = check_inZonotope(myZ, X(3*(j-1)+1:3*j, i), links{j}.info.position_dimensions);
        if dist > maxdist
            maxdist = dist;
            maxdist_idx = [i;j];
            maxdist_pt = X(3*(j-1)+1:3*j, i);
            maxdist_Z = Z;
        end
        if dist > 1
            flag = 1;
        end
    end
end
toc
disp(['If this number is less than 1, then all traj points contained in zonotopes: ', num2str(maxdist)]);
% disp(flag);

%% now, try to do IK that solves for fetch generalized coordinates that produce the desired joint location trajectories

Q = get_fetch_q_from_traj(X);
checkX = zeros(9, size(Q, 2));
% check that the FK of Q lines up with X
for i = 1:size(Q, 2)
    checkX(:, i) = joints_FK(Q(:, i));
end

for j = 1:(size(checkX, 1)/3)
   plot3(checkX(3*(j-1)+1, :)',  checkX(3*(j-1)+2, :)', checkX(3*(j-1)+3, :)', 'r--', 'LineWidth', 4);
end
