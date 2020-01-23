% going to try obstacle intersection for 1 link

clear all; clc;
figure(1); clf; hold on; view(3); axis equal;

teston = 0;
testgradon = 1;

% where's your obstacle?
% obs_center = [cos(pi/4)*0.33; sin(pi/4)*0.33; 0.08];
obs_center = [0.2; 0.2; -0.11];
obs_width = [0.1];
% obs_center = [0.08; 0.3; -0.15];
% obs_width = [0.1];

O = box_obstacle_zonotope('center', obs_center(:), 'side_lengths', [obs_width, obs_width, obs_width]);
plot(O);

obstacles{1} = O;

% q_0 = zeros(4, 1);
q_0 = [0; 0];
% q_dot_0 = [pi/2; 0.4];
q_dot_0 = [pi/2; pi/2];

% slice_pt1 = pi/6-0.001;
% slice_pt2 = -pi/6+0.001;
slice_pt = [ pi/6 - 0.001; pi/6 - 0.001];

tic
FRS = robot_arm_FRS_rotatotope_1link(q_0, q_dot_0);
toc
% FRS.plot(10)
% FRS.plot_slice(ones(2,1)*slice_pt1, 2)
% FRS.plot_slice(slice_pt, 10, {'r'});

tic
FRS = FRS.generate_polytope_normals(obstacles);
toc

pause(0.05);

xlim([-0.33, 0.33]);
ylim([-0.33, 0.33]);
zlim([-0.33, 0.33]);

box on;
xlabel('X');

% now test the constraints in FRS.A
if teston
    A = FRS.A;
    
    c_k = FRS.c_k;
    g_k = FRS.g_k;
    
    figure(2); clf; hold on; axis equal;
    lims = [-g_k(1) -g_k(1) g_k(1) g_k(1) -g_k(1); -g_k(2) g_k(2) g_k(2) -g_k(2) -g_k(2)];
    plot(lims(1, :)', lims(2, :)', 'k--', 'LineWidth', 4);
    
    myk = linspace(-g_k(1), g_k(1), 50);
    [Xk, Yk] = meshgrid(myk, myk);
    Zk_save = {};
    % Zk = zeros(size(Xk));
    for i = 1:length(myk)
        for j = 1:length(myk)
%             for k = 1:length(FRS.A{1}{1})
%                 if isempty(FRS.A{1}{1}{k})
%                     continue;
%                 end
                K = [Xk(i, j); Yk(i, j)];
%                 lambda = c_k + (K./g_k);
%                 lambdas = k_con{1}{1}{k}.*lambda;
%                 
%                 % dumb way to do this... want to multiply rows of lambdas
%                 % together, replacing zeros with ones
%                 lambdas(~lambdas) = 1;
%                 lambdas = prod(lambdas, 1)';
%                 
%                 Zk = A_con{1}{1}{k}*lambdas - b_con{1}{1}{k};
                h = FRS.evaluate_sliced_constraints(K, obstacles);
                Zk = max(h);
                if Zk > 0
                    plot(Xk(i, j), Yk(i, j), 'r.', 'MarkerSize', 20, 'LineWidth', 6);
                end
%             end
                k_save{i}{j} = K;
                Zk_save{i}{j} = Zk;
        end
    end
    
    disp('Click a point!');
    [p1, p2] = ginput(1);
    plot(p1, p2, 'kx', 'MarkerSize', 16, 'LineWidth', 6);
    
    figure(1);
    FRS.plot_slice([p1;p2], 1, {'b'});
end

if testgradon
    k1 = [-0.1; 0.5];
    k_step_size = 1e-6;
    deltak = k_step_size*[1; 0];
    k1plus = k1 + deltak;
    
    [h1, gradh1] = FRS.evaluate_sliced_constraints(k1, obstacles);
    [h1plus, gradh1plus] = FRS.evaluate_sliced_constraints(k1plus, obstacles);
    
    gradcheck = (h1plus - h1)./k_step_size - gradh1(1, :)';
    disp(gradcheck);
end
            