% going to try obstacle intersection for 1 link

clear all; clc;

teston = 1;

% where's your obstacle?
% obs_center = [cos(pi/4)*0.33; sin(pi/4)*0.33; 0.08];
obs_center = [0.175; 0.175; -0.11];
obs_width = [0.075];

O1 = box_obstacle_zonotope('center', obs_center(:), 'side_lengths', [obs_width, obs_width, obs_width]);

obs_center = [0.3; 0.1; -0.11];
obs_width = [0.05];

O2 = box_obstacle_zonotope('center', obs_center(:), 'side_lengths', [obs_width, obs_width, obs_width]);

obs_center = [0.2; 0.3; -0.11];
obs_width = [0.05];

O3 = box_obstacle_zonotope('center', obs_center(:), 'side_lengths', [obs_width, obs_width, obs_width]);

obs_center = [0.5; 0.1; -0.11];
obs_width = [0.075];

O4 = box_obstacle_zonotope('center', obs_center(:), 'side_lengths', [obs_width, obs_width, obs_width]);

obstacles{1} = O1;
obstacles{2} = O2;
obstacles{3} = O3;
obstacles{4} = O4;

% q_0 = zeros(4, 1);
q_0 = [0; 0];
q_dot_0 = [pi/2; pi/2];

slice_pt1 = pi/6-0.001;
slice_pt2 = -pi/6+0.001;


FRS = robot_arm_FRS_rotatotope_1link(q_0, q_dot_0);

% FRS.plot(1)
% FRS.plot_slice(ones(2,1)*slice_pt1, 2)
% FRS.plot_slice(ones(2,1)*slice_pt2, 2, {'r'});

for i = 1:length(obstacles)
    FRS = FRS.generate_constraints(obstacles(1:i));
end

figure(1); clf; hold on; view(3); axis equal;
for i = 1:length(obstacles)
    plot(obstacles{i});
    hold on;
end

pause(0.05);

xlim([-0.33, 0.33]);
ylim([-0.33, 0.33]);
zlim([-0.33, 0.33]);

box on;
xlabel('X');

% now test the constraints in FRS.A_con and FRS.b_con
if teston
    A_con = FRS.A_con;
    b_con = FRS.b_con;
    k_con = FRS.k_con;
    
    mex_A_con = FRS.mex_A_con;
    mex_b_con = FRS.mex_b_con;
    mex_k_con = FRS.mex_k_con;
    
    diff = []; % difference in Zk between original code and mex code
    
    c_k = FRS.c_k;
    g_k = FRS.g_k;
    
    figure(2); clf; hold on;
    lims = [-g_k(1) -g_k(1) g_k(1) g_k(1) -g_k(1); -g_k(2) g_k(2) g_k(2) -g_k(2) -g_k(2)];
    plot(lims(1, :)', lims(2, :)', 'k--', 'LineWidth', 4);
    
    myk = linspace(-g_k(1), g_k(1), 25);
    [Xk, Yk] = meshgrid(myk, myk);
    % Zk = zeros(size(Xk));
    for i = 1:length(myk)
        for j = 1:length(myk)
            for k = 1:length(A_con{1}{1})
                if isempty(k_con{1}{1}{k})
                    continue;
                end
                K = [Xk(i, j); Yk(i, j)];
                lambda = c_k + (K./g_k);
                lambdas = k_con{1}{1}{k}.*lambda;
                
                % dumb way to do this... want to multiply rows of lambdas
                % together, replacing zeros with ones
                lambdas(~lambdas) = 1;
                lambdas = prod(lambdas, 1)';
                
                Zk = A_con{1}{1}{k}*lambdas - b_con{1}{1}{k};
                Zk = max(Zk);
            end
            
            for k = 1:length(mex_A_con{1}{1})
                if isempty(mex_k_con{1}{1}{k})
                    continue;
                end
                K = [Xk(i, j); Yk(i, j)];
                lambda = c_k + (K./g_k);
                mex_lambdas = mex_k_con{1}{1}{k}.*lambda;
                
                % dumb way to do this... want to multiply rows of lambdas
                % together, replacing zeros with ones
                mex_lambdas(~mex_lambdas) = 1;
                mex_lambdas = prod(mex_lambdas, 1)';
                
                mex_Zk = mex_A_con{1}{1}{k}*mex_lambdas - mex_b_con{1}{1}{k};
                mex_Zk = max(mex_Zk);
                if mex_Zk <= 0
                    plot(Xk(i, j), Yk(i, j), 'r.', 'MarkerSize', 20, 'LineWidth', 6);
                end
            end
            
            diff = [diff, mex_Zk - Zk];
        end
    end
    
    disp('Click a point!');
    [p1, p2] = ginput(1);
    plot(p1, p2, 'kx', 'MarkerSize', 16, 'LineWidth', 6);
    
    figure(1);
    FRS.plot_slice([p1;p2], 1);
end

mean(abs(diff))
max(abs(diff))
            